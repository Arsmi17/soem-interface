#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>

#define NUM_SLAVES 3
volatile sig_atomic_t keep_running = 1;

// Adjust this structure to match your PDO mapping for Cyclic Synchronous Velocity mode
typedef struct __attribute__((packed))
{
    uint16_t control_word;          // 0x6040
    int32_t target_velocity;        // 0x60FF - Target velocity for Cyclic Synchronous Velocity Mode
    uint8_t mode_of_operation;      // 0x6060
} el7_out_t;

typedef struct __attribute__((packed))
{
    uint16_t status_word;           // 0x6041
    int32_t actual_position;        // 0x6064
    int32_t actual_velocity;        // 0x606C
    int16_t actual_torque;          // 0x6077
    uint8_t mode_of_operation_display; // 0x6061
} el7_in_t;

char IOmap[4096];
el7_out_t *out_data[NUM_SLAVES];
el7_in_t *in_data[NUM_SLAVES];

void setup_keyboard()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void restore_keyboard()
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

void send_and_receive()
{
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    usleep(1000);
}

// Set mode to Cyclic Synchronous Velocity Mode via SDO
void set_csv_mode(int slave_idx)
{
    uint8_t mode = 9;  // Cyclic Synchronous Velocity Mode

    printf("Setting slave %d to Cyclic Synchronous Velocity Mode (9)...\n", slave_idx + 1);
    if (ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE)) {
        printf("Successfully set mode of operation to Cyclic Synchronous Velocity Mode (9)\n");
    } else {
        printf("Failed to set mode of operation! Will try via PDO.\n");
        // Try via PDO as fallback
        out_data[slave_idx]->mode_of_operation = 9;
        send_and_receive();
    }
    usleep(50000);  // Give time for mode change to take effect
}

// Verify the current mode
void verify_mode_of_operation(int slave_idx)
{
    uint8_t mode;
    int size = sizeof(mode);
    int result = ec_SDOread(slave_idx + 1, 0x6061, 0x00, FALSE, &size, &mode, EC_TIMEOUTSAFE);
    
    if (result == 1) {
        printf("Current mode of operation for slave %d is: %d\n", slave_idx + 1, mode);
        if (mode != 9) {
            printf("WARNING: Actual mode is not Cyclic Synchronous Velocity Mode (9)!\n");
        }
    } else {
        printf("Failed to read mode of operation display for slave %d\n", slave_idx + 1);
    }
}

// Set velocity for cyclic synchronous velocity mode
void set_velocity(int slave_idx, int32_t velocity)
{
    // In CSV mode, we directly update target_velocity which is sent every cycle
    out_data[slave_idx]->target_velocity = velocity;
    
    // Make sure control word enables operation
    if ((in_data[slave_idx]->status_word & 0x006F) != 0x0027) {  // Check if not enabled
        out_data[slave_idx]->control_word = 0x000F;  // Enable operation
    }
    
    // No need to read back status in cyclic mode as it's continuously updated
}

void stop_motor(int slave_idx)
{
    // In CSV mode, just set velocity to 0
    out_data[slave_idx]->target_velocity = 0;
}

void reset_fault(int slave_idx)
{
    printf("Resetting fault on drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x80;
    send_and_receive();
    usleep(10000);
    
    // Clear the fault bit and restore operation
    out_data[slave_idx]->control_word = 0x0F;
    send_and_receive();
}

void enable_drive(int slave_idx) {
    printf("Enabling drive %d...\n", slave_idx + 1);
    
    // First check for faults and reset if needed
    send_and_receive();
    uint16_t status_word = in_data[slave_idx]->status_word;
    
    // Check if fault bit (bit 3) is set
    if (status_word & 0x0008) {
        printf("Fault detected on drive %d (Status: 0x%04X). Resetting...\n", 
               slave_idx + 1, status_word);
        reset_fault(slave_idx);
        usleep(50000); // Give time for fault to clear
    }

    // State transition sequence
    printf("State transition sequence...\n");
    
    // Step 1: Shutdown (if not already in that state)
    out_data[slave_idx]->control_word = 0x0006; // Shutdown
    send_and_receive();
    usleep(50000);
    
    // Step 2: Switch On
    out_data[slave_idx]->control_word = 0x0007; // Switch On
    send_and_receive();
    usleep(50000);
    
    // Step 3: Enable Operation
    out_data[slave_idx]->control_word = 0x000F; // Enable Operation
    send_and_receive();
    usleep(50000);
    
    // Check final status
    send_and_receive();
    status_word = in_data[slave_idx]->status_word;
    printf("Drive %d status after enable sequence: 0x%04X\n", slave_idx + 1, status_word);
    
    // Check if we're in "Operation enabled" state (bits 0,1,2,5 set: 0x0027)
    if ((status_word & 0x006F) == 0x0027) {
        printf("Drive %d successfully enabled\n", slave_idx + 1);
    } else {
        printf("WARNING: Drive %d may not be fully enabled (Status: 0x%04X)\n", 
               slave_idx + 1, status_word);
    }
}

void disable_drive(int slave_idx)
{
    printf("Disabling drive %d...\n", slave_idx + 1);
    // First stop the motor
    stop_motor(slave_idx);
    usleep(50000);
    // Then disable the drive
    out_data[slave_idx]->control_word = 0x0000;
    send_and_receive();
}

void int_handler(int sig)
{
    (void)sig;
    printf("\nCaught SIGINT (Ctrl+C), stopping motors...\n");

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        stop_motor(i);
        usleep(10000);
        disable_drive(i);
    }

    ec_close();
    restore_keyboard();

    exit(0);
}

int selected_slave = 0;  // Default to first slave

void select_slave(int slave_idx)
{
    if (slave_idx < 0 || slave_idx >= NUM_SLAVES) {
        printf("Invalid slave index %d. Valid range is 0 to %d.\n", slave_idx, NUM_SLAVES - 1);
        return;
    }
    selected_slave = slave_idx;
    printf("Slave %d selected for movement.\n", selected_slave + 1);
}

int main(int argc, char **argv)
{
    signal(SIGINT, int_handler);
    
    // Network interface
    char *ifname = "enp0s31f6";  // Default interface
    
    // Check if interface name is provided as argument
    if (argc > 1) {
        ifname = argv[1];
    }
        
    printf("Using network interface: %s\n", ifname);
    
    if (!ec_init(ifname))  // Use provided or default interface
    {
        printf("Failed to initialize EtherCAT interface.\n");
        return -1;
    }

    if (ec_config_init(FALSE) < NUM_SLAVES)
    {
        printf("Not enough slaves found!\n");
        ec_close();
        return -1;
    }

    ec_config_map(&IOmap);

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        out_data[i] = (el7_out_t *)ec_slave[i + 1].outputs;
        in_data[i] = (el7_in_t *)ec_slave[i + 1].inputs;

        // Set important motion parameters 
        uint32_t max_torque = 1000;       // Maximum torque value
        int32_t max_velocity = 500000;    // Maximum velocity value

        // Configure parameters via SDO
        ec_SDOwrite(i + 1, 0x6072, 0x00, FALSE, sizeof(max_torque), &max_torque, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);

        // Set to Cyclic Synchronous Velocity Mode
        set_csv_mode(i);
    }

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_OPERATIONAL;
        ec_writestate(i + 1);
    }
    usleep(100000);  // Longer delay to ensure transition

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        if (ec_statecheck(i + 1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
        {
            printf("Slave %d failed to reach OPERATIONAL state.\n", i + 1);
            ec_close();
            return -1;
        }
    }

    // Verify that the mode was properly set
    for (int i = 0; i < NUM_SLAVES; ++i) {
        verify_mode_of_operation(i);
        // Initialize target velocity to 0
        out_data[i]->target_velocity = 0;
    }

    printf("Slaves are OPERATIONAL. Use keys:\n");
    printf("[1-5] Select slave  [i] Enable all drives  [o] Disable all drives  [p] Reset faults  [c] Verify mode\n");
    printf("[q] Press and hold for forward motion  [w] Press and hold for reverse motion\n");
    printf("[+] Increase speed  [-] Decrease speed\n");
    printf("[ESC] Quit\n");

    setup_keyboard();
    int running = 1;
    int velocity_level = 160000;  // Start with a lower velocity (adjust as needed)
    int velocity_increment = 500;
    int q_pressed = 0;
    int w_pressed = 0;
    
    // Print initial status
    printf("Starting control loop, selected slave: %d\n", selected_slave + 1);
    printf("Current velocity level: %d\n", velocity_level);

    while (running)
    {
        // Read keyboard input
        int ch = getchar();
        
        // Handle key press
        switch (ch)
        {
            case '1':
            case '2':
            case '3': 
            case '4':
            case '5':
                {
                    int idx = ch - '1';  // Convert ASCII character to zero-based index
                    select_slave(idx);
                }
                break;

            case 'i':
                for (int i = 0; i < NUM_SLAVES; i++)
                    enable_drive(i);
                break;

            case 'o':
                for (int i = 0; i < NUM_SLAVES; i++)
                    disable_drive(i);
                break;

            case 'p':
                for (int i = 0; i < NUM_SLAVES; i++)
                    reset_fault(i);
                break;

            case 'c':
                for (int i = 0; i < NUM_SLAVES; i++)
                    verify_mode_of_operation(i);
                break;

            case 'q':
                q_pressed = 1;  // Mark key as pressed
                printf("Forward motion at velocity %d for slave %d (hold key)...\n", 
                       velocity_level, selected_slave + 1);
                break;

            case 'w':
                w_pressed = 1;  // Mark key as pressed
                printf("Reverse motion at velocity %d for slave %d (hold key)...\n", 
                       -velocity_level, selected_slave + 1);
                break;

            case '+': // Increase speed
                velocity_level += velocity_increment;
                printf("Velocity level increased to %d\n", velocity_level);
                break;

            case '-': // Decrease speed
                velocity_level -= velocity_increment;
                if (velocity_level < velocity_increment)
                    velocity_level = velocity_increment;
                printf("Velocity level decreased to %d\n", velocity_level);
                break;

            case 27: // ESC key
                running = 0;
                break;
        }

        // Check for specific key release events
    // In non-blocking mode, we need a better way to detect key releases
    // We'll use a counter-based approach to debounce and detect real key releases
    static int key_absent_count = 0;
    const int KEY_RELEASE_THRESHOLD = 5; // Wait for several cycles without key input to consider a release
    
    if (ch == -1) {
        // No new key pressed this cycle
        key_absent_count++;
        
        // If we've gone several cycles without a key press and a key was previously marked as pressed
        if (key_absent_count >= KEY_RELEASE_THRESHOLD && (q_pressed || w_pressed)) {
            // Check if another key press is actively happening
            int another_key = 0;
            
            // Try to read any pending keys in the buffer
            char buffer[10];
            int n = read(STDIN_FILENO, buffer, sizeof(buffer));
            
            if (n > 0) {
                // Check for our movement keys in the buffer
                for (int i = 0; i < n; i++) {
                    if (buffer[i] == 'q' || buffer[i] == 'w') {
                        another_key = buffer[i];
                        break;
                    }
                }
            }
            
            // If we found our movement key still in the buffer, keep the state
            if (another_key == 'q') {
                q_pressed = 1;
                w_pressed = 0;
                key_absent_count = 0;
            } else if (another_key == 'w') {
                q_pressed = 0;
                w_pressed = 1;
                key_absent_count = 0;
            } else {
                // No movement keys found in buffer, consider released
                q_pressed = 0;
                w_pressed = 0;
                stop_motor(selected_slave);
                printf("Key released - stopping slave %d\n", selected_slave + 1);
            }
        }
    } else {
        // Reset counter when a key is detected
        key_absent_count = 0;
    }

        // Set velocity based on key state
        if (q_pressed) {
            set_velocity(selected_slave, velocity_level);
        } else if (w_pressed) {
            set_velocity(selected_slave, -velocity_level);
        }

        // Process EtherCAT communication
        send_and_receive();

        // Print status periodically (uncomment if needed)
        /*
        static int counter = 0;
        if (++counter % 100 == 0) {  // Print every 100 cycles
            printf("Slave %d -> Status: 0x%04X | Vel: %d | Pos: %d\n",
                   selected_slave + 1,
                   in_data[selected_slave]->status_word,
                   in_data[selected_slave]->actual_velocity,
                   in_data[selected_slave]->actual_position);
        }
        */

        // Status printing - uncomment to see more details
        /*
        static int print_counter = 0;
        if (++print_counter % 100 == 0) {
            printf("Status: q=%d, w=%d, vel=%d, actual_vel=%d\n", 
                q_pressed, w_pressed, 
                out_data[selected_slave]->target_velocity,
                in_data[selected_slave]->actual_velocity);
        }
        */
        
        // Short sleep to prevent CPU overload and maintain cycle time
        usleep(5000);  // 5ms cycle time
    }

    // Clean up before exiting
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        stop_motor(i);
        usleep(10000);
        disable_drive(i);
    }

    restore_keyboard();
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_INIT;
        ec_writestate(i + 1);
    }
    ec_close();
    printf("\nExited.\n");
    return 0;
}