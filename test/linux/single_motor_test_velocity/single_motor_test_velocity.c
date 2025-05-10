#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>

#define NUM_SLAVES 1
volatile sig_atomic_t keep_running = 1;

// Adjust this structure to match your PDO mapping for velocity mode
typedef struct __attribute__((packed))
{
    uint16_t control_word;     // 0x6040
    int32_t target_velocity;   // 0x60FF - Target velocity for Profile Velocity Mode
    uint8_t mode_of_operation; // 0x6060
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

// Set mode to Profile Velocity Mode via SDO
void set_velocity_mode(int slave_idx)
{
    uint8_t mode = 3;  // Profile Velocity Mode

    printf("Setting slave %d to Profile Velocity Mode (3)...\n", slave_idx + 1);
    if (ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE)) {
        printf("Successfully set mode of operation to Profile Velocity Mode (3)\n");
    } else {
        printf("Failed to set mode of operation! Will try via PDO.\n");
        // Try via PDO as fallback
        out_data[slave_idx]->mode_of_operation = 3;
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
        if (mode != 3) {
            printf("WARNING: Actual mode is not Profile Velocity Mode (3)!\n");
        }
    } else {
        printf("Failed to read mode of operation display for slave %d\n", slave_idx + 1);
    }
}

// Set velocity directly via SDO and PDO
void set_velocity(int slave_idx, int32_t velocity)
{
    printf("Setting velocity for slave %d to %" PRId32 "\n", slave_idx + 1, velocity);
    
    // First, try setting via SDO for initial setup
    if (ec_SDOwrite(slave_idx + 1, 0x60FF, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE)) {
        printf("Successfully set target velocity via SDO\n");
    } else {
        printf("Failed to set target velocity via SDO, will use PDO\n");
    }
    
    // Always update via PDO for continuous operation
    out_data[slave_idx]->target_velocity = velocity;
    
    // Make sure control word enables operation
    out_data[slave_idx]->control_word = 0x000F;  // Enable operation
    
    send_and_receive();
    
    // Read back status word
    uint16_t status = in_data[slave_idx]->status_word;
    printf("Status Word after set_velocity: 0x%04X\n", status);
    
    // Read back the actual velocity to verify
    int32_t actual_velocity;
    int size = sizeof(actual_velocity);
    if (ec_SDOread(slave_idx + 1, 0x606C, 0x00, FALSE, &size, &actual_velocity, EC_TIMEOUTSAFE) == 1) {
        printf("Current actual velocity: %" PRId32 "\n", actual_velocity);
    }
}

void stop_motor(int slave_idx)
{
    printf("Stopping motor %d\n", slave_idx + 1);
    
    // Set velocity to 0
    set_velocity(slave_idx, 0);
    
    send_and_receive();
    usleep(10000);
}

void quick_stop_motor(int slave_idx)
{
    printf("Quick stopping motor %d\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x0002;  // Quick stop command
    send_and_receive();
    usleep(10000);
    
    // After quick stop, you may need to transition back to operation enabled
    out_data[slave_idx]->control_word = 0x000F;
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

int selected_slave = -1;  // Global variable to track the selected slave (-1 means no slave is selected)

void select_slave(int slave_idx)
{
    if (slave_idx < 0 || slave_idx >= NUM_SLAVES) {
        printf("Invalid slave index %d. Valid range is 0 to %d.\n", slave_idx, NUM_SLAVES - 1);
        return;
    }
    selected_slave = slave_idx;
    printf("Slave %d selected for movement.\n", selected_slave + 1);
}

int main()
{
    signal(SIGINT, int_handler);

    if (!ec_init("enp0s31f6"))  // Replace with your network interface
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

        // Set important motion parameters for velocity mode
        uint32_t profile_acceleration = 500000;
        uint32_t profile_deceleration = 500000;
        uint32_t max_profile_velocity = 500000;

        // Configure parameters via SDO
        ec_SDOwrite(i + 1, 0x6083, 0x00, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6084, 0x00, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(max_profile_velocity), &max_profile_velocity, EC_TIMEOUTSAFE);

        // Set to Profile Velocity Mode
        set_velocity_mode(i);
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
    }

    printf("Slaves are OPERATIONAL. Use keys:\n");
    printf("[s] Select slave  [1] Enable drives  [2] Disable drives  [3] Reset fault  [4] Verify mode\n");
    printf("[q] Forward motion  [w] Reverse motion  [space] Stop motion\n");
    printf("[+] Increase speed  [-] Decrease speed\n");
    printf("[ESC] Quit\n");

    setup_keyboard();
    int running = 1;
    int velocity_level = 160000;  // Start with a lower velocity (adjust as needed)
    int velocity_increment = 500;

    while (running)
    {
        send_and_receive();

        // for (int i = 0; i < NUM_SLAVES; ++i)
        // {
        //     printf("Slave %d -> Status: 0x%04X | Vel: %d | Pos: %d\n",
        //            i + 1,
        //            in_data[i]->status_word,
        //            in_data[i]->actual_velocity,
        //            in_data[i]->actual_position);
        // }

        int ch = getchar();
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
            if (selected_slave == -1) {
                printf("No slave selected. Press 's' to select a slave first.\n");
            } else {
                printf("Moving forward at velocity %d for slave %d...\n", velocity_level, selected_slave + 1);
                set_velocity(selected_slave, velocity_level);
            }
            break;

        case 'w':
            if (selected_slave == -1) {
                printf("No slave selected. Press 's' to select a slave first.\n");
            } else {
                printf("Moving in reverse at velocity %d for slave %d...\n", -velocity_level, selected_slave + 1);
                set_velocity(selected_slave, -velocity_level);
            }
            break;

        case ' ': // Space key to stop
            if (selected_slave == -1) {
                printf("No slave selected. Press 's' to select a slave first.\n");
            } else {
                printf("Stopping motion for slave %d...\n", selected_slave + 1);
                stop_motor(selected_slave);
            }
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

        usleep(20000);
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