#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>

#define NUM_SLAVES 8
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
    uint16_t status_word;              // 0x6041
    int32_t actual_position;           // 0x6064
    int32_t actual_velocity;           // 0x606C
    int16_t actual_torque;             // 0x6077
    uint8_t mode_of_operation_display; // 0x6061
} el7_in_t;

char IOmap[4096];
el7_out_t *out_data[NUM_SLAVES];
el7_in_t *in_data[NUM_SLAVES];
int selected_slave = -1; // Global variable to track the selected slave (-1 means no slave is selected)
int velocity_level = 4000; // Global variable for velocity level
int velocity_increment = 500; // Global variable for velocity increment

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
    uint8_t mode = 3; // Profile Velocity Mode

    printf("Setting slave %d to Profile Velocity Mode (3)...\n", slave_idx + 1);
    if (ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE))
    {
        printf("Successfully set mode of operation to Profile Velocity Mode (3)\n");
    }
    else
    {
        printf("Failed to set mode of operation! Will try via PDO.\n");
        // Try via PDO as fallback
        out_data[slave_idx]->mode_of_operation = 3;
        send_and_receive();
    }
    usleep(50000); // Give time for mode change to take effect
}

int set_absolute_encoder(uint16_t slave_idx, uint32_t value)
{
    uint16_t index = 0x2015; // P0.15 Absolute Encoder Settings
    uint8_t subindex = 0x00;
    int size = sizeof(value);

    // Ensure slave is in PRE-OPERATIONAL state
    ec_slave[slave_idx + 1].state = EC_STATE_PRE_OP;
    ec_writestate(slave_idx + 1);
    if (ec_statecheck(slave_idx + 1, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP)
    {
        printf("Failed to set slave %d to PRE-OPERATIONAL state\n", slave_idx + 1);
        return 0;
    }

    // Write P0.15 to enable multiturn absolute mode
    printf("Setting P0.15 (0x2015) to %d for slave %d...\n", value, slave_idx + 1);
    int success = ec_SDOwrite(slave_idx + 1, index, subindex, FALSE, size, &value, EC_TIMEOUTSAFE);
    if (!success)
    {
        printf("Failed to set P0.15 for slave %d\n", slave_idx + 1);
        return 0;
    }

    // Transition to INIT for software reset
    ec_slave[slave_idx + 1].state = EC_STATE_INIT;
    ec_writestate(slave_idx + 1);
    usleep(50000);
    ec_slave[slave_idx + 1].state = EC_STATE_PRE_OP;
    ec_writestate(slave_idx + 1);
    if (ec_statecheck(slave_idx + 1, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP)
    {
        printf("Failed to return slave %d to PRE-OPERATIONAL state after restart\n", slave_idx + 1);
        return 0;
    }

    // Verify the setting
    uint32_t read_value;
    size = sizeof(read_value);
    if (ec_SDOread(slave_idx + 1, index, subindex, FALSE, &size, &read_value, EC_TIMEOUTSAFE))
    {
        printf("Verified P0.15 = %d for slave %d\n", read_value, slave_idx + 1);
        return (read_value == value);
    }
    else
    {
        printf("Failed to verify P0.15 for slave %d\n", slave_idx + 1);
        return 0;
    }
}

int write_leadshine_param_home(uint16 slave, uint16 param_number, uint32 value)
{
    uint16 index = 0x3000 + param_number;
    uint8 subindex = 0x00;
    int size = sizeof(value);

    int success = ec_SDOwrite(slave + 1, index, subindex, FALSE, size, &value, EC_TIMEOUTRXM);
    if (success)
    {
        printf("✅ Successfully wrote Pr0.%d = %d to slave %d (index 0x%04X)\n",
               param_number, value, slave, index);
    }
    else
    {
        printf("❌ Failed to write Pr0.%d to slave %d (index 0x%04X)\n",
               param_number, slave, index);
    }
    return success;
}

// Verify the current mode
void verify_mode_of_operation(int slave_idx)
{
    uint8_t mode;
    int size = sizeof(mode);
    int result = ec_SDOread(slave_idx + 1, 0x6061, 0x00, FALSE, &size, &mode, EC_TIMEOUTSAFE);

    if (result == 1)
    {
        printf("Current mode of operation for slave %d is: %d\n", slave_idx + 1, mode);
        if (mode != 3)
        {
            printf("WARNING: Actual mode is not Profile Velocity Mode (3)!\n");
        }
    }
    else
    {
        printf("Failed to read mode of operation display for slave %d\n", slave_idx + 1);
    }
}

// Modified set_velocity with even/odd velocity limits
void set_velocity(int slave_idx, int32_t velocity)
{
    // Determine max velocity based on whether slave index is even or odd
    int32_t max_velocity = (slave_idx % 2 == 0) ? 70000 : 4000;

    // Clamp velocity to the allowed range
    if (velocity > max_velocity)
    {
        velocity = max_velocity;
        printf("Velocity clamped to max %d for slave %d (even/odd rule)\n", max_velocity, slave_idx + 1);
    }
    else if (velocity < -max_velocity)
    {
        velocity = -max_velocity;
        printf("Velocity clamped to min %d for slave %d (even/odd rule)\n", -max_velocity, slave_idx + 1);
    }

    printf("Setting velocity for slave %d to %" PRId32 "\n", slave_idx + 1, velocity);

    // First, try setting via SDO for initial setup
    if (ec_SDOwrite(slave_idx + 1, 0x60FF, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE))
    {
        printf("Successfully set target velocity via SDO\n");
    }
    else
    {
        printf("Failed to set target velocity via SDO, will use PDO\n");
    }

    // Always update via PDO for continuous operation
    out_data[slave_idx]->target_velocity = velocity;

    // Make sure control word enables operation
    out_data[slave_idx]->control_word = 0x000F; // Enable operation

    send_and_receive();

    // Read back status word
    uint16_t status = in_data[slave_idx]->status_word;
    printf("Status Word after set_velocity: 0x%04X\n", status);

    // Read back the actual velocity to verify
    int32_t actual_velocity;
    int size = sizeof(actual_velocity);
    if (ec_SDOread(slave_idx + 1, 0x606C, 0x00, FALSE, &size, &actual_velocity, EC_TIMEOUTSAFE) == 1)
    {
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
    out_data[slave_idx]->control_word = 0x0002; // Quick stop command
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

void enable_drive(int slave_idx)
{
    printf("Enabling drive %d...\n", slave_idx + 1);

    // First check for faults and reset if needed
    send_and_receive();
    uint16_t status_word = in_data[slave_idx]->status_word;

    // Check if fault bit (bit 3) is set
    if (status_word & 0x0008)
    {
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
    if ((status_word & 0x006F) == 0x0027)
    {
        printf("Drive %d successfully enabled\n", slave_idx + 1);
    }
    else
    {
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

// Modified select_slave to set velocity_level
void select_slave(int slave_idx)
{
    if (slave_idx < 0 || slave_idx >= NUM_SLAVES)
    {
        printf("Invalid slave index %d. Valid range is 0 to %d.\n", slave_idx, NUM_SLAVES - 1);
        return;
    }
    selected_slave = slave_idx;
    // Set initial velocity_level based on whether slave is even or odd
    // even : odd // movement : rotate
    velocity_level = (slave_idx % 2 == 0) ? 70000 : 4000;
    printf("Slave %d selected for movement. Initial velocity level set to %d.\n", selected_slave + 1, velocity_level);
}

int main()
{
    signal(SIGINT, int_handler);

    if (!ec_init("enp0s31f6")) // Replace with your network interface
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
    usleep(100000); // Longer delay to ensure transition

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
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        verify_mode_of_operation(i);
    }

    printf("Slaves are OPERATIONAL. Use keys:\n");
    printf("[1-8] Select slave  [i] Enable drives  [o] Disable drives  [p] Reset fault  [c] Verify mode\n");
    printf("[q] Forward motion  [w] Reverse motion  [space] Stop motion\n");
    printf("[+] Increase speed  [-] Decrease speed  [a] Set absolute encoder\n");
    printf("[ESC] Quit\n");

    setup_keyboard();
    int running = 1;

    while (running)
    {
        send_and_receive();

        int ch = getchar();
        switch (ch)
        {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        {
            int idx = ch - '1'; // Convert ASCII character to zero-based index
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
            if (selected_slave == -1)
            {
                printf("No slave selected. Press '1-8' to select a slave first.\n");
            }
            else
            {
                printf("Moving forward at velocity %d for slave %d...\n", velocity_level, selected_slave + 1);
                set_velocity(selected_slave, velocity_level);
            }
            break;

        case 'w':
            if (selected_slave == -1)
            {
                printf("No slave selected. Press '1-8' to select a slave first.\n");
            }
            else
            {
                printf("Moving in reverse at velocity %d for slave %d...\n", -velocity_level, selected_slave + 1);
                set_velocity(selected_slave, -velocity_level);
            }
            break;

        case 'a': // Set P0.15 for selected slave
            if (selected_slave < 0 || selected_slave >= ec_slavecount)
            {
                printf("No slave selected. Select a slave first (1-%d).\n", ec_slavecount);
            }
            else
            {
                printf("Setting absolute encoder for slave %d...\n", selected_slave + 1);
                if (!set_absolute_encoder(selected_slave, 1))
                {
                    printf("Failed to set absolute encoder for slave %d\n", selected_slave + 1);
                }
                else
                {
                    printf("Please power-cycle slave %d to apply P0.15 settings.\n", selected_slave + 1);
                    restore_keyboard();
                    printf("Press Enter after power-cycling slave %d...\n", selected_slave + 1);
                    getchar();
                    setup_keyboard();
                    // Reinitialize slave
                    ec_slave[selected_slave + 1].state = EC_STATE_INIT;
                    ec_writestate(selected_slave + 1);
                    ec_slave[selected_slave + 1].state = EC_STATE_PRE_OP;
                    ec_writestate(selected_slave + 1);
                    if (ec_statecheck(selected_slave + 1, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP)
                    {
                        printf("Failed to return slave %d to PRE-OPERATIONAL state\n", selected_slave + 1);
                    }
                    else
                    {
                        set_velocity_mode(selected_slave);
                        enable_drive(selected_slave);
                    }
                }
            }
            break;

        case ' ': // Space key to stop
            if (selected_slave == -1)
            {
                printf("No slave selected. Press '1-8' to select a slave first.\n");
            }
            else
            {
                printf("Stopping motion for slave %d...\n", selected_slave + 1);
                stop_motor(selected_slave);
            }
            break;

        case '+': // Increase speed
            if (selected_slave == -1)
            {
                printf("No slave selected. Press '1-8' to select a slave first.\n");
            }
            else
            {
                int max_velocity = (selected_slave % 2 == 0) ? 70000 : 4000;
                velocity_level += velocity_increment;
                if (velocity_level > max_velocity)
                {
                    velocity_level = max_velocity;
                    printf("Velocity level capped at %d for slave %d (even/odd rule)\n", max_velocity, selected_slave + 1);
                }
                printf("Velocity level increased to %d\n", velocity_level);
            }
            break;

        case '-': // Decrease speed
            if (selected_slave == -1)
            {
                printf("No slave selected. Press '1-8' to select a slave first.\n");
            }
            else
            {
                velocity_level -= velocity_increment;
                if (velocity_level < velocity_increment)
                    velocity_level = velocity_increment;
                printf("Velocity level decreased to %d\n", velocity_level);
            }
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