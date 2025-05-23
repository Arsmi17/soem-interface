#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define NUM_SLAVES 8
volatile sig_atomic_t keep_running = 1;

typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t target_position;
    uint8_t mode_of_operation;
} el7_out_t;

typedef struct __attribute__((packed))
{
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
} el7_in_t;


int clockwiseDirections[4] = {1, 1, 1, 1};


// Anti-clockwise for all
int anticlockwiseDirections[4] = {-1, -1, -1, -1};


// Mixed directions (e.g., robot curve turn)
int mixedDirections[4] = {1, -1, -1, 1};



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

#define POSITION_ACTUAL_VALUE_OFFSET 4 // Assuming position data starts from offset 4
#define RAW_STEPS_PER_METER 806074880  // Raw steps you're receiving for 1 meter
#define ACTUAL_STEPS_PER_METER 3148730 // Actual steps it should represent

#define ACTUAL_STEPS_PER_METER_ROTATION 49500 // Actual steps it should represent

#define MOVEMENT_MOTOR_1 0
#define MOVEMENT_MOTOR_2 2
#define MOVEMENT_MOTOR_3 4
#define MOVEMENT_MOTOR_4 6

#define ROTATION_MOTOR_1 1
#define ROTATION_MOTOR_2 3
#define ROTATION_MOTOR_3 5
#define ROTATION_MOTOR_4 7

float read_current_position(int slave_id)
{
    if (slave_id < 0 || !ec_slave[slave_id + 1].inputs)
    {
        fprintf(stderr, "Invalid slave ID or no input data available.\n");
        return 0.0;
    }

    // Pointer to inputs
    uint8_t *inputs = ec_slave[slave_id + 1].inputs;

    // Read Actual Motor Position (0x6064)
    uint8_t *pos_ptr = inputs + POSITION_ACTUAL_VALUE_OFFSET;

    // Interpret 4 bytes as int32_t (little-endian format)
    int32_t raw_position = (int32_t)((pos_ptr[3] << 24) | // Most significant byte
                                     (pos_ptr[2] << 16) |
                                     (pos_ptr[1] << 8) |
                                     pos_ptr[0]);

    // Apply scaling to get correct position
    float scale_factor = (float)ACTUAL_STEPS_PER_METER / (float)RAW_STEPS_PER_METER;
    float scaled_position = (float)raw_position * scale_factor;

    float position_in_meters = scaled_position / (float)ACTUAL_STEPS_PER_METER;
    printf("=======================================\n");
    printf("Slave ID         : %d\n", slave_id);
    printf("Raw Position     : %d\n", raw_position);
    printf("Scaled Position  : %.3f\n", scaled_position);
    printf("Position (meters): %.4f m\n", position_in_meters);
    printf("=======================================\n\n");

    return scaled_position;
}

// #define POSITION_DEMAND_VALUE_OFFSET 2
// int32_t read_current_position(int slave_id) {
//     if (slave_id < 0 || !ec_slave[slave_id + 1].inputs) {
//         fprintf(stderr, "Invalid slave ID or no input data.\n");
//         return 0;
//     }

//     // Pointer to inputs
//     uint8_t *inputs = ec_slave[slave_id + 1].inputs;

//     // Read Position Demand Value (0x6062)
//     uint8_t *pos_ptr = inputs + POSITION_DEMAND_VALUE_OFFSET;

//     int32_t position = (int32_t)(
//         (pos_ptr[3] << 24) |
//         (pos_ptr[2] << 16) |
//         (pos_ptr[1] << 8) |
//         pos_ptr[0]
//     );

//     return position;
// }

void move_to_absolute(int slave_idx, int position)
{
    out_data[slave_idx]->target_position = position;
    out_data[slave_idx]->control_word = 0x0F | 0x10; // New setpoint
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F; // Clear new setpoint
    send_and_receive();
    usleep(10000);
}

void reset_fault(int slave_idx)
{
    printf("Resetting fault on drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x80;
    send_and_receive();
    usleep(10000);
}

boolean is_drive_enabled(int slave_index)
{
    // Ensure the slave index is valid
    if (slave_index < 0 || slave_index > ec_slavecount)
    {
        printf("Invalid slave index: %d\n", slave_index);
        return FALSE;
    }

    // Read the state of all slaves
    ec_readstate();

    // Get the state of the specific slave
    uint16 slave_state = ec_slave[slave_index].state;

    // Check if the slave is in the OPERATIONAL state
    if (slave_state != EC_STATE_OPERATIONAL)
    {
        printf("Drive at slave index %d is not in OPERATIONAL state. Current state: %d\n", slave_index, slave_state);
        return FALSE;
    }

    // Read the Status Word (Object 0x6041) using SDO or PDO
    uint16 status_word = 0;         // Variable to store the Status Word
    int size = sizeof(status_word); // Variable to hold the size of the data (now declared as int)
    int wkc = ec_SDOread(slave_index + 1, 0x6041, 0x00, FALSE, &size, &status_word, EC_TIMEOUTRXM);

    if (wkc <= 0)
    {
        printf("Failed to read Status Word from slave %d\n", slave_index);
        return FALSE;
    }

    // Check if the "Operation Enabled" bit (Bit 2) is set
    boolean is_enabled = (status_word & (1 << 2)) != 0;

    if (is_enabled)
    {
        printf("Drive at slave index %d is enabled.\n", slave_index);
    }
    else
    {
        printf("Drive at slave index %d is not enabled. Status Word: 0x%04X\n", slave_index, status_word);
    }

    return is_enabled;
}

boolean are_all_drives_enabled()
{
    boolean all_enabled = TRUE;

    // Loop through all slaves and check each one
    for (int slave_index = 0; slave_index < NUM_SLAVES; slave_index++)
    {
        // Check if the current slave is enabled
        boolean is_enabled = is_drive_enabled(slave_index);

        // If any slave is not enabled, set the result to FALSE
        if (!is_enabled)
        {
            printf("Not all drives are enabled (slave %d is disabled).\n", slave_index + 1);
            all_enabled = FALSE;
            // Don't return immediately - check all slaves so user knows which ones need attention
        }
    }

    if (all_enabled)
    {
        printf("All drives are enabled and operational.\n");
    }

    return all_enabled;
}

void enable_drive(int slave_idx)
{
    printf("Enabling drive %d...\n", slave_idx + 1);
    int max_retries = 3;
    int retry_count = 0;
    uint16_t status_word;
    boolean enabled = FALSE;

    // First check for faults and reset if needed
    send_and_receive();
    status_word = in_data[slave_idx]->status_word;

    // Check if fault bit (bit 3) is set
    if (status_word & 0x0008)
    {
        printf("Fault detected on drive %d (Status: 0x%04X). Resetting...\n",
               slave_idx + 1, status_word);
        reset_fault(slave_idx);
        usleep(50000); // Give time for fault to clear
    }

    while (retry_count < max_retries && !enabled)
    {
        // State transition sequence
        out_data[slave_idx]->control_word = 0x06; // "Switch on disabled"
        send_and_receive();
        usleep(10000);

        out_data[slave_idx]->control_word = 0x07; // "Switch on"
        send_and_receive();
        usleep(10000);

        out_data[slave_idx]->control_word = 0x0F; // "Enable operation"
        send_and_receive();
        usleep(10000);

        // Check status word to verify enabled state
        send_and_receive();
        status_word = in_data[slave_idx]->status_word;

        // Check if we're in "Operation enabled" state (bits 6-8: 00100111)
        if ((status_word & 0x006F) == 0x0027)
        {
            enabled = TRUE;
            printf("Drive %d successfully enabled (Status: 0x%04X)\n",
                   slave_idx + 1, status_word);
        }
        else
        {
            // If not enabled, check if fault occurred during attempt
            if (status_word & 0x0008)
            {
                printf("Fault occurred during enable attempt %d (Status: 0x%04X)\n",
                       retry_count + 1, status_word);
                reset_fault(slave_idx);
                usleep(50000);
            }

            retry_count++;
            printf("Drive %d enable attempt %d failed (Status: 0x%04X). Retrying...\n",
                   slave_idx + 1, retry_count, status_word);
            usleep(50000); // Longer delay between retries
        }
    }

    // if (!enabled) {
    //     printf("ERROR: Failed to enable drive %d after %d attempts (Status: 0x%04X)\n",
    //            slave_idx + 1, max_retries, status_word);
    // }
}

void disable_drive(int slave_idx)
{
    printf("Disabling drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x00;
    send_and_receive();
}

void move_to_zero(int slave_idx)
{
    printf("Moving slave %d to position 0...\n", slave_idx + 1);

    // Set target position to 0
    out_data[slave_idx]->target_position = 0;
    out_data[slave_idx]->control_word = 0x0F | 0x10; // Enable + New Setpoint
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F; // Clear new setpoint bit
    send_and_receive();
    // Wait for motor to reach position 0
}

void wait_until_reached(int slave, int target_position)
{
    int retries = 0;
    while (retries < 10000)
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        float current_pos = read_current_position(slave);

        if (retries % 100 == 0)
        {
            printf("Slave %d current position: %.2f, Target: %d\n",
                   slave, current_pos, target_position);
        }

        // Use same comparison logic that worked before
        if (fabsf(current_pos - target_position) < 1000)
        {
            printf("Slave %d reached target position %d\n", slave, target_position);
            // Same delay pattern that worked before
            for (int i = 0; i < 100; i++)
            {
                send_and_receive();
                usleep(10000);
            }

            return;
        }

        usleep(10000);
        retries++;
    }

    printf("WARNING: Slave %d timed out while moving to position %d\n",
           slave, target_position);
}

void delay_with_communication(int milliseconds)
{
    printf("Waiting %d ms while maintaining communication...\n", milliseconds);
    int iterations = milliseconds / 1; // 1ms per iteration

    for (int i = 0; i < iterations; i++)
    {
        // Keep the EtherCAT communication active
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        usleep(1000); // 1ms sleep
    }
}

void safe_move_to_absolute(int slave_idx, int target_position)
{
    // if (!are_all_drives_enabled(NUM_SLAVES))
    // {
    //     printf("Error: Not all drives are enabled. Cannot move drive %d.\n", slave_idx + 1);
    //     return; // Exit early without moving
    // }

    // If all drives are enabled, perform move
    move_to_absolute(slave_idx, target_position);
}

void template_1()
{
    printf("Running Template 1 Sequence...\n");

    int slave_Index = 4;
    // Move to 0.2 meters
    int target_pos = 0.2 * ACTUAL_STEPS_PER_METER;
    safe_move_to_absolute(slave_Index, target_pos);

    // Wait until first position is reached
    wait_until_reached(slave_Index, target_pos);

    delay_with_communication(5000);
    printf("Moving back to zero...\n");
    safe_move_to_absolute(slave_Index, 0);

    // Wait until zero position is reached
    wait_until_reached(slave_Index, 0);

    printf("Sequence completed.\n");
}

void template_1_loop()
{
    printf("Running Template 1 Sequence in loop. Press SPACEBAR to stop...\n");

    int running_loop = 1;
    int loop_count = 0;

    while (running_loop)
    {
        printf("\nStarting loop iteration #%d\n", ++loop_count);

        // Move to 0.2 meters
        int target_pos = 0.2 * ACTUAL_STEPS_PER_METER;

        safe_move_to_absolute(0, target_pos);

        // Wait until first position is reached
        wait_until_reached(0, target_pos);

        // Wait additional time while maintaining communication
        delay_with_communication(2000); // 2 seconds delay

        printf("Moving back to zero...\n");
        safe_move_to_absolute(0, 0);

        // Wait until zero position is reached
        wait_until_reached(0, 0);

        // Wait additional time while maintaining communication
        delay_with_communication(1000); // 1 second delay

        // Check for spacebar during the delay
        for (int i = 0; i < 100; i++)
        {
            int ch = getchar();
            if (ch == ' ')
            {
                printf("Spacebar pressed. Stopping loop.\n");
                running_loop = 0;
                break;
            }
            delay_with_communication(10);
        }
    }

    printf("Template 1 loop stopped.\n");
}

void firstMovement()
{
    int quarterMile = 0.2 * ACTUAL_STEPS_PER_METER;
    int secondMile = 0.1 * ACTUAL_STEPS_PER_METER;
    safe_move_to_absolute(2, quarterMile);
    delay_with_communication(2);
    safe_move_to_absolute(1, secondMile);
    delay_with_communication(2);
    safe_move_to_absolute(0, 0);
    delay_with_communication(2);
    wait_until_reached(2, quarterMile);
    wait_until_reached(1, secondMile);
    delay_with_communication(1000);

    // int negate_quarterMile = 0.25 * ACTUAL_STEPS_PER_METER;
    // safe_move_to_absolute(0, negate_quarterMile);

    // safe_move_to_absolute(1, negate_quarterMile);
    // delay_with_communication(10);
    // wait_until_reached(0, negate_quarterMile);
    // wait_until_reached(1, negate_quarterMile);
    // delay_with_communication(2000); // 2 seconds delay
    // int lastEnding = 1.4 * ACTUAL_STEPS_PER_METER;
    // int middleEnding = 0.7 * ACTUAL_STEPS_PER_METER;
    // int negate_lastEnding = -1.4 * ACTUAL_STEPS_PER_METER;
    // int negate_middleEnding = -0.7 * ACTUAL_STEPS_PER_METER;

    // safe_move_to_absolute(4, lastEnding);
    // delay_with_communication(10);
    // safe_move_to_absolute(0, negate_lastEnding);
    // delay_with_communication(10);
    // safe_move_to_absolute(3, middleEnding);
    // delay_with_communication(10);
    // safe_move_to_absolute(1, negate_middleEnding);

    // wait_until_reached(4, lastEnding);
    // wait_until_reached(0, negate_lastEnding);
    // delay_with_communication(1000); // 2 seconds delay
}

void configure_slave_with_params(int index, uint32_t profile_velocity,
                                 uint32_t max_velocity, uint32_t acceleration, uint32_t deceleration)
{
    int slave = index + 1;
    out_data[index] = (el7_out_t *)ec_slave[slave].outputs;
    in_data[index] = (el7_in_t *)ec_slave[slave].inputs;
    ec_SDOwrite(slave, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x607F, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x6083, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave, 0x6084, 0x00, FALSE, sizeof(deceleration), &deceleration, EC_TIMEOUTSAFE);
}

void secondMovement()
{
    int homePosition = 0;
    safe_move_to_absolute(0, homePosition);
    delay_with_communication(2);
    safe_move_to_absolute(1, homePosition);
    delay_with_communication(2);
    safe_move_to_absolute(2, homePosition);
    delay_with_communication(2);
    wait_until_reached(2, homePosition);
    wait_until_reached(1, homePosition);
    wait_until_reached(0, homePosition);
    delay_with_communication(1000);
}

void sampleHomeTest()
{
    safe_move_to_absolute(MOVEMENT_MOTOR_4, 10000);
    delay_with_communication(10);
    wait_until_reached(MOVEMENT_MOTOR_4, 10000);
}

void defaultMovementSteps(int motorName, int position, int delayTime)
{
    safe_move_to_absolute(motorName, position);
    delay_with_communication(delayTime);
}

void moveToCenterFromHomePosition()
{
    // int sameSpeed = 400000;
    // int sameAcceleration = 200000;
    // int sameDeceleration = 200000;
    // configure_slave_with_params(MOVEMENT_MOTOR_4, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    // configure_slave_with_params(MOVEMENT_MOTOR_3, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    // configure_slave_with_params(MOVEMENT_MOTOR_2, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    // configure_slave_with_params(MOVEMENT_MOTOR_1, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);

    int sameSpeed = 600000;
    int sameAcceleration = 400000;
    int sameDeceleration = 400000;
    int divideFactor =3;
    configure_slave_with_params(MOVEMENT_MOTOR_4, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    configure_slave_with_params(MOVEMENT_MOTOR_3, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration/divideFactor, sameDeceleration/divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_2, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration/divideFactor, sameDeceleration/divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_1, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);


    int quarterMile = 1.18 * ACTUAL_STEPS_PER_METER;
    int delayTime = 30;

    defaultMovementSteps(MOVEMENT_MOTOR_4, quarterMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_3, quarterMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_2, quarterMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_1, quarterMile, delayTime);
    wait_until_reached(MOVEMENT_MOTOR_1, quarterMile);
    printf("Movement to position 0 completed for slave 7.\n");
}



void SeperateEquallyfromCenter()
{
    int sameSpeed = 600000;
    int sameAcceleration = 400000;
    int sameDeceleration = 400000;
    int divideFactor =3;
    configure_slave_with_params(MOVEMENT_MOTOR_4, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    configure_slave_with_params(MOVEMENT_MOTOR_3, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration/divideFactor, sameDeceleration/divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_2, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration/divideFactor, sameDeceleration/divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_1, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);

    int fourthMile = (1.18 * ACTUAL_STEPS_PER_METER) + (1.20 * ACTUAL_STEPS_PER_METER);
    int thirdMile = (1.18 * ACTUAL_STEPS_PER_METER) + (0.4 * ACTUAL_STEPS_PER_METER);
    int secondMile = (1.18 * ACTUAL_STEPS_PER_METER) - (0.4 * ACTUAL_STEPS_PER_METER);
    int firstMile = (1.18 * ACTUAL_STEPS_PER_METER) - (1.20 * ACTUAL_STEPS_PER_METER);

    int delayTime = 30;

    defaultMovementSteps(MOVEMENT_MOTOR_4, fourthMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_3, thirdMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_2, secondMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_1, firstMile, delayTime);
    wait_until_reached(MOVEMENT_MOTOR_1, firstMile);
    wait_until_reached(MOVEMENT_MOTOR_3, thirdMile);
    printf("Movement to position 0 completed for slave 7.\n");
}

void SeperateEquallySecondaryfromCenter()
{
    int sameSpeed = 600000;
    int sameAcceleration = 400000;
    int sameDeceleration = 400000;
    int divideFactor =3;
    configure_slave_with_params(MOVEMENT_MOTOR_4, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    configure_slave_with_params(MOVEMENT_MOTOR_3, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration/divideFactor, sameDeceleration/divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_2, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration/divideFactor, sameDeceleration/divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_1, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);

    int fourthMile = (1.18 * ACTUAL_STEPS_PER_METER) + (0.6 * ACTUAL_STEPS_PER_METER);
    int thirdMile = (1.18 * ACTUAL_STEPS_PER_METER) + (0.2 * ACTUAL_STEPS_PER_METER);
    int secondMile = (1.18 * ACTUAL_STEPS_PER_METER) - (0.2 * ACTUAL_STEPS_PER_METER);
    int firstMile = (1.18 * ACTUAL_STEPS_PER_METER) - (0.6 * ACTUAL_STEPS_PER_METER);

    int delayTime = 30;

    defaultMovementSteps(MOVEMENT_MOTOR_4, fourthMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_3, thirdMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_2, secondMile, delayTime);
    defaultMovementSteps(MOVEMENT_MOTOR_1, firstMile, delayTime);
    wait_until_reached(MOVEMENT_MOTOR_1, firstMile);
    wait_until_reached(MOVEMENT_MOTOR_3, thirdMile);
    printf("Movement to position 0 completed for slave 7.\n");
}

void Rotate(int directions[4]) {
    int sameSpeed = 4000;
    int sameAcceleration = 2000;
    int sameDeceleration = 2000;
    int divideFactor = 3;

    // Configure movement motors
    configure_slave_with_params(MOVEMENT_MOTOR_4, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);
    configure_slave_with_params(MOVEMENT_MOTOR_3, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration / divideFactor, sameDeceleration / divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_2, sameSpeed / divideFactor, sameSpeed / divideFactor, sameAcceleration / divideFactor, sameDeceleration / divideFactor);
    configure_slave_with_params(MOVEMENT_MOTOR_1, sameSpeed, sameSpeed, sameAcceleration, sameDeceleration);

    int delayTime = 30;

    // Calculate target positions based on direction
    int mile = ACTUAL_STEPS_PER_METER_ROTATION;

    int targets[4] = {
        directions[0] * mile,
        directions[1] * mile,
        directions[2] * mile,
        directions[3] * mile
    };

    // Send movement commands
    defaultMovementSteps(ROTATION_MOTOR_4, targets[0], delayTime);
    defaultMovementSteps(ROTATION_MOTOR_3, targets[1], delayTime);
    defaultMovementSteps(ROTATION_MOTOR_2, targets[2], delayTime);
    defaultMovementSteps(ROTATION_MOTOR_1, targets[3], delayTime);

    // Wait for key motors to finish (you can customize which to wait on)
    wait_until_reached(ROTATION_MOTOR_1, targets[3]);
    wait_until_reached(ROTATION_MOTOR_3, targets[1]);
    printf("Movement to position completed.\n");

    int homePosition = 0;
    // Move all back to home
    defaultMovementSteps(ROTATION_MOTOR_4, homePosition, delayTime);
    defaultMovementSteps(ROTATION_MOTOR_3, homePosition, delayTime);
    defaultMovementSteps(ROTATION_MOTOR_2, homePosition, delayTime);
    defaultMovementSteps(ROTATION_MOTOR_1, homePosition, delayTime);

    wait_until_reached(ROTATION_MOTOR_1, homePosition);
    wait_until_reached(ROTATION_MOTOR_3, homePosition);
    printf("Returned to home position.\n");
}





void testFirstMovement()
{
   moveToCenterFromHomePosition();
   SeperateEquallySecondaryfromCenter();
   
    // delay_with_communication(1000);
    // SeperateEquallyfromCenter();
    
}

void testSecondMovement()
{
    
    //SeperateEquallyfromCenter();
    Rotate(mixedDirections);
    SeperateEquallyfromCenter();
    Rotate(clockwiseDirections);
}

void negate_test()
{
    int negateEnding = -0.2 * ACTUAL_STEPS_PER_METER;
    safe_move_to_absolute(0, negateEnding);
    wait_until_reached(0, negateEnding);
    delay_with_communication(2000);
    safe_move_to_absolute(0, 0);
    wait_until_reached(0, 0);
    delay_with_communication(2000);
}

void template_2()
{
    printf("Running Template 1 Sequence in loop. Press SPACEBAR to stop...\n");
    testFirstMovement();
    testSecondMovement();
    // testMovement();
    printf("Template 1 loop stopped.\n");
}

void template_2_loop()
{
    printf("Running Template 1 Sequence in loop. Press SPACEBAR to stop...\n");

    int running_loop = 1;
    int loop_count = 0;

    while (running_loop)
    {
        printf("\nStarting loop iteration #%d\n", ++loop_count);

        testFirstMovement();

        testSecondMovement();

        // Check for spacebar during the delay
        for (int i = 0; i < 100; i++)
        {
            int ch = getchar();
            if (ch == ' ')
            {
                printf("Spacebar pressed. Stopping loop.\n");
                running_loop = 0;
                break;
            }
            delay_with_communication(10);
        }
    }

    printf("Template 1 loop stopped.\n");
}

// void template_2_loop()
// {
//     printf("Running Template 1 Sequence in loop. Press SPACEBAR to stop...\n");

//     int running_loop = 1;
//     int loop_count = 0;

//     while (running_loop)
//     {
//         printf("\nStarting loop iteration #%d\n", ++loop_count);

//         int endPosition = 2 * ACTUAL_STEPS_PER_METER;

//         for (int i = 4; i >= 0; i--)
//         {
//             safe_move_to_absolute(i, endPosition);
//         }

//         // Wait until first position is reached
//         for (int i = 4; i >= 0; i--)
//         {
//             wait_until_reached(i, endPosition);
//         }

//         // Wait additional time while maintaining communication
//         delay_with_communication(4000); // 2 seconds delay

//         printf("Moving back to zero...\n");
//         for (int i = 0; i >= 4; i++)
//         {
//             safe_move_to_absolute(i, 0);
//         }

//         // Wait until zero position is reached
//         for (int i = 0; i >= 4; i++)
//         {
//             wait_until_reached(i, 0);
//         }

//         // Wait additional time while maintaining communication
//         delay_with_communication(4000); // 1 second delay

//         // Check for spacebar during the delay
//         // for (int i = 0; i < 100; i++)
//         // {
//         //     int ch = getchar();
//         //     if (ch == ' ')
//         //     {
//         //         printf("Spacebar pressed. Stopping loop.\n");
//         //         running_loop = 0;
//         //         break;
//         //     }
//         //     delay_with_communication(10);
//         // }
//     }

//     printf("Template 1 loop stopped.\n");
// }

#define BASE_STEP 629746
#define TOTAL_MOVES 5

// void move_steps(int start_slave) {
//     for (int i = 0; i < TOTAL_MOVES; i++) {
//         int slave = start_slave + (TOTAL_MOVES - 1) - i; // Start from the highest slave
//         int position = BASE_STEP * (i + 1);              // 1x, 2x, 3x, 4x, 5x
//         move_to_absolute(slave, position);
//     }
// }

void int_handler(int sig)
{
    (void)sig;
    printf("\nCaught SIGINT (Ctrl+C), stopping motors...\n");

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        disable_drive(i); // Make sure this stops the motor or puts it in safe state
    }

    ec_close();         // Close EtherCAT connection if open
    restore_keyboard(); // Optional: if you used raw mode for input

    exit(0); // Exit safely
}

void set_position_as_zero(int slave_idx)
{
    printf("Setting current position as zero for motor %d...\n", slave_idx + 1);

    // Write 0 to the actual position register to set it as the new zero
    int32_t zero_position = 0;
    ec_SDOwrite(slave_idx + 1, 0x6064, 0x00, FALSE, sizeof(zero_position), &zero_position, EC_TIMEOUTSAFE);
    usleep(10000);

    printf("Motor position set to zero.\n");
}

void perform_homing(int slave_idx)
{
    printf("Starting homing procedure for drive %d...\n", slave_idx + 1);

    // Set mode of operation to Homing Mode (6)
    uint8_t mode = 6;
    ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
    usleep(10000);

    // Set homing method (example: 35)
    int8_t homing_method = 35;
    ec_SDOwrite(slave_idx + 1, 0x6098, 0x00, FALSE, sizeof(homing_method), &homing_method, EC_TIMEOUTSAFE);
    usleep(10000);

    // Start homing
    out_data[slave_idx]->control_word = 0x1F; // Enable operation + Start homing
    send_and_receive();
    usleep(10000);

    // Monitor homing
    printf("Waiting for homing to complete...\n");
    uint16_t status_word;
    int homing_complete = 0;
    int timeout_counter = 0;
    while (!homing_complete && timeout_counter < 500)
    {
        send_and_receive();
        status_word = in_data[slave_idx]->status_word;

        // Print status bits for debugging
        printf("Status Word: 0x%04X\n", status_word);

        // Bit 12 (homing attained), Bit 10 (target reached)
        if ((status_word & 0x3400) == 0x3400)
        {
            homing_complete = 1;
            printf("Homing completed for drive %d.\n", slave_idx + 1);
        }

        usleep(20000);
        timeout_counter++;
    }

    if (!homing_complete)
    {
        printf("Homing timed out or failed for drive %d.\n", slave_idx + 1);
    }

    // Return to Profile Position Mode (1)
    mode = 1;
    ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
    usleep(10000);
}

int main()
{
    signal(SIGINT, int_handler);

    if (!ec_init("enp0s31f6"))
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

        uint8_t mode = 1;

        // Default motion parameters
        uint32_t profile_velocity = 80000;
        uint32_t max_velocity = 80000;
        uint32_t acceleration = 6000;
        uint32_t deceleration = 6000;

        // if (i == 1)
        // {
        //     profile_velocity /= 1.5;
        //     max_velocity /= 1.5;
        //     acceleration /= 1.5;
        //     deceleration /= 1.5;
        // }

        // if (i == 0)
        // {
        //     profile_velocity /= 3;
        //     max_velocity /= 3;
        //     acceleration /= 3;
        //     deceleration /= 3;
        // }

        // Setting it to Profile Position Mode
        ec_SDOwrite(i + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6083, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6084, 0x00, FALSE, sizeof(deceleration), &deceleration, EC_TIMEOUTSAFE);
    }

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_OPERATIONAL;
        ec_writestate(i + 1);
    }
    usleep(10000);

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        if (ec_statecheck(i + 1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
        {
            printf("Slave %d failed to reach OPERATIONAL state.\n", i + 1);
            ec_close();
            return -1;
        }
    }

    printf("Slaves are OPERATIONAL. Use keys:\n");
    printf("[1-2] Enable  [q,w] Position  [a,s] Reset Fault  [z,x] Disable  [h] Homing  [ESC] Quit\n");

    setup_keyboard();
    int running = 1;

    while (running)
    {
        send_and_receive();

        for (int i = 0; i < NUM_SLAVES; ++i)
        {
            read_current_position(i);
        }

        int ch = getchar();
        switch (ch)
        {
        case '1':
            for (int i = 0; i < NUM_SLAVES; i++)
                enable_drive(i);
            break;
        case '2':
            for (int i = 0; i < NUM_SLAVES; i++)
                disable_drive(i);
            break;
        case '3':
            for (int i = 0; i < NUM_SLAVES; i++)
                reset_fault(i);
            break;
        case '4':
            for (int i = 0; i < NUM_SLAVES; i++)
                is_drive_enabled(i);
            break;
        case 'q':
            template_2_loop();
            break;
            break;
        case 'a':
            testFirstMovement();
            break;
        case 's':
            testSecondMovement();
            break;
        // case 'w':
        //     template_2();
        //     break;

        // Homing
        case 'h':
            for (int i = 0; i < NUM_SLAVES; i++)
                move_to_zero(i);
            break;

        case 27:
            running = 0;
            break; // ESC key
        }

        usleep(20000);
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