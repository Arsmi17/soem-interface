#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define NUM_SLAVES 4
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

#define POSITION_ACTUAL_VALUE_OFFSET 4
#define RAW_STEPS_PER_METER 202985985
#define ACTUAL_STEPS_PER_METER 792628.75

#define MOVEMENT_MOTOR_COUNT 4

#define MOVEMENT_MOTOR_1 0
#define MOVEMENT_MOTOR_2 1
#define MOVEMENT_MOTOR_3 2
#define MOVEMENT_MOTOR_4 3

int motors[4] = {MOVEMENT_MOTOR_1, MOVEMENT_MOTOR_2, MOVEMENT_MOTOR_3, MOVEMENT_MOTOR_4};

float read_current_position(int slave_id)
{
    if (slave_id < 0 || !ec_slave[slave_id + 1].inputs)
    {
        fprintf(stderr, "Invalid slave ID or no input data available.\n");
        return 0.0;
    }

    uint8_t *inputs = ec_slave[slave_id + 1].inputs;
    uint8_t *pos_ptr = inputs + POSITION_ACTUAL_VALUE_OFFSET;

    int32_t raw_position = (int32_t)((pos_ptr[3] << 24) | 
                                     (pos_ptr[2] << 16) |
                                     (pos_ptr[1] << 8) |
                                     pos_ptr[0]);

    float scale_factor = (float)ACTUAL_STEPS_PER_METER / (float)RAW_STEPS_PER_METER;
    float scaled_position = (float)raw_position * scale_factor;

    float position_in_meters = scaled_position / (float)ACTUAL_STEPS_PER_METER;
    printf("=======================================\n");
    printf("Slave ID         : %d\n", slave_id);
    printf("Position (meters): %.4f m\n", position_in_meters);
    printf("=======================================\n\n");

    return scaled_position;
}

void move_to_absolute(int slave_idx, int position)
{
    out_data[slave_idx]->target_position = position;
    out_data[slave_idx]->control_word = 0x0F | 0x10;
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F;
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
    if (slave_index < 0 || slave_index > ec_slavecount)
    {
        printf("Invalid slave index: %d\n", slave_index);
        return FALSE;
    }

    ec_readstate();
    uint16 slave_state = ec_slave[slave_index].state;

    if (slave_state != EC_STATE_OPERATIONAL)
    {
        printf("Drive at slave index %d is not in OPERATIONAL state. Current state: %d\n", slave_index, slave_state);
        return FALSE;
    }

    uint16 status_word = 0;
    int size = sizeof(status_word);
    int wkc = ec_SDOread(slave_index + 1, 0x6041, 0x00, FALSE, &size, &status_word, EC_TIMEOUTRXM);

    if (wkc <= 0)
    {
        printf("Failed to read Status Word from slave %d\n", slave_index);
        return FALSE;
    }

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

    for (int slave_index = 0; slave_index < NUM_SLAVES; slave_index++)
    {
        boolean is_enabled = is_drive_enabled(slave_index);

        if (!is_enabled)
        {
            printf("Not all drives are enabled (slave %d is disabled).\n", slave_index + 1);
            all_enabled = FALSE;
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

    send_and_receive();
    status_word = in_data[slave_idx]->status_word;

    if (status_word & 0x0008)
    {
        printf("Fault detected on drive %d (Status: 0x%04X). Resetting...\n",
               slave_idx + 1, status_word);
        reset_fault(slave_idx);
        usleep(50000);
    }

    while (retry_count < max_retries && !enabled)
    {
        out_data[slave_idx]->control_word = 0x06;
        send_and_receive();
        usleep(10000);

        out_data[slave_idx]->control_word = 0x07;
        send_and_receive();
        usleep(10000);

        out_data[slave_idx]->control_word = 0x0F;
        send_and_receive();
        usleep(10000);

        send_and_receive();
        status_word = in_data[slave_idx]->status_word;

        if ((status_word & 0x006F) == 0x0027)
        {
            enabled = TRUE;
            printf("Drive %d successfully enabled (Status: 0x%04X)\n",
                   slave_idx + 1, status_word);
        }
        else
        {
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
            usleep(50000);
        }
    }
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
    out_data[slave_idx]->target_position = 0;
    out_data[slave_idx]->control_word = 0x0F | 0x10;
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F;
    send_and_receive();
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

        if (fabsf(current_pos - target_position) < 1000)
        {
            printf("Slave %d reached target position %d\n", slave, target_position);
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
    int iterations = milliseconds / 1;

    for (int i = 0; i < iterations; i++)
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        usleep(1000);
    }
}

void safe_move_to_absolute(int slave_idx, int target_position)
{
    if (!are_all_drives_enabled())
    {
        printf("Error: Not all drives are enabled. Cannot move drive %d.\n", slave_idx + 1);
        return;
    }

    move_to_absolute(slave_idx, target_position);
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

void configure_motor_with_factor(
    int motorId,
    int baseSpeed,
    int baseAcceleration,
    int baseDeceleration,
    int factor)
{
    int speed = baseSpeed / factor;
    int acceleration = baseAcceleration / factor;
    int deceleration = baseDeceleration / factor;

    configure_slave_with_params(motorId, speed, speed, acceleration, deceleration);
}

void moveMotorsWithSteps(
    int motorSteps[4],
    int baseParams[3],
    int divideFactor,
    int delayTime,
    int direction,
    const char *logLabel)
{
    int factor = divideFactor != 0 ? divideFactor : 1;
    int motorCount = sizeof(motors) / sizeof(motors[0]);

    for (int i = 0; i < motorCount; i++)
    {
        int index = motorCount - 1 - i;
        int currentFactor = (i == 1 || i == 2) ? factor : 1;

        int speed = baseParams[0] / currentFactor;
        int accel = baseParams[1] / currentFactor;
        int decel = baseParams[2] / currentFactor;

        configure_slave_with_params(motors[index], speed, speed, accel, decel);
    }

    for (int i = 0; i < motorCount; i++)
    {
        int index = direction ? i : (motorCount - 1 - i);
        safe_move_to_absolute(motors[index], motorSteps[index]);
        delay_with_communication(delayTime);
    }

    for (int i = 0; i < motorCount; i++)
    {
        int index = direction ? i : (motorCount - 1 - i);
        wait_until_reached(motors[index], motorSteps[index]);
    }
    printf("Movement completed: %s\n", logLabel);
}

void moveMotorSequentially(
    int motorSteps[4],
    int baseParams[3],
    int divideFactor,
    int delayTime,
    int direction,
    const char *logLabel)
{
    int factor = divideFactor != 0 ? divideFactor : 1;
    int motorCount = sizeof(motors) / sizeof(motors[0]);

    for (int i = 0; i < motorCount; i++)
    {
        int index = motorCount - 1 - i;
        int currentFactor = (i == 1 || i == 2) ? factor : 1;

        int speed = baseParams[0] / currentFactor;
        int accel = baseParams[1] / currentFactor;
        int decel = baseParams[2] / currentFactor;

        configure_slave_with_params(motors[index], speed, speed, accel, decel);
    }

    for (int i = 0; i < motorCount; i++)
    {
        int index = direction ? i : (motorCount - 1 - i);
        safe_move_to_absolute(motors[index], motorSteps[index]);
        delay_with_communication(delayTime);
        wait_until_reached(motors[index], motorSteps[index]);
    }

    printf("Movement completed: %s\n", logLabel);
}

typedef enum
{
    STEPS_HOME,
    STEPS_END,
    STEPS_SPREAD,
    STEP_PATTERN_COUNT
} StepPattern;

int motorCount = sizeof(motors) / sizeof(motors[0]);

const float stepOffsets[STEP_PATTERN_COUNT][MOVEMENT_MOTOR_COUNT] = {
    {0.0, 0.0, 0.0, 0.0},
    {-1.1, -1.1, 1.1, 1.1},
    {-0.9, -0.3, 0.3, 0.9} // adjusted spread
};
int speed[] = {70000, 35000, 35000};

void tesHomeMovement(int sequential)
{
    int steps[4];
    for (int i = 0; i < motorCount; i++)
    {
        steps[i] = stepOffsets[STEPS_HOME][i] * ACTUAL_STEPS_PER_METER;
    }

    if (sequential == 0)
    {
        moveMotorsWithSteps(steps, speed, 0, 30, 1, "HOME Movement");
    }
    else
    {
        moveMotorSequentially(steps, speed, 0, 30, 1, "HOME Movement");
    }
    printf("Steps: %d, %d, %d, %d\n", steps[0], steps[1], steps[2], steps[3]);
}

void testEndMovement(int sequential)
{
    int steps[4];
    for (int i = 0; i < motorCount; i++)
    {
        steps[i] = stepOffsets[STEPS_END][i] * ACTUAL_STEPS_PER_METER;
    }

    if (sequential == 0)
    {
        moveMotorsWithSteps(steps, speed, 0, 30, 0, "END Movement");
    }
    else
    {
        moveMotorSequentially(steps, speed, 0, 30, 0, "END Movement");
    }

    printf("Steps: %d, %d, %d, %d\n", steps[0], steps[1], steps[2], steps[3]);
    delay_with_communication(2000);
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

void int_handler(int sig)
{
    (void)sig;
    printf("\nCaught SIGINT (Ctrl+C), stopping motors...\n");

    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        disable_drive(i);
    }

    ec_close();
    restore_keyboard();
    exit(0);
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
        uint32_t profile_velocity = 80000;
        uint32_t max_velocity = 80000;
        uint32_t acceleration = 6000;
        uint32_t deceleration = 6000;

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
    printf("[i] Enable all  [o] Disable all  [u] Reset Fault all  [l] Check status  [ESC] Quit\n");

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
        case 'i':
            for (int i = 0; i < NUM_SLAVES; i++)
                enable_drive(i);
            break;
        case 'o':
            for (int i = 0; i < NUM_SLAVES; i++)
                disable_drive(i);
            break;
        case 'u':
            for (int i = 0; i < NUM_SLAVES; i++)
                reset_fault(i);
            break;
        case 'l':
            for (int i = 0; i < NUM_SLAVES; i++)
                is_drive_enabled(i);
            break;
        case 'h':
            tesHomeMovement(0);
            break;
        case 27:
            running = 0;
            break;
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