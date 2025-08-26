
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <lo/lo.h>

#define TARGET_HOST "192.168.1.179"
#define TARGET_PORT 8000
#define OSC_PATH "/test"

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

typedef enum
{
    STEPS_HOME,
    STEPS_END,
    STEPS_SPREAD,
    STEPS_SPREAD_EQUAL,
    STEPS_LEFT,
    STEPS_RIGHT,
    STEP_PATTERN_COUNT
} StepPattern;

int motorCount = sizeof(motors) / sizeof(motors[0]);

const float stepOffsets[STEP_PATTERN_COUNT][MOVEMENT_MOTOR_COUNT] = {
    {0.0, 0.0, 0.0, 0.0}, // STEPS_HOME
    {-1.1, -1.1, 1.1, 1.1}, // STEPS_END
    {-1.1, 0, 0, 1.1}, // STEPS_SPREAD
    {-1.1, -0.4, 0.4, 1.1}, // STEPS_SPREAD_EQUAL
    {-1.1, -1.1, -1.1, -1.1}, // STEPS_LEFT
    {1.1, 1.1, 1.1, 1.1}, // STEPS_RIGHT
    // {-0.8, -0.25, 0.25, 0.8} // STEPS_SPREAD
};
int speed[] = {125000, 62500, 62500};
// int speed[] = {175000, 87500, 87500};
// int speed[] = {250000, 125000, 125000};

int delay_times[] = {75115, 3125};
int num_delays = sizeof(delay_times) / sizeof(delay_times[0]);
float video_number_offset = 5.0f; // Adjust as needed

float read_current_scaled_position(int slave_id)
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

    // float position_in_meters = scaled_position / (float)ACTUAL_STEPS_PER_METER;
    // printf("=======================================\n");
    // printf("Slave ID         : %d\n", slave_id);
    // printf("Scaled Position  : %.4f steps\n", scaled_position);
    // printf("Position (meters): %.4f m\n", position_in_meters);
    // printf("=======================================\n\n");

    return scaled_position;
}

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

    // printf("=======================================\n");
    // printf("Slave ID         : %d\n", slave_id);
    // printf("Scale Factor     : %.6f\n", scale_factor);
    // printf("Scaled Position  : %.4f steps\n", scaled_position);
    // printf("Position (meters): %.4f m\n", position_in_meters);
    // printf("=======================================\n\n");

    return position_in_meters;
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
        // printf("Drive at slave index %d is enabled.\n", slave_index);
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
        // printf("All drives are enabled and operational.\n");
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

void wait_until_reached(int slave, int target_position)
{
    int retries = 0;
    while (retries < 10000)
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        float current_pos = read_current_scaled_position(slave);

        if (retries % 100 == 0)
        {
            // printf("Slave %d current position: %.2f, Target: %d\n",slave, current_pos, target_position);
        }

        if (fabsf(current_pos - target_position) < 1000.0f) //< 1000
        // if ((int)current_pos == target_position)
        {
            // printf("Slave %d reached target position %d\n", slave, target_position);
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

    printf("WARNING: Slave %d timed out while moving to position %d\n",slave, target_position);
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

// void get_move_order(int motorSteps[4], int order[4], int indegree[4])
// {
//     char dir[4];
//     float current_meters[4];
//     float target_meters[4];
//     float target_sum = 0.0;
//     for (int i = 0; i < 4; i++)
//     {
//         current_meters[i] = read_current_position(i);
//         target_meters[i] = (float)motorSteps[i] / ACTUAL_STEPS_PER_METER;
//         target_sum += fabsf(target_meters[i]);
//         int delta = motorSteps[i] - (int)(current_meters[i] * ACTUAL_STEPS_PER_METER);
//         if (delta < 0)
//             dir[i] = 'L'; // Moving to negative direction
//         else if (delta > 0)
//             dir[i] = 'R'; // Moving to positive direction
//         else
//             dir[i] = '0';
//         // printf("Motor %d: current %.4f m, target %.4f m, dir %c\n", i, current_meters[i], target_meters[i], dir[i]);
//     }

//     // Detect spreading vs. converging: large target_sum (e.g., 4.4) for spreading, small (e.g., 2.2, 0) for converging
//     int is_spreading = (target_sum > 3.0); // Threshold for Scenario 1 (4.4) vs. Scenarios 2 (2.2), 3 (0)

//     int num_succ[4] = {0};
//     int successors[4][3] = {{0}};
//     for (int i = 0; i < 4; i++)
//     {
//         indegree[i] = 0; // Initialize indegree
//     }

//     for (int i = 0; i < 4; i++)
//     {
//         for (int j = 0; j < 4; j++)
//         {
//             if (i == j || dir[i] == '0' || dir[j] == '0')
//                 continue;

//             // Check if motors are at the same position
//             if (fabsf(current_meters[i] - current_meters[j]) < 0.001)
//             {
//                 float dist_i = fabsf(target_meters[i]);
//                 float dist_j = fabsf(target_meters[j]);
//                 if (is_spreading)
//                 {
//                     // Spreading: prioritize farther from center
//                     if (dist_i > dist_j)
//                     {
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (same pos, spreading, %d to farther %.4f)\n", i, j, i, target_meters[i]);
//                     }
//                     else if (dist_j > dist_i)
//                     {
//                         successors[j][num_succ[j]++] = i;
//                         indegree[i]++;
//                         // printf("Added edge %d -> %d (same pos, spreading, %d to farther %.4f)\n", j, i, j, target_meters[j]);
//                     }
//                     // Equal distances: higher index for 'R', lower for 'L'
//                     else if (dir[i] == 'R' && dir[j] == 'R' && i > j)
//                     {
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (R, same pos, spreading, higher index)\n", i, j);
//                     }
//                     else if (dir[i] == 'L' && dir[j] == 'L' && i < j)
//                     {
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         printf("Added edge %d -> %d (L, same pos, spreading, lower index)\n", i, j);
//                     }
//                 }
//                 else
//                 {
//                     // Converging: prioritize closer to center
//                     if (dist_i < dist_j)
//                     {
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (same pos, converging, %d to closer %.4f)\n", i, j, i, target_meters[i]);
//                     }
//                     else if (dist_j < dist_i)
//                     {
//                         successors[j][num_succ[j]++] = i;
//                         indegree[i]++;
//                         // printf("Added edge %d -> %d (same pos, converging, %d to closer %.4f)\n", j, i, j, target_meters[j]);
//                     }
//                     // Equal distances: lower index for 'R', higher for 'L'
//                     else if (dir[i] == 'R' && dir[j] == 'R' && i < j)
//                     {
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (R, same pos, converging, lower index)\n", i, j);
//                     }
//                     else if (dir[i] == 'L' && dir[j] == 'L' && i > j)
//                     {
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (L, same pos, converging, higher index)\n", i, j);
//                     }
//                 }
//             }
//             // Check for potential collision based on movement paths
//             else
//             {
//                 // Motor i moving right, j is in its path to target
//                 if (dir[i] == 'R' && current_meters[i] < current_meters[j] && target_meters[i] >= current_meters[j])
//                 {
//                     if (is_spreading)
//                     {
//                         // Spreading: prioritize i (moving to outer position)
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (R, spreading, i at %.4f to %.4f, j at %.4f)\n", i, j, current_meters[i], target_meters[i], current_meters[j]);
//                     }
//                     else
//                     {
//                         // Converging: prioritize j (clear inner position first)
//                         successors[j][num_succ[j]++] = i;
//                         indegree[i]++;
//                         // printf("Added edge %d -> %d (R, converging, j at %.4f in i's path to %.4f)\n", j, i, current_meters[j], target_meters[i]);
//                     }
//                 }
//                 // Motor i moving left, j is in its path to target
//                 else if (dir[i] == 'L' && current_meters[i] > current_meters[j] && target_meters[i] <= current_meters[j])
//                 {
//                     if (is_spreading)
//                     {
//                         // Spreading: prioritize i (moving to outer position)
//                         successors[i][num_succ[i]++] = j;
//                         indegree[j]++;
//                         // printf("Added edge %d -> %d (L, spreading, i at %.4f to %.4f, j at %.4f)\n", i, j, current_meters[i], target_meters[i], current_meters[j]);
//                     }
//                     else
//                     {
//                         // Converging: prioritize j (clear inner position first)
//                         successors[j][num_succ[j]++] = i;
//                         indegree[i]++;
//                         // printf("Added edge %d -> %d (L, converging, j at %.4f in i's path to %.4f)\n", j, i, current_meters[j], target_meters[i]);
//                     }
//                 }
//             }
//         }
//     }

//     int q[4];
//     int front = 0, rear = 0;
//     for (int i = 0; i < 4; i++)
//     {
//         if (indegree[i] == 0)
//         {
//             q[rear++] = i;
//             // printf("Initial queue: added %d\n", i);
//         }
//     }

//     int ord_idx = 0;
//     while (front < rear)
//     {
//         int u = q[front++];
//         order[ord_idx++] = u;
//         // printf("Processing motor %d\n", u);
//         for (int j = 0; j < num_succ[u]; j++)
//         {
//             int v = successors[u][j];
//             indegree[v]--;
//             // printf("Reduced indegree[%d] to %d\n", v, indegree[v]);
//             if (indegree[v] == 0)
//             {
//                 q[rear++] = v;
//                 // printf("Added to queue: %d\n", v);
//             }
//         }
//     }

//     if (ord_idx != 4)
//     {
//         printf("Error in computing order\n");
//         for (int i = 0; i < 4; i++)
//         {
//             order[i] = i;
//             indegree[i] = 0;
//         }
//     }
// }

void get_move_order(int motorSteps[4], int order[4], int indegree[4])
{
    char dir[4];
    float current_meters[4];
    float target_meters[4];
    float target_sum = 0.0;
    for (int i = 0; i < 4; i++)
    {
        current_meters[i] = read_current_position(i);
        target_meters[i] = (float)motorSteps[i] / ACTUAL_STEPS_PER_METER;
        target_sum += fabsf(target_meters[i]);
        int delta = motorSteps[i] - (int)(current_meters[i] * ACTUAL_STEPS_PER_METER);
        if (delta < 0)
            dir[i] = 'L'; // Moving to negative direction
        else if (delta > 0)
            dir[i] = 'R'; // Moving to positive direction
        else
            dir[i] = '0';
        // printf("Motor %d: current %.4f m, target %.4f m, dir %c\n", i, current_meters[i], target_meters[i], dir[i]);
    }

    // Detect spreading vs. converging: large target_sum (e.g., 4.4) for spreading, small (e.g., 2.2, 0) for converging
    int is_spreading = (target_sum > 3.0); // Threshold for Scenario 1 (4.4) vs. Scenarios 2 (2.2), 3 (0)

    int num_succ[4] = {0};
    int successors[4][3] = {{0}};
    for (int i = 0; i < 4; i++)
    {
        indegree[i] = 0; // Initialize indegree
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j || dir[i] == '0' || dir[j] == '0')
                continue;

            // Check if motors are at the same position
            if (fabsf(current_meters[i] - current_meters[j]) < 0.001)
            {
                float dist_i = fabsf(target_meters[i]);
                float dist_j = fabsf(target_meters[j]);
                if (is_spreading)
                {
                    // Spreading: prioritize farther from center
                    if (dist_i > dist_j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (same pos, spreading, %d to farther %.4f)\n", i, j, i, target_meters[i]);
                    }
                    else if (dist_j > dist_i)
                    {
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        // printf("Added edge %d -> %d (same pos, spreading, %d to farther %.4f)\n", j, i, j, target_meters[j]);
                    }
                    // Equal distances: higher index for 'R', lower for 'L'
                    else if (dir[i] == 'R' && dir[j] == 'R' && i > j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (R, same pos, spreading, higher index)\n", i, j);
                    }
                    else if (dir[i] == 'L' && dir[j] == 'L' && i < j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (L, same pos, spreading, lower index)\n", i, j);
                    }
                }
                else
                {
                    // Converging: prioritize closer to center
                    if (dist_i < dist_j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (same pos, converging, %d to closer %.4f)\n", i, j, i, target_meters[i]);
                    }
                    else if (dist_j < dist_i)
                    {
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        // printf("Added edge %d -> %d (same pos, converging, %d to closer %.4f)\n", j, i, j, target_meters[j]);
                    }
                    // Equal distances: higher index for 'R' (inner before outer), lower for 'L' (inner before outer)
                    else if (dir[i] == 'R' && dir[j] == 'R' && i > j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (R, same pos, converging, higher index)\n", i, j);
                    }
                    else if (dir[i] == 'L' && dir[j] == 'L' && i < j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (L, same pos, converging, lower index)\n", i, j);
                    }
                }
            }
            // Check for potential collision based on movement paths
            else
            {
                // Motor i moving right, j is in its path to target
                if (dir[i] == 'R' && current_meters[i] < current_meters[j] && target_meters[i] >= current_meters[j])
                {
                    if (is_spreading)
                    {
                        // Spreading: prioritize i (moving to outer position)
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (R, spreading, i at %.4f to %.4f, j at %.4f)\n", i, j, current_meters[i], target_meters[i], current_meters[j]);
                    }
                    else
                    {
                        // Converging: prioritize j (clear inner position first)
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        // printf("Added edge %d -> %d (R, converging, j at %.4f in i's path to %.4f)\n", j, i, current_meters[j], target_meters[i]);
                    }
                }
                // Motor i moving left, j is in its path to target
                else if (dir[i] == 'L' && current_meters[i] > current_meters[j] && target_meters[i] <= current_meters[j])
                {
                    if (is_spreading)
                    {
                        // Spreading: prioritize i (moving to outer position)
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        // printf("Added edge %d -> %d (L, spreading, i at %.4f to %.4f, j at %.4f)\n", i, j, current_meters[i], target_meters[i], current_meters[j]);
                    }
                    else
                    {
                        // Converging: prioritize j (clear inner position first)
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        // printf("Added edge %d -> %d (L, converging, j at %.4f in i's path to %.4f)\n", j, i, current_meters[j], target_meters[i]);
                    }
                }
            }
        }
    }

    int q[4];
    int front = 0, rear = 0;
    for (int i = 0; i < 4; i++)
    {
        if (indegree[i] == 0)
        {
            q[rear++] = i;
            // printf("Initial queue: added %d\n", i);
        }
    }

    int ord_idx = 0;
    while (front < rear)
    {
        int u = q[front++];
        order[ord_idx++] = u;
        // printf("Processing motor %d\n", u);
        for (int j = 0; j < num_succ[u]; j++)
        {
            int v = successors[u][j];
            indegree[v]--;
            // printf("Reduced indegree[%d] to %d\n", v, indegree[v]);
            if (indegree[v] == 0)
            {
                q[rear++] = v;
                // printf("Added to queue: %d\n", v);
            }
        }
    }

    if (ord_idx != 4)
    {
        printf("Error in computing order\n");
        for (int i = 0; i < 4; i++)
        {
            order[i] = i;
            indegree[i] = 0;
        }
    }
}

void moveMotorsWithSteps(
    int motorSteps[4],
    int baseParams[3],
    int divideFactor,
    int delayTime,
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

    int order[4];
    int indegree[4];
    get_move_order(motorSteps, order, indegree);

    printf("Move order: ");
    for (int i = 0; i < 4; i++)
    {
        printf("%d ", order[i]);
    }
    printf("\n");

    // Track which motors need to move
    int needs_move[4] = {0};
    for (int k = 0; k < motorCount; k++)
    {
        int index = order[k];
        if (motorSteps[index] != (int)(read_current_position(index) * ACTUAL_STEPS_PER_METER))
        {
            needs_move[k] = 1;
        }
    }

    // Start motors in topological order with minor delay within levels
    for (int k = 0; k < motorCount; k++)
    {
        int index = order[k];
        if (needs_move[k])
        {
            // printf("Starting motor %d to position %d\n", index, motorSteps[index]);
            safe_move_to_absolute(motors[index], motorSteps[index]);
            // Minor delay between motors in the same level
            if (k < motorCount - 1 && needs_move[k + 1] && indegree[order[k + 1]] == indegree[index])
            {
                delay_with_communication(10); // 10ms delay between motors in the same level
            }
        }
        // Apply level delay if next motor is in a new level or at the end
        if (k == motorCount - 1 || (k < motorCount - 1 && indegree[order[k + 1]] != indegree[index]))
        {
            delay_with_communication(delayTime);
        }
    }

    // Wait for completion in topological order
    for (int k = 0; k < motorCount; k++)
    {
        int index = order[k];
        if (needs_move[k])
        {
            wait_until_reached(motors[index], motorSteps[index]);
        }
    }
    printf("Movement completed: %s\n", logLabel);
}

void moveMotorSequentially(
    int motorSteps[4],
    int baseParams[3],
    int divideFactor,
    int delayTime,
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

    int order[4];
    int indegree[4];
    get_move_order(motorSteps, order, indegree);

    printf("Move order: ");
    for (int i = 0; i < 4; i++)
    {
        printf("%d ", order[i]);
    }
    printf("\n");

    for (int k = 0; k < motorCount; k++)
    {
        int index = order[k];
        if (motorSteps[index] != (int)(read_current_position(index) * ACTUAL_STEPS_PER_METER))
        {
            printf("Starting motor %d to position %d\n", index, motorSteps[index]);
            safe_move_to_absolute(motors[index], motorSteps[index]);
            delay_with_communication(delayTime);
            wait_until_reached(motors[index], motorSteps[index]);
        }
    }

    printf("Movement completed: %s\n", logLabel);
}

void testMovement(StepPattern pattern, int sequential, int divideFactor, int delayTime, const char *label)
{
    int steps[motorCount];
    for (int i = 0; i < motorCount; i++)
    {
        steps[i] = stepOffsets[pattern][i] * ACTUAL_STEPS_PER_METER;
    }

    if (sequential == 0)
    {
        moveMotorsWithSteps(steps, speed, divideFactor, delayTime, label);
    }
    else
    {
        moveMotorSequentially(steps, speed, divideFactor, delayTime, label);
    }

    printf("%s Steps: %d, %d, %d, %d\n", 
           label, steps[0], steps[1], steps[2], steps[3]);
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

void send_osc(float value)
{
    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", TARGET_PORT); // Convert int to string
    lo_address target = lo_address_new(TARGET_HOST, port_str);

    int ret = lo_send(target, OSC_PATH, "f", value);

    if (ret == -1)
    {
        fprintf(stderr, "OSC send failed: %s\n", lo_address_errstr(target));
    }
    else
    {
        printf("Sent OSC message '%s' with float value %.2f to %s:%d\n", OSC_PATH, value, TARGET_HOST, TARGET_PORT);
    }

    lo_address_free(target);
}

void semicom_wrap(){
    struct timespec step_start, step_end;
    double step_time;
    int sequential = 0; // Set to 1 for sequential movement, 0 for parallel 
    
    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_HOME, sequential, 0,   30, "HOME Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for HOME Movement: %.3f seconds\n", step_time);

    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_END, sequential, 1,   30,  "END Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for END Movement: %.3f seconds\n", step_time);

    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_SPREAD_EQUAL, sequential, 0,    30,  "SPREAD Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for SPREAD Movement: %.3f seconds\n", step_time);

    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_HOME, sequential, 0,   30, "HOME Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for HOME Movement: %.3f seconds\n", step_time);
}

void template_semicom()
{
    int sequential = 0; // Set to 1 for sequential movement, 0 for parallel
    
    struct timespec total_start, total_end;
    struct timespec step_start, step_end;
    double total_time, step_time;
    
    clock_gettime(CLOCK_MONOTONIC, &total_start);

    printf("Running Template 1 Sequence...\n");
    
    semicom_wrap();

    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_LEFT, sequential, 0,   30, "STEPS_LEFT Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for final STEPS_LEFT Movement: %.3f seconds\n", step_time);

    semicom_wrap();

    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_RIGHT, sequential, 0,   30, "STEPS_RIGHT Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for final STEPS_RIGHT Movement: %.3f seconds\n", step_time);

    printf("Ending Template 1 Sequence...\n");
    
    clock_gettime(CLOCK_MONOTONIC, &total_end);
    total_time = (total_end.tv_sec - total_start.tv_sec) + (double)(total_end.tv_nsec - total_start.tv_nsec) / 1e9;
    printf("Total time for Template 1 Sequence: %.3f seconds\n", total_time);
}

void template_dell(int video_delay_time, float video_number)
{
    int sequential = 0; // Set to 1 for sequential movement, 0 for parallel
    
    struct timespec total_start, total_end;
    struct timespec step_start, step_end;
    double total_time, step_time;
    
    clock_gettime(CLOCK_MONOTONIC, &total_start);
    
    video_number = video_number - 1.0f;

    printf("Running Template 1 Sequence...\n");
    send_osc(video_number + video_number_offset);
    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_HOME, sequential, 0,   30, "HOME Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for HOME Movement: %.3f seconds\n", step_time);
    printf("--------STEP 1 FINISH : HOME --------\n");

    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_END, sequential, 1,   30,  "END Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for END Movement: %.3f seconds\n", step_time);
    printf("--------STEP 3 FINISH : END --------\n");


    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_SPREAD, sequential, 0,    30,  "SPREAD Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for SPREAD Movement: %.3f seconds\n", step_time);
    printf("--------STEP 5 FINISH : SPREAD --------\n");


    clock_gettime(CLOCK_MONOTONIC, &step_start);
    testMovement(STEPS_HOME, sequential, 0,   30, "HOME Movement");
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for final HOME Movement: %.3f seconds\n", step_time);
    printf("--------STEP 7 FINISH : HOME --------\n");

    // video_delay_time = video_delay_time > 0 ? video_delay_time : 6150; 
    send_osc(video_number);
    clock_gettime(CLOCK_MONOTONIC, &step_start);
    delay_with_communication(video_delay_time);  //6150 for 10sec
    clock_gettime(CLOCK_MONOTONIC, &step_end);
    step_time = (step_end.tv_sec - step_start.tv_sec) + (double)(step_end.tv_nsec - step_start.tv_nsec) / 1e9;
    printf("Time taken for delay after HOME: %.3f seconds\n", step_time);
    printf("--------STEP 2 FINISH : DELAY --------\n");

    printf("Ending Template 1 Sequence...\n");
    
    clock_gettime(CLOCK_MONOTONIC, &total_end);
    total_time = (total_end.tv_sec - total_start.tv_sec) + (double)(total_end.tv_nsec - total_start.tv_nsec) / 1e9;
    printf("Total time for Template 1 Sequence: %.3f seconds\n", total_time);
}

void template_dell_loop()
{
    printf("Running Template 1 Sequence in loop. Press SPACEBAR to stop...\n");

    int running_loop = 1;
    int loop_count = 0;
    struct timespec loop_start, loop_end;
    double loop_time;

    while (running_loop)
    {
        printf("\nStarting loop iteration #%d\n", ++loop_count);

        clock_gettime(CLOCK_MONOTONIC, &loop_start);

        int delay_index = (loop_count - 1) % num_delays;
        int video_delay_time = delay_times[delay_index];
        float video_number = (float)(delay_index + 1);
        (void)video_delay_time;
        (void)video_number;
        // send_osc(video_number);
        
        // template_dell(video_delay_time, video_number);
        template_semicom();

        clock_gettime(CLOCK_MONOTONIC, &loop_end);
        loop_time = (loop_end.tv_sec - loop_start.tv_sec) + (double)(loop_end.tv_nsec - loop_start.tv_nsec) / 1e9;
        printf("Time taken for loop iteration #%d: %.3f seconds\n", loop_count, loop_time);

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

    printf("Template dell loop stopped.\n");
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
        case 'w':
            template_dell(6150, 1.0f);
            break;
        case 'q':
            template_dell_loop();
            break;
        case 'h':
            testMovement(STEPS_HOME, 0, 0,   30, "HOME Movement");
            break;
        case 'z':
            testMovement(STEPS_LEFT, 0, 0,  30,  "END Movement");
            break;
        case 's':
            template_semicom();
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