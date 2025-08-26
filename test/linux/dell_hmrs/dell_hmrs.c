#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <gtk/gtk.h>

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

// GTK-related global variables
static GtkTextBuffer *status_buffer;
static GtkTextView *status_textview;
static GMainLoop *main_loop;

// Function to append text to the status TextView
void append_to_status(const char *text) {
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(status_buffer, &end);
    gtk_text_buffer_insert(status_buffer, &end, text, -1);
    gtk_text_buffer_insert(status_buffer, &end, "\n", -1);
    
    // Scroll to the end
    GtkTextMark *mark = gtk_text_buffer_get_insert(status_buffer);
    gtk_text_view_scroll_to_mark(status_textview, mark, 0.0, FALSE, 0.0, 0.0);
}

// Redirect printf to status TextView
int vprintf(const char *format, va_list args) {
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    append_to_status(buffer);
    return vfprintf(stdout, format, args); // Also print to console
}

int printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int ret = vprintf(format, args);
    va_end(args);
    return ret;
}

void send_and_receive()
{
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
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
        send_and_receive();
        float current_pos = read_current_scaled_position(slave);

        if (fabsf(current_pos - target_position) < 1000)
        {
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

    printf("WARNING: Slave %d timed out while moving to position %d\n", slave, target_position);
}

void delay_with_communication(int milliseconds)
{
    printf("Waiting %d ms while maintaining communication...\n", milliseconds);
    int iterations = milliseconds;

    for (int i = 0; i < iterations; i++)
    {
        send_and_receive();
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
            dir[i] = 'L';
        else if (delta > 0)
            dir[i] = 'R';
        else
            dir[i] = '0';
        printf("Motor %d: current %.4f m, target %.4f m, dir %c\n", 
               i, current_meters[i], target_meters[i], dir[i]);
    }

    int is_spreading = (target_sum > 3.0);

    int num_succ[4] = {0};
    int successors[4][3] = {{0}};
    for (int i = 0; i < 4; i++)
    {
        indegree[i] = 0;
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j || dir[i] == '0' || dir[j] == '0')
                continue;

            if (fabsf(current_meters[i] - current_meters[j]) < 0.001)
            {
                float dist_i = fabsf(target_meters[i]);
                float dist_j = fabsf(target_meters[j]);
                if (is_spreading)
                {
                    if (dist_i > dist_j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (same pos, spreading, %d to farther %.4f)\n", i, j, i, target_meters[i]);
                    }
                    else if (dist_j > dist_i)
                    {
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        printf("Added edge %d -> %d (same pos, spreading, %d to farther %.4f)\n", j, i, j, target_meters[j]);
                    }
                    else if (dir[i] == 'R' && dir[j] == 'R' && i > j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (R, same pos, spreading, higher index)\n", i, j);
                    }
                    else if (dir[i] == 'L' && dir[j] == 'L' && i < j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (L, same pos, spreading, lower index)\n", i, j);
                    }
                }
                else
                {
                    if (dist_i < dist_j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (same pos, converging, %d to closer %.4f)\n", i, j, i, target_meters[i]);
                    }
                    else if (dist_j < dist_i)
                    {
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        printf("Added edge %d -> %d (same pos, converging, %d to closer %.4f)\n", j, i, j, target_meters[j]);
                    }
                    else if (dir[i] == 'R' && dir[j] == 'R' && i < j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (R, same pos, converging, lower index)\n", i, j);
                    }
                    else if (dir[i] == 'L' && dir[j] == 'L' && i > j)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (L, same pos, converging, higher index)\n", i, j);
                    }
                }
            }
            else
            {
                if (dir[i] == 'R' && current_meters[i] < current_meters[j] && target_meters[i] >= current_meters[j])
                {
                    if (is_spreading)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (R, spreading, i at %.4f to %.4f, j at %.4f)\n", i, j, current_meters[i], target_meters[i], current_meters[j]);
                    }
                    else
                    {
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        printf("Added edge %d -> %d (R, converging, j at %.4f in i's path to %.4f)\n", j, i, current_meters[j], target_meters[i]);
                    }
                }
                else if (dir[i] == 'L' && current_meters[i] > current_meters[j] && target_meters[i] <= current_meters[j])
                {
                    if (is_spreading)
                    {
                        successors[i][num_succ[i]++] = j;
                        indegree[j]++;
                        printf("Added edge %d -> %d (L, spreading, i at %.4f to %.4f, j at %.4f)\n", i, j, current_meters[i], target_meters[i], current_meters[j]);
                    }
                    else
                    {
                        successors[j][num_succ[j]++] = i;
                        indegree[i]++;
                        printf("Added edge %d -> %d (L, converging, j at %.4f in i's path to %.4f)\n", j, i, current_meters[j], target_meters[i]);
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
            printf("Initial queue: added %d\n", i);
        }
    }

    int ord_idx = 0;
    while (front < rear)
    {
        int u = q[front++];
        order[ord_idx++] = u;
        printf("Processing motor %d\n", u);
        for (int j = 0; j < num_succ[u]; j++)
        {
            int v = successors[u][j];
            indegree[v]--;
            printf("Reduced indegree[%d] to %d\n", v, indegree[v]);
            if (indegree[v] == 0)
            {
                q[rear++] = v;
                printf("Added to queue: %d\n", v);
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

    int needs_move[4] = {0};
    for (int k = 0; k < motorCount; k++)
    {
        int index = order[k];
        if (motorSteps[index] != (int)(read_current_position(index) * ACTUAL_STEPS_PER_METER))
        {
            needs_move[k] = 1;
        }
    }

    for (int k = 0; k < motorCount; k++)
    {
        int index = order[k];
        if (needs_move[k])
        {
            printf("Starting motor %d to position %d\n", index, motorSteps[index]);
            safe_move_to_absolute(motors[index], motorSteps[index]);
            if (k < motorCount - 1 && needs_move[k + 1] && indegree[order[k + 1]] == indegree[index])
            {
                delay_with_communication(10);
            }
        }
        if (k == motorCount - 1 || (k < motorCount - 1 && indegree[order[k + 1]] != indegree[index]))
        {
            delay_with_communication(delayTime);
        }
    }

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
    {-0.8, -0.25, 0.25, 0.8}
};
int speed[] = {70000, 35000, 35000};

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

void template_dell()
{
    int sequential = 0;

    printf("Running Template 1 Sequence...\n");
    delay_with_communication(6250);

    testMovement(STEPS_END, sequential, 1, 30, "END Movement");
    delay_with_communication(500);

    testMovement(STEPS_SPREAD, sequential, 0, 30, "SPREAD Movement");
    delay_with_communication(500);

    testMovement(STEPS_HOME, sequential, 0, 30, "HOME Movement");

    printf("Ending Template 1 Sequence...\n");
}

static gboolean template_loop_running = FALSE;
static guint template_loop_source_id = 0;

gboolean template_dell_loop(gpointer user_data)
{
    (void)user_data; 
    static int loop_count = 0;
    if (!template_loop_running)
        return FALSE;

    printf("\nStarting loop iteration #%d\n", ++loop_count);
    template_dell();
    delay_with_communication(1000);
    return TRUE;
}

gboolean periodic_communication(gpointer user_data G_GNUC_UNUSED)
{
    if (!keep_running)
        return FALSE;

    send_and_receive();
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        read_current_position(i);
    }
    return TRUE;
}

// GTK signal handlers
void on_enable_button_clicked(GtkButton *button G_GNUC_UNUSED, gpointer user_data G_GNUC_UNUSED)
{
    for (int i = 0; i < NUM_SLAVES; i++)
        enable_drive(i);
}

void on_disable_button_clicked(GtkButton *button G_GNUC_UNUSED, gpointer user_data G_GNUC_UNUSED)
{
    for (int i = 0; i < NUM_SLAVES; i++)
        disable_drive(i);
}

void on_reset_button_clicked(GtkButton *button G_GNUC_UNUSED, gpointer user_data G_GNUC_UNUSED)
{
    for (int i = 0; i < NUM_SLAVES; i++)
        reset_fault(i);
}

void on_status_button_clicked(GtkButton *button G_GNUC_UNUSED, gpointer user_data G_GNUC_UNUSED)
{
    for (int i = 0; i < NUM_SLAVES; i++)
        is_drive_enabled(i);
}

void on_template_button_clicked(GtkButton *button G_GNUC_UNUSED, gpointer user_data G_GNUC_UNUSED)
{
    template_dell();
}

void on_template_loop_button_clicked(GtkButton *button, gpointer user_data G_GNUC_UNUSED)
{
    if (!template_loop_running)
    {
        template_loop_running = TRUE;
        template_loop_source_id = g_idle_add(template_dell_loop, NULL);
        gtk_button_set_label(button, "Stop Template Loop");
    }
    else
    {
        template_loop_running = FALSE;
        if (template_loop_source_id != 0)
        {
            g_source_remove(template_loop_source_id);
            template_loop_source_id = 0;
        }
        gtk_button_set_label(button, "Run Template Loop");
    }
}

void on_quit_button_clicked(GtkButton *button G_GNUC_UNUSED, gpointer user_data G_GNUC_UNUSED)
{
    keep_running = 0;
    template_loop_running = FALSE;
    if (template_loop_source_id != 0)
    {
        g_source_remove(template_loop_source_id);
        template_loop_source_id = 0;
    }
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_INIT;
        ec_writestate(i + 1);
    }
    ec_close();
    g_main_loop_quit(main_loop);
}

void int_handler(int sig)
{
    (void)sig;
    printf("\nCaught SIGINT (Ctrl+C), stopping motors...\n");
    keep_running = 0;
    template_loop_running = FALSE;
    if (template_loop_source_id != 0)
    {
        g_source_remove(template_loop_source_id);
        template_loop_source_id = 0;
    }
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        disable_drive(i);
    }
    ec_close();
    g_main_loop_quit(main_loop);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, int_handler);

    // Initialize GTK
    gtk_init(&argc, &argv);

    // Load the UI file
    GtkBuilder *builder = gtk_builder_new();
    GError *error = NULL;
    if (!gtk_builder_add_from_file(builder, "interface.ui", &error))
    {
        fprintf(stderr, "Cannot load UI file: %s\n", error->message);
        g_error_free(error);
        return -1;
    }

    // Get the main window and text view
    GtkWidget *window = GTK_WIDGET(gtk_builder_get_object(builder, "main_window"));
    status_textview = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "status_textview"));
    status_buffer = gtk_text_view_get_buffer(status_textview);
    gtk_builder_connect_signals(builder, NULL);

    // Initialize EtherCAT
    if (!ec_init("enp0s31f6"))
    {
        printf("Failed to initialize EtherCAT interface.\n");
        g_object_unref(builder);
        return -1;
    }

    if (ec_config_init(FALSE) < NUM_SLAVES)
    {
        printf("Not enough slaves found!\n");
        ec_close();
        g_object_unref(builder);
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
            g_object_unref(builder);
            return -1;
        }
    }

    printf("Slaves are OPERATIONAL.\n");

    // Show the window
    gtk_widget_show_all(window);

    // Start periodic communication
    g_timeout_add(20, periodic_communication, NULL);

    // Run the main loop
    main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(main_loop);

    // Cleanup
    g_object_unref(builder);
    g_main_loop_unref(main_loop);
    printf("\nExited.\n");
    return 0;
}