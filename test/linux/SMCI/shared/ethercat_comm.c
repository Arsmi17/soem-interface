#include "ethercat_comm.h"
#include "app_context.h"    // AppContext must define: app.total_slaves, app_update_slave_count()
#include "ethercat.h"       // SOEM EtherCAT master library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <glib.h>
#include <stdbool.h>   // For bool type
#include <inttypes.h>
#include <math.h>  // For fabs()
static pthread_mutex_t ethercat_mutex = PTHREAD_MUTEX_INITIALIZER;


// Internal global variables
static char IOmap[4096];
static el7_out_t *out_data[NUM_SLAVES];
static el7_in_t  *in_data[NUM_SLAVES];
static int connected_slave_count = 0;
static volatile gboolean keep_monitoring = TRUE;
static GThread *monitor_thread = NULL;
static pthread_t comm_thread;

volatile gboolean running = TRUE;
GAsyncQueue *button_command_queue = NULL; 

static bool ethercat_initialized = false;

// Forward declarations
static void *ethercat_comm_thread(void *data);
static void *monitor_slave_thread(gpointer data);
static void send_and_receive(void);
gboolean update_app_slave_count_idle(gpointer data);

// Safety constants
#define MIN_DISTANCE_MM 100.0f  // Minimum 100mm distance between slaves
#define METERS_TO_MM 1000.0f    // Conversion factor

static bool movement_monitoring_active = false;
static int moving_slave_idx = -1;
static bool moving_direction_forward = false;
static int check_slave_idx = -1;
extern void set_velocity(int slave_idx, int32_t velocity);

// Structure for movement monitoring data
typedef struct {
    int slave_idx;
    bool is_forward;
    int check_slave;
} MovementMonitorData;


float read_current_position(int slave_id) {
    // Check for valid slave ID and input availability
    if (slave_id < 0 || slave_id >= connected_slave_count || !ec_slave[slave_id + 1].inputs) {
        // Update display with error state
        update_position_display(slave_id, 0, 0.0f);
        return 0.0f;
    }

    // Get pointer to input data
    uint8_t *inputs = ec_slave[slave_id + 1].inputs;
    uint8_t *pos_ptr = inputs + POSITION_ACTUAL_VALUE_OFFSET;

    // Convert 4 bytes to signed 32-bit integer (little-endian)
    int32_t raw_position = (int32_t)((pos_ptr[3] << 24) | (pos_ptr[2] << 16) | 
                                     (pos_ptr[1] << 8) | pos_ptr[0]);

    // Apply scaling
    float scale_factor = (float)ACTUAL_STEPS_PER_METER / (float)RAW_STEPS_PER_METER;
    float scaled_position = (float)raw_position * scale_factor;

    // Convert to meters
    float position_in_meters = scaled_position / (float)ACTUAL_STEPS_PER_METER;

    // Update the corresponding display widget
    update_position_display(slave_id, raw_position, position_in_meters);

    return scaled_position;
}
// --- Public API ---
#define MODE_OF_OPERATION 3;
bool ethercat_comm_init(const char *iface) {
    printf("[EtherCAT] Initializing on interface: %s\n", iface);
    

    if (!ec_init(iface)) {
        fprintf(stderr, "[EtherCAT] Failed to initialize interface %s\n", iface);
        return false;
    }

    if (ec_config_init(FALSE) <= 0) {
        fprintf(stderr, "[EtherCAT] No slaves found!\n");
        ec_close();
        return false;
    }

    ec_config_map(&IOmap);
    ec_configdc();

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    int chk = 40;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL);

    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        fprintf(stderr, "[EtherCAT] Slaves failed to reach OPERATIONAL state\n");
        ec_close();
        return false;
    }

    connected_slave_count = ec_slavecount;
    if (connected_slave_count > NUM_SLAVES) {
        fprintf(stderr, "[EtherCAT] Too many slaves (%d) for allocated (%d)!\n",
                connected_slave_count, NUM_SLAVES);
        ec_close();
        return false;
    }

    // Assign input/output pointers
    for (int i = 0; i < connected_slave_count; ++i) {
        out_data[i] = (el7_out_t *) ec_slave[i + 1].outputs;
        in_data[i]  = (el7_in_t *) ec_slave[i + 1].inputs;

        // **CRITICAL: Initialize target velocity to 0**
        if (out_data[i]) {
            out_data[i]->target_velocity = 0;
            out_data[i]->control_word = 0x00; // Disabled state
        }
        // Setup motor/network parameters
        uint8_t mode = MODE_OF_OPERATION; // Profile position
        uint32_t velocity = 80000;
        uint32_t accel = 6000;
        uint32_t decel = 6000;

        ec_SDOwrite(i + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6081, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6083, 0x00, FALSE, sizeof(accel), &accel, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6084, 0x00, FALSE, sizeof(decel), &decel, EC_TIMEOUTSAFE);
    }

        // After slaves reach OPERATIONAL, add settling time
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("[EtherCAT] All slaves operational, allowing settling time...\n");
        usleep(500000); // 500ms settling time
        
        // Verify all individual slaves
        for (int i = 1; i <= connected_slave_count; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                printf("[EtherCAT] Warning: Slave %d not fully operational: 0x%04X\n", 
                       i, ec_slave[i].state);
            }
        }
    }

    printf("[EtherCAT] Initialization complete with %d slaves\n", connected_slave_count);

    // From your snippet: Create queue and start background thread
    button_command_queue = g_async_queue_new();
    if (pthread_create(&comm_thread, NULL, ethercat_comm_thread, NULL) != 0) {
        fprintf(stderr, "[EtherCAT] Failed to create communication thread\n");
        ec_close();
        return false;
    }

    // Launch slave monitor thread (30s check)
    monitor_thread = g_thread_new("EC-SlaveMonitor", monitor_slave_thread, NULL);

    ethercat_initialized = true;

    g_idle_add(update_app_slave_count_idle, GINT_TO_POINTER(connected_slave_count));

    return true;
}

void ethercat_comm_cleanup(void) {
    printf("[EtherCAT] Cleaning up...\n");

    // From your snippet: Shutdown thread and queue
    running = FALSE;
    if (button_command_queue) {
        g_async_queue_unref(button_command_queue);
        button_command_queue = NULL;
    }
    pthread_join(comm_thread, NULL);

    keep_monitoring = FALSE;
    if (monitor_thread) {
        g_thread_join(monitor_thread);
        monitor_thread = NULL;
    }

    // Bring each slave to INIT and clean
    for (int i = 0; i < connected_slave_count; ++i) {
        ec_slave[i + 1].state = EC_STATE_INIT;
        ec_writestate(i + 1);
    }

    ec_close();
    printf("[EtherCAT] Closed successfully.\n");
}

int ethercat_get_slave_count(void) {
    return connected_slave_count;
}

void ethercat_trigger_slave_check(void) {
    g_idle_add(update_app_slave_count_idle, GINT_TO_POINTER(connected_slave_count));
}

gboolean update_app_slave_count_idle(gpointer data) {
    int new_count = GPOINTER_TO_INT(data);
    app.total_slaves = new_count;
    app_update_slave_count(new_count);
    return G_SOURCE_REMOVE; // run once
}

void send_and_receive() {
    pthread_mutex_lock(&ethercat_mutex);
    
    int wkc = ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    if (wkc < connected_slave_count) {
        printf("[EtherCAT] Warning: WKC=%d, expected=%d. Possible communication issue.\n", 
               wkc, connected_slave_count);
        ec_readstate();
        for (int i = 1; i <= connected_slave_count; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                printf("[EtherCAT] Slave %d not in OPERATIONAL state: 0x%04X\n", 
                       i, ec_slave[i].state);
            }
        }
    }
    
    pthread_mutex_unlock(&ethercat_mutex);
}

static void *monitor_slave_thread(gpointer data) {
    (void)data;
    int actual_connected = 0;

    while (keep_monitoring) {
        ec_readstate();
        actual_connected = 0;

        // Count only slaves in OPERATIONAL or at least SAFE_OP
        for (int i = 1; i <= ec_slavecount; ++i) {
            if (ec_slave[i].state == 0x00) {
                printf("[Monitor] Slave %d is OFFLINE.\n", i);
            }

            if (ec_slave[i].state >= EC_STATE_SAFE_OP) {
                actual_connected++;
            }
        }

        if (actual_connected != connected_slave_count) {
            connected_slave_count = actual_connected;
            g_idle_add(update_app_slave_count_idle, GINT_TO_POINTER(actual_connected));
        }

        sleep(5);
    }

    return NULL;
}

static void recover_slaves(void) {
    printf("[EtherCAT] Starting slave recovery...\n");
    
    for (int i = 1; i <= connected_slave_count; i++) {
        if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
            printf("[EtherCAT] Recovering slave %d from state 0x%04X\n", 
                   i, ec_slave[i].state);
            
            // Gradual state transition
            ec_slave[i].state = EC_STATE_INIT;
            ec_writestate(i);
            usleep(50000);
            
            ec_slave[i].state = EC_STATE_PRE_OP;
            ec_writestate(i);
            usleep(50000);
            
            ec_slave[i].state = EC_STATE_SAFE_OP;
            ec_writestate(i);
            usleep(50000);
            
            ec_slave[i].state = EC_STATE_OPERATIONAL;
            ec_writestate(i);
            usleep(100000);
            
            // Verify recovery
            ec_readstate();
            if (ec_slave[i].state == EC_STATE_OPERATIONAL) {
                printf("[EtherCAT] Slave %d recovered successfully\n", i);
            } else {
                printf("[EtherCAT] Failed to recover slave %d\n", i);
            }
        }
    }
}

static float get_slave_position_meters(int slave_idx) {
    if (slave_idx < 0 || slave_idx >= connected_slave_count) {
        return 0.0f;  // Invalid slave, return 0
    }
    
    // Use the existing read_current_position function but don't log to UI
    if (!ec_slave[slave_idx + 1].inputs) {
        return 0.0f;
    }
    
    uint8_t *inputs = ec_slave[slave_idx + 1].inputs;
    uint8_t *pos_ptr = inputs + POSITION_ACTUAL_VALUE_OFFSET;
    
    // Convert 4 bytes to signed 32-bit integer (little-endian)
    int32_t raw_position = (int32_t)((pos_ptr[3] << 24) | (pos_ptr[2] << 16) | 
                                     (pos_ptr[1] << 8) | pos_ptr[0]);
    
    // Apply scaling
    float scale_factor = (float)ACTUAL_STEPS_PER_METER / (float)RAW_STEPS_PER_METER;
    float scaled_position = (float)raw_position * scale_factor;
    
    // Convert to meters
    return scaled_position / (float)ACTUAL_STEPS_PER_METER;
}

static void get_adjacent_slaves_for_direction(int slave_idx, bool is_forward, int *check_slave) {
    *check_slave = -1;  // Invalid by default
    
    if (strcmp(app.selection_type, "Matrix") == 0) {
        // Matrix layout: 3 rows, arranged vertically then horizontally
        // Row 0: 0, 3, 6, 9,  12, 15, ...
        // Row 1: 1, 4, 7, 10, 13, 16, ...  
        // Row 2: 2, 5, 8, 11, 14, 17, ...
        
        int row = slave_idx % 3;  // Which row (0, 1, or 2)
        
        if (row == 0) {
            // Row 0: Check Row 1 only when moving backward
            if (!is_forward) {
                int corresponding_row1_slave = slave_idx + 1;
                if (corresponding_row1_slave < connected_slave_count) {
                    *check_slave = corresponding_row1_slave;
                }
            }
            // Forward movement: no check needed
        } else if (row == 1) {
            // Row 1: Check Row 0 when forward, Row 2 when backward
            if (is_forward) {
                int corresponding_row0_slave = slave_idx - 1;
                if (corresponding_row0_slave >= 0) {
                    *check_slave = corresponding_row0_slave;
                }
            } else {
                int corresponding_row2_slave = slave_idx + 1;
                if (corresponding_row2_slave < connected_slave_count) {
                    *check_slave = corresponding_row2_slave;
                }
            }
        } else if (row == 2) {
            // Row 2: Check Row 1 only when moving forward
            if (is_forward) {
                int corresponding_row1_slave = slave_idx - 1;
                if (corresponding_row1_slave >= 0) {
                    *check_slave = corresponding_row1_slave;
                }
            }
            // Backward movement: no check needed
        }
    } else if (strcmp(app.selection_type, "HMRS") == 0) {
        // HMRS layout: 2 rows, arranged horizontally then vertically
        // Row 0: 0, 2, 4, 6, ... (even indices - no checks needed)
        // Row 1: 1, 3, 5, 7, ... (odd indices - check side by side)
        
        if (slave_idx % 2 == 1) {  // Only odd-indexed slaves need checks
            if (is_forward) {
                // Check next odd slave (slave_idx + 2)
                int next_odd_slave = slave_idx + 2;
                if (next_odd_slave < connected_slave_count) {
                    *check_slave = next_odd_slave;
                }
            } else {
                // Check previous odd slave (slave_idx - 2)
                int prev_odd_slave = slave_idx - 2;
                if (prev_odd_slave >= 0) {
                    *check_slave = prev_odd_slave;
                }
            }
        }
        // Even-indexed slaves (0, 2, 4, 6...) don't need position checks
    }
}

static bool is_movement_safe_directional(int slave_idx, int32_t target_velocity, bool is_forward) {
    (void)target_velocity;  // Unused in this context, but can be used for logging
    // Get current position of the slave
    float current_pos = get_slave_position_meters(slave_idx);
    
    // Get the slave to check based on direction
    int check_slave;
    get_adjacent_slaves_for_direction(slave_idx, is_forward, &check_slave);
    
    // If no slave needs checking, movement is safe
    if (check_slave == -1) {
        printf("[Safety] No adjacent slave check needed for slave %d moving %s\n", 
               slave_idx, is_forward ? "forward" : "backward");
        return true;
    }
    
    // Check distance with the relevant slave
    float check_pos = get_slave_position_meters(check_slave);
    float distance_mm = fabs(current_pos - check_pos) * METERS_TO_MM;
    
    if (distance_mm < MIN_DISTANCE_MM) {
        printf("[Safety] Insufficient distance between slaves: %.2fmm (min: %.0fmm)\n", 
               distance_mm, MIN_DISTANCE_MM);
        
        // Create detailed error message
        char error_msg[256];
        snprintf(error_msg, sizeof(error_msg), 
                "Can't apply velocity - Current [%d: %.3fm], %s [%d: %.3fm]",
                slave_idx, current_pos,
                is_forward ? "Target check" : "Target check",
                check_slave, check_pos);
        
        printf("[Safety] %s\n", error_msg);
        return false;
    }
    
    printf("[Safety] Safe distance confirmed: slave %d (%.3fm) to slave %d (%.3fm) = %.2fmm\n",
           slave_idx, current_pos, check_slave, check_pos, distance_mm);
    
    return true;  // Movement is safe
}

static void start_movement_monitoring(int slave_idx, bool is_forward) {
    movement_monitoring_active = true;
    moving_slave_idx = slave_idx;
    moving_direction_forward = is_forward;
    
    // Get the slave to monitor based on direction
    get_adjacent_slaves_for_direction(slave_idx, is_forward, &check_slave_idx);
    
    printf("[Safety Monitor] Started monitoring slave %d moving %s (checking against slave %d)\n",
           slave_idx, is_forward ? "forward" : "backward", check_slave_idx);
}

static void stop_movement_monitoring(void) {
    if (movement_monitoring_active) {
        printf("[Safety Monitor] Stopping monitoring for slave %d\n", moving_slave_idx);
    }
    
    // Reset all monitoring variables
    movement_monitoring_active = false;
    moving_slave_idx = -1;
    moving_direction_forward = false;
    check_slave_idx = -1;
    
    printf("[Safety Monitor] All monitoring variables reset\n");
}

static void monitor_movement_safety(void) {
    if (!movement_monitoring_active || moving_slave_idx == -1 || check_slave_idx == -1) {
        return;  // No active monitoring
    }
    
    // Get current positions
    float moving_pos = get_slave_position_meters(moving_slave_idx);
    float check_pos = get_slave_position_meters(check_slave_idx);
    float distance_mm = fabs(moving_pos - check_pos) * METERS_TO_MM;
    
    // Check if distance is too small
    if (distance_mm <= MIN_DISTANCE_MM) {
        printf("[Safety Monitor] COLLISION RISK! Distance: %.2fmm (min: %.0fmm)\n", 
               distance_mm, MIN_DISTANCE_MM);
        printf("[Safety Monitor] Emergency stop for slave %d\n", moving_slave_idx);
        
        set_velocity(moving_slave_idx, 0);  // Stop the moving slave
        send_and_receive();
        
        // Stop monitoring
        stop_movement_monitoring();
        
        // Log the emergency stop
        char emergency_msg[256];
        snprintf(emergency_msg, sizeof(emergency_msg),
                "EMERGENCY STOP: Slave %d stopped due to collision risk. Distance to slave %d: %.2fmm",
                moving_slave_idx, check_slave_idx, distance_mm);
        printf("[Safety Monitor] %s\n", emergency_msg);
        
        // You can also send this to UI log if needed
        // LOG_ERROR(emergency_msg);
        
    } else if (distance_mm <= (MIN_DISTANCE_MM + 20.0f)) {
        // Warning zone (120mm or less)
        printf("[Safety Monitor] WARNING: Close proximity - Distance: %.2fmm\n", distance_mm);
    }
}

void set_velocity(int slave_idx, int32_t velocity) {
    if (slave_idx < 0 || slave_idx >= connected_slave_count) {
        printf("Invalid slave index: %d\n", slave_idx);
        return;
    }

     if (velocity == 0) {
        if (moving_slave_idx == slave_idx) {
            stop_movement_monitoring();
        }
    }

    bool is_forward = (velocity > 0);

         // **NEW: Directional safety check before movement**
    if (!is_movement_safe_directional(slave_idx, velocity, is_forward)) {
        printf("[Safety] Movement blocked for slave %d moving %s\n", 
               slave_idx, is_forward ? "forward" : "backward");
        return;  // Don't proceed with movement
    }

    // printf("Setting velocity for slave %d to %" PRId32 "\n", slave_idx + 1, velocity);

    // // Ensure slave is in velocity mode and operational
    // ec_readstate();
    // if (ec_slave[slave_idx + 1].state != EC_STATE_OPERATIONAL) {
    //     printf("Slave %d not operational, current state: 0x%04X\n", 
    //            slave_idx + 1, ec_slave[slave_idx + 1].state);
    //     return;
    // }

    // // Verify we're in velocity mode
    uint8_t current_mode;
    int size = sizeof(current_mode);
    // if (ec_SDOread(slave_idx + 1, 0x6061, 0x00, FALSE, &size, &current_mode, EC_TIMEOUTSAFE)) {
    //     if (current_mode != 3) {  // 3 = Profile Velocity Mode
    //         printf("Warning: Slave %d not in velocity mode (current mode: %d)\n", 
    //                slave_idx + 1, current_mode);
    //     }
    // }

    // Check current status before proceeding
    uint16_t current_status = in_data[slave_idx]->status_word;
    printf("Current status word for slave %d: 0x%04X\n", slave_idx, current_status);
    
    // Set target velocity via PDO (primary method)
    out_data[slave_idx]->target_velocity = velocity;
    // out_data[slave_idx]->control_word = 0x000F;  // Enable operation

    send_and_receive();
    usleep(10000);

    // Optional: Also set via SDO as backup
    if (!ec_SDOwrite(slave_idx + 1, 0x60FF, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE)) {
        printf("Warning: Failed to set target velocity via SDO for slave %d\n", slave_idx + 1);
    }

    // Verify the command was accepted
    uint16_t status = in_data[slave_idx]->status_word;
    printf("Status Word after set_velocity: 0x%04X\n", status);
    
    // Read back actual velocity for verification
    int32_t actual_velocity;
    size = sizeof(actual_velocity);
    if (ec_SDOread(slave_idx + 1, 0x606C, 0x00, FALSE, &size, &actual_velocity, EC_TIMEOUTSAFE)) {
        printf("Current actual velocity: %" PRId32 "\n", actual_velocity);
    }
        // **NEW: Start continuous movement monitoring**
    start_movement_monitoring(slave_idx, is_forward);

    printf("[EtherCAT] Velocity set for slave %d completed.\n", slave_idx + 1);
    send_and_receive();
    usleep(10000); // Allow time for state change
}

void reset_fault(int slave_idx) {
    if (!ethercat_initialized) {
        fprintf(stderr, "[EtherCAT] ERROR: Cannot reset fault because EtherCAT not initialized!\n");
        return;
    }
    if (slave_idx < 0 || slave_idx >= connected_slave_count || !out_data[slave_idx]) {
        fprintf(stderr, "[EtherCAT] ERROR: Invalid slave index %d in reset_fault\n", slave_idx);
        return;
    }
    printf("[EtherCAT] Resetting fault on slave %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x80; // Fault reset command
    send_and_receive();
    usleep(10000);
    disable_drive(slave_idx); // Disable drive after fault reset
    send_and_receive();
}

void enable_drive(int slave_idx) {
    if (slave_idx < 0 || slave_idx >= connected_slave_count || !out_data[slave_idx]) {
        fprintf(stderr, "[EtherCAT] ERROR: Invalid slave index %d or null pointer in enable_drive\n", slave_idx);
        return;
    }

    printf("[EtherCAT] Enabling drive %d...\n", slave_idx + 1);
    
    // **CRITICAL FIX: Set target velocity to 0 FIRST**
    out_data[slave_idx]->target_velocity = 0;
    send_and_receive();
    usleep(10000);
    
    int max_retries = 3;
    int retry_count = 0;
    bool enabled = false;
    uint16_t status_word = 0;

    // Check if there's a fault, try to clear
    send_and_receive();
    status_word = in_data[slave_idx]->status_word;
    if (status_word & 0x0008) {
        printf("[EtherCAT] Fault detected on drive %d (status 0x%04X), resetting...\n", slave_idx + 1, status_word);
        reset_fault(slave_idx);
        usleep(50000);
    }

    // Attempt enabling sequence
    while (retry_count < max_retries && !enabled) {
        // **ENSURE velocity is 0 before each enable attempt**
        out_data[slave_idx]->target_velocity = 0;
        
        out_data[slave_idx]->control_word = 0x06; // Shutdown
        send_and_receive();
        usleep(10000);

        out_data[slave_idx]->control_word = 0x07; // Switch on
        send_and_receive();
        usleep(10000);

        out_data[slave_idx]->control_word = 0x0F; // Enable operation
        send_and_receive();
        usleep(10000);

        send_and_receive(); // Extra cycle for status update
        status_word = in_data[slave_idx]->status_word;

        // Operational enabled state (bits 6-8 == 0x27)
        if ((status_word & 0x006F) == 0x0027) {
            enabled = true;
            printf("[EtherCAT] Drive %d enabled successfully.\n", slave_idx + 1);
            // **REDUNDANT SAFETY: Ensure velocity is still 0**
            out_data[slave_idx]->target_velocity = 0;
            send_and_receive();
        } else {
            if (status_word & 0x0008) {
                printf("[EtherCAT] Fault detected during enable attempt %d on drive %d. Resetting...\n", retry_count + 1, slave_idx + 1);
                reset_fault(slave_idx);
                usleep(50000);
            }
            retry_count++;
        }
    }

    send_and_receive();
    usleep(10000);

    if (!enabled) {
        printf("[EtherCAT] ERROR: Failed to enable drive %d after %d attempts.\n", slave_idx + 1, max_retries);
    }
}

void disable_drive(int slave_idx) {
    if (slave_idx < 0 || slave_idx >= connected_slave_count || !out_data[slave_idx]) {
        fprintf(stderr, "[EtherCAT] ERROR: Invalid slave index %d or null pointer in disable_drive\n", slave_idx);
        return;
    }
    printf("[EtherCAT] Disabling drive %d...\n", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x00;
    send_and_receive();
}

bool is_drive_enabled(int slave_index)
{
    if (slave_index < 0 || slave_index >= connected_slave_count)
    {
        printf("Invalid slave index: %d\n", slave_index);
        LOG_ERROR("Invalid slave index: %d", slave_index);
        return FALSE;
    }

    ec_readstate();
    uint16 slave_state = ec_slave[slave_index + 1].state;

    if (slave_state != EC_STATE_OPERATIONAL)
    {
        printf("Drive at slave index %d is not in OPERATIONAL state. Current state: %d\n", slave_index, slave_state);
        LOG_WARN("Drive at slave index %d is not in OPERATIONAL state. Current state: %d", slave_index, slave_state);
        return FALSE;
    }

    uint16 status_word = 0;
    int size = sizeof(status_word);
    int wkc = ec_SDOread(slave_index + 1, 0x6041, 0x00, FALSE, &size, &status_word, EC_TIMEOUTRXM);

    if (wkc <= 0)
    {
        printf("Failed to read Status Word from slave %d\n", slave_index);
        LOG_ERROR("Failed to read Status Word from slave %d", slave_index);
        return FALSE;
    }

    boolean is_enabled = (status_word & (1 << 2)) != 0;

    if (is_enabled)
    {
        printf("Drive at slave index %d is enabled.\n", slave_index);
        LOG_SUCCESS("Drive at slave index %d is enabled.", slave_index);
    }
    else
    {
        printf("Drive at slave index %d is not enabled. Status Word: 0x%04X\n", slave_index, status_word);
        LOG_WARN("Drive at slave index %d is not enabled. Status Word: 0x%04X", slave_index, status_word);
    }

    return is_enabled;
}

// --- Background communication thread ---
static void *ethercat_comm_thread(void *data) {  // Your ec_thread_func
    (void)data;
        static int position_log_counter = 0;
    static int safety_monitor_counter = 0;

    while (running) {
        // Maintain communication
        send_and_receive();
        // send_osc_all_slaves(); // Uncomment if needed (from history)

        // Process queued commands from UI
        Command *cmd = g_async_queue_try_pop(button_command_queue);
        if (cmd) {
            switch (cmd->id) {
                case CMD_RESET_FAULTS:
                    for (int i = 0; i < connected_slave_count; i++) reset_fault(i);
                    break;
                case CMD_ENABLE_ALL:
                    for (int i = 0; i < connected_slave_count; i++) enable_drive(i);
                    break;
                case CMD_DISABLE_ALL:
                    for (int i = 0; i < connected_slave_count; i++) disable_drive(i);
                    break;
                case CMD_CHECK_STATUS:
                    for (int i = 0; i < connected_slave_count; i++) is_drive_enabled(i);
                    break;
                 case CMD_COMMAND_FORWARD:
                    printf("[EtherCAT] Executing forward motion command for slave %d\n", cmd->slave_index);
                    set_velocity(cmd->slave_index, cmd->velocity);
                    break;
                    
                case CMD_COMMAND_BACKWARD:
                    printf("[EtherCAT] Executing backward motion command for slave %d\n", cmd->slave_index);
                    set_velocity(cmd->slave_index, -cmd->velocity); // Negative for backward
                    break;
                    
                case CMD_STOP_VELOCITY_MOTION:
                    printf("[EtherCAT] Executing stop motion command for slave %d\n", cmd->slave_index);
                    set_velocity(cmd->slave_index, cmd->velocity);
                    break;
            }
            g_free(cmd);
        }  

        // Enhanced periodic state check
        static int state_check_counter = 0;
        if (++state_check_counter % 50 == 0) { // Check more frequently
            pthread_mutex_lock(&ethercat_mutex);
            ec_readstate();
            
            bool recovery_needed = false;
            for (int i = 1; i <= connected_slave_count; i++) {
                if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                    printf("Slave %d dropped out of OPERATIONAL state: 0x%04X\n", 
                           i, ec_slave[i].state);
                    recovery_needed = true;
                }
            }
            
            if (recovery_needed) {
                if (movement_monitoring_active) {
                    stop_movement_monitoring();
                }
                recover_slaves();
                printf("--------------------------------\n");
            }
            
            pthread_mutex_unlock(&ethercat_mutex);
            state_check_counter = 0;
        }
    
        // **NEW: High-frequency safety monitoring during movement**
          // **ONLY monitor if movement is active (button was clicked)**
        if (movement_monitoring_active) {
            safety_monitor_counter++;
            if (safety_monitor_counter >= 2) { // Check every 2 cycles (10ms)
                pthread_mutex_lock(&ethercat_mutex);
                monitor_movement_safety();
                pthread_mutex_unlock(&ethercat_mutex);
                safety_monitor_counter = 0;
            }
        } else {
            // Reset counter when not monitoring
            safety_monitor_counter = 0;
        }

        // **NEW: Log positions every 100 cycles (about 500ms at 5ms cycle time)**
        position_log_counter++;
        if (position_log_counter >= 100) { // Every 100 cycles (about 500ms at 5ms cycle time)
            for (int i = 0; i < connected_slave_count; i++) {
                read_current_position(i);
            }
            position_log_counter = 0;
        }

        usleep(5000); // 5 ms cycle

    }
    return NULL;
}
