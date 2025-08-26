#ifndef ETHERCAT_COMM_H
#define ETHERCAT_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include <glib.h>
#define NUM_SLAVES 18
#define POSITION_ACTUAL_VALUE_OFFSET 4
#define RAW_STEPS_PER_METER 206820498 //202985985
#define ACTUAL_STEPS_PER_METER 807936
// Structure for output process data (to slave)
typedef struct {
    uint16_t control_word;
    int32_t target_position;
    int32_t target_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint8_t mode_of_operation;
} el7_out_t;

// Structure for input process data (from slave)
typedef struct {
    uint16_t status_word;
    int32_t position_actual_value;
    uint8_t mode_of_operation_display;
} el7_in_t;

// Enum of commands for queue
typedef enum {
    CMD_ENABLE_ALL,
    CMD_DISABLE_ALL,
    CMD_RESET_FAULTS,
    CMD_CHECK_STATUS,
    // CMD_SET_VELOCITY_MODE, 
    CMD_COMMAND_FORWARD,  
    CMD_COMMAND_BACKWARD,   
    CMD_STOP_VELOCITY_MOTION,      
    // CMD_SINGLE_SLAVE_QUEUED_MOVEMENT,  
    // CMD_FLUSH_QUEUED_MOVEMENTS,    
    // CMD_STOP_POSITION_MOVEMENT,
    // Add other commands as needed
} CommandID;

typedef struct {
    CommandID id;
    int slave_index;
    int32_t velocity;     // For Velocity commands
    float target_position;  // For position commands
    int *slave_indices;     // Array of slave indices for multi-slave commands
    int slave_count;        // Number of slaves in the array
    int *base_params;       // Array for [speed, accel, decel] parameters
    char direction[16];
    gpointer callback_data; 
} Command;

// API functions
bool ethercat_comm_init(const char *iface);
void ethercat_comm_cleanup(void);
int  ethercat_get_slave_count(void);
void ethercat_trigger_slave_check(void);
gboolean update_app_slave_count_idle(gpointer data);

// Drive control functions (called from background thread)
void reset_fault(int slave_idx);
void enable_drive(int slave_idx);
void disable_drive(int slave_idx);
bool is_drive_enabled(int slave_idx);  // New: for CMD_CHECK_STATUS

// New: Mode setting and verification functions
void set_velocity_mode(int slave_idx);
void verify_mode_of_operation(int slave_idx);
gboolean on_velocity_mode_complete(gpointer data);


// Add function declarations
void move_single_slave(int slave_idx, float target_position_meters, int *base_params);
void flush_slave_commands(void);


// Global queue for commands (declared externally)
extern GAsyncQueue *button_command_queue;

#endif // ETHERCAT_COMM_H
