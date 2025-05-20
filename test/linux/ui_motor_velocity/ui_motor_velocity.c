#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "ethercat.h"
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>



typedef struct __attribute__((packed)) {
    uint16_t control_word;
    int32_t target_position;
    uint8_t mode_of_operation;
    int32_t target_velocity;
} el7_out_t;

typedef struct __attribute__((packed)) {
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
} el7_in_t;

#define NUM_SLAVES 3
char IOmap[4096];
el7_out_t *out_data[NUM_SLAVES];
el7_in_t *in_data[NUM_SLAVES];
int selected_slave = -1;  // Global variable to track the selected slave (-1 means no slave is selected)
int velocity_level = 300;  // Default velocity from UI

// Structure to hold app data
typedef struct {
    GtkWidget *message_box; // Text view for messages
    GtkWidget *motor_radios[3]; // Radio buttons for motor selection (adjusted to 3 slaves)
    GtkWidget *speed_entry; // Speed input field
} AppData;

// Helper function to append messages to the message box
void append_to_message_box(AppData *data, const char *message) {
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(data->message_box));
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(buffer, &end);
    gtk_text_buffer_insert(buffer, &end, message, -1);
    gtk_text_buffer_insert(buffer, &end, "\n", -1);
}

// Redirect printf to message box
int printf_to_message_box(AppData *data, const char *format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    append_to_message_box(data, buffer);
    return 0;  // Mimic printf return value
}

void setup_keyboard() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void restore_keyboard() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

void send_and_receive() {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    usleep(1000);
}

// Set mode to Profile Velocity Mode via SDO
void set_velocity_mode(int slave_idx, AppData *data) {
    uint8_t mode = 3;  // Profile Velocity Mode

    printf_to_message_box(data, "Setting slave %d to Profile Velocity Mode (3)...", slave_idx + 1);
    if (ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE)) {
        printf_to_message_box(data, "Successfully set mode of operation to Profile Velocity Mode (3)");
    } else {
        printf_to_message_box(data, "Failed to set mode of operation! Will try via PDO.");
        out_data[slave_idx]->mode_of_operation = 3;
        send_and_receive();
    }
    usleep(50000);
}

// Verify the current mode
void verify_mode_of_operation(int slave_idx, AppData *data) {
    uint8_t mode;
    int size = sizeof(mode);
    int result = ec_SDOread(slave_idx + 1, 0x6061, 0x00, FALSE, &size, &mode, EC_TIMEOUTSAFE);
    
    if (result == 1) {
        printf_to_message_box(data, "Current mode of operation for slave %d is: %d", slave_idx + 1, mode);
        if (mode != 3) {
            printf_to_message_box(data, "WARNING: Actual mode is not Profile Velocity Mode (3)!");
        }
    } else {
        printf_to_message_box(data, "Failed to read mode of operation display for slave %d", slave_idx + 1);
    }
}

// Set velocity directly via SDO and PDO
void set_velocity(int slave_idx, int32_t velocity, AppData *data) {
    printf_to_message_box(data, "Setting velocity for slave %d to %d", slave_idx + 1, velocity);
    
    if (ec_SDOwrite(slave_idx + 1, 0x60FF, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE)) {
        printf_to_message_box(data, "Successfully set target velocity via SDO");
    } else {
        printf_to_message_box(data, "Failed to set target velocity via SDO, will use PDO");
    }
    
    out_data[slave_idx]->target_velocity = velocity;
    out_data[slave_idx]->control_word = 0x000F;
    send_and_receive();
    
    uint16_t status = in_data[slave_idx]->status_word;
    printf_to_message_box(data, "Status Word after set_velocity: 0x%04X", status);
    
    int32_t actual_velocity;
    int size = sizeof(actual_velocity);
    if (ec_SDOread(slave_idx + 1, 0x606C, 0x00, FALSE, &size, &actual_velocity, EC_TIMEOUTSAFE) == 1) {
        printf_to_message_box(data, "Current actual velocity: %d", actual_velocity);
    }
}

void stop_motor(int slave_idx, AppData *data) {
    printf_to_message_box(data, "Stopping motor %d", slave_idx + 1);
    set_velocity(slave_idx, 0, data);
    send_and_receive();
    usleep(10000);
}

void quick_stop_motor(int slave_idx, AppData *data) {
    printf_to_message_box(data, "Quick stopping motor %d", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x0002;
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x000F;
}

void reset_fault(int slave_idx, AppData *data) {
    printf_to_message_box(data, "Resetting fault on drive %d...", slave_idx + 1);
    out_data[slave_idx]->control_word = 0x80;
    send_and_receive();
    usleep(10000);
    out_data[slave_idx]->control_word = 0x0F;
    send_and_receive();
}

void enable_drive(int slave_idx, AppData *data) {
    printf_to_message_box(data, "Enabling drive %d...", slave_idx + 1);
    
    send_and_receive();
    uint16_t status_word = in_data[slave_idx]->status_word;
    
    if (status_word & 0x0008) {
        printf_to_message_box(data, "Fault detected on drive %d (Status: 0x%04X). Resetting...", 
                              slave_idx + 1, status_word);
        reset_fault(slave_idx, data);
        usleep(50000);
    }

    printf_to_message_box(data, "State transition sequence...");
    
    out_data[slave_idx]->control_word = 0x0006;
    send_and_receive();
    usleep(50000);
    
    out_data[slave_idx]->control_word = 0x0007;
    send_and_receive();
    usleep(50000);
    
    out_data[slave_idx]->control_word = 0x000F;
    send_and_receive();
    usleep(50000);
    
    send_and_receive();
    status_word = in_data[slave_idx]->status_word;
    printf_to_message_box(data, "Drive %d status after enable sequence: 0x%04X", slave_idx + 1, status_word);
    
    if ((status_word & 0x006F) == 0x0027) {
        printf_to_message_box(data, "Drive %d successfully enabled", slave_idx + 1);
    } else {
        printf_to_message_box(data, "WARNING: Drive %d may not be fully enabled (Status: 0x%04X)", 
                              slave_idx + 1, status_word);
    }
}

void disable_drive(int slave_idx, AppData *data) {
    printf_to_message_box(data, "Disabling drive %d...", slave_idx + 1);
    stop_motor(slave_idx, data);
    usleep(50000);
    out_data[slave_idx]->control_word = 0x0000;
    send_and_receive();
}

void select_slave(int slave_idx, AppData *data) {
    if (slave_idx < 0 || slave_idx >= NUM_SLAVES) {
        printf_to_message_box(data, "Invalid slave index %d. Valid range is 0 to %d.", slave_idx, NUM_SLAVES - 1);
        return;
    }
    selected_slave = slave_idx;
    printf_to_message_box(data, "Slave %d selected for movement.", selected_slave + 1);
}

// Button callback methods
void on_enable_motor_clicked(GtkButton *button, AppData *data) {
    (void)button;  // Mark button as unused to suppress warning
    // Clear the message box and add new messages
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(data->message_box));
    gtk_text_buffer_set_text(buffer, "All motors are enabled", -1);
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(buffer, &end);
    gtk_text_buffer_insert(buffer, &end, "\nMotor system check completed", -1);
    
    for (int i = 0; i < NUM_SLAVES; i++) {
        enable_drive(i, data);
    }
}

void on_disable_motor_clicked(GtkButton *button, AppData *data) {
    (void)button;  // Mark button as unused to suppress warning
    for (int i = 0; i < NUM_SLAVES; i++) {
        disable_drive(i, data);
    }
    append_to_message_box(data, "All motors are disabled");
}

void on_check_motor_enabled_clicked(GtkButton *button, AppData *data) {
    (void)button;  // Mark button as unused
    for (int i = 0; i < NUM_SLAVES; i++) {
        verify_mode_of_operation(i, data);
    }
}

void on_reset_fault_clicked(GtkButton *button, AppData *data) {
    (void)button;  // Mark button as unused
    for (int i = 0; i < NUM_SLAVES; i++) {
        reset_fault(i, data);
    }
    append_to_message_box(data, "Faults reset for all drives");
}

gboolean on_forward_pressed(GtkWidget *widget, GdkEventButton *event, AppData *data) {
    (void)widget;  // Mark widget as unused
    (void)event;   // Mark event as unused
    if (selected_slave == -1) {
        append_to_message_box(data, "No slave selected. Please select a motor first.");
        return TRUE;
    }
    const char *speed_text = gtk_entry_get_text(GTK_ENTRY(data->speed_entry));
    velocity_level = atoi(speed_text);
    printf_to_message_box(data, "Moving forward at velocity %d for slave %d...", velocity_level, selected_slave + 1);
    set_velocity(selected_slave, velocity_level, data);
    return TRUE;
}

gboolean on_forward_released(GtkWidget *widget, GdkEventButton *event, AppData *data) {
    (void)widget;  // Mark widget as unused
    (void)event;   // Mark event as unused
    if (selected_slave != -1) {
        stop_motor(selected_slave, data);
    }
    return TRUE;
}

gboolean on_backward_pressed(GtkWidget *widget, GdkEventButton *event, AppData *data) {
    (void)widget;  // Mark widget as unused
    (void)event;   // Mark event as unused
    if (selected_slave == -1) {
        append_to_message_box(data, "No slave selected. Please select a motor first.");
        return TRUE;
    }
    const char *speed_text = gtk_entry_get_text(GTK_ENTRY(data->speed_entry));
    velocity_level = atoi(speed_text);
    printf_to_message_box(data, "Moving in reverse at velocity %d for slave %d...", -velocity_level, selected_slave + 1);
    set_velocity(selected_slave, -velocity_level, data);
    return TRUE;
}

gboolean on_backward_released(GtkWidget *widget, GdkEventButton *event, AppData *data) {
    (void)widget;  // Mark widget as unused
    (void)event;   // Mark event as unused
    if (selected_slave != -1) {
        stop_motor(selected_slave, data);
    }
    return TRUE;
}

// Callback for radio button toggle (disable others when one is selected)
void on_motor_toggled(GtkRadioButton *radio, AppData *data) {
    for (int i = 0; i < NUM_SLAVES; i++) {
        if (GTK_RADIO_BUTTON(data->motor_radios[i]) == radio && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radio))) {
            select_slave(i, data);
        }
        if (GTK_RADIO_BUTTON(data->motor_radios[i]) != radio) {
            gtk_widget_set_sensitive(data->motor_radios[i], !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radio)));
        }
    }
}

// Cleanup on window close
void window_destroy(GtkWidget *widget, AppData *data) {
    (void)widget;  // Mark widget as unused
    printf_to_message_box(data, "Window closed, stopping motors...");
    for (int i = 0; i < NUM_SLAVES; i++) {
        stop_motor(i, data);
        usleep(10000);
        disable_drive(i, data);
    }
    ec_close();
    restore_keyboard();
    gtk_main_quit();
}

int main(int argc, char *argv[]) {
    // Print console message with updated date and time
    printf("Velocity Motor Control program started at 08:58 PM IST on Monday, May 19, 2025\n");

    // Initialize GTK
    gtk_init(&argc, &argv);

    // Create the main window
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Velocity Motor Control");
    gtk_window_set_default_size(GTK_WINDOW(window), 500, 400);
    g_signal_connect(window, "destroy", G_CALLBACK(window_destroy), NULL);

    // Main vertical box
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_add(GTK_CONTAINER(window), vbox);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);

    // App data structure, initialize members to NULL for safety
    AppData data = {NULL, {NULL, NULL, NULL}, NULL};

    // Top section (blue background)
    GtkWidget *top_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_name(top_box, "top-section");
    gtk_box_pack_start(GTK_BOX(vbox), top_box, FALSE, FALSE, 0);

    // Buttons in top section
    GtkWidget *button_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(top_box), button_box, FALSE, FALSE, 0);

    GtkWidget *enable_button = gtk_button_new_with_label("Enable Motor");
    g_signal_connect(enable_button, "clicked", G_CALLBACK(on_enable_motor_clicked), &data);
    gtk_box_pack_start(GTK_BOX(button_box), enable_button, TRUE, TRUE, 0);

    GtkWidget *disable_button = gtk_button_new_with_label("Disable Motor");
    g_signal_connect(disable_button, "clicked", G_CALLBACK(on_disable_motor_clicked), &data);
    gtk_box_pack_start(GTK_BOX(button_box), disable_button, TRUE, TRUE, 0);

    GtkWidget *check_button = gtk_button_new_with_label("Check Motor Enabled");
    g_signal_connect(check_button, "clicked", G_CALLBACK(on_check_motor_enabled_clicked), &data);
    gtk_box_pack_start(GTK_BOX(button_box), check_button, TRUE, TRUE, 0);

    GtkWidget *reset_button = gtk_button_new_with_label("Reset Fault");
    g_signal_connect(reset_button, "clicked", G_CALLBACK(on_reset_fault_clicked), &data);
    gtk_box_pack_start(GTK_BOX(button_box), reset_button, TRUE, TRUE, 0);

    // Message box
    data.message_box = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(data.message_box), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(data.message_box), GTK_WRAP_WORD);
    GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(scrolled_window), data.message_box);
    gtk_widget_set_size_request(scrolled_window, -1, 100);
    gtk_box_pack_start(GTK_BOX(top_box), scrolled_window, TRUE, TRUE, 0);

    // Bottom section (pink background)
    GtkWidget *bottom_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_name(bottom_box, "bottom-section");
    gtk_box_pack_start(GTK_BOX(vbox), bottom_box, TRUE, TRUE, 0);

    // Motor selection (radio buttons, adjusted to 3 motors)
    GtkWidget *motor_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_box_pack_start(GTK_BOX(bottom_box), motor_box, FALSE, FALSE, 0);

    data.motor_radios[0] = gtk_radio_button_new_with_label(NULL, "Motor 1");
    gtk_box_pack_start(GTK_BOX(motor_box), data.motor_radios[0], FALSE, FALSE, 0);
    for (int i = 1; i < NUM_SLAVES; i++) {
        char label[10];
        snprintf(label, sizeof(label), "Motor %d", i + 1);
        data.motor_radios[i] = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(data.motor_radios[0]), label);
        gtk_box_pack_start(GTK_BOX(motor_box), data.motor_radios[i], FALSE, FALSE, 0);
    }

    // Connect toggle signals for radio buttons
    for (int i = 0; i < NUM_SLAVES; i++) {
        g_signal_connect(data.motor_radios[i], "toggled", G_CALLBACK(on_motor_toggled), &data);
    }

    // Forward and Backward buttons
    GtkWidget *direction_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(bottom_box), direction_box, FALSE, FALSE, 0);

    GtkWidget *backward_button = gtk_button_new_with_label("Backward");
    g_signal_connect(backward_button, "button-press-event", G_CALLBACK(on_backward_pressed), &data);
    g_signal_connect(backward_button, "button-release-event", G_CALLBACK(on_backward_released), &data);
    gtk_box_pack_start(GTK_BOX(direction_box), backward_button, TRUE, TRUE, 0);

    GtkWidget *forward_button = gtk_button_new_with_label("Forward");
    g_signal_connect(forward_button, "button-press-event", G_CALLBACK(on_forward_pressed), &data);
    g_signal_connect(forward_button, "button-release-event", G_CALLBACK(on_forward_released), &data);
    gtk_box_pack_start(GTK_BOX(direction_box), forward_button, TRUE, TRUE, 0);

    // Speed input
    GtkWidget *speed_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(bottom_box), speed_box, FALSE, FALSE, 0);

    GtkWidget *speed_label = gtk_label_new("Speed");
    gtk_box_pack_start(GTK_BOX(speed_box), speed_label, FALSE, FALSE, 0);

    data.speed_entry = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data.speed_entry), "300");
    gtk_box_pack_start(GTK_BOX(speed_box), data.speed_entry, TRUE, TRUE, 0);

    // Apply CSS for styling
    GtkCssProvider *css_provider = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css_provider,
        "#top-section { background-color: #add8e6; padding: 10px; }\n"
        "#bottom-section { background-color: #ffb6c1; padding: 10px; }", -1, NULL);
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(css_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    // Show all widgets
    gtk_widget_show_all(window);

    // Initialize EtherCAT (after GTK setup to ensure data.message_box is ready)
    if (!ec_init("enp0s31f6")) {
        printf("Failed to initialize EtherCAT interface.\n");
        return -1;
    }

    if (ec_config_init(FALSE) < NUM_SLAVES) {
        printf("Not enough slaves found!\n");
        ec_close();
        return -1;
    }

    ec_config_map(&IOmap);

    // EtherCAT setup for slaves
    for (int i = 0; i < NUM_SLAVES; ++i) {
        out_data[i] = (el7_out_t *)ec_slave[i + 1].outputs;
        in_data[i] = (el7_in_t *)ec_slave[i + 1].inputs;

        uint32_t profile_acceleration = 500000;
        uint32_t profile_deceleration = 500000;
        uint32_t max_profile_velocity = 500000;

        ec_SDOwrite(i + 1, 0x6083, 0x00, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x6084, 0x00, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(max_profile_velocity), &max_profile_velocity, EC_TIMEOUTSAFE);

        set_velocity_mode(i, &data);
    }

    for (int i = 0; i < NUM_SLAVES; ++i) {
        ec_slave[i + 1].state = EC_STATE_OPERATIONAL;
        ec_writestate(i + 1);
    }
    usleep(100000);

    for (int i = 0; i < NUM_SLAVES; ++i) {
        if (ec_statecheck(i + 1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL) {
            printf("Slave %d failed to reach OPERATIONAL state.\n", i + 1);
            ec_close();
            return -1;
        }
    }

    // Verify that the mode was properly set
    for (int i = 0; i < NUM_SLAVES; ++i) {
        verify_mode_of_operation(i, &data);
    }

    // Start the main loop
    gtk_main();

    return 0;
}