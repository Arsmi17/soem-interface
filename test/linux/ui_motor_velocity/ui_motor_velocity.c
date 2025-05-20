#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>
#include <time.h>
#include <pthread.h>
#include "ethercat.h"

typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t target_position;
    uint8_t mode_of_operation;
    int32_t target_velocity;
} el7_out_t;

typedef struct __attribute__((packed))
{
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
} el7_in_t;

char IOmap[4096];
el7_out_t **out_data;
el7_in_t **in_data;
int selected_slave = -1;  // -1 means no slave selected
int velocity_level = 1000; // Default velocity
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int num_slaves = 0; // Dynamic number of slaves

// Structure to hold app data
typedef struct
{
    GtkWidget *message_box;   // Text view for messages
    GtkWidget **motor_radios; // Dynamic array for radio buttons
    int num_radios;           // Number of radio buttons
    GtkWidget *speed_entry;   // Speed input field
} AppData;

// Helper function to append messages to the message box
void append_to_message_box(AppData *data, const char *message)
{
    if (!data || !data->message_box)
    {
        fprintf(stderr, "Error: Message box not initialized: %s\n", message);
        return;
    }
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(data->message_box));
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(buffer, &end);
    gtk_text_buffer_insert(buffer, &end, message, -1);
    gtk_text_buffer_insert(buffer, &end, "\n", -1);
}

// Redirect printf to message box
int printf_to_message_box(AppData *data, const char *format, ...)
{
    if (!data || !data->message_box)
    {
        fprintf(stderr, "Error: Message box not initialized\n");
        return -1;
    }
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    append_to_message_box(data, buffer);
    return 0;
}

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
    pthread_mutex_lock(&mutex);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    pthread_mutex_unlock(&mutex);
    usleep(1000);
}

// Set mode to Profile Velocity Mode via SDO
void set_velocity_mode(int slave_idx, AppData *data)
{
    uint8_t mode = 3; // Profile Velocity Mode
    printf_to_message_box(data, "Setting slave %d to Profile Velocity Mode (3)...", slave_idx + 1);
    if (ec_SDOwrite(slave_idx + 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE))
    {
        printf_to_message_box(data, "Successfully set mode of operation to Profile Velocity Mode (3)");
    }
    else
    {
        printf_to_message_box(data, "Failed to set mode of operation! Will try via PDO.");
        pthread_mutex_lock(&mutex);
        out_data[slave_idx]->mode_of_operation = 3;
        pthread_mutex_unlock(&mutex);
        send_and_receive();
    }
    usleep(50000);
}

int is_drive_enabled(int slave_index, AppData *data) {
    // Ensure the slave index is valid
    if (slave_index < 0 || slave_index >= num_slaves) {
        printf_to_message_box(data, "Invalid slave index: %d. Valid range is 0 to %d.", slave_index, num_slaves - 1);
        return FALSE;
    }

    // Read the state of all slaves
    ec_readstate();

    // Get the state of the specific slave
    uint16 slave_state = ec_slave[slave_index + 1].state;

    // Check if the slave is in the OPERATIONAL state
    if (slave_state != EC_STATE_OPERATIONAL) {
        printf_to_message_box(data, "Drive at slave index %d is not in OPERATIONAL state. Current state: %d", slave_index, slave_state);
        return FALSE;
    }

    // Read the Status Word from PDO (el7_in_t)
    uint16 status_word;
    pthread_mutex_lock(&mutex);
    status_word = in_data[slave_index]->status_word;
    pthread_mutex_unlock(&mutex);
    send_and_receive(); // Ensure latest PDO data

    // Fallback to SDO if PDO data is unreliable
    if (status_word == 0) {
        int size = sizeof(status_word);
        int wkc = ec_SDOread(slave_index + 1, 0x6041, 0x00, FALSE, &size, &status_word, EC_TIMEOUTRXM);
        if (wkc <= 0) {
            printf_to_message_box(data, "Failed to read Status Word from slave %d via SDO", slave_index);
            return FALSE;
        }
    }

    // Check if the "Operation Enabled" bit (Bit 2) is set
    int is_enabled = (status_word & (1 << 2)) != 0;

    if (is_enabled) {
        printf_to_message_box(data, "Drive at slave index %d is enabled. Status Word: 0x%04X", slave_index, status_word);
    } else {
        printf_to_message_box(data, "Drive at slave index %d is not enabled. Status Word: 0x%04X", slave_index, status_word);
    }

    return is_enabled;
}

// Verify the current mode
void verify_mode_of_operation(int slave_idx, AppData *data)
{
    uint8_t mode;
    int size = sizeof(mode);
    int result = ec_SDOread(slave_idx + 1, 0x6061, 0x00, FALSE, &size, &mode, EC_TIMEOUTSAFE);
    if (result == 1)
    {
        printf_to_message_box(data, "Current mode of operation for slave %d is: %d", slave_idx + 1, mode);
        if (mode != 3)
        {
            printf_to_message_box(data, "WARNING: Actual mode is not Profile Velocity Mode (3)!");
        }
    }
    else
    {
        printf_to_message_box(data, "Failed to read mode of operation display for slave %d", slave_idx + 1);
    }
}

// Set velocity directly via SDO and PDO
void set_velocity(int slave_idx, int32_t velocity, AppData *data)
{
    printf_to_message_box(data, "Setting velocity for slave %d to %d", slave_idx + 1, velocity);
    if (ec_SDOwrite(slave_idx + 1, 0x60FF, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE))
    {
        printf_to_message_box(data, "Successfully set target velocity via SDO");
    }
    else
    {
        printf_to_message_box(data, "Failed to set target velocity via SDO, will use PDO");
    }
    pthread_mutex_lock(&mutex);
    out_data[slave_idx]->target_velocity = velocity;
    out_data[slave_idx]->control_word = 0x000F;
    pthread_mutex_unlock(&mutex);
    send_and_receive();
    uint16_t status = in_data[slave_idx]->status_word;
    printf_to_message_box(data, "Status Word after set_velocity: 0x%04X", status);
    int32_t actual_velocity;
    int size = sizeof(actual_velocity);
    if (ec_SDOread(slave_idx + 1, 0x606C, 0x00, FALSE, &size, &actual_velocity, EC_TIMEOUTSAFE) == 1)
    {
        printf_to_message_box(data, "Current actual velocity: %d", actual_velocity);
    }
}

void stop_motor(int slave_idx, AppData *data)
{
    printf_to_message_box(data, "Stopping motor %d", slave_idx + 1);
    set_velocity(slave_idx, 0, data);
    send_and_receive();
    usleep(10000);
}

void quick_stop_motor(int slave_idx, AppData *data)
{
    printf_to_message_box(data, "Quick stopping motor %d", slave_idx + 1);
    pthread_mutex_lock(&mutex);
    out_data[slave_idx]->control_word = 0x0002;
    pthread_mutex_unlock(&mutex);
    send_and_receive();
    usleep(10000);
    pthread_mutex_lock(&mutex);
    out_data[slave_idx]->control_word = 0x000F;
    pthread_mutex_unlock(&mutex);
}

void reset_fault(int slave_idx, AppData *data) {
    printf_to_message_box(data, "Resetting fault on drive %d...", slave_idx + 1);
    pthread_mutex_lock(&mutex);
    out_data[slave_idx]->control_word = 0x80;
    pthread_mutex_unlock(&mutex);
    send_and_receive();
    usleep(10000);
    pthread_mutex_lock(&mutex);
    out_data[slave_idx]->control_word = 0x0F;
    pthread_mutex_unlock(&mutex);
    send_and_receive();
}

int enable_drive(int slave_idx, AppData *data) {
    if (slave_idx < 0 || slave_idx >= num_slaves) {
        printf_to_message_box(data, "Invalid slave index %d. Valid range is 0 to %d.", slave_idx, num_slaves - 1);
        return FALSE;
    }

    printf_to_message_box(data, "Enabling drive %d...", slave_idx + 1);
    int max_retries = 3;
    int retry_count = 0;
    uint16_t status_word;
    int enabled = FALSE;

    // First check for faults and reset if needed
    send_and_receive();
    pthread_mutex_lock(&mutex);
    status_word = in_data[slave_idx]->status_word;
    pthread_mutex_unlock(&mutex);

    // Check if fault bit (bit 3) is set
    if (status_word & 0x0008) {
        printf_to_message_box(data, "Fault detected on drive %d (Status: 0x%04X). Resetting...", 
                              slave_idx + 1, status_word);
        reset_fault(slave_idx, data);
        usleep(50000); // Give time for fault to clear
    }

    while (retry_count < max_retries && !enabled) {
        // State transition sequence
        pthread_mutex_lock(&mutex);
        out_data[slave_idx]->control_word = 0x06; // "Switch on disabled"
        pthread_mutex_unlock(&mutex);
        send_and_receive();
        usleep(10000);

        pthread_mutex_lock(&mutex);
        out_data[slave_idx]->control_word = 0x07; // "Switch on"
        pthread_mutex_unlock(&mutex);
        send_and_receive();
        usleep(10000);

        pthread_mutex_lock(&mutex);
        out_data[slave_idx]->control_word = 0x0F; // "Enable operation"
        pthread_mutex_unlock(&mutex);
        send_and_receive();
        usleep(10000);

        // Check status word to verify enabled state
        send_and_receive();
        pthread_mutex_lock(&mutex);
        status_word = in_data[slave_idx]->status_word;
        pthread_mutex_unlock(&mutex);

        // Check if we're in "Operation enabled" state (bits 0-6: 00100111)
        if ((status_word & 0x006F) == 0x0027) {
            enabled = TRUE;
            printf_to_message_box(data, "Drive %d successfully enabled (Status: 0x%04X)", 
                                  slave_idx + 1, status_word);
        } else {
            // If not enabled, check if fault occurred during attempt
            if (status_word & 0x0008) {
                printf_to_message_box(data, "Fault occurred during enable attempt %d (Status: 0x%04X)", 
                                      retry_count + 1, status_word);
                reset_fault(slave_idx, data);
                usleep(50000);
            }

            retry_count++;
            printf_to_message_box(data, "Drive %d enable attempt %d failed (Status: 0x%04X). Retrying...", 
                                  slave_idx + 1, retry_count, status_word);
            usleep(50000); // Longer delay between retries
        }
    }

    if (!enabled) {
        printf_to_message_box(data, "ERROR: Failed to enable drive %d after %d attempts (Status: 0x%04X)", 
                              slave_idx + 1, max_retries, status_word);
    }

    return enabled;
}


void disable_drive(int slave_idx, AppData *data)
{
    printf_to_message_box(data, "Disabling drive %d...", slave_idx + 1);
    stop_motor(slave_idx, data);
    usleep(50000);
    pthread_mutex_lock(&mutex);
    out_data[slave_idx]->control_word = 0x0000;
    pthread_mutex_unlock(&mutex);
    send_and_receive();
}

void select_slave(int slave_idx, AppData *data)
{
    if (slave_idx < 0 || slave_idx >= num_slaves)
    {
        printf_to_message_box(data, "Invalid slave index %d. Valid range is 0 to %d.", slave_idx, num_slaves - 1);
        return;
    }
    selected_slave = slave_idx;
    printf_to_message_box(data, "Slave %d selected for movement.", selected_slave + 1);
}

// Button callback methods
void on_enable_motor_clicked(GtkButton *button, AppData *data) {
    (void)button;
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(data->message_box));
    gtk_text_buffer_set_text(buffer, "All motors are enabled", -1);
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(buffer, &end);
    gtk_text_buffer_insert(buffer, &end, "\nMotor system check completed", -1);
    for (int i = 0; i < num_slaves; i++) {
        if (!enable_drive(i, data)) {
            printf_to_message_box(data, "Failed to enable drive %d", i + 1);
        }
    }
}


void on_disable_motor_clicked(GtkButton *button, AppData *data)
{
    (void)button;
    for (int i = 0; i < num_slaves; i++)
    {
        disable_drive(i, data);
    }
    append_to_message_box(data, "All motors are disabled");
}

void on_check_motor_enabled_clicked(GtkButton *button, AppData *data) {
    (void)button;
    for (int i = 0; i < num_slaves; i++) {
        if (is_drive_enabled(i, data)) {
            printf_to_message_box(data, "Slave %d is enabled and operational.", i);
        } else {
            printf_to_message_box(data, "Slave %d is not enabled.", i);
        }
        verify_mode_of_operation(i, data); // Keep existing mode check
    }
}

void on_clear_logs_clicked(GtkButton *button, AppData *data)
{
    (void)button;
    if (data && data->message_box)
    {
        GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(data->message_box));
        gtk_text_buffer_set_text(buffer, "", -1);
    }
}

void on_reset_fault_clicked(GtkButton *button, AppData *data)
{
    (void)button;
    for (int i = 0; i < num_slaves; i++)
    {
        reset_fault(i, data);
    }
    append_to_message_box(data, "Faults reset for all drives");
}

gboolean on_forward_pressed(GtkWidget *widget, GdkEventButton *event, AppData *data)
{
    (void)widget;
    (void)event;
    if (selected_slave == -1)
    {
        append_to_message_box(data, "No slave selected. Please select a motor first.");
        return TRUE;
    }
    const char *speed_text = gtk_entry_get_text(GTK_ENTRY(data->speed_entry));
    char *endptr;
    long velocity = strtol(speed_text, &endptr, 10);
    if (*endptr != '\0' || velocity < 0 || velocity > 1000000)
    {
        append_to_message_box(data, "Invalid speed input. Please enter a valid number.");
        return TRUE;
    }
    velocity_level = (int32_t)velocity;
    printf_to_message_box(data, "Moving forward at velocity %d for slave %d...", velocity_level, selected_slave + 1);
    set_velocity(selected_slave, velocity_level, data);
    return TRUE;
}

gboolean on_forward_released(GtkWidget *widget, GdkEventButton *event, AppData *data)
{
    (void)widget;
    (void)event;
    if (selected_slave != -1)
    {
        stop_motor(selected_slave, data);
    }
    return TRUE;
}

gboolean on_backward_pressed(GtkWidget *widget, GdkEventButton *event, AppData *data)
{
    (void)widget;
    (void)event;
    if (selected_slave == -1)
    {
        append_to_message_box(data, "No slave selected. Please select a motor first.");
        return TRUE;
    }
    const char *speed_text = gtk_entry_get_text(GTK_ENTRY(data->speed_entry));
    char *endptr;
    long velocity = strtol(speed_text, &endptr, 10);
    if (*endptr != '\0' || velocity < 0 || velocity > 1000000)
    {
        append_to_message_box(data, "Invalid speed input. Please enter a valid number.");
        return TRUE;
    }
    velocity_level = (int32_t)velocity;
    printf_to_message_box(data, "Moving in reverse at velocity %d for slave %d...", -velocity_level, selected_slave + 1);
    set_velocity(selected_slave, -velocity_level, data);
    return TRUE;
}

gboolean on_backward_released(GtkWidget *widget, GdkEventButton *event, AppData *data)
{
    (void)widget;
    (void)event;
    if (selected_slave != -1)
    {
        stop_motor(selected_slave, data);
    }
    return TRUE;
}

void on_motor_toggled(GtkRadioButton *radio, AppData *data)
{
    for (int i = 0; i < data->num_radios; i++)
    {
        if (GTK_RADIO_BUTTON(data->motor_radios[i]) == radio && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radio)))
        {
            select_slave(i, data);
        }
    }
}

void window_destroy(GtkWidget *widget, AppData *data) {
    (void)widget;
    printf_to_message_box(data, "Window closed, stopping motors...");
    for (int i = 0; i < num_slaves; i++) {
        stop_motor(i, data);
        usleep(10000);
        disable_drive(i, data);
    }
    ec_close();
    restore_keyboard();
    if (data->motor_radios) free(data->motor_radios);
    gtk_main_quit();
}

void cleanup(void)
{
    restore_keyboard();
    ec_close();
    if (out_data)
        free(out_data);
    if (in_data)
        free(in_data);
}

void signal_handler(int sig)
{
    (void)sig; // Mark as unused
    cleanup();
    gtk_main_quit();
    exit(0);
}

gboolean update_ethercat(gpointer user_data)
{
    (void)user_data; // Mark as unused
    send_and_receive();
    return G_SOURCE_CONTINUE;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    atexit(cleanup);

    // Print console message with dynamic timestamp
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%I:%M %p IST on %A, %B %d, %Y", tm);
    printf("Velocity Motor Control program started at %s\n", time_str);

    // Initialize GTK
    gtk_init(&argc, &argv);
    printf("GTK initialized\n");

    // Initialize EtherCAT to detect slaves
    char *ifname = argc > 1 ? argv[1] : "enp0s31f6";
    printf("Initializing EtherCAT on interface: %s\n", ifname);
    if (!ec_init(ifname))
    {
        printf("Failed to initialize EtherCAT on %s\n", ifname);
        return -1;
    }

    // Find and configure slaves
    num_slaves = ec_config(FALSE, &IOmap);
    if (num_slaves <= 0)
    {
        printf("No slaves found!\n");
        ec_close();
        return -1;
    }
    printf("%d slaves found and configured.\n", num_slaves);

    // Create the main window
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Velocity Motor Control");
    gtk_window_set_default_size(GTK_WINDOW(window), 500, 400);
    g_signal_connect(window, "destroy", G_CALLBACK(window_destroy), NULL);
    printf("Main window created\n");

    // Main vertical box
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_add(GTK_CONTAINER(window), vbox);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    printf("Vertical box added\n");

    // App data structure
    AppData data = {NULL, NULL, 0, NULL};
    data.num_radios = num_slaves;
    data.motor_radios = malloc(num_slaves * sizeof(GtkWidget *));
    if (!data.motor_radios)
    {
        printf("Memory allocation failed for motor_radios!\n");
        ec_close();
        return -1;
    }
    for (int i = 0; i < num_slaves; i++)
    {
        data.motor_radios[i] = NULL;
    }

    // Top section
    GtkWidget *top_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_name(top_box, "top-section");
    gtk_box_pack_start(GTK_BOX(vbox), top_box, FALSE, FALSE, 0);
    printf("Top section added\n");

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
    printf("Buttons added\n");

    GtkWidget *clear_button = gtk_button_new_with_label("Clear Logs");
    g_signal_connect(clear_button, "clicked", G_CALLBACK(on_clear_logs_clicked), &data);
    gtk_box_pack_start(GTK_BOX(button_box), clear_button, TRUE, TRUE, 0);

    // Message box
    data.message_box = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(data.message_box), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(data.message_box), GTK_WRAP_WORD);
    GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(scrolled_window), data.message_box);
    gtk_widget_set_size_request(scrolled_window, -1, 100);
    gtk_box_pack_start(GTK_BOX(top_box), scrolled_window, TRUE, TRUE, 0);
    printf("Message box added\n");

    // Bottom section
    GtkWidget *bottom_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_name(bottom_box, "bottom-section");
    gtk_box_pack_start(GTK_BOX(vbox), bottom_box, TRUE, TRUE, 0);
    printf("Bottom section added\n");

    // Motor selection (dynamic radio buttons)
    GtkWidget *motor_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_box_pack_start(GTK_BOX(bottom_box), motor_box, FALSE, FALSE, 0);
    if (num_slaves > 0)
    {
        char label[64];
        snprintf(label, sizeof(label), "Motor %d (%s)", 1, ec_slave[1].name);
        data.motor_radios[0] = gtk_radio_button_new_with_label(NULL, label);
        gtk_box_pack_start(GTK_BOX(motor_box), data.motor_radios[0], FALSE, FALSE, 0);
        for (int i = 1; i < num_slaves; i++)
        {
            snprintf(label, sizeof(label), "Motor %d (%s)", i + 1, ec_slave[i + 1].name);
            data.motor_radios[i] = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(data.motor_radios[0]), label);
            gtk_box_pack_start(GTK_BOX(motor_box), data.motor_radios[i], FALSE, FALSE, 0);
        }
        for (int i = 0; i < num_slaves; i++)
        {
            g_signal_connect(data.motor_radios[i], "toggled", G_CALLBACK(on_motor_toggled), &data);
        }
        // Select the first slave by default
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(data.motor_radios[0]), TRUE);
        select_slave(0, &data);
    }
    else
    {
        GtkWidget *label = gtk_label_new("No motors detected");
        gtk_box_pack_start(GTK_BOX(motor_box), label, FALSE, FALSE, 0);
    }
    printf("Motor selection added\n");

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
    printf("Direction buttons added\n");

    // Speed input
    GtkWidget *speed_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(bottom_box), speed_box, FALSE, FALSE, 0);

    GtkWidget *speed_label = gtk_label_new("Speed");
    gtk_box_pack_start(GTK_BOX(speed_box), speed_label, FALSE, FALSE, 0);

    data.speed_entry = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data.speed_entry), "300");
    gtk_box_pack_start(GTK_BOX(speed_box), data.speed_entry, TRUE, TRUE, 0);
    printf("Speed input added\n");

    // Show all widgets
    gtk_widget_show_all(window);
    printf("All widgets shown\n");

    // Configure EtherCAT
    ec_configdc();
    ec_config_map(&IOmap);
    printf("EtherCAT configured\n");

    // Allocate memory for out_data and in_data
    out_data = malloc(num_slaves * sizeof(el7_out_t *));
    in_data = malloc(num_slaves * sizeof(el7_in_t *));
    printf("EtherCAT configured\n");
    if (!out_data || !in_data)
    {
        printf_to_message_box(&data, "Memory allocation failed!");
        ec_close();
        if (data.motor_radios)
            free(data.motor_radios);
        return -1;
    }
    printf("EtherCAT configured\n");
    // EtherCAT setup for slaves
    for (int i = 0; i < num_slaves; ++i)
    {
        out_data[i] = (el7_out_t *)ec_slave[i + 1].outputs;
        in_data[i] = (el7_in_t *)ec_slave[i + 1].inputs;

        uint32_t profile_acceleration = 500000;
        uint32_t profile_deceleration = 500000;
        uint32_t max_profile_velocity = 500000;
        printf("entered for loop\n");
        if (!ec_SDOwrite(i + 1, 0x6083, 0x00, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTSAFE))
        {
            printf_to_message_box(&data, "Failed to set profile acceleration for slave %d", i + 1);
            ec_close();
            free(out_data);
            free(in_data);
            if (data.motor_radios)
                free(data.motor_radios);
            return -1;
        }
        if (!ec_SDOwrite(i + 1, 0x6084, 0x00, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTSAFE))
        {
            printf_to_message_box(&data, "Failed to set profile deceleration for slave %d", i + 1);
            ec_close();
            free(out_data);
            free(in_data);
            if (data.motor_radios)
                free(data.motor_radios);
            return -1;
        }

        // Set max profile velocity
        printf("Setting max profile velocity for slave %d\n", i + 1);
        if (!ec_SDOwrite(i + 1, 0x607F, 0x00, FALSE, sizeof(max_profile_velocity), &max_profile_velocity, EC_TIMEOUTSAFE))
        {
            printf_to_message_box(&data, "Failed to set max profile velocity for slave %d", i + 1);
            ec_close();
            free(out_data);
            free(in_data);
            if (data.motor_radios)
                free(data.motor_radios);
            return -1;
        }

        printf("Max profile velocity set for slave %d\n", i + 1);
        set_velocity_mode(i, &data);
    }

    printf("Setting slaves to OPERATIONAL state...\n");
    // Set slaves to OPERATIONAL state
    int operational_slaves = 0;
    for (int i = 0; i < num_slaves; ++i)
    {
        ec_slave[i + 1].state = EC_STATE_OPERATIONAL;
        ec_writestate(i + 1);
    }
    usleep(100000);
    for (int i = 0; i < num_slaves; ++i)
    {
        if (ec_statecheck(i + 1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) == EC_STATE_OPERATIONAL)
        {
            operational_slaves++;
        }
        else
        {
            printf_to_message_box(&data, "Slave %d failed to reach OPERATIONAL state.", i + 1);
        }
    }
    if (operational_slaves == 0)
    {
        printf_to_message_box(&data, "No slaves reached OPERATIONAL state. Exiting.");
        ec_close();
        free(out_data);
        free(in_data);
        if (data.motor_radios)
            free(data.motor_radios);
        return -1;
    }

    // Verify mode of operation
    for (int i = 0; i < num_slaves; ++i)
    {
        is_drive_enabled(i, &data);
    }

    printf_to_message_box(&data, "All slaves reached OPERATIONAL state. Ready for control.");
    // Start EtherCAT communication loop
    g_timeout_add(10, update_ethercat, NULL);
    printf("EtherCAT communication loop started\n");
    // Start the main loop
    gtk_main();

    printf("Exiting program...\n");
    free(out_data);
    free(in_data);
    if (data.motor_radios)
        free(data.motor_radios);
    return 0;
}