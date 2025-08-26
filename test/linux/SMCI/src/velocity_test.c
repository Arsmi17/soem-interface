#include <gtk/gtk.h>
#include "app_context.h"
#include "slave_selector.h"
#include "ethercat_comm.h"
#include <windows.h>
#include <shlwapi.h> 

static GtkWidget *vel_win = NULL;
int velocity = 7000;
int increment = 100;
// Global state
static GtkWidget *last_selected_velocity_slave_btn = NULL;
static GtkWidget *motion_forward_btn = NULL;
static GtkWidget *motion_backward_btn = NULL;
bool is_motion_active = false;
char current_motion_direction[16] = "";  // "forward" or "backward"
static int selected_slave_index = -1;

extern void on_slave_button_clicked_velocity(GtkButton *btn, gpointer user_data);

static void on_back_clicked(GtkButton *btn, gpointer user_data) {
    (void)user_data;
    (void)btn;
    gtk_widget_hide(vel_win);
    gtk_widget_show_all(app.main_window);
    if (app.log_scroll) {
        gtk_widget_hide(app.log_scroll);
    }
}

void on_slave_button_clicked_velocity(GtkButton *button, gpointer user_data) {
    (void)user_data;

    if (is_motion_active) {
        const gchar *current_slave = gtk_button_get_label(GTK_BUTTON(last_selected_velocity_slave_btn));
        char msg[128];
        snprintf(msg, sizeof(msg), "%s movement [%s] is in progress. Stop it before selecting a new slave.", current_slave, current_motion_direction);
        LOG_WARN(msg);
        return;
    }

    if (last_selected_velocity_slave_btn == GTK_WIDGET(button)) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
        gtk_style_context_remove_class(ctx, "slave-selected");
        last_selected_velocity_slave_btn = NULL;
        selected_slave_index = -1;
        LOG_WARN("Slave deselected.");
        return;
    }

    for (int i = 0; i < app.total_slaves; i++) {
        if (slave_buttons[i]) {
            GtkStyleContext *ctx = gtk_widget_get_style_context(slave_buttons[i]);
            gtk_style_context_remove_class(ctx, "slave-selected");
        }
    }

    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    gtk_style_context_add_class(ctx, "slave-selected");
    last_selected_velocity_slave_btn = GTK_WIDGET(button);

    // Store slave index globally
    selected_slave_index = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(button), "slave-index"));

    const gchar *label = gtk_button_get_label(button);
    char msg[64];
    snprintf(msg, sizeof(msg), "Slave %s selected", label);
    LOG_SUCCESS(msg);
}

void on_stop_button_clicked(GtkButton *button, gpointer user_data) {
    (void)button;
    (void)user_data;
    
    is_motion_active = false;
    strcpy(current_motion_direction, "");

    GtkStyleContext *f_ctx = gtk_widget_get_style_context(motion_forward_btn);
    GtkStyleContext *b_ctx = gtk_widget_get_style_context(motion_backward_btn);

    gtk_style_context_remove_class(f_ctx, "motion-active");
    gtk_style_context_remove_class(b_ctx, "motion-active");

    // Send command to move forward
    Command *cmd = g_new(Command, 1);
    cmd->id = CMD_STOP_VELOCITY_MOTION;
    cmd->slave_index = selected_slave_index;
    cmd->velocity = 0;
    strcpy(cmd->direction, "stop");
    
    // Queue the command for execution in the EtherCAT thread
    g_async_queue_push(button_command_queue, cmd);

    LOG_DEFAULT("STOP button pressed");
}

void on_inc_plus_clicked(GtkButton *button, gpointer entry) {
    (void)button;
    increment += 100;
    char text[16];
    snprintf(text, sizeof(text), "%d", increment);
    gtk_entry_set_text(GTK_ENTRY(entry), text);
    LOG_DEFAULT("INCREMENT increased");
}

void on_inc_minus_clicked(GtkButton *button, gpointer entry) {
    (void)button;
    increment = (increment >= 100) ? increment - 100 : 0;
    char text[16];
    snprintf(text, sizeof(text), "%d", increment);
    gtk_entry_set_text(GTK_ENTRY(entry), text);
    LOG_DEFAULT("INCREMENT decreased");
}

void on_vel_plus_clicked(GtkButton *button, gpointer entry) {
    (void)button;
    velocity += increment;
    char text[16];
    snprintf(text, sizeof(text), "%d", velocity);
    gtk_entry_set_text(GTK_ENTRY(entry), text);
    LOG_DEFAULT("VELOCITY increased");
}

void on_vel_minus_clicked(GtkButton *button, gpointer entry) {
    (void)button;
    velocity = (velocity >= increment) ? velocity - increment : 0;
    char text[16];
    snprintf(text, sizeof(text), "%d", velocity);
    gtk_entry_set_text(GTK_ENTRY(entry), text);
    LOG_DEFAULT("VELOCITY decreased");
}

// Function to get the index of the currently selected slave
int get_selected_slave_index(void) {
    if (last_selected_velocity_slave_btn == NULL) {
        return -1;  // No slave selected
    }
    
    // Get the button label (e.g., "Slave 0", "Slave 1", etc.)
    const gchar *label = gtk_button_get_label(GTK_BUTTON(last_selected_velocity_slave_btn));
    
    if (label == NULL) {
        return -1;  // Invalid label
    }
    
    // Extract slave number from label
    // Expected format: "Slave X" where X is the slave number
    if (strncmp(label, "Slave ", 6) == 0) {
        int slave_num = atoi(label + 6);  // Skip "Slave " and convert number
        
        // Validate the slave number
        if (slave_num >= 0 && slave_num < app.total_slaves) {
            return slave_num;  // Return 0-based index
        }
    }
    
    // Alternative: If your button labels use different format like "0", "1", "2"
    else {
        int slave_num = atoi(label);
        if (slave_num >= 0 && slave_num < app.total_slaves) {
            return slave_num;
        }
    }
    
    printf("[Error] Invalid slave selection: %s\n", label);
    return -1;  // Invalid selection
}

void on_forward_clicked(GtkButton *button, gpointer user_data) {
    GtkBuilder *builder = GTK_BUILDER(user_data);

    if (last_selected_velocity_slave_btn == NULL) {
        LOG_ERROR("No Slave Selected");
        return;
    }

    GtkEntry *velocity_entry = GTK_ENTRY(gtk_builder_get_object(builder, "velocity_entry"));
    const gchar *velocity_text = gtk_entry_get_text(velocity_entry);

    if (velocity_text == NULL || strlen(velocity_text) == 0 || atoi(velocity_text) == 0) {
        LOG_ERROR("Entered Velocity Speed is 0, thus Returning - No Movement");
        return;
    }

    // Get selected slave index
    int selected_slave_index = get_selected_slave_index();
    if (selected_slave_index < 0) {
        LOG_ERROR("Invalid slave selection");
        return;
    }

    // Mark motion as active
    is_motion_active = true;
    strcpy(current_motion_direction, "forward");

    // Update button styles
    GtkStyleContext *f_ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    GtkStyleContext *b_ctx = gtk_widget_get_style_context(GTK_WIDGET(motion_backward_btn));
    gtk_style_context_add_class(f_ctx, "motion-active");
    gtk_style_context_remove_class(b_ctx, "motion-active");

    // Create and queue the command (safety check happens in set_velocity)
    Command *cmd = g_new(Command, 1);
    cmd->id = CMD_COMMAND_FORWARD;
    cmd->slave_index = selected_slave_index;
    cmd->velocity = atoi(velocity_text); // Positive velocity for forward
    strcpy(cmd->direction, "forward");
    
    // Queue the command for execution in the EtherCAT thread
    g_async_queue_push(button_command_queue, cmd);

    const gchar *slave_label = gtk_button_get_label(GTK_BUTTON(last_selected_velocity_slave_btn));
    char msg[128];
    snprintf(msg, sizeof(msg), "[Forward] %s movement requested with velocity %s", 
             slave_label, velocity_text);
    LOG_SUCCESS(msg);
}

void on_backward_clicked(GtkButton *button, gpointer user_data) {
    GtkBuilder *builder = GTK_BUILDER(user_data);

    if (last_selected_velocity_slave_btn == NULL) {
        LOG_ERROR("No Slave Selected");
        return;
    }

    GtkEntry *velocity_entry = GTK_ENTRY(gtk_builder_get_object(builder, "velocity_entry"));
    const gchar *velocity_text = gtk_entry_get_text(velocity_entry);

    if (velocity_text == NULL || strlen(velocity_text) == 0 || atoi(velocity_text) == 0) {
        LOG_ERROR("Entered Velocity Speed is 0, thus Returning - No Movement");
        return;
    }

    // Get selected slave index
    int selected_slave_index = get_selected_slave_index();
    if (selected_slave_index < 0) {
        LOG_ERROR("Invalid slave selection");
        return;
    }

    // Mark motion as active
    is_motion_active = true;
    strcpy(current_motion_direction, "backward");

    // Update button styles
    GtkStyleContext *b_ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    GtkStyleContext *f_ctx = gtk_widget_get_style_context(GTK_WIDGET(motion_forward_btn));
    gtk_style_context_add_class(b_ctx, "motion-active");
    gtk_style_context_remove_class(f_ctx, "motion-active");

    // Create and queue the command (safety check happens in set_velocity)
    Command *cmd = g_new(Command, 1);
    cmd->id = CMD_COMMAND_BACKWARD;
    cmd->slave_index = selected_slave_index;
    cmd->velocity = atoi(velocity_text); // Will be made negative in set_velocity
    strcpy(cmd->direction, "backward");
    
    // Queue the command for execution in the EtherCAT thread
    g_async_queue_push(button_command_queue, cmd);

    const gchar *slave_label = gtk_button_get_label(GTK_BUTTON(last_selected_velocity_slave_btn));
    char msg[128];
    snprintf(msg, sizeof(msg), "[Backward] %s movement requested with velocity %s", 
             slave_label, velocity_text);
    LOG_SUCCESS(msg);
}

void on_destroy(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    g_print("Destroy signal received, cleaning up...\n");
    gtk_main_quit();
}

void open_velocity_test(GtkButton *button, gpointer user_data) {
    (void)button;
    (void)user_data;
    
    app_load_css(NULL);

    char exe_dir[MAX_PATH];
    GetModuleFileNameA(NULL, exe_dir, MAX_PATH);
    PathRemoveFileSpecA(exe_dir);

    char ui_path[MAX_PATH];
    _snprintf_s(ui_path, MAX_PATH, _TRUNCATE, "%s\\builders\\velocityTest.ui", exe_dir);
    GtkBuilder *builder = load_ui(ui_path);
    
    vel_win = GTK_WIDGET(gtk_builder_get_object(builder, "velocity_test_window"));
    if (!vel_win) {
        g_critical("Failed to load velocity test window");
        return;
    }
    gtk_builder_connect_signals(builder, NULL);

    g_signal_connect(gtk_builder_get_object(builder, "back_button"), "clicked", G_CALLBACK(on_back_clicked), NULL);

    attach_shared_title_with_back(builder, "title_placeholder", "<b>Velocity Test</b>", G_CALLBACK(on_back_clicked), NULL);
    
    // g_thread_new("ethercat_init", ethercat_init_thread, builder);
    // g_timeout_add(10, (GSourceFunc)trigger_slave_check, NULL);

    GtkBuilder *shared_builder = attach_shared_panel(builder, "right_panel");
    app_connect_shared_right_panel_signals();

    app.log_view = GTK_WIDGET(gtk_builder_get_object(shared_builder, "logs_view"));
    app.log_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(app.log_view));
    setup_log_tags(app.log_buffer);

    // Set up toggle buttons styling
    if (app.status_toggle_button) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(app.status_toggle_button);
        gtk_button_set_label(GTK_BUTTON(app.status_toggle_button),
        app.status_enabled ? "Status: ENABLE" : "Status: Disable");
        gtk_style_context_add_class(ctx, app.status_enabled ? "status-enabled" : "status-disabled");
    }

    if (app.break_toggle_button) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(app.break_toggle_button);
        gtk_button_set_label(GTK_BUTTON(app.break_toggle_button),
        app.breaks_enabled ? "Breaks: ON" : "Breaks: OFF");
        gtk_style_context_add_class(ctx, app.breaks_enabled ? "breaks-on" : "breaks-off");
    }

    if (app.reset_fault_btn) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(app.reset_fault_btn);
        gtk_style_context_add_class(ctx, "reset-btn");
    }

    app_update_slave_count(app.total_slaves);
    slave_total_available = app.total_slaves;

    generate_slave_buttons(builder,
                       "slave_grid",
                       app.selection_type,
                       on_slave_button_clicked_velocity,
                       NULL);

    // Input fields
    GtkEntry *velocity_entry = GTK_ENTRY(gtk_builder_get_object(builder, "velocity_entry"));
    GtkEntry *increment_entry = GTK_ENTRY(gtk_builder_get_object(builder, "increment_entry"));
    gtk_entry_set_text(velocity_entry, "7000");
    gtk_entry_set_text(increment_entry, "100");

    // Velocity and Increment ± buttons
    g_signal_connect(gtk_builder_get_object(builder, "vel_plus_btn"),  "clicked",
                     G_CALLBACK(on_vel_plus_clicked), velocity_entry);
    g_signal_connect(gtk_builder_get_object(builder, "vel_minus_btn"), "clicked",
                     G_CALLBACK(on_vel_minus_clicked), velocity_entry);

    g_signal_connect(gtk_builder_get_object(builder, "inc_plus_btn"), "clicked",
                     G_CALLBACK(on_inc_plus_clicked), increment_entry);
    g_signal_connect(gtk_builder_get_object(builder, "inc_minus_btn"), "clicked",
                     G_CALLBACK(on_inc_minus_clicked), increment_entry);

    // Motion control buttons
    motion_forward_btn = GTK_WIDGET(gtk_builder_get_object(builder, "forward_btn"));
    motion_backward_btn = GTK_WIDGET(gtk_builder_get_object(builder, "backward_btn"));
    GtkButton *stop_btn = GTK_BUTTON(gtk_builder_get_object(builder, "stop_btn"));

    g_signal_connect(motion_forward_btn, "clicked", G_CALLBACK(on_forward_clicked), builder);
    g_signal_connect(motion_backward_btn, "clicked", G_CALLBACK(on_backward_clicked), builder);
    g_signal_connect(stop_btn, "clicked", G_CALLBACK(on_stop_button_clicked), NULL);

    g_signal_connect(vel_win, "destroy", G_CALLBACK(on_destroy), NULL);

    gtk_widget_hide(app.main_window);
    gtk_widget_show_all(vel_win);
}