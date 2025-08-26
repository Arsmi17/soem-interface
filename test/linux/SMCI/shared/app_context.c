#include "app_context.h"
#include "ethercat_comm.h"  // To access EtherCAT drive control functions
#include <math.h>
AppContext app = {
    .log_scroll = NULL, // main page log_view
    .total_slaves = 0, // ← Set default to 0
    .selection_type = "Matrix", // "Matrix", "HMRS"
};


#define DEFAULT_CSS_PATH "css/style.css"  // You can override this as needed

void app_load_css(const char *css_path) {
    const char *path = css_path ? css_path : DEFAULT_CSS_PATH;

    GtkCssProvider *provider = gtk_css_provider_new();
    GError *error = NULL;

    if (!gtk_css_provider_load_from_path(provider, path, &error)) {
        g_warning("❌ Failed to load CSS file: %s", error->message);
        g_error_free(error);
        g_object_unref(provider);
        return;
    }

    gtk_style_context_add_provider_for_screen(
        gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(provider),
        GTK_STYLE_PROVIDER_PRIORITY_USER
    );

    g_message("✅ CSS loaded from %s", path);
    g_object_unref(provider);
}

GtkBuilder* load_ui(const char *filename) {
    GtkBuilder *builder = gtk_builder_new();
    GError *error = NULL;

    if (!gtk_builder_add_from_file(builder, filename, &error)) {
        g_critical("Error loading UI file: %s", error->message);
        g_error_free(error);
    }

    return builder;
}

void setup_log_tags(GtkTextBuffer *buffer) {
    if (!buffer) return;

    gtk_text_buffer_create_tag(buffer, "log-info",     "foreground", "#3498db", NULL);
    gtk_text_buffer_create_tag(buffer, "log-warning",  "foreground", "#f39c12", NULL);
    gtk_text_buffer_create_tag(buffer, "log-danger",   "foreground", "#e74c3c", NULL);
    gtk_text_buffer_create_tag(buffer, "log-success",  "foreground", "#27ae60", NULL);
    gtk_text_buffer_create_tag(buffer, "log-primary",  "foreground", "#1abc9c", NULL);
}

static gboolean update_slave_position_display(gpointer data) {
    PositionUpdateData *pos_data = (PositionUpdateData *)data;

    if (pos_data->slave_id >= 0 && pos_data->slave_id < app.total_slaves &&
        app.slave_position_labels && app.slave_position_labels[pos_data->slave_id]) {

        char position_text[256];
        snprintf(position_text, sizeof(position_text),
                "Slave %-3d  |  %+12d  |  %+8.3f m",
                pos_data->slave_id,
                pos_data->raw_position,
                pos_data->position_meters);

        gtk_label_set_text(GTK_LABEL(app.slave_position_labels[pos_data->slave_id]), position_text);

        // Optional: Add color coding based on position or status
        GtkStyleContext *context = gtk_widget_get_style_context(app.slave_position_labels[pos_data->slave_id]);
        gtk_style_context_remove_class(context, "error");
        gtk_style_context_remove_class(context, "warning");

        // Example: Add warning class if position is very large (now with math.h included)
        if (fabs(pos_data->position_meters) > 100.0) {
            gtk_style_context_add_class(context, "warning");
        }
    }

    g_free(pos_data);
    return G_SOURCE_REMOVE;
}

void update_position_display(int slave_id, int32_t raw_position, float position_meters) {
    PositionUpdateData *pos_data = g_new(PositionUpdateData, 1);
    pos_data->slave_id = slave_id;
    pos_data->raw_position = raw_position;
    pos_data->position_meters = position_meters;

    // Schedule GUI update in main thread
    g_idle_add(update_slave_position_display, pos_data);
}

void append_unstyled_log(const char *fmt, ...) {
    if (!app.log_buffer) return;

    char buffer[1024];  // Adjust buffer size as needed
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // Trim old logs (keep last 500 lines)
    GtkTextIter start, trim_iter;
    int total_lines = gtk_text_buffer_get_line_count(app.log_buffer);
    if (total_lines > 500) {
        gtk_text_buffer_get_start_iter(app.log_buffer, &start);
        gtk_text_buffer_get_iter_at_line(app.log_buffer, &trim_iter, total_lines - 500);
        gtk_text_buffer_delete(app.log_buffer, &start, &trim_iter);
    }

    // Append new log without tags
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(app.log_buffer, &end);
    gtk_text_buffer_insert(app.log_buffer, &end, buffer, -1);
    gtk_text_buffer_insert(app.log_buffer, &end, "\n", 1);

    // 🔽 Auto-scroll to bottom
    GtkTextMark *mark = gtk_text_buffer_create_mark(app.log_buffer, NULL, &end, FALSE);
    gtk_text_view_scroll_mark_onscreen(GTK_TEXT_VIEW(app.log_view), mark);
}

void append_colored_log(const char *tag, const char *fmt, ...) {
    if (!app.log_buffer) return;

    char buffer[1024];  // Adjust size as needed
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // Trim old logs (keep last 500 lines)
    GtkTextIter start, trim_iter;
    int total_lines = gtk_text_buffer_get_line_count(app.log_buffer);
    if (total_lines > 500) {
        gtk_text_buffer_get_start_iter(app.log_buffer, &start);
        gtk_text_buffer_get_iter_at_line(app.log_buffer, &trim_iter, total_lines - 500);
        gtk_text_buffer_delete(app.log_buffer, &start, &trim_iter);
    }

    // Append new log with tag
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(app.log_buffer, &end);
    gtk_text_buffer_insert_with_tags_by_name(app.log_buffer, &end, buffer, -1, tag, NULL);
    gtk_text_buffer_insert(app.log_buffer, &end, "\n", 1);

    // 🔽 Auto-scroll to bottom
    GtkTextMark *mark = gtk_text_buffer_create_mark(app.log_buffer, NULL, &end, FALSE);
    gtk_text_view_scroll_mark_onscreen(GTK_TEXT_VIEW(app.log_view), mark);

}

void create_slave_position_displays(GtkWidget *parent_container) {
    // Clear existing widgets if any
    if (app.slave_position_labels) {
        for (int i = 0; i < app.total_slaves; i++) {
            if (app.slave_position_labels[i]) {
                gtk_widget_destroy(app.slave_position_labels[i]);
            }
        }
        g_free(app.slave_position_labels);
    }

    // Allocate memory for position labels
    app.slave_position_labels = g_malloc(sizeof(GtkWidget*) * app.total_slaves);

    // Create a vertical box to hold all position displays
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_margin_top(vbox, 10);
    gtk_widget_set_margin_bottom(vbox, 10);
    gtk_widget_set_margin_start(vbox, 10);
    gtk_widget_set_margin_end(vbox, 10);

    // Add header
    GtkWidget *header_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(header_label),
        "<b>Slave ID    |    Raw Position    |    Position (m)</b>");
    gtk_widget_set_halign(header_label, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(vbox), header_label, FALSE, FALSE, 0);

    // Add separator
    GtkWidget *separator = gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_box_pack_start(GTK_BOX(vbox), separator, FALSE, FALSE, 5);

    // Create individual labels for each slave
    for (int i = 0; i < app.total_slaves; i++) {
        GtkWidget *slave_frame = gtk_frame_new(NULL);
        gtk_frame_set_shadow_type(GTK_FRAME(slave_frame), GTK_SHADOW_IN);

        app.slave_position_labels[i] = gtk_label_new("Initializing...");
        gtk_widget_set_halign(app.slave_position_labels[i], GTK_ALIGN_START);
        gtk_widget_set_margin_top(app.slave_position_labels[i], 5);
        gtk_widget_set_margin_bottom(app.slave_position_labels[i], 5);
        gtk_widget_set_margin_start(app.slave_position_labels[i], 10);
        gtk_widget_set_margin_end(app.slave_position_labels[i], 10);

        // Set monospace font for better alignment
        GtkStyleContext *context = gtk_widget_get_style_context(app.slave_position_labels[i]);
        gtk_style_context_add_class(context, "monospace");

        gtk_container_add(GTK_CONTAINER(slave_frame), app.slave_position_labels[i]);
        gtk_box_pack_start(GTK_BOX(vbox), slave_frame, FALSE, FALSE, 2);
    }

    // Add the vbox to parent container
    gtk_container_add(GTK_CONTAINER(parent_container), vbox);
    gtk_widget_show_all(parent_container);

    app.position_container = vbox;
}


GtkBuilder* attach_shared_panel(GtkBuilder *builder, const char *right_panel_id) {
    // Load the shared UI
    GtkBuilder *shared_builder = load_ui("builders/shared_right_panel.ui");
    app.main_builder = shared_builder;

    // Load CSS
    app_load_css(NULL);

    // Widgets
    GtkWidget *shared_panel = GTK_WIDGET(gtk_builder_get_object(shared_builder, "shared_right_panel"));
    GtkWidget *target_panel = GTK_WIDGET(gtk_builder_get_object(builder, right_panel_id));
    (void)shared_panel; // Avoid unused warning

    // LOGS
    GtkWidget *logs_view = GTK_WIDGET(gtk_builder_get_object(shared_builder, "logs_textview")); // Correct ID
    GtkTextBuffer *log_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(logs_view));
    app.log_view = logs_view;
    app.log_buffer = log_buffer;

    // Apply .monospace class to logs
    GtkStyleContext *log_ctx = gtk_widget_get_style_context(logs_view);
    gtk_style_context_add_class(log_ctx, "monospace");


    // Positions Frame
    GtkWidget *position_frame = GTK_WIDGET(gtk_builder_get_object(shared_builder, "position_frame"));
    if (position_frame) {
        GList *children = gtk_container_get_children(GTK_CONTAINER(position_frame));
        for (GList *iter = children; iter != NULL; iter = g_list_next(iter)) {
            gtk_widget_destroy(GTK_WIDGET(iter->data));
        }
        g_list_free(children);

        // create_slave_position_displays(position_frame);
    }

    // Clear and attach to target panel
    GList *children = gtk_container_get_children(GTK_CONTAINER(target_panel));
    for (GList *iter = children; iter != NULL; iter = g_list_next(iter)) {
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    }
    g_list_free(children);

    // Attach and display
    gtk_box_pack_start(GTK_BOX(target_panel), shared_panel, TRUE, TRUE, 0);
    gtk_widget_show_all(target_panel);

    return shared_builder;
}

void attach_shared_title_with_back(GtkBuilder *builder, const char *placeholder_id,
                                    const char *title_text, GCallback back_callback, gpointer user_data) {
    GtkBuilder *shared_builder = load_ui("builders/shared_title_bar.ui");

    GtkWidget *title_bar = GTK_WIDGET(gtk_builder_get_object(shared_builder, "shared_title_bar"));
    GtkLabel *title_label = GTK_LABEL(gtk_builder_get_object(shared_builder, "page_title_label"));
    GtkWidget *back_button = GTK_WIDGET(gtk_builder_get_object(shared_builder, "back_button"));
    GtkBox *target = GTK_BOX(gtk_builder_get_object(builder, placeholder_id));

    gtk_label_set_markup(title_label, title_text);

    if (back_callback) {
        g_signal_connect(back_button, "clicked", back_callback, user_data);
    }

    GList *children = gtk_container_get_children(GTK_CONTAINER(target));
    for (GList *l = children; l != NULL; l = l->next) {
        gtk_widget_destroy(GTK_WIDGET(l->data));
    }
    g_list_free(children);

    GtkWidget *reset_fault_btn = GTK_WIDGET(gtk_builder_get_object(shared_builder, "reset_fault_btn"));
    GtkWidget *break_toggle = GTK_WIDGET(gtk_builder_get_object(shared_builder, "break_toggle_button"));
    GtkWidget *status_toggle = GTK_WIDGET(gtk_builder_get_object(shared_builder, "status_toggle_button"));
    GtkWidget *check_status_button = GTK_WIDGET(gtk_builder_get_object(shared_builder, "check_status_button"));
    GtkWidget *slave_count_label = GTK_WIDGET(gtk_builder_get_object(shared_builder, "connected_slaves_label"));

    app.break_toggle_button = break_toggle;
    app.status_toggle_button = status_toggle;
    app.check_status_button = check_status_button;
    app.reset_fault_btn = reset_fault_btn;
    app.connected_slaves_label = slave_count_label;

    gtk_box_pack_start(target, title_bar, FALSE, FALSE, 0);
    gtk_widget_show_all(GTK_WIDGET(title_bar));

    g_object_unref(shared_builder);
}

void app_update_slave_count(int count) {
    app.total_slaves = count;
    printf("AppContext updated: total_slaves = %d\n", app.total_slaves);

    // Optionally update a GTK label if you have one
    if (app.connected_slaves_label) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Connected Slaves: %d", count);
        gtk_label_set_text(GTK_LABEL(app.connected_slaves_label), msg);
    }

    if(app.total_slaves > 0){
        GtkWidget *position_frame = GTK_WIDGET(gtk_builder_get_object(app.main_builder, "position_frame"));
        if (position_frame) {
            create_slave_position_displays(position_frame);
        }
    }
}

static void on_break_toggle_clicked(GtkButton *button, gpointer user_data) {
    (void)user_data;

    app.breaks_enabled = !app.breaks_enabled;

    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));

    if (app.breaks_enabled) {
        gtk_button_set_label(button, "Breaks: ON");
        gtk_style_context_remove_class(ctx, "breaks-off");
        gtk_style_context_add_class(ctx, "breaks-on");
        LOG_SUCCESS("🟢 Breaks turned ON");
    } else {
        gtk_button_set_label(button, "Breaks: OFF");
        gtk_style_context_remove_class(ctx, "breaks-on");
        gtk_style_context_add_class(ctx, "breaks-off");
        LOG_ERROR("⚠️ Breaks turned OFF");
    }
}

static void on_check_status_button(GtkButton *button, gpointer user_data) {
    (void)user_data;
    (void)button;
    app.breaks_enabled = !app.breaks_enabled;

    Command *cmd = g_new(Command, 1);
    cmd->id = CMD_CHECK_STATUS;
    g_async_queue_push(button_command_queue, cmd);
    printf("Check status command queued.\n");

}

static void on_reset_fault_clicked(GtkButton *button, gpointer user_data) {
    (void)button;
    (void)user_data;

    LOG_PRIMARY("🔧 Reset Fault clicked");

    Command *cmd = g_new(Command, 1);
    cmd->id = CMD_RESET_FAULTS;
    g_async_queue_push(button_command_queue, cmd);
    printf("Reset faults command queued.\n");

    // After resetting faults, disable the status if currently enabled
    if (app.status_enabled) {
        app.status_enabled = FALSE;
        gtk_button_set_label(GTK_BUTTON(app.status_toggle_button), "Status: Disable");
        GtkStyleContext *ctx = gtk_widget_get_style_context(app.status_toggle_button);
        gtk_style_context_remove_class(ctx, "status-enabled");
        gtk_style_context_add_class(ctx, "status-disabled");

        LOG_WARN("⛔ Reset Fault → Status disabled");
    }
}

static void on_status_toggle_clicked(GtkButton *button, gpointer user_data) {
    (void)user_data;

    app.status_enabled = !app.status_enabled;

    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));


    Command *cmd = g_new(Command, 1);
    cmd->id = app.status_enabled ? CMD_ENABLE_ALL : CMD_DISABLE_ALL;
    g_async_queue_push(button_command_queue, cmd);
    printf("%s all drives command queued.\n", app.status_enabled ? "Enable" : "Disable");

    if (app.status_enabled) {
        gtk_button_set_label(button, "Status: ENABLE");
        gtk_style_context_remove_class(ctx, "status-disabled");
        gtk_style_context_add_class(ctx, "status-enabled");
        LOG_SUCCESS("✅ Status enabled");
    } else {
        gtk_button_set_label(button, "Status: Disable");
        gtk_style_context_remove_class(ctx, "status-enabled");
        gtk_style_context_add_class(ctx, "status-disabled");
        LOG_WARN("⏺ Status disabled");
    }
}

void app_connect_shared_right_panel_signals(void) {
    if (app.status_toggle_button)
        g_signal_connect(app.status_toggle_button, "clicked", G_CALLBACK(on_status_toggle_clicked), NULL);

    if (app.break_toggle_button)
        g_signal_connect(app.break_toggle_button, "clicked", G_CALLBACK(on_break_toggle_clicked), NULL);

    if (app.check_status_button)
        g_signal_connect(app.check_status_button, "clicked", G_CALLBACK(on_check_status_button), NULL);

    if (app.reset_fault_btn)
        g_signal_connect(app.reset_fault_btn, "clicked", G_CALLBACK(on_reset_fault_clicked), NULL);
}