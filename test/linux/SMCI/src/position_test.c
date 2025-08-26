#include <gtk/gtk.h>
#include "app_context.h"
#include "slave_selector.h"
#include "ethercat_comm.h"
#include <windows.h>
#include <shlwapi.h> 

static GtkWidget *pos_win = NULL;
static GtkWidget *last_selected_slave_btn = NULL;
static GtkWidget *last_selected_column_btn = NULL;
static GtkWidget *last_selected_row_btn = NULL;
static GtkWidget *last_selected_hmrs_zone_btn = NULL;

extern void on_slave_button_clicked_position(GtkButton *btn, gpointer user_data);

void reset_all_slave_styles(void) {
    for (int i = 0; i < app.total_slaves; i++) {
        if (slave_buttons[i]) {
            GtkStyleContext *ctx = gtk_widget_get_style_context(slave_buttons[i]);
            gtk_style_context_remove_class(ctx, "slave-selected");
            gtk_style_context_remove_class(ctx, "column-highlight");
            gtk_style_context_remove_class(ctx, "row-highlight");
            gtk_style_context_remove_class(ctx, "zone-highlight");
        }
    }
}

void on_slave_button_clicked_position(GtkButton *button, gpointer user_data) {
    (void)user_data;

    // Check if already selected
    if (last_selected_slave_btn == GTK_WIDGET(button)) {
        // Deselect current
        GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
        gtk_style_context_remove_class(ctx, "slave-selected");
        last_selected_slave_btn = NULL;
        LOG_WARN("Slave deselected.");
        return;
    }

    // Clear highlights from other selectors and slaves
    reset_all_slave_styles();

    if (last_selected_column_btn || last_selected_row_btn || last_selected_hmrs_zone_btn) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(last_selected_column_btn);
        gtk_style_context_remove_class(ctx, "selected");
        GtkStyleContext *rtx = gtk_widget_get_style_context(last_selected_row_btn);
        gtk_style_context_remove_class(rtx, "selected");
        GtkStyleContext *htx = gtk_widget_get_style_context(last_selected_hmrs_zone_btn);
        gtk_style_context_remove_class(htx, "selected");
    }

    // Reset selection buttons
    last_selected_column_btn = NULL;
    last_selected_row_btn = NULL;
    last_selected_hmrs_zone_btn = NULL;

    // Mark slave as selected
    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    gtk_style_context_add_class(ctx, "slave-selected");
    last_selected_slave_btn = GTK_WIDGET(button);

    // Log
    const gchar *label = gtk_button_get_label(button);
    char msg[64];
    snprintf(msg, sizeof(msg), "Slave %s selected", label);
    LOG_SUCCESS(msg);
}

void on_column_button_clicked(GtkButton *button, gpointer user_data) {
    int column = GPOINTER_TO_INT(user_data);

    reset_all_slave_styles();

    if (last_selected_row_btn) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(last_selected_row_btn);
        gtk_style_context_remove_class(ctx, "selected");
        last_selected_row_btn = NULL;
    }

    last_selected_slave_btn = NULL;
    last_selected_row_btn = NULL;
    last_selected_hmrs_zone_btn = NULL;

        // Compute indexes for column slaves
    int indices[] = { column * 3 + 0, column * 3 + 1, column * 3 + 2 };

    for (int i = 0; i < 3; i++) {
        if (indices[i] < app.total_slaves && slave_buttons[indices[i]]) {
            GtkStyleContext *ctx = gtk_widget_get_style_context(slave_buttons[indices[i]]);
            gtk_style_context_add_class(ctx, "column-highlight");
        }
    }

    if (last_selected_column_btn && last_selected_column_btn != GTK_WIDGET(button)) {
        GtkStyleContext *prev_ctx = gtk_widget_get_style_context(last_selected_column_btn);
        gtk_style_context_remove_class(prev_ctx, "selected");
    }

    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    gtk_style_context_add_class(ctx, "selected");
    last_selected_column_btn = GTK_WIDGET(button);

    gchar *log = g_strdup_printf("📊 Column %d highlighted: [%d, %d, %d]", column, indices[0], indices[1], indices[2]);
    LOG_INFO(log);
    g_free(log);
}

void on_row_button_clicked(GtkButton *button, gpointer user_data) {
    int row = GPOINTER_TO_INT(user_data);

    if (last_selected_column_btn) {
        GtkStyleContext *ctx = gtk_widget_get_style_context(last_selected_column_btn);
        gtk_style_context_remove_class(ctx, "selected");
        last_selected_column_btn = NULL;
    }

    reset_all_slave_styles();
    last_selected_slave_btn = NULL;
    last_selected_column_btn = NULL;
    last_selected_hmrs_zone_btn = NULL;

    // Remove highlight from previously selected
    if (last_selected_row_btn && last_selected_row_btn != GTK_WIDGET(button)) {
        GtkStyleContext *prev_ctx = gtk_widget_get_style_context(last_selected_row_btn);
        gtk_style_context_remove_class(prev_ctx, "selected");
    }

    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    gtk_style_context_add_class(ctx, "selected");
    last_selected_row_btn = GTK_WIDGET(button);

    // Highlight slaves for this row
    GString *log = g_string_new(NULL);
    g_string_append_printf(log, "📎 Row %d highlighted: ", row + 1);

    for (int i = 0; i < 6; i++) {
        int index = row + i * 3;
        if (index >= app.total_slaves) break;

        if (slave_buttons[index]) {
            GtkStyleContext *sctx = gtk_widget_get_style_context(slave_buttons[index]);
            gtk_style_context_add_class(sctx, "row-highlight");
            g_string_append_printf(log, "%d", index);
            if (i < 5 && index + 3 < app.total_slaves)
                g_string_append(log, ", ");
        }
    }

    LOG_INFO(log->str);
    g_string_free(log, TRUE);
}

void on_hmrs_zone_clicked(GtkButton *button, gpointer user_data) {
    const gchar *zone = (const gchar *)user_data;

    reset_all_slave_styles();
    last_selected_slave_btn = NULL;
    last_selected_column_btn = NULL;
    last_selected_row_btn = NULL;

    if (last_selected_hmrs_zone_btn && last_selected_hmrs_zone_btn != GTK_WIDGET(button)) {
        GtkStyleContext *prev_ctx = gtk_widget_get_style_context(last_selected_hmrs_zone_btn);
        gtk_style_context_remove_class(prev_ctx, "selected");
    }

    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    gtk_style_context_add_class(ctx, "selected");
    last_selected_hmrs_zone_btn = GTK_WIDGET(button);

    int indices[4];
    if (g_strcmp0(zone, "linear") == 0) {
        indices[0] = 1; indices[1] = 3; indices[2] = 5; indices[3] = 7;
    } else {
        indices[0] = 0; indices[1] = 2; indices[2] = 4; indices[3] = 6;
    }

    for (int i = 0; i < 4; i++) {
        int idx = indices[i];
        if (idx < app.total_slaves && slave_buttons[idx]) {
            GtkStyleContext *sctx = gtk_widget_get_style_context(slave_buttons[idx]);
            gtk_style_context_add_class(sctx, "zone-highlight");
        }
    }

    gchar *log = g_strdup_printf("🛠️ HMRS %s zone highlighted", g_ascii_strup(zone, -1));
    LOG_INFO(log);
    g_free(log);
}

void generate_column_buttons(GtkBuilder *builder) {
    GtkGrid *col_grid = GTK_GRID(gtk_builder_get_object(builder, "column_grid"));

    GList *children = gtk_container_get_children(GTK_CONTAINER(col_grid));
    for (GList *iter = children; iter != NULL; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    if (g_strcmp0(app.selection_type, "Matrix") != 0)
        return;

    int num_columns = app.total_slaves / 3;

    for (int col = 0; col < num_columns; ++col) {
        gchar *label = g_strdup_printf("Column %d", col);
        GtkWidget *btn = gtk_button_new_with_label(label);

        g_signal_connect(btn, "clicked", G_CALLBACK(on_column_button_clicked), GINT_TO_POINTER(col));

        gtk_grid_attach(col_grid, btn, col % 3, col / 3, 1, 1);
        g_free(label);
    }

    gtk_widget_show_all(GTK_WIDGET(col_grid));
}

void generate_row_buttons(GtkBuilder *builder) {
    GtkGrid *row_grid = GTK_GRID(gtk_builder_get_object(builder, "row_grid"));

    GList *children = gtk_container_get_children(GTK_CONTAINER(row_grid));
    for (GList *iter = children; iter != NULL; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    if (g_strcmp0(app.selection_type, "Matrix") != 0)
        return;

    int row_count = 0;

    for (int row = 0; row < 3; ++row) {
        gboolean has_any = FALSE;
        for (int offset = 0; offset < 6; ++offset) {
            int index = row + offset * 3;
            if (index < app.total_slaves) {
                has_any = TRUE;
                break;
            }
        }

        if (!has_any) continue;

        gchar *label = g_strdup_printf("Row %d", row + 1);
        GtkWidget *btn = gtk_button_new_with_label(label);

        g_signal_connect(btn, "clicked", G_CALLBACK(on_row_button_clicked), GINT_TO_POINTER(row));

        gtk_grid_attach(row_grid, btn, row_count++, 0, 1, 1);
        g_free(label);
    }

    gtk_widget_show_all(GTK_WIDGET(row_grid));
}

void generate_hmrs_zone_buttons(GtkBuilder *builder) {
    GtkGrid *zone_grid = GTK_GRID(gtk_builder_get_object(builder, "column_grid"));  // reused

    GList *children = gtk_container_get_children(GTK_CONTAINER(zone_grid));
    for (GList *iter = children; iter != NULL; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    if (g_strcmp0(app.selection_type, "HMRS") != 0)
        return;

    GtkWidget *lin_btn = gtk_button_new_with_label("Linear");
    g_signal_connect(lin_btn, "clicked", G_CALLBACK(on_hmrs_zone_clicked), "linear");
    gtk_grid_attach(zone_grid, lin_btn, 0, 0, 1, 1);

    GtkWidget *rot_btn = gtk_button_new_with_label("Rotation");
    g_signal_connect(rot_btn, "clicked", G_CALLBACK(on_hmrs_zone_clicked), "rotation");
    gtk_grid_attach(zone_grid, rot_btn, 1, 0, 1, 1);

    gtk_widget_show_all(GTK_WIDGET(zone_grid));
}

void update_position_test_layout(GtkBuilder *builder, const gchar *type) {
    GtkWidget *column_frame = GTK_WIDGET(gtk_builder_get_object(builder, "column_frame"));
    GtkWidget *row_frame = GTK_WIDGET(gtk_builder_get_object(builder, "row_frame"));

    if (!column_frame || !row_frame) {
        g_warning("Could not find column_frame or row_frame in UI.");
        return;
    }

    if (g_strcmp0(type, "HMRS") == 0) {
        gtk_frame_set_label(GTK_FRAME(column_frame), "Movement Selection");
        gtk_widget_hide(row_frame);
    } else if (g_strcmp0(type, "Matrix") == 0) {
        gtk_frame_set_label(GTK_FRAME(column_frame), "Column Selection");
        gtk_widget_show(row_frame);
    } else {
        gtk_frame_set_label(GTK_FRAME(column_frame), "Unknown Mode");
        gtk_widget_hide(row_frame);
    }

    // Optional log
    gchar *log = g_strdup_printf("Layout updated for type: %s", type);
    LOG_DEFAULT(log);
    g_free(log);
}

static void on_back_clicked(GtkButton *btn, gpointer user_data) {
    (void)user_data;
    (void)btn;

    gtk_widget_hide(pos_win);
    gtk_widget_show_all(app.main_window);
}
typedef struct {
    GtkEntry *position_entry;
    GtkEntry *speed_entry;
    GtkWidget *start_button;
} StartPositionData;

void on_start_position_clicked(GtkButton *button, gpointer user_data) {
    StartPositionData *data = (StartPositionData *)user_data;
    GtkEntry *position_entry = data->position_entry;
    GtkEntry *speed_entry = data->speed_entry;

    const gchar *position_text = gtk_entry_get_text(position_entry);
    const gchar *speed_text = gtk_entry_get_text(speed_entry);

    // Validate position input
    char *endptr;
    double position = g_ascii_strtod(position_text, &endptr);
    if (endptr == position_text || *endptr != '\0') {
        LOG_ERROR("Invalid position: Please enter a numeric value.");
        return;
    }
    if (position < HOME_POSITION || position > END_POSITION) {
        LOG_WARN("Target Position out of range (0.0 to 0.9)");
        return;
    }

    // Validate speed input
    int speed = (int)g_ascii_strtod(speed_text, &endptr);
    if (endptr == speed_text || *endptr != '\0' || speed <= 0) {
        LOG_ERROR("Invalid speed: Please enter a positive numeric value.");
        return;
    }

    // Get selected slaves
    GPtrArray *active_slaves = g_ptr_array_new();
    for (int i = 0; i < app.total_slaves; i++) {
        if (!slave_buttons[i]) continue;
        GtkStyleContext *ctx = gtk_widget_get_style_context(slave_buttons[i]);

        if (gtk_style_context_has_class(ctx, "slave-selected") ||
            gtk_style_context_has_class(ctx, "column-highlight") ||
            gtk_style_context_has_class(ctx, "row-highlight") ||
            gtk_style_context_has_class(ctx, "zone-highlight")) {
                g_ptr_array_add(active_slaves, GINT_TO_POINTER(i));
        }
    }

    if (active_slaves->len == 0) {
        LOG_ERROR("❌ No Slave Selected");
        g_ptr_array_free(active_slaves, TRUE);
        return;
    }

    // Update UI style
    GtkStyleContext *ctx = gtk_widget_get_style_context(GTK_WIDGET(button));
    gtk_style_context_remove_class(ctx, "start-button");
    gtk_style_context_add_class(ctx, "start-active");

    gchar *log = g_strdup_printf("🚀 Moving to %.2f at speed %d on %u slave%s",
                                 position, speed, active_slaves->len,
                                 (active_slaves->len == 1 ? "" : "s"));
    LOG_PRIMARY(log);
    g_free(log);

    g_ptr_array_free(active_slaves, TRUE);
}


void on_stop_position_btn_clicked(GtkButton *button, gpointer user_data) {
    (void)button; // Unused parameter
    StartPositionData *start_data = (StartPositionData *)user_data;

    // If you want to update the start button UI, include start_button in StartPositionData struct
    GtkWidget *start_button = start_data->start_button;
    GtkStyleContext *ctx = gtk_widget_get_style_context(start_button);
    gtk_style_context_remove_class(ctx, "start-active");
    gtk_style_context_add_class(ctx, "start-button");

    // Get selected slaves (same logic as start)
    GPtrArray *active_slaves = g_ptr_array_new();
    for (int i = 0; i < app.total_slaves; i++) {
        if (!slave_buttons[i]) continue;
        GtkStyleContext *ctx = gtk_widget_get_style_context(slave_buttons[i]);

        if (gtk_style_context_has_class(ctx, "slave-selected") ||
            gtk_style_context_has_class(ctx, "column-highlight") ||
            gtk_style_context_has_class(ctx, "row-highlight") ||
            gtk_style_context_has_class(ctx, "zone-highlight")) {
                g_ptr_array_add(active_slaves, GINT_TO_POINTER(i));
        }
    }

    if (active_slaves->len == 0) {
        LOG_ERROR("❌ No Slave Selected to Stop");
        g_ptr_array_free(active_slaves, TRUE);
        return;
    }

    gchar *log = g_strdup_printf("🛑 Stopping movement for %u slave%s",
                                 active_slaves->len,
                                 (active_slaves->len == 1 ? "" : "s"));
    LOG_WARN(log);
    g_free(log);

    g_ptr_array_free(active_slaves, TRUE);
}

void open_position_test(GtkButton *button, gpointer user_data) {
    (void)button;
    (void)user_data;

    app_load_css(NULL);

    const gchar *type = app.selection_type; // Assume this is either "HMRS" or "Matrix"

    char exe_dir[MAX_PATH];
    GetModuleFileNameA(NULL, exe_dir, MAX_PATH);
    PathRemoveFileSpecA(exe_dir);

    char ui_path[MAX_PATH];
    _snprintf_s(ui_path, MAX_PATH, _TRUNCATE, "%s\\builders\\velocityTest.ui", exe_dir);
    GtkBuilder *builder = load_ui(ui_path);

    pos_win = GTK_WIDGET(gtk_builder_get_object(builder, "position_test_window"));
    gtk_builder_connect_signals(builder, NULL);

    attach_shared_title_with_back(builder, "title_placeholder", "<b>Position Test</b>", G_CALLBACK(on_back_clicked), NULL);
    g_signal_connect(gtk_builder_get_object(builder, "back_button"), "clicked", G_CALLBACK(on_back_clicked), NULL);

    GtkBuilder *shared_builder = attach_shared_panel(builder, "right_panel");
    app_connect_shared_right_panel_signals(); // Set up signal callbacks

    app.log_view = GTK_WIDGET(gtk_builder_get_object(shared_builder, "logs_view"));
    app.log_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(app.log_view));
    setup_log_tags(app.log_buffer);

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
    update_position_test_layout(builder, type);

    slave_total_available = app.total_slaves;
    generate_slave_buttons(builder,
                       "slave_grid",
                       app.selection_type,
                       on_slave_button_clicked_position,
                       NULL);

    if (g_strcmp0(type, "Matrix") == 0) {
        generate_column_buttons(builder);
        generate_row_buttons(builder);
    } else {
        generate_hmrs_zone_buttons(builder);
    }


    // position operate
    GtkEntry *target_position_entry = GTK_ENTRY(gtk_builder_get_object(builder, "target_position_entry"));
    GtkEntry *target_speed_entry = GTK_ENTRY(gtk_builder_get_object(builder, "target_speed_entry"));
    GtkWidget *start_btn = GTK_WIDGET(gtk_builder_get_object(builder, "start_btn"));
    GtkStyleContext *srt_ctx = gtk_widget_get_style_context(GTK_WIDGET(start_btn));
    gtk_style_context_add_class(srt_ctx, "start-button");

    GtkButton *stop_btn = GTK_BUTTON(gtk_builder_get_object(builder, "stop_btn"));
    GtkStyleContext *stp_ctx = gtk_widget_get_style_context(GTK_WIDGET(stop_btn));
    gtk_style_context_add_class(stp_ctx, "stop-button");

    StartPositionData *start_data = g_malloc(sizeof(StartPositionData));
    start_data->position_entry = GTK_ENTRY(target_position_entry);
    start_data->speed_entry = GTK_ENTRY(target_speed_entry);
    start_data->start_button = GTK_WIDGET(start_btn);

    g_signal_connect(start_btn, "clicked", G_CALLBACK(on_start_position_clicked), start_data);

    g_signal_connect(stop_btn, "clicked", G_CALLBACK(on_stop_position_btn_clicked), start_data);

    gtk_widget_hide(app.main_window);
    gtk_widget_show_all(pos_win);
}