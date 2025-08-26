#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <json-glib/json-glib.h>
#include "app_context.h"
#include <windows.h>
#include <shlwapi.h> 

static GtkWidget *tmpl_win = NULL;
const char *TEMPLATE_DIR = "templates"; // relative path


static const char *IGNORED_FUNCTIONS[] = {
    "delay_with_communication", NULL
};

gboolean is_ignored_function(const char *func) {
    for (int i = 0; IGNORED_FUNCTIONS[i] != NULL; i++) {
        if (g_strcmp0(func, IGNORED_FUNCTIONS[i]) == 0) {
            return TRUE;
        }
    }
    return FALSE;
}

GPtrArray *parse_run_template_functions(const char *filepath) {
    gchar *contents = NULL;
    gsize length = 0;
    GError *error = NULL;

    if (!g_file_get_contents(filepath, &contents, &length, &error)) {
        g_warning("Could not read file: %s", error->message);
        g_error_free(error);
        return NULL;
    }

    GPtrArray *function_names = g_ptr_array_new_with_free_func(g_free);
    gchar **lines = g_strsplit(contents, "\n", -1);
    gboolean in_run_template = FALSE;

    for (gint i = 0; lines[i] != NULL; i++) {
        gchar *line = g_strstrip(lines[i]);

        // Match "void run_template()" or "run_template()" with optional whitespace
        if (!in_run_template && (g_str_has_prefix(line, "void run_template()") || g_str_has_prefix(line, "run_template()"))) {
            in_run_template = TRUE;
            continue;
        }

        if (in_run_template) {
            if (g_str_has_prefix(line, "{"))
                continue;

            if (g_str_has_prefix(line, "}")) {
                break;
            }

            if (g_str_has_suffix(line, ";")) {
                // Remove comments if any
                gchar *semicolon = strchr(line, ';');
                if (!semicolon) continue;

                gchar *comment = strchr(line, '/');
                if (comment) *comment = '\0';

                line = g_strndup(line, semicolon - line); // keep up to ';'
                gchar *func_call = g_strstrip(line);

                if (!is_ignored_function(func_call)) {
                    g_ptr_array_add(function_names, g_strdup(func_call));
                }
                g_free(line);
            }
        }
    }

    g_strfreev(lines);
    g_free(contents);
    return function_names;
}

void generate_template_function_buttons(GtkBuilder *builder, const gchar *group_id, const gchar *template_filename) {
    GtkWidget *function_box = GTK_WIDGET(gtk_builder_get_object(builder, "execution_function_list_box"));
    if (!function_box) {
        g_warning("execution_function_list_box not found.");
        return;
    }

    // Clear any existing buttons
    GList *children = gtk_container_get_children(GTK_CONTAINER(function_box));
    for (GList *iter = children; iter != NULL; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    // Build full path: templates/<group_id>/<template_file>
    gchar *filepath = g_build_filename("templates", group_id, template_filename, NULL);
    GPtrArray *functions = parse_run_template_functions(filepath);
    g_free(filepath);

    if (!functions || functions->len == 0) {
        append_colored_log("⚠ No valid steps found in template.", "log-warning");
        if (functions) g_ptr_array_free(functions, TRUE);
        return;
    }

    for (guint i = 0; i < functions->len; i++) {
        const gchar *func = g_ptr_array_index(functions, i);
        gchar **parts = g_strsplit(func, "_", -1);
        GString *title = g_string_new(NULL);

        for (gint j = 0; parts[j] != NULL; j++) {
            if (strlen(parts[j]) == 0) continue;
            gchar *word = parts[j];
            gchar capitalized[128];
            snprintf(capitalized, sizeof(capitalized), "%c%s", g_ascii_toupper(word[0]), word + 1);
            g_string_append(title, capitalized);
            if (parts[j + 1] != NULL)
                g_string_append_c(title, ' ');
        }

        g_strfreev(parts);

        gchar *btn_label = g_strdup_printf("Step %d : %s", i + 1, title->str);
        g_string_free(title, TRUE);

        GtkWidget *btn = gtk_button_new_with_label(btn_label);
        gtk_box_pack_start(GTK_BOX(function_box), btn, FALSE, FALSE, 2);

        gchar *log_msg = g_strdup_printf("✅ Executed: %s", func);
        g_signal_connect_swapped(btn, "clicked", G_CALLBACK(append_colored_log), log_msg);

        g_free(btn_label);
    }

    gtk_widget_show_all(function_box);
    g_ptr_array_free(functions, TRUE);
}


// TEMPLATE: Button click handler for template selection
void on_run_once_clicked(GtkButton *button, gpointer builder_ptr) {
    (void)button; // Unused parameter
    (void)builder_ptr; // Unused parameter
    append_colored_log("▶ Running selected template ONCE", "log-primary");
    // simulate or call step logic
}

void on_run_loop_clicked(GtkButton *button, gpointer builder_ptr) {
    (void)button; // Unused parameter
    (void)builder_ptr; // Unused parameter
    append_colored_log("🔁 Running selected template in LOOP", "log-primary");
}

void on_emergency_stop_clicked(GtkButton *button, gpointer builder_ptr) {
    (void)button; // Unused parameter
    (void)builder_ptr; // Unused parameter
    append_colored_log("⛔ EMERGENCY stop activated", "log-danger");
}

void on_return_clicked(GtkButton *btn, gpointer user_data) {
    (void)btn;
    GtkBuilder *builder = GTK_BUILDER(user_data);
    GtkStack *stack = GTK_STACK(gtk_builder_get_object(builder, "left_stack"));
    gtk_stack_set_visible_child_name(stack, "select_view");

    // Reset JSON file chooser
    GtkFileChooser *chooser = GTK_FILE_CHOOSER(gtk_builder_get_object(builder, "json_file_button"));
    gtk_file_chooser_unselect_all(chooser);

    append_colored_log("↩ Returned to template selection", "log-info");
}

// LOAD JSOn

void apply_template_execution_view(GtkBuilder *builder, const gchar *label) {
    GtkStack *stack = GTK_STACK(gtk_builder_get_object(builder, "left_stack"));
    gtk_stack_set_visible_child_name(stack, "exec_view");

    // Convert snake_case label → Title Case
    gchar **parts = g_strsplit(label, "_", -1);
    GString *formatted_label = g_string_new(NULL);
    for (gint i = 0; parts[i] != NULL; i++) {
        if (strlen(parts[i]) == 0) continue;
        gchar *word = parts[i];
        g_string_append_printf(formatted_label, "%c%s", g_ascii_toupper(word[0]), word + 1);
        if (parts[i + 1]) g_string_append_c(formatted_label, ' ');
    }
    g_strfreev(parts);

    // Create full markup string with new line and bold
    gchar *markup = g_strdup_printf("Template Name:\n<b>%s</b>", formatted_label->str);
    gtk_label_set_markup(GTK_LABEL(gtk_builder_get_object(builder, "selected_template_label")), markup);
    g_free(markup);
    g_string_free(formatted_label, TRUE);

    GtkButton *run_once = GTK_BUTTON(gtk_builder_get_object(builder, "run_once_button"));
    GtkButton *run_loop = GTK_BUTTON(gtk_builder_get_object(builder, "run_loop_button"));
    GtkWidget *stop_btn = GTK_WIDGET(gtk_builder_get_object(builder, "emergency_stop_button"));
    GtkStyleContext *stop_ctx = gtk_widget_get_style_context(stop_btn);
    gtk_style_context_add_class(stop_ctx, "stop-button");

    GtkWidget *return_btn = GTK_WIDGET(gtk_builder_get_object(builder, "return_button"));
    GtkStyleContext *return_ctx = gtk_widget_get_style_context(return_btn);
    gtk_style_context_add_class(return_ctx, "return-button");


    g_signal_connect(run_once,  "clicked", G_CALLBACK(on_run_once_clicked), builder);
    g_signal_connect(run_loop,  "clicked", G_CALLBACK(on_run_loop_clicked), builder);
    g_signal_connect(return_btn,"clicked", G_CALLBACK(on_return_clicked), builder);
    g_signal_connect(stop_btn,  "clicked", G_CALLBACK(on_emergency_stop_clicked), builder);
}

void on_load_json_btn_clicked(GtkButton *button, gpointer user_data) {
    (void)button;

    GtkBuilder *builder = GTK_BUILDER(user_data);
    GtkWidget *file_chooser = GTK_WIDGET(gtk_builder_get_object(builder, "json_file_button"));
    gchar *filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(file_chooser));

    if (!filename) {
        append_colored_log("⚠ No file selected.", "log-warning");
        return;
    }

    // Ensure selection_type is set first
    if (!(g_strcmp0(app.selection_type, "Matrix") == 0 || g_strcmp0(app.selection_type, "HMRS") == 0)) {
        append_colored_log("❌ Please select a valid group (Matrix or HMRS) before loading JSON.", "log-danger");
        g_free(filename);
        return;
    }

    // Parse JSON file
    JsonParser *parser = json_parser_new();
    GError *error = NULL;
    if (!json_parser_load_from_file(parser, filename, &error)) {
        append_colored_log("❌ Failed to load JSON file.", "log-danger");
        g_error_free(error);
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    JsonNode *root = json_parser_get_root(parser);
    if (!JSON_NODE_HOLDS_OBJECT(root)) {
        append_colored_log("❌ Invalid JSON: root is not an object.", "log-danger");
        g_object_unref(parser);
        g_free(filename);
        return;
    }
    JsonObject *root_obj = json_node_get_object(root);

    // ✅ Type validation (should match selected)
    const gchar *json_type = json_object_get_string_member(root_obj, "type");
    if (!json_type || g_strcmp0(json_type, app.selection_type) != 0) {
        gchar *msg = g_strdup_printf("❌ JSON 'type' must match selected type: %s", app.selection_type);
        append_colored_log(msg, "log-danger");
        g_free(msg);
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    // ✅ Validate total_slaves <= app.total_slaves
    if (!json_object_has_member(root_obj, "total_slaves")) {
        append_colored_log("❌ Missing 'total_slaves' field in JSON.", "log-danger");
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    gint json_total_slaves = json_object_get_int_member(root_obj, "total_slaves");
    if (json_total_slaves > app.total_slaves) {
        gchar *msg = g_strdup_printf("❌ JSON 'total_slaves' (%d) exceeds available (%d).", json_total_slaves, app.total_slaves);
        append_colored_log(msg, "log-danger");
        g_free(msg);
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    // Validate template field
    if (!json_object_has_member(root_obj, "template")) {
        append_colored_log("❌ JSON missing required \"template\" array.", "log-danger");
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    JsonArray *templates = json_object_get_array_member(root_obj, "template");
    if (!templates || json_array_get_length(templates) == 0) {
        append_colored_log("❌ JSON 'template' array is empty or invalid.", "log-danger");
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    JsonObject *first_template = json_array_get_object_element(templates, 0);
    if (!first_template) {
        append_colored_log("❌ Template structure missing inner data.", "log-danger");
        g_object_unref(parser);
        g_free(filename);
        return;
    }
    // --- UI logic: switch to exec_view just like template button ---
    const gchar *template_name = json_object_get_string_member(first_template, "name");
    apply_template_execution_view(builder, template_name ? template_name : "Unnamed Template");

    // --- Generate buttons for each pattern step ---
    GtkWidget *btn_box = GTK_WIDGET(gtk_builder_get_object(builder, "execution_function_list_box"));
    GList *children = gtk_container_get_children(GTK_CONTAINER(btn_box));
    for (GList *iter = children; iter != NULL; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    JsonArray *steps = json_object_get_array_member(first_template, "steps");
    if (!steps || json_array_get_length(steps) == 0) {
        append_colored_log("❌ No steps found in template.", "log-danger");
        g_object_unref(parser);
        g_free(filename);
        return;
    }

    for (guint i = 0; i < json_array_get_length(steps); i++) {
        JsonObject *step = json_array_get_object_element(steps, i);
        if (!step) continue;

        const gchar *pattern_name = json_object_get_string_member(step, "pattern_name");
        if (!pattern_name) continue;

        // Convert snake_case to Title Case
        gchar **parts = g_strsplit(pattern_name, "_", -1);
        GString *title = g_string_new(NULL);
        for (gint j = 0; parts[j] != NULL; j++) {
            if (j > 0) g_string_append_c(title, ' ');
            if (parts[j][0])
                g_string_append_printf(title, "%c%s", g_ascii_toupper(parts[j][0]), parts[j] + 1);
        }
        g_strfreev(parts);

        gchar *btn_label = g_strdup_printf("Step %d : %s", i + 1, title->str);
        GtkWidget *btn = gtk_button_new_with_label(btn_label);
        gtk_box_pack_start(GTK_BOX(btn_box), btn, FALSE, FALSE, 4);

        gchar *log_msg = g_strdup_printf("✅ Executed: %s", pattern_name);
        g_signal_connect_swapped(btn, "clicked", G_CALLBACK(append_colored_log), log_msg);

        g_free(btn_label);
        g_string_free(title, TRUE);
    }

    gtk_widget_show_all(btn_box);
    append_colored_log("✅ JSON and template loaded. Ready to execute patterns!", "log-success");

    g_object_unref(parser);
    g_free(filename);
}

// TEMPLATE: Hook Button Connections After View Switch
void on_template_btn_clicked(GtkButton *button, gpointer user_data) {
    GtkBuilder *builder = GTK_BUILDER(user_data);
    const gchar *label = gtk_button_get_label(button); // e.g., demo

    // Get selected group (Matrix1, HMRS2, etc.)
    GtkComboBox *combo = GTK_COMBO_BOX(gtk_builder_get_object(builder, "slave_column_dropdown"));
    const gchar *group_id = gtk_combo_box_get_active_id(combo); // e.g., "Matrix1"

    if (!group_id) {
        append_colored_log("⚠ No group selected. Cannot load template.", "log-warning");
        return;
    }

    gchar *template_file = g_strdup_printf("%s.c", label);      // demo.c

    apply_template_execution_view(builder, label);
    generate_template_function_buttons(builder, group_id, template_file);

    // Reset JSON file chooser
    GtkFileChooser *chooser = GTK_FILE_CHOOSER(gtk_builder_get_object(builder, "json_file_button"));
    gtk_file_chooser_unselect_all(chooser);

    g_free(template_file);
}

void generate_template_buttons_with_folder(const gchar *folder_name, GtkBuilder *builder) {
    GtkWidget *box = GTK_WIDGET(gtk_builder_get_object(builder, "template_list_box"));
    if (!box) return;

    // Clear previous buttons
    GList *children = gtk_container_get_children(GTK_CONTAINER(box));
    for (GList *iter = children; iter != NULL; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    gchar *folder_path = g_build_filename(TEMPLATE_DIR, folder_name, NULL);
    GDir *dir = g_dir_open(folder_path, 0, NULL);

    if (!dir) {
        gchar *err = g_strdup_printf("⚠ Unable to read directory: %s", folder_path);
        LOG_WARN(err);
        g_free(err);
        g_free(folder_path);
        return;
    }

    const gchar *file;
    while ((file = g_dir_read_name(dir)) != NULL) {
        if (g_str_has_suffix(file, ".c")) {
            gchar *name = g_strndup(file, strlen(file) - 2); // remove .c
            GtkWidget *btn = gtk_button_new_with_label(name);
            gtk_box_pack_start(GTK_BOX(box), btn, FALSE, FALSE, 5);
            g_signal_connect(btn, "clicked", G_CALLBACK(on_template_btn_clicked), builder);
            g_free(name);
        }
    }

    g_dir_close(dir);
    g_free(folder_path);
    gtk_widget_show_all(box);
}

void on_dropdown_changed(GtkComboBoxText *combo, gpointer user_data) {
    const gchar *dir_name = gtk_combo_box_get_active_id(GTK_COMBO_BOX(combo)); // ✅ the actual folder name

    if (!dir_name || strlen(dir_name) == 0) {
        LOG_WARN("⚠ No template group ID selected.");
        return;
    }

    const gchar *label = gtk_combo_box_text_get_active_text(combo);
    gchar *log = g_strdup_printf("🧠 Selected Slave Group: %s", label);
    LOG_PRIMARY(log);
    g_free(log);

    // ✅ Use directory-safe ID for actual lookup
    generate_template_buttons_with_folder(dir_name, GTK_BUILDER(user_data));
}

static void on_back_clicked(GtkButton *btn, gpointer user_data) {
    (void)user_data;
    (void)btn;
    gtk_widget_hide(tmpl_win);
    gtk_widget_show_all(app.main_window);
}

void populate_slave_group_dropdown(GtkBuilder *builder) {
    const gchar *type = app.selection_type;
    gint total = app.total_slaves;

    GtkComboBoxText *dropdown = GTK_COMBO_BOX_TEXT(gtk_builder_get_object(builder, "slave_column_dropdown"));
    if (!dropdown) {
        LOG_ERROR("Dropdown 'slave_column_dropdown' not found.");
        return;
    }

    // Reset existing entries and show
    gtk_combo_box_text_remove_all(dropdown);
    gtk_widget_show(GTK_WIDGET(dropdown));

    if (!(g_strcmp0(type, "Matrix") == 0 || g_strcmp0(type, "HMRS") == 0)) {
        gtk_widget_hide(GTK_WIDGET(dropdown));
        return;
    }

    if (g_strcmp0(type, "Matrix") == 0) {
        int num_columns = total / 3;
        for (int i = 0; i < num_columns; i++) {
            // Instead of group-specific range, we show full range (0 to last)
            int end_slave = (i + 1) * 3 - 1;
            if (end_slave >= total) end_slave = total - 1;

            gchar *id = g_strdup_printf("Matrix%d", i + 1);
            gchar *label = g_strdup_printf("Matrix%d (0 to %d)", i + 1, end_slave);

            gtk_combo_box_text_append(dropdown, id, label);
            g_free(id);
            g_free(label);
        }
    } else if (g_strcmp0(type, "HMRS") == 0) {
        for (int i = 0; i < 4; i++) {
            int count = (i + 1) * 2;
            if (count > total)
                break;

            int start = 0;
            int end = count - 1;

            gchar *id = g_strdup_printf("HMRS%d", i + 1);
            gchar *label = g_strdup_printf("HMRS%d (%d to %d)", i + 1, start, end);

            gtk_combo_box_text_append(dropdown, id, label);

            g_free(id);
            g_free(label);
        }
    }

    // Set last option as selected and manually trigger change
    GtkTreeModel *model = gtk_combo_box_get_model(GTK_COMBO_BOX(dropdown));
    gint num_items = gtk_tree_model_iter_n_children(model, NULL);
    if (num_items > 0) {
        gtk_combo_box_set_active(GTK_COMBO_BOX(dropdown), num_items - 1);
        on_dropdown_changed(GTK_COMBO_BOX_TEXT(dropdown), builder);  // ⬅️ manually trigger once
    }

    // Ensure signal is connected
    g_signal_connect(dropdown, "changed", G_CALLBACK(on_dropdown_changed), builder);
}

void open_template(GtkButton *button, gpointer user_data) {
    (void)user_data;
    (void)button;
    
    app_load_css(NULL);

    char exe_dir[MAX_PATH];
    GetModuleFileNameA(NULL, exe_dir, MAX_PATH);
    PathRemoveFileSpecA(exe_dir);

    char ui_path[MAX_PATH];
    _snprintf_s(ui_path, MAX_PATH, _TRUNCATE, "%s\\builders\\velocityTest.ui", exe_dir);
    GtkBuilder *builder = load_ui(ui_path);

    tmpl_win = GTK_WIDGET(gtk_builder_get_object(builder, "templates_window"));
    gtk_builder_connect_signals(builder, NULL);

    g_signal_connect(gtk_builder_get_object(builder, "back_button"), "clicked", G_CALLBACK(on_back_clicked), NULL);

    attach_shared_title_with_back(builder, "title_placeholder", "<b>Template Execution</b>", G_CALLBACK(on_back_clicked), NULL);
    g_signal_connect(gtk_builder_get_object(builder, "back_button"), "clicked", G_CALLBACK(on_back_clicked), NULL);

    GtkBuilder *shared_builder = attach_shared_panel(builder, "right_panel");
    app_connect_shared_right_panel_signals(); // Set up signal callbacks

    populate_slave_group_dropdown(builder);

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

    // template page controls
    GtkWidget *file_chooser = GTK_WIDGET(gtk_builder_get_object(builder, "json_file_button"));
    GtkFileFilter *json_filter = gtk_file_filter_new();
    gtk_file_filter_set_name(json_filter, "JSON files");
    gtk_file_filter_add_pattern(json_filter, "*.json");  // Important!
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(file_chooser), json_filter);
    gtk_file_chooser_set_filter(GTK_FILE_CHOOSER(file_chooser), json_filter);  // Optional: make it default
    GtkWidget *json_btn = GTK_WIDGET(gtk_builder_get_object(builder, "load_json_btn"));
    g_signal_connect(json_btn, "clicked", G_CALLBACK(on_load_json_btn_clicked), builder);
    // generate_template_buttons(builder);

    // g_signal_connect(GTK_WIDGET(dropdown), "changed", G_CALLBACK(on_dropdown_changed), NULL);

    gtk_widget_hide(app.main_window);
    gtk_widget_show_all(tmpl_win);
}