#include <gtk/gtk.h>
#include "app_context.h"
#include "ethercat_comm.h"
#include <glib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <windows.h>
#include <shlwapi.h>

// Forward declarations for functions in other files
void open_velocity_test(GtkButton *button, gpointer user_data);
void open_position_test(GtkButton *button, gpointer user_data);
void open_template(GtkButton *button, gpointer user_data);

gboolean showing_logs = FALSE;

// Utility to get multiple widgets from builder
void get_widgets(GtkBuilder *builder, GtkWidget **widgets, const char *ids[], int count) {
    for (int i = 0; i < count; i++) {
        widgets[i] = GTK_WIDGET(gtk_builder_get_object(builder, ids[i]));
    }
}

// Utility to set sensitive (enabled/disabled) state
void set_widgets_sensitive(GtkWidget **widgets, int count, gboolean state) {
    for (int i = 0; i < count; i++) {
        if (widgets[i]) {
            gtk_widget_set_sensitive(widgets[i], state);
        }
    }
}

// Utility to show/hide widget groups
void set_widgets_visible(GtkWidget **widgets, int count, gboolean visible) {
    for (int i = 0; i < count; i++) {
        if (widgets[i]) {
            if (visible)
                gtk_widget_show(widgets[i]);
            else
                gtk_widget_hide(widgets[i]);
        }
    }
}

void toggle_log_view(GtkButton *button, gpointer user_data) {
    GtkBuilder *builder = GTK_BUILDER(user_data);
    GtkWidget *log_scroll = GTK_WIDGET(gtk_builder_get_object(builder, "log_scroll"));
    GtkTextView *log_view = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "log_view"));
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(log_view);

    const char *button_ids[] = {
        "velocity_test_button",
        "position_test_button",
        "run_templates_button",
        "exit_button"
    };
    GtkWidget *buttons[4];
    get_widgets(builder, buttons, button_ids, 4);

    if (!showing_logs) {
        const char *log_text =
            "Lorem ipsum dolor sit amet, consectetur adipiscing elit. "
            "Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. "
            "Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.";

        gtk_text_buffer_set_text(buffer, log_text, -1);
        set_widgets_visible(buttons, 4, FALSE);
        gtk_widget_show(log_scroll);
        gtk_button_set_label(button, "Back");
        showing_logs = TRUE;
    } else {
        gtk_text_buffer_set_text(buffer, "", -1);
        set_widgets_visible(buttons, 4, TRUE);
        gtk_widget_hide(log_scroll);
        gtk_button_set_label(button, "View Logs");
        showing_logs = FALSE;
    }
}

int main(int argc, char *argv[]) {
    gtk_init(&argc, &argv);

    char exe_dir[MAX_PATH];
    GetModuleFileNameA(NULL, exe_dir, MAX_PATH);
    PathRemoveFileSpecA(exe_dir);

    char ui_path[MAX_PATH];
    _snprintf_s(ui_path, MAX_PATH, _TRUNCATE, "%s\\builders\\main.ui", exe_dir);
    GtkBuilder *builder = load_ui(ui_path);
    
    app.main_window = GTK_WIDGET(gtk_builder_get_object(builder, "main_window"));
    if (!app.main_window) {
        g_critical("Failed to load main window");
        return 1;
    }
    gtk_builder_connect_signals(builder, NULL);

    const char *button_ids[] = {
        "velocity_test_button",
        "position_test_button",
        "run_templates_button"
    };
    GtkWidget *buttons[3];
    get_widgets(builder, buttons, button_ids, 3);

    g_signal_connect(buttons[0], "clicked", G_CALLBACK(open_velocity_test), NULL);
    g_signal_connect(buttons[1], "clicked", G_CALLBACK(open_position_test), NULL);
    g_signal_connect(buttons[2], "clicked", G_CALLBACK(open_template), NULL);
    g_signal_connect(gtk_builder_get_object(builder, "exit_button"), "clicked", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(gtk_builder_get_object(builder, "view_logs_button"), "clicked", G_CALLBACK(toggle_log_view), builder);

    GtkWidget *connected_slaves_label = GTK_WIDGET(gtk_builder_get_object(builder, "connected_slaves_label"));
    GtkWidget *log_scroll = GTK_WIDGET(gtk_builder_get_object(builder, "log_scroll"));
    app.log_scroll = log_scroll;
    
    gtk_widget_show_all(app.main_window);
    
    if (log_scroll) {
        gtk_widget_hide(log_scroll);
    }

    if (connected_slaves_label) {
        gtk_label_set_markup(GTK_LABEL(connected_slaves_label), "<big><big><b>Initializing EtherCAT...</b></big></big>");
    }

    gtk_main();

    return 0;
}