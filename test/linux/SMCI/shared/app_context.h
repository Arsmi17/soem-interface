#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <gtk/gtk.h>

typedef struct {
    GtkWidget *main_window;
    GtkWidget *log_scroll;

    // Logging
    GtkWidget *log_view;
    GtkTextBuffer *log_buffer;
   GtkWidget **slave_position_labels;
    GtkWidget *position_container;
    GtkBuilder *main_builder;
    GtkBuilder *velocity_test_builder;


    // Slave management
    gint total_slaves;
    gchar selection_type[16]; // "HMRS" or "Matrix"

    // Shared widgets
    GtkWidget *connected_slaves_label;
    GtkWidget *status_toggle_button;
    GtkWidget *check_status_button;
    GtkWidget *break_toggle_button;
    GtkWidget *reset_fault_btn;

    // Application-wide shared state
    gboolean status_enabled;
    gboolean breaks_enabled;

} AppContext;

// Global instance
extern AppContext app;

typedef struct {
    int slave_id;
    int32_t raw_position;
    float position_meters;
} PositionUpdateData;

// Logging setup and functions
void setup_log_tags(GtkTextBuffer *buffer);
void append_unstyled_log(const char *fmt, ...); // Internal use
void append_colored_log(const char *tag, const char *fmt, ...) ;
void app_load_css(const char *css_path);
void update_position_display(int slave_id, int32_t raw_position, float position_meters);

#define LOG_INFO(fmt, ...)    append_colored_log("log-info", fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)    append_colored_log("log-warning", fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   append_colored_log("log-danger", fmt, ##__VA_ARGS__)
#define LOG_SUCCESS(fmt, ...) append_colored_log("log-success", fmt, ##__VA_ARGS__)
#define LOG_PRIMARY(fmt, ...) append_colored_log("log-primary", fmt, ##__VA_ARGS__)
#define LOG_DEFAULT(fmt, ...) append_unstyled_log(fmt, ##__VA_ARGS__)

#define MATRIX_MAX_SLAVES 18
#define HMRS_MAX_SLAVES    8
#define HOME_POSITION 0
#define END_POSITION 0.9

// UI Loading and attachment
GtkBuilder* load_ui(const char *filename);
GtkBuilder* attach_shared_panel(GtkBuilder *builder, const char *right_panel_id);
void attach_shared_title_with_back(GtkBuilder *builder, const char *placeholder_id, const char *title_text, GCallback back_callback, gpointer user_data);

// Shared right-panel features
void app_connect_shared_right_panel_signals(void);
void app_update_slave_count(int count);



// NEW: Add these function declarations
void disable_velocity_ui_elements(GtkBuilder *builder);
void enable_velocity_ui_elements(GtkBuilder *builder);
void disable_all_slave_buttons(void);
void enable_all_slave_buttons(void);
gboolean on_velocity_mode_complete(gpointer data);

// NEW: Declare the global builder variable
extern GtkBuilder *velocity_test_builder;

#endif