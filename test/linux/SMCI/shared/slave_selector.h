#ifndef SLAVE_SELECTOR_H
#define SLAVE_SELECTOR_H

#include <gtk/gtk.h>

#define MAX_SLAVES 32

// Global slave button widgets
extern GtkWidget *slave_buttons[MAX_SLAVES];
extern int slave_total_available;

// Callback type for slave click events
typedef void (*SlaveClickCallback)(GtkButton *button, gpointer user_data);

// Generate slave buttons (reusable across modules)
void generate_slave_buttons(GtkBuilder *builder,
                            const char *grid_id,
                            const char *type,
                            SlaveClickCallback callback,
                            gpointer user_data);

#endif
