#include "slave_selector.h"

GtkWidget *slave_buttons[MAX_SLAVES] = { NULL };
gint slave_total_available = 0;

void generate_slave_buttons(GtkBuilder *builder, const gchar *grid_id,
                            const gchar *type, SlaveClickCallback click_handler,
                            gpointer handler_user_data) {
    GtkGrid *grid = GTK_GRID(gtk_builder_get_object(builder, grid_id));
    if (!grid) {
        g_warning("❌ slave grid with id '%s' not found.", grid_id);
        return;
    }

    gchar mode[16];
    strncpy(mode, type, sizeof(mode));

    const int max_per_type = (g_strcmp0(mode, "Matrix") == 0) ? 18 : 8;
    const int count = (slave_total_available < max_per_type)
                      ? slave_total_available : max_per_type;

    // Clear grid
    GList *children = gtk_container_get_children(GTK_CONTAINER(grid));
    for (GList *iter = children; iter; iter = iter->next)
        gtk_widget_destroy(GTK_WIDGET(iter->data));
    g_list_free(children);

    for (int i = 0; i < count; i++) {
        gchar *label = g_strdup_printf("%d", i);
        GtkWidget *btn = gtk_button_new_with_label(label);
        slave_buttons[i] = btn;

        // 🔧 Set slave index for later retrieval
        g_object_set_data(G_OBJECT(btn), "slave-index", GINT_TO_POINTER(i));

        // Connect using supplied callback
        g_signal_connect(btn, "clicked", G_CALLBACK(click_handler), handler_user_data);

        int row, col;
        if (g_strcmp0(type, "Matrix") == 0) {
            row = i % 3; col = i / 3;
        } else {
            row = i % 2; col = i / 2;
        }

        gtk_grid_attach(grid, btn, col, row, 1, 1);
        g_free(label);
    }

    gtk_widget_show_all(GTK_WIDGET(grid));
}
