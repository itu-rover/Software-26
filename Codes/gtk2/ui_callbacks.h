#ifndef UI_CALLBACKS_H
#define UI_CALLBACKS_H

#include "app_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void on_add_camera_clicked(GtkButton *button, CustomData *data);
void on_remove_camera_clicked(GtkButton *button, CustomData *data);
void on_camera_selection_changed(GtkTreeSelection *selection, CustomData *data);

void on_connect_clicked(GtkButton *button, CustomData *data);
void on_disconnect_clicked(GtkButton *button, CustomData *data);
void on_apply_settings_clicked(GtkButton *btn, CustomData *data);

/* Service management callbacks */
/* void on_create_service_clicked(GtkButton *button, CustomData *data);
void on_start_service_clicked(GtkButton *button, CustomData *data);
void on_stop_service_clicked(GtkButton *button, CustomData *data);
void on_remove_service_clicked(GtkButton *button, CustomData *data);
void on_remote_service_toggled(GtkToggleButton *toggle, CustomData *data); */

void on_remote_service_toggled_details(GtkToggleButton *toggle, CustomData *data);

gboolean main_window_delete_event(GtkWidget *w, GdkEvent *e, CustomData *data);

/* Helpers exposed so other modules can refresh the UI video or details pane */
void switch_video_display(CustomData *data, GtkWidget *new_video_widget_to_show);
void update_details_pane_for_selection(CustomData *data);

#ifdef __cplusplus
}
#endif

#endif /* UI_CALLBACKS_H */ 