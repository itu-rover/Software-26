#include "ui_callbacks.h"
#include "ui_setup.h"
#include "systemd_service_manager.h"
// #include "service_dialog.h" 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <gtk/gtk.h>

// Simple stub functions for missing dependencies
static void free_camera_stream_info(CameraStreamInfo *stream) {
    if (!stream) return;
    g_free(stream->name);
    g_free(stream->remote_ip);
    g_free(stream->encoding);
    g_free(stream->device_path);
    g_free(stream->service_name);
    g_free(stream->remote_host);
    g_free(stream->remote_user);
    g_free(stream->flip_method);
    g_free(stream);
}

static void add_camera_to_app_state(CustomData *data, const gchar *name, const gchar *ip, 
                                   gint stream_port, gint ctrl_port, const gchar *encoding,
                                   gint width, gint height, gint framerate) {
    CameraStreamInfo *stream = g_new0(CameraStreamInfo, 1);
    stream->name = g_strdup(name);
    stream->remote_ip = g_strdup(ip);
    stream->streaming_port = stream_port;
    stream->control_port = ctrl_port;
    stream->encoding = g_strdup(encoding);
    stream->width = width;
    stream->height = height;
    stream->framerate = framerate;
    stream->connected = FALSE;
    stream->app_data = data;
    
    // Initialize service management fields
    stream->device_path = g_strdup("/dev/video0");         // Default device path
    stream->service_name = NULL;                           // Will be generated on connect
    stream->service_template = SERVICE_TEMPLATE_SW_H264;   // Default template
    stream->input_format = INPUT_FORMAT_YUYV;            // Default input format
    stream->bitrate = 1000000;                           // Default 1Mbps
    stream->flip_method = g_strdup("none");              // Default to no flip
    stream->is_remote_service = FALSE;                   // Default to local
    stream->remote_host = g_strdup("192.168.1.100");    // Default remote host
    stream->remote_user = g_strdup("ubuntu");            // Default remote user
    stream->service_status = SERVICE_STATUS_UNKNOWN;       // Initial status
    
    // Add to list
    data->camera_streams = g_list_append(data->camera_streams, stream);
    
    // Add to tree view
    gchar *info = g_strdup_printf("%s:%d", ip, stream_port);
    gtk_list_store_append(data->camera_list_store, &stream->iter);
    gtk_list_store_set(data->camera_list_store, &stream->iter,
                      COLUMN_NAME, name,
                      COLUMN_INFO, info,
                      COLUMN_STATUS, "Disconnected",
                      COLUMN_POINTER, stream,
                      -1);
    g_free(info);
}

void switch_video_display(CustomData *data, GtkWidget *new_video_widget_to_show)
{
    // Simplified - just print debug info
    if (new_video_widget_to_show) {
        g_print("DEBUG: Would show video widget %p\n", new_video_widget_to_show);
    } else {
        g_print("DEBUG: Would hide video display\n");
    }
    (void)data; // Suppress unused parameter warning
}

void update_details_pane_for_selection(CustomData *data)
{
    CameraStreamInfo *s = data->selected_stream;

    if (s) {
        gtk_widget_set_sensitive(data->details_pane_grid, TRUE);
        gchar *title = g_markup_printf_escaped("<b>%s</b>", s->name);
        gtk_label_set_markup(GTK_LABEL(data->label_cam_name_dynamic), title);
        g_free(title);

        gtk_entry_set_text(GTK_ENTRY(data->entry_cam_ip), s->remote_ip);
        gtk_entry_set_text(GTK_ENTRY(data->entry_streaming_port), g_strdup_printf("%d", s->streaming_port));
        gtk_entry_set_text(GTK_ENTRY(data->entry_width), g_strdup_printf("%d", s->width));
        gtk_entry_set_text(GTK_ENTRY(data->entry_height), g_strdup_printf("%d", s->height));
        gtk_entry_set_text(GTK_ENTRY(data->entry_framerate), g_strdup_printf("%d", s->framerate));
        gtk_entry_set_text(GTK_ENTRY(data->entry_control_port), g_strdup_printf("%d", s->control_port));
        gtk_entry_set_text(GTK_ENTRY(data->entry_bitrate_details), g_strdup_printf("%d", s->bitrate));

        // Set device path, template, input format, remote details
        gtk_entry_set_text(GTK_ENTRY(data->entry_device_path), s->device_path ? s->device_path : "");
        gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_service_template), s->service_template);
        gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_input_format), s->input_format);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(data->check_remote_service), s->is_remote_service);
        gtk_entry_set_text(GTK_ENTRY(data->entry_remote_host), s->remote_host ? s->remote_host : "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_remote_user), s->remote_user ? s->remote_user : "");
        // Manually trigger the toggle handler to set sensitivity of remote host/user entries
        on_remote_service_toggled_details(GTK_TOGGLE_BUTTON(data->check_remote_service), data);

        // Set encoding combo
        if (g_strcmp0(s->encoding, "H264") == 0)
            gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_encoding), 0);
        else if (g_strcmp0(s->encoding, "H265") == 0)
            gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_encoding), 1);
        else
            gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_encoding), 2);

        // Set flip method combo
        const gchar *flip_options[] = {"none", "clockwise", "rotate-180", "counter-clockwise", "horizontal-flip", "vertical-flip", "upper-left-diagonal", "upper-right-diagonal"};
        for (gint i = 0; i < G_N_ELEMENTS(flip_options); ++i) {
            if (g_strcmp0(s->flip_method, flip_options[i]) == 0) {
                gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_flip_method_details), i);
                break;
            }
        }

        gtk_widget_set_sensitive(data->button_connect, !s->connected);
        gtk_widget_set_sensitive(data->button_disconnect, s->connected);
        gtk_widget_set_sensitive(data->button_apply_settings, TRUE);

        const gchar *status_txt = s->connected ? "Status: Connected" : "Status: Disconnected";
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), status_txt);
    } else {
        /* No stream selected */
        gtk_widget_set_sensitive(data->details_pane_grid, FALSE);
        gtk_label_set_markup(GTK_LABEL(data->label_cam_name_dynamic), "<b>No Camera Selected</b>");
        gtk_entry_set_text(GTK_ENTRY(data->entry_cam_ip), "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_streaming_port), "");
        gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_encoding), -1);
        gtk_entry_set_text(GTK_ENTRY(data->entry_width), "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_height), "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_framerate), "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_control_port), "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_bitrate_details), "");
        gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_flip_method_details), -1);
        // Clear service-related fields as well
        gtk_entry_set_text(GTK_ENTRY(data->entry_device_path), "");
        gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_service_template), -1);
        gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_input_format), -1);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(data->check_remote_service), FALSE);
        gtk_entry_set_text(GTK_ENTRY(data->entry_remote_host), "");
        gtk_entry_set_text(GTK_ENTRY(data->entry_remote_user), "");

        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Select a camera from the list or add a new one.");
    }
}

void on_camera_selection_changed(GtkTreeSelection *sel, CustomData *data)
{
    GtkTreeModel *model;
    GtkTreeIter   iter;
    CameraStreamInfo *new_s = NULL;

    if (gtk_tree_selection_get_selected(sel, &model, &iter)) {
        gtk_tree_model_get(model, &iter, COLUMN_POINTER, &new_s, -1);
    }

    if (data->selected_stream == new_s)
        return; /* No change */

    data->selected_stream = new_s;
    update_details_pane_for_selection(data);
}

// Simple add camera dialog
typedef struct {
    GtkWidget *dialog;
    GtkWidget *entry_name;
    GtkWidget *entry_ip;
    GtkWidget *entry_stream_port;
    GtkWidget *entry_ctrl_port;
    GtkWidget *combo_encoding;
    GtkWidget *entry_width;
    GtkWidget *entry_height;
    GtkWidget *entry_framerate;
    CustomData *app_data;
} SimpleAddCameraDialog;

static void on_simple_add_camera_response(GtkDialog *dlg, gint response, SimpleAddCameraDialog *d)
{
    (void)dlg;
    if (response == GTK_RESPONSE_OK) {
        const gchar *name = gtk_entry_get_text(GTK_ENTRY(d->entry_name));
        const gchar *ip = gtk_entry_get_text(GTK_ENTRY(d->entry_ip));
        gint stream_port = atoi(gtk_entry_get_text(GTK_ENTRY(d->entry_stream_port)));
        gint ctrl_port = atoi(gtk_entry_get_text(GTK_ENTRY(d->entry_ctrl_port)));
        gchar *encoding = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(d->combo_encoding));
        gint width = atoi(gtk_entry_get_text(GTK_ENTRY(d->entry_width)));
        gint height = atoi(gtk_entry_get_text(GTK_ENTRY(d->entry_height)));
        gint framerate = atoi(gtk_entry_get_text(GTK_ENTRY(d->entry_framerate)));

        if (g_str_equal(name, "") || g_str_equal(ip, "") || stream_port <= 0 || ctrl_port <= 0 ||
            width <= 0 || height <= 0 || framerate <= 0 || !encoding) {
            GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(d->dialog),
                                                    GTK_DIALOG_DESTROY_WITH_PARENT | GTK_DIALOG_MODAL,
                                                    GTK_MESSAGE_ERROR, GTK_BUTTONS_OK,
                                                    "All fields must be filled correctly and numeric fields > 0.");
            gtk_dialog_run(GTK_DIALOG(err));
            gtk_widget_destroy(err);
            if (encoding) g_free(encoding);
            return;
        }

        add_camera_to_app_state(d->app_data, name, ip, stream_port, ctrl_port,
                                encoding, width, height, framerate);
        
        if (encoding) g_free(encoding);
    }
    
    gtk_widget_destroy(d->dialog);
    g_free(d);
}

void on_add_camera_clicked(GtkButton *unused, CustomData *data)
{
    (void)unused;
    
    SimpleAddCameraDialog *dlg = g_new0(SimpleAddCameraDialog, 1);
    dlg->app_data = data;

    dlg->dialog = gtk_dialog_new_with_buttons("Add New Camera",
                                             GTK_WINDOW(data->main_window),
                                             GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                             "_OK", GTK_RESPONSE_OK,
                                             "_Cancel", GTK_RESPONSE_CANCEL,
                                             NULL);
    GtkWidget *content = gtk_dialog_get_content_area(GTK_DIALOG(dlg->dialog));
    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(grid), 10);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
    gtk_container_add(GTK_CONTAINER(content), grid);

    int row = 0;
    
    // Name
    dlg->entry_name = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_name), "New Camera");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Name:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_name, 1, row++, 1, 1);

    // IP
    dlg->entry_ip = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_ip), "192.168.1.100");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("IP Address:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_ip, 1, row++, 1, 1);

    // Stream Port
    dlg->entry_stream_port = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_stream_port), "5000");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Streaming Port:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_stream_port, 1, row++, 1, 1);

    // Control Port
    dlg->entry_ctrl_port = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_ctrl_port), "9999");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Control Port:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_ctrl_port, 1, row++, 1, 1);

    // Encoding
    dlg->combo_encoding = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(dlg->combo_encoding), "H264");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(dlg->combo_encoding), "H265");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(dlg->combo_encoding), "MJPEG");
    gtk_combo_box_set_active(GTK_COMBO_BOX(dlg->combo_encoding), 0);
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Encoding:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->combo_encoding, 1, row++, 1, 1);

    // Width
    dlg->entry_width = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_width), "640");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Width:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_width, 1, row++, 1, 1);

    // Height
    dlg->entry_height = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_height), "480");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Height:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_height, 1, row++, 1, 1);

    // Framerate
    dlg->entry_framerate = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(dlg->entry_framerate), "30");
    gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Framerate:"), 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), dlg->entry_framerate, 1, row++, 1, 1);

    gtk_widget_show_all(dlg->dialog);
    g_signal_connect(dlg->dialog, "response", G_CALLBACK(on_simple_add_camera_response), dlg);
}

void on_remove_camera_clicked(GtkButton *btn, CustomData *data)
{
    (void)btn;
    if (!data->selected_stream) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Error: No camera selected to remove.");
        return;
    }

    CameraStreamInfo *s = data->selected_stream;

    // If connected, try to disconnect and stop service first
    if (s->connected && s->service_name) {
        g_print("INFO: Camera %s is connected. Stopping service %s before removal.\n", s->name, s->service_name);
        GError *stop_error = NULL;
        stop_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &stop_error);
        if (stop_error) {
            g_warning("Failed to stop service %s during camera removal: %s", s->service_name, stop_error->message);
            g_error_free(stop_error);
            // Not returning, will still try to remove camera state
        }
        // Also remove the service file itself
        GError *remove_error = NULL;
        remove_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &remove_error);
        if (remove_error) {
            g_warning("Failed to remove service files for %s during camera removal: %s", s->service_name, remove_error->message);
            g_error_free(remove_error);
        }
        g_free(s->service_name);
        s->service_name = NULL;
        s->connected = FALSE; // Mark as disconnected
    }

    gchar *msg = g_strdup_printf("Are you sure you want to remove camera '%s'?", s->name);
    GtkWidget *dlg = gtk_message_dialog_new(GTK_WINDOW(data->main_window),
                                            GTK_DIALOG_DESTROY_WITH_PARENT | GTK_DIALOG_MODAL,
                                            GTK_MESSAGE_QUESTION, GTK_BUTTONS_YES_NO,
                                            "%s", msg);
    g_free(msg);
    gint res = gtk_dialog_run(GTK_DIALOG(dlg));
    gtk_widget_destroy(dlg);
    if (res != GTK_RESPONSE_YES)
        return;

    /* Deselection & UI update */
    data->selected_stream = NULL;
    gtk_tree_selection_unselect_iter(gtk_tree_view_get_selection(GTK_TREE_VIEW(data->camera_list_treeview)), &s->iter);
    update_details_pane_for_selection(data);

    gtk_list_store_remove(data->camera_list_store, &s->iter);
    data->camera_streams = g_list_remove(data->camera_streams, s);
    free_camera_stream_info(s);

    gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Info: Camera removed.");
}

void on_connect_clicked(GtkButton *btn, CustomData *data)
{
    (void)btn;
    if (!data->selected_stream) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Error: No camera selected to connect.");
        return;
    }
    
    CameraStreamInfo *s = data->selected_stream;
    if (s->connected) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Info: Already connected.");
        return;
    }

    GError *error = NULL;

    // 1. If a service already exists for this camera, stop and remove it first.
    if (s->service_name) {
        g_print("INFO: Existing service %s found for camera %s. Stopping and removing it first.\n", s->service_name, s->name);
        stop_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &error);
        if (error) {
            g_warning("Failed to stop existing service %s: %s. Continuing to recreate.", s->service_name, error->message);
            g_error_free(error);
            error = NULL;
        }
        remove_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &error);
        if (error) {
            g_warning("Failed to remove existing service %s: %s. Continuing to recreate.", s->service_name, error->message);
            g_error_free(error);
            error = NULL;
        }
        g_free(s->service_name);
        s->service_name = NULL;
    }

    // 2. Create ServiceConfig from CameraStreamInfo
    ServiceConfig *config = service_config_new();
    
    // Sanitize camera name for service name using GRegex
    GRegex *regex = g_regex_new("[^a-zA-Z0-9_]", 0, 0, NULL);
    gchar *sanitized_name = g_regex_replace_literal(regex, s->name, -1, 0, "", 0, NULL);
    g_regex_unref(regex);
    
    config->service_name = g_strdup_printf("cam_%s.service", sanitized_name ? sanitized_name : "default");
    g_free(sanitized_name);

    config->device_path = g_strdup(s->device_path);
    config->target_ip = g_strdup(s->remote_ip);
    config->port = s->streaming_port;
    config->width = s->width;
    config->height = s->height;
    config->framerate = s->framerate;
    config->bitrate = s->bitrate;
    config->template_type = s->service_template;
    config->input_format = s->input_format;
    config->flip_method = s->flip_method ? g_strdup(s->flip_method) : g_strdup("none");
    config->is_remote = s->is_remote_service;
    if (s->is_remote_service) {
        config->remote_host = g_strdup(s->remote_host);
        config->remote_user = g_strdup(s->remote_user);
    }

    g_print("INFO: Creating service %s for camera %s...\n", config->service_name, s->name);
    gtk_label_set_text(GTK_LABEL(data->label_status_selected), 
                      g_strdup_printf("Status: Creating service %s...", config->service_name));

    // 3. Create the systemd service
    if (!create_systemd_service(config, &error)) {
        gchar *err_msg = g_strdup_printf("Error: Failed to create service %s: %s", 
                                        config->service_name, 
                                        error ? error->message : "Unknown error");
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), err_msg);
        g_warning("%s", err_msg);
        g_free(err_msg);
        if (error) g_error_free(error);
        service_config_free(config);
        return;
    }

    // Store the generated service name back to the stream info
    if (s->service_name) g_free(s->service_name);
    s->service_name = g_strdup(config->service_name);

    g_print("INFO: Starting service %s for camera %s...\n", s->service_name, s->name);
    gtk_label_set_text(GTK_LABEL(data->label_status_selected), 
                      g_strdup_printf("Status: Starting service %s...", s->service_name));

    // 4. Start the systemd service
    if (!start_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &error)) {
        gchar *err_msg = g_strdup_printf("Error: Failed to start service %s: %s", 
                                        s->service_name, 
                                        error ? error->message : "Unknown error");
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), err_msg);
        g_warning("%s", err_msg);
        g_free(err_msg);
        if (error) g_error_free(error);
        
        // Attempt to remove the just-created service files if start fails
        GError *remove_err = NULL;
        remove_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &remove_err);
        if(remove_err) {
            g_warning("Failed to cleanup service files for %s after start failure: %s", 
                     s->service_name, remove_err->message);
            g_error_free(remove_err);
        }
        g_free(s->service_name);
        s->service_name = NULL;
        service_config_free(config);
        return;
    }

    service_config_free(config);

    // 5. Update UI
    s->connected = TRUE;
    gtk_list_store_set(data->camera_list_store, &s->iter, COLUMN_STATUS, "Connected", -1);
    gtk_label_set_text(GTK_LABEL(data->label_status_selected), 
                      g_strdup_printf("Status: Connected (Service: %s)", s->service_name));
    update_details_pane_for_selection(data);
    
    g_print("INFO: Camera %s connected. Service %s started.\n", s->name, s->service_name);
}

void on_disconnect_clicked(GtkButton *btn, CustomData *data)
{
    (void)btn;
    if (!data->selected_stream || !data->selected_stream->connected) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Info: Not connected or none selected.");
        return;
    }

    CameraStreamInfo *s = data->selected_stream;
    
    if (!s->service_name) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Error: No service name associated with this stream. Cannot disconnect.");
        g_warning("Attempted to disconnect camera %s but no service_name is set.", s->name);
        // Mark as disconnected anyway in UI to allow attempt to reconnect
        s->connected = FALSE;
        gtk_list_store_set(data->camera_list_store, &s->iter, COLUMN_STATUS, "Disconnected", -1);
        update_details_pane_for_selection(data);
        return;
    }

    GError *error = NULL;
    g_print("INFO: Stopping service %s for camera %s...\n", s->service_name, s->name);
    gtk_label_set_text(GTK_LABEL(data->label_status_selected), g_strdup_printf("Status: Stopping service %s...", s->service_name));

    if (!stop_systemd_service(s->service_name, s->is_remote_service, s->remote_host, s->remote_user, &error)) {
        gchar *err_msg = g_strdup_printf("Error: Failed to stop service %s: %s", s->service_name, error ? error->message : "Unknown error");
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), err_msg);
        g_warning("%s", err_msg);
        g_free(err_msg);
        if (error) g_error_free(error);
        // Don't return, still update UI to disconnected state to allow re-connect attempt
    } else {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Status: Disconnected.");
        g_print("INFO: Service %s stopped successfully.\n", s->service_name);
    }
    
    s->connected = FALSE;
    gtk_list_store_set(data->camera_list_store, &s->iter, COLUMN_STATUS, "Disconnected", -1);
    update_details_pane_for_selection(data); // Enables connect, disables disconnect
}

void on_apply_settings_clicked(GtkButton *btn, CustomData *data)
{
    (void)btn;
    if (!data->selected_stream) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Error: No camera selected to apply settings.");
        return;
    }

    CameraStreamInfo *s = data->selected_stream;

    gint new_width = atoi(gtk_entry_get_text(GTK_ENTRY(data->entry_width)));
    gint new_height = atoi(gtk_entry_get_text(GTK_ENTRY(data->entry_height)));
    gint new_fps = atoi(gtk_entry_get_text(GTK_ENTRY(data->entry_framerate)));
    gint new_stream_pt = atoi(gtk_entry_get_text(GTK_ENTRY(data->entry_streaming_port)));
    gint new_ctrl_pt = atoi(gtk_entry_get_text(GTK_ENTRY(data->entry_control_port)));
    gchar *new_enc = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(data->combo_encoding));
    gint new_bitrate = atoi(gtk_entry_get_text(GTK_ENTRY(data->entry_bitrate_details)));
    gchar *new_flip_method = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details));

    // Get service related settings
    const gchar *new_device_path = gtk_entry_get_text(GTK_ENTRY(data->entry_device_path));
    gint new_service_template_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(data->combo_service_template));
    gint new_input_format_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(data->combo_input_format));
    gboolean new_is_remote = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(data->check_remote_service));
    const gchar *new_remote_host = gtk_entry_get_text(GTK_ENTRY(data->entry_remote_host));
    const gchar *new_remote_user = gtk_entry_get_text(GTK_ENTRY(data->entry_remote_user));

    if (new_width <= 0 || new_height <= 0 || new_fps <= 0 || new_stream_pt <= 0 || new_ctrl_pt <= 0 || !new_enc || new_bitrate <= 0 || !new_flip_method ||
        g_str_equal(new_device_path, "") || new_service_template_idx < 0 || new_input_format_idx < 0 ||
        (new_is_remote && (g_str_equal(new_remote_host, "") || g_str_equal(new_remote_user, "")))) {
        gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Error: Invalid settings. Please fill all fields.");
        if (new_enc) g_free(new_enc);
        if (new_flip_method) g_free(new_flip_method);
        return;
    }

    // Update settings
    s->width = new_width;
    s->height = new_height;
    s->framerate = new_fps;
    s->streaming_port = new_stream_pt;
    s->control_port = new_ctrl_pt;
    if (s->encoding) g_free(s->encoding);
    s->encoding = g_strdup(new_enc);
    s->bitrate = new_bitrate;
    if (s->flip_method) g_free(s->flip_method);
    s->flip_method = g_strdup(new_flip_method);

    // Update service related settings
    if(s->device_path) g_free(s->device_path);
    s->device_path = g_strdup(new_device_path);
    s->service_template = new_service_template_idx;
    s->input_format = new_input_format_idx;
    s->is_remote_service = new_is_remote;
    if(s->remote_host) g_free(s->remote_host);
    s->remote_host = g_strdup(new_remote_host);
    if(s->remote_user) g_free(s->remote_user);
    s->remote_user = g_strdup(new_remote_user);

    // Update the info column in the tree view
    gchar *inf = g_strdup_printf("%s:%d", s->remote_ip, s->streaming_port);
    gtk_list_store_set(data->camera_list_store, &s->iter, COLUMN_INFO, inf, -1);
    g_free(inf);

    gtk_label_set_text(GTK_LABEL(data->label_status_selected), "Info: Settings updated.");
    
    if (new_enc) g_free(new_enc);
    if (new_flip_method) g_free(new_flip_method);
    g_print("DEBUG: Applied settings: %dx%d@%dfps, encoding: %s, bitrate: %d, flip: %s, device: %s, template: %d, input: %d, remote: %d, host: %s, user: %s\n", 
            new_width, new_height, new_fps, s->encoding, s->bitrate, s->flip_method, 
            s->device_path, s->service_template, s->input_format, s->is_remote_service, s->remote_host, s->remote_user);
}

void on_remote_service_toggled_details(GtkToggleButton *toggle, CustomData *data) {
    gboolean is_remote = gtk_toggle_button_get_active(toggle);
    gtk_widget_set_sensitive(data->entry_remote_host, is_remote);
    gtk_widget_set_sensitive(data->entry_remote_user, is_remote);
}

gboolean main_window_delete_event(GtkWidget *w, GdkEvent *e, CustomData *data)
{
    (void)w; (void)e;
    g_print("Main window delete event. Cleaning up...\n");

    // Clean up camera streams
    for (GList *it = data->camera_streams; it; it = it->next) {
        CameraStreamInfo *s = (CameraStreamInfo *)it->data;
        if (s) {
            free_camera_stream_info(s);
        }
    }

    gtk_main_quit();
    return TRUE;
}

 