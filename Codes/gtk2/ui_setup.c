#include "ui_setup.h"
#include "ui_callbacks.h"
#include "systemd_service_manager.h"
#include <gtk/gtk.h>
#include <string.h>
#include <pango/pango.h>

void create_ui(CustomData *data)
{
    GtkWidget *main_paned;
    GtkWidget *left_pane_vbox, *right_pane_vbox;
    GtkWidget *scrolled_window_treeview;
    GtkWidget *button_hbox_left;
    GtkWidget *add_camera_button, *remove_camera_button;
    GtkCellRenderer *renderer;
    GtkTreeViewColumn *column;

    data->main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(data->main_window), "Camera Stream Viewer");
    gtk_window_set_default_size(GTK_WINDOW(data->main_window), 1200, 700);
    g_signal_connect(data->main_window, "delete-event", G_CALLBACK(main_window_delete_event), data);

    main_paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_container_add(GTK_CONTAINER(data->main_window), main_paned);

    /* ================= LEFT PANE (Camera List) ================= */
    left_pane_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_set_border_width(GTK_CONTAINER(left_pane_vbox), 5);
    gtk_paned_add1(GTK_PANED(main_paned), left_pane_vbox);
    gtk_widget_set_size_request(left_pane_vbox, 350, -1);

    scrolled_window_treeview = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window_treeview), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_box_pack_start(GTK_BOX(left_pane_vbox), scrolled_window_treeview, TRUE, TRUE, 0);

    data->camera_list_store = gtk_list_store_new(NUM_COLUMNS, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_POINTER);
    data->camera_list_treeview = gtk_tree_view_new_with_model(GTK_TREE_MODEL(data->camera_list_store));
    gtk_container_add(GTK_CONTAINER(scrolled_window_treeview), data->camera_list_treeview);

    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes("Name", renderer, "text", COLUMN_NAME, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(data->camera_list_treeview), column);
    column = gtk_tree_view_column_new_with_attributes("Info (IP:Port)", renderer, "text", COLUMN_INFO, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(data->camera_list_treeview), column);
    column = gtk_tree_view_column_new_with_attributes("Status", renderer, "text", COLUMN_STATUS, NULL);
    gtk_tree_view_append_column(GTK_TREE_VIEW(data->camera_list_treeview), column);

    GtkTreeSelection *selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(data->camera_list_treeview));
    g_signal_connect(selection, "changed", G_CALLBACK(on_camera_selection_changed), data);

    button_hbox_left = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_button_box_set_layout(GTK_BUTTON_BOX(button_hbox_left), GTK_BUTTONBOX_CENTER);
    gtk_box_pack_start(GTK_BOX(left_pane_vbox), button_hbox_left, FALSE, FALSE, 5);

    add_camera_button = gtk_button_new_with_label("Add Camera");
    g_signal_connect(add_camera_button, "clicked", G_CALLBACK(on_add_camera_clicked), data);
    gtk_container_add(GTK_CONTAINER(button_hbox_left), add_camera_button);

    remove_camera_button = gtk_button_new_with_label("Remove Selected");
    g_signal_connect(remove_camera_button, "clicked", G_CALLBACK(on_remove_camera_clicked), data);
    gtk_container_add(GTK_CONTAINER(button_hbox_left), remove_camera_button);

    /* ================= RIGHT PANE (Details & Video) ================= */
    right_pane_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_set_border_width(GTK_CONTAINER(right_pane_vbox), 5);
    gtk_paned_add2(GTK_PANED(main_paned), right_pane_vbox);

    /* ---- Details & Controls Section ---- */
    GtkWidget *details_frame = gtk_frame_new("Camera Details & Controls");
    gtk_box_pack_start(GTK_BOX(right_pane_vbox), details_frame, FALSE, FALSE, 0);

    data->details_pane_grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(details_frame), data->details_pane_grid);
    gtk_grid_set_column_spacing(GTK_GRID(data->details_pane_grid), 10);
    gtk_grid_set_row_spacing(GTK_GRID(data->details_pane_grid), 5);
    gtk_container_set_border_width(GTK_CONTAINER(data->details_pane_grid), 10);

    gint row = 0;
    data->label_cam_name_dynamic = gtk_label_new("No Camera Selected");
    PangoAttribute *size_attr = pango_attr_size_new_absolute(14 * PANGO_SCALE);
    PangoAttrList *attrs = pango_attr_list_new();
    pango_attr_list_insert(attrs, size_attr);
    gtk_label_set_attributes(GTK_LABEL(data->label_cam_name_dynamic), attrs);
    pango_attr_list_unref(attrs);
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->label_cam_name_dynamic, 0, row++, 2, 1);

    /* IP */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Remote IP:"), 0, row, 1, 1);
    data->entry_cam_ip = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_cam_ip), "192.168.1.100");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_cam_ip, 1, row++, 1, 1);

    /* Streaming port */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Streaming Port:"), 0, row, 1, 1);
    data->entry_streaming_port = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_streaming_port), "5000");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_streaming_port, 1, row++, 1, 1);

    /* Encoding combo */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Encoding:"), 0, row, 1, 1);
    data->combo_encoding = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_encoding), "H264");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_encoding), "H265");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_encoding), "MJPEG");
    gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_encoding), 0);
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->combo_encoding, 1, row++, 1, 1);

    /* Width */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Width:"), 0, row, 1, 1);
    data->entry_width = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_width), "640");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_width, 1, row++, 1, 1);

    /* Height */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Height:"), 0, row, 1, 1);
    data->entry_height = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_height), "480");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_height, 1, row++, 1, 1);

    /* FPS */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Framerate:"), 0, row, 1, 1);
    data->entry_framerate = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_framerate), "30");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_framerate, 1, row++, 1, 1);

    /* Control port */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Control Port:"), 0, row, 1, 1);
    data->entry_control_port = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_control_port), "9999");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_control_port, 1, row++, 1, 1);

    /* Bitrate (details pane) */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Bitrate (bps):"), 0, row, 1, 1);
    data->entry_bitrate_details = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_bitrate_details), "1000000"); // Default 1Mbps
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_bitrate_details, 1, row++, 1, 1);

    /* Flip Method (details pane) */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Flip Method:"), 0, row, 1, 1);
    data->combo_flip_method_details = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "none");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "clockwise");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "rotate-180");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "counter-clockwise");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "horizontal-flip");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "vertical-flip");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "upper-left-diagonal");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_flip_method_details), "upper-right-diagonal");
    gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_flip_method_details), 0); // Default to none
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->combo_flip_method_details, 1, row++, 1, 1);

    /* Device Path */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Device Path:"), 0, row, 1, 1);
    data->entry_device_path = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_device_path), "/dev/video0");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_device_path, 1, row++, 1, 1);

    /* Service Template */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Pipeline Template:"), 0, row, 1, 1);
    data->combo_service_template = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_service_template), "Jetson HW H264");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_service_template), "Jetson HW H265");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_service_template), "Software H264");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_service_template), "Software H265");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_service_template), "MJPEG Stream");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_service_template), "Rockchip MPP H265");
    gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_service_template), 2); // Default to SW H264
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->combo_service_template, 1, row++, 1, 1);

    /* Input Format */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Input Format (for v4l2src):"), 0, row, 1, 1);
    data->combo_input_format = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_input_format), "YUYV");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(data->combo_input_format), "MJPEG");
    gtk_combo_box_set_active(GTK_COMBO_BOX(data->combo_input_format), 0); // Default to YUYV
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->combo_input_format, 1, row++, 1, 1);

    /* Remote Service Checkbox */
    data->check_remote_service = gtk_check_button_new_with_label("Deploy to Remote Host");
    g_signal_connect(data->check_remote_service, "toggled", G_CALLBACK(on_remote_service_toggled_details), data);
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->check_remote_service, 0, row++, 2, 1);

    /* Remote Host */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Remote Host:"), 0, row, 1, 1);
    data->entry_remote_host = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_remote_host), "192.168.1.100");
    gtk_widget_set_sensitive(data->entry_remote_host, FALSE);
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_remote_host, 1, row++, 1, 1);

    /* Remote User */
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), gtk_label_new("Remote User:"), 0, row, 1, 1);
    data->entry_remote_user = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(data->entry_remote_user), "ubuntu");
    gtk_widget_set_sensitive(data->entry_remote_user, FALSE);
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->entry_remote_user, 1, row++, 1, 1);

    /* Button box */
    GtkWidget *button_box_controls = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box_controls), GTK_BUTTONBOX_CENTER);
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), button_box_controls, 0, row++, 2, 1);

    data->button_connect = gtk_button_new_with_label("Connect");
    g_signal_connect(data->button_connect, "clicked", G_CALLBACK(on_connect_clicked), data);
    gtk_container_add(GTK_CONTAINER(button_box_controls), data->button_connect);

    data->button_disconnect = gtk_button_new_with_label("Disconnect");
    g_signal_connect(data->button_disconnect, "clicked", G_CALLBACK(on_disconnect_clicked), data);
    gtk_container_add(GTK_CONTAINER(button_box_controls), data->button_disconnect);

    data->button_apply_settings = gtk_button_new_with_label("Apply Settings");
    g_signal_connect(data->button_apply_settings, "clicked", G_CALLBACK(on_apply_settings_clicked), data);
    gtk_container_add(GTK_CONTAINER(button_box_controls), data->button_apply_settings);

    data->label_status_selected = gtk_label_new("Select a camera or add a new one.");
    gtk_grid_attach(GTK_GRID(data->details_pane_grid), data->label_status_selected, 0, row++, 2, 1);

    gtk_widget_set_sensitive(data->details_pane_grid, FALSE);

    gtk_paned_set_position(GTK_PANED(main_paned), 320);
    gtk_widget_show_all(data->main_window);

    /* Ensure details pane is disabled after show */
    gtk_widget_set_sensitive(data->details_pane_grid, FALSE);
} 