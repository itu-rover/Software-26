#ifndef APP_DATA_H
#define APP_DATA_H

#include <gtk/gtk.h>
#include <gst/gst.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
typedef struct _CustomData CustomData;
typedef struct _CameraStreamInfo CameraStreamInfo;

/* Structure to hold info for a single camera stream */
struct _CameraStreamInfo {
    gchar *name;                // User-defined name for the camera
    gchar *remote_ip;           // IP address of the camera stream source
    gint streaming_port;        // Port for the video stream (UDP)
    gint control_port;          // Port for sending control commands
    gchar *encoding;            // "H264", "H265", "MJPEG"
    gint width;                 // Target width
    gint height;                // Target height
    gint framerate;             // Target framerate

    gboolean connected;         // Connection status
    GtkTreeIter iter;           // Iterator for this stream's row in the GtkTreeView
    CustomData *app_data;       // Pointer back to the main application data
    
    /* Service management fields */
    gchar *device_path;         // e.g., "/dev/video0"
    gchar *service_name;        // e.g., "cam0.service"
    gint service_template;      // ServiceTemplateType enum value
    gint input_format;          // InputFormatType enum value
    gint bitrate;               // Encoding bitrate
    gboolean is_remote_service; // TRUE for remote deployment
    gchar *remote_host;         // SSH host for remote deployment
    gchar *remote_user;         // SSH username
    gint service_status;        // ServiceStatus enum value
    gchar *flip_method;         // Flip method e.g., "none", "horizontal-flip"
};

/* Main application data structure */
struct _CustomData {
    GList *camera_streams;            
    CameraStreamInfo *selected_stream;

    /* --- Widgets owned by the main UI --- */
    GtkWidget *main_window;
    GtkWidget *camera_list_treeview;
    GtkListStore *camera_list_store;

    GtkWidget *details_pane_grid;
    GtkWidget *label_cam_name_dynamic;
    GtkWidget *entry_cam_ip;
    GtkWidget *entry_streaming_port;
    GtkWidget *combo_encoding;
    GtkWidget *entry_width;
    GtkWidget *entry_height;
    GtkWidget *entry_framerate;
    GtkWidget *entry_control_port;
    GtkWidget *entry_bitrate_details; // For main details pane
    GtkWidget *combo_flip_method_details; // For main details pane
    GtkWidget *button_connect;
    GtkWidget *button_disconnect;
    GtkWidget *button_apply_settings;
    GtkWidget *label_status_selected;
    
    /* Service management widgets */
    GtkWidget *entry_device_path;
    GtkWidget *combo_service_template;
    GtkWidget *combo_input_format;
    GtkWidget *entry_bitrate;
    GtkWidget *check_remote_service;
    GtkWidget *entry_remote_host;
    GtkWidget *entry_remote_user;
    GtkWidget *button_create_service;
    GtkWidget *button_start_service;
    GtkWidget *button_stop_service;
    GtkWidget *button_remove_service;
    GtkWidget *label_service_status;
};

/* Columns for the GtkTreeView */
enum {
    COLUMN_NAME = 0,
    COLUMN_INFO,
    COLUMN_STATUS,
    COLUMN_POINTER,
    NUM_COLUMNS
};

#ifdef __cplusplus
}
#endif

#endif /* APP_DATA_H */ 