#include "app_data.h"
#include "ui_setup.h"
#include <gtk/gtk.h>
#include <gst/gst.h>

// Simple stub function for cleanup
static void free_camera_stream_info(CameraStreamInfo *stream) {
    if (!stream) return;
    g_free(stream->name);
    g_free(stream->remote_ip);
    g_free(stream->encoding);
    g_free(stream);
}

int main(int argc, char *argv[])
{
    /* Zero-initialize the main application struct */
    CustomData data = {0};

    /* Initialize GTK and GStreamer */
    gtk_init(&argc, &argv);
    gst_init(&argc, &argv);

    /* Create the UI */
    create_ui(&data);

    /* Run the main loop */
    gtk_main();

    /* Final cleanup: free all remaining streams */
    g_list_free_full(data.camera_streams, (GDestroyNotify)free_camera_stream_info);
    data.camera_streams = NULL;

    g_print("Exiting application.\n");
    return 0;
} 