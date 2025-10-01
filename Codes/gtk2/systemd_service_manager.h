#ifndef SYSTEMD_SERVICE_MANAGER_H
#define SYSTEMD_SERVICE_MANAGER_H

#include "app_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Service template types */
typedef enum {
    SERVICE_TEMPLATE_JETSON_HW_H264,
    SERVICE_TEMPLATE_JETSON_HW_H265,
    SERVICE_TEMPLATE_SW_H264,
    SERVICE_TEMPLATE_SW_H265,
    SERVICE_TEMPLATE_MJPEG,
    SERVICE_TEMPLATE_ROCKCHIP_MPP_H265,
    SERVICE_TEMPLATE_COUNT
} ServiceTemplateType;

/* Camera input format types */
typedef enum {
    INPUT_FORMAT_YUYV,
    INPUT_FORMAT_MJPEG,
    INPUT_FORMAT_COUNT
} InputFormatType;

/* Service configuration structure */
typedef struct {
    gchar *service_name;        // e.g., "cam0.service"
    gchar *device_path;         // e.g., "/dev/video0"
    gchar *target_ip;           // IP where to send stream
    gint port;                  // UDP port for streaming
    gint width;                 // Video width
    gint height;                // Video height
    gint framerate;             // Target framerate
    gint bitrate;               // Encoding bitrate
    ServiceTemplateType template_type;
    InputFormatType input_format;
    gboolean is_remote;         // TRUE for remote deployment, FALSE for local
    gchar *remote_host;         // SSH host for remote deployment
    gchar *remote_user;         // SSH username
    gchar *flip_method;         // GStreamer flip method string
} ServiceConfig;

/* Service status */
typedef enum {
    SERVICE_STATUS_UNKNOWN,
    SERVICE_STATUS_ACTIVE,
    SERVICE_STATUS_INACTIVE,
    SERVICE_STATUS_FAILED,
    SERVICE_STATUS_ACTIVATING,
    SERVICE_STATUS_DEACTIVATING
} ServiceStatus;

/* Function declarations */
const gchar* get_template_name(ServiceTemplateType type);
const gchar* get_input_format_name(InputFormatType type);

gchar* generate_gstreamer_pipeline(const ServiceConfig *config);
gchar* generate_service_unit_content(const ServiceConfig *config);
gchar* generate_service_script_content(const ServiceConfig *config);

gboolean create_systemd_service(const ServiceConfig *config, GError **error);
gboolean start_systemd_service(const gchar *service_name, gboolean is_remote, 
                               const gchar *remote_host, const gchar *remote_user, GError **error);
gboolean stop_systemd_service(const gchar *service_name, gboolean is_remote,
                              const gchar *remote_host, const gchar *remote_user, GError **error);
gboolean remove_systemd_service(const gchar *service_name, gboolean is_remote,
                                const gchar *remote_host, const gchar *remote_user, GError **error);

ServiceStatus get_service_status(const gchar *service_name, gboolean is_remote,
                                const gchar *remote_host, const gchar *remote_user);

ServiceConfig* service_config_new(void);
void service_config_free(ServiceConfig *config);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEMD_SERVICE_MANAGER_H */ 