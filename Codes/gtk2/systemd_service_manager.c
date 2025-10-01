#include "systemd_service_manager.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <glib.h>
#include <gtk/gtk.h>

/* Template names */
static const gchar* template_names[] = {
    "Jetson HW H264",
    "Jetson HW H265", 
    "Software H264",
    "Software H265",
    "MJPEG Stream",
    "Rockchip MPP H265"
};

/* Input format names */
static const gchar* input_format_names[] = {
    "YUYV",
    "MJPEG"
};

const gchar* get_template_name(ServiceTemplateType type) {
    if (type >= 0 && type < SERVICE_TEMPLATE_COUNT) {
        return template_names[type];
    }
    return "Unknown";
}

const gchar* get_input_format_name(InputFormatType type) {
    if (type >= 0 && type < INPUT_FORMAT_COUNT) {
        return input_format_names[type];
    }
    return "Unknown";
}

gchar* generate_gstreamer_pipeline(const ServiceConfig *config) {
    if (!config) return NULL;

    GString *pipeline_str = g_string_new("");

    // 1. v4l2src element
    if (config->template_type == SERVICE_TEMPLATE_ROCKCHIP_MPP_H265) {
        g_string_append_printf(pipeline_str, "v4l2src device=%s io-mode=4 do-timestamp=true ! ", config->device_path);
    } else {
        g_string_append_printf(pipeline_str, "v4l2src device=%s do-timestamp=true ! ", config->device_path);
    }

    // 2. Input capabilities and jpegdec if needed
    if (config->input_format == INPUT_FORMAT_MJPEG) {
        g_string_append_printf(pipeline_str, "image/jpeg,width=%d,height=%d,framerate=%d/1 ! jpegdec ! ",
                               config->width, config->height, config->framerate);
    } else { // YUYV or other raw formats
        g_string_append_printf(pipeline_str, "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! ",
                               config->width, config->height, config->framerate);
    }
    
    // 3. Videoflip element (if flip_method is not "none")
    if (config->flip_method && g_strcmp0(config->flip_method, "none") != 0) {
        // We assume flip_method string directly maps to video-direction property value
        g_string_append_printf(pipeline_str, "videoflip video-direction=%s ! ", config->flip_method);
    }

    // 4. videoconvert and caps for encoder input (NV12 for most hardware encoders)
    // This might need adjustment based on specific encoder requirements.
    // For Rockchip MPP, it expects NV12 after videoconvert.
    // For Jetson nvvidconv, it handles conversion and expects raw before it.

    const gchar *encoder_element = "";
    const gchar *payloader = "";
    gint payload_type = 96;
    gboolean needs_videoconvert_to_nv12 = FALSE;

    switch (config->template_type) {
        case SERVICE_TEMPLATE_JETSON_HW_H264:
            // nvvidconv handles conversion. Input to nvvidconv is after jpegdec (if any) or raw caps.
            // flip-method for nvvidconv: 0=none, 1=counterclockwise, 2=rotate-180, 3=clockwise, 4=horizontal flip, 5=vertical flip, 6=upper left diagonal, 7=upper right diagonal
            // We need a mapping from config->flip_method string to these integer values if we use nvvidconv's flip.
            // For now, we're using videoflip element before this stage.
            encoder_element = g_strdup_printf("nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h264enc bitrate=%d", config->bitrate);
            payloader = "rtph264pay";
            break;
        case SERVICE_TEMPLATE_JETSON_HW_H265:
            encoder_element = g_strdup_printf("nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h265enc bitrate=%d", config->bitrate);
            payloader = "rtph265pay";
            break;
        case SERVICE_TEMPLATE_SW_H264:
            needs_videoconvert_to_nv12 = TRUE; // x264enc prefers I420 or NV12, let videoconvert choose or specify.
            encoder_element = g_strdup_printf("x264enc tune=zerolatency speed-preset=ultrafast bitrate=%d", config->bitrate / 1000);
            payloader = "rtph264pay";
            break;
        case SERVICE_TEMPLATE_SW_H265:
            needs_videoconvert_to_nv12 = TRUE;
            encoder_element = g_strdup_printf("x265enc tune=zerolatency speed-preset=ultrafast bitrate=%d", config->bitrate / 1000);
            payloader = "rtph265pay";
            break;
        case SERVICE_TEMPLATE_MJPEG: // This is MJPEG *output* stream
            encoder_element = g_strdup("jpegenc");
            payloader = "rtpjpegpay";
            payload_type = 26;
            if (config->input_format != INPUT_FORMAT_MJPEG) { // If input is raw, need videoconvert before jpegenc
                 needs_videoconvert_to_nv12 = TRUE; // Or some other format jpegenc accepts if not from jpegdec
            }
            break;
        case SERVICE_TEMPLATE_ROCKCHIP_MPP_H265:
            needs_videoconvert_to_nv12 = TRUE; // mpph265enc expects NV12
            encoder_element = g_strdup_printf("mpph265enc rc-mode=cbr bps=%d gop=15", config->bitrate);
            payloader = "rtph265pay";
            break;
        default:
            needs_videoconvert_to_nv12 = TRUE;
            encoder_element = g_strdup_printf("x264enc tune=zerolatency speed-preset=ultrafast bitrate=%d", config->bitrate / 1000);
            payloader = "rtph264pay";
            break;
    }

    if (needs_videoconvert_to_nv12) {
        // If input was MJPEG, jpegdec already converted it to a raw format.
        // If input was YUY2, it's raw.
        // We need videoconvert to ensure format is NV12 for encoders like mpph265enc or x264/x265enc.
        // If the element before this point was `jpegdec ! videoflip !`, then we add `videoconvert ! video/x-raw,format=NV12 !`
        // If it was `video/x-raw,... ! videoflip !`, then we add `videoconvert ! video/x-raw,format=NV12 !`
        // Exception: If SERVICE_TEMPLATE_MJPEG and input was already MJPEG, jpegdec is there but jpegenc is the target, so this videoconvert might not be needed or different.
        // For SERVICE_TEMPLATE_MJPEG where input is YUYV, we need videoconvert ! jpegenc.
        if (config->template_type == SERVICE_TEMPLATE_MJPEG && config->input_format != INPUT_FORMAT_MJPEG) {
            g_string_append_printf(pipeline_str, "videoconvert ! "); // jpegenc will follow
        } else if (config->template_type != SERVICE_TEMPLATE_JETSON_HW_H264 && 
                   config->template_type != SERVICE_TEMPLATE_JETSON_HW_H265 && 
                   config->template_type != SERVICE_TEMPLATE_MJPEG) { // Avoid double videoconvert or if nvvidconv handles it
             g_string_append_printf(pipeline_str, "videoconvert ! video/x-raw,format=NV12 ! ");
        }
    }

    // 5. Queue, watchdog (optional but good practice)
    g_string_append_printf(pipeline_str, "queue max-size-buffers=1 max-size-bytes=0 leaky=downstream ! watchdog timeout=2000000000 name=wd ! ");

    // 6. Encoder element
    g_string_append(pipeline_str, encoder_element);
    g_string_append(pipeline_str, " ! ");

    // 7. Parser (for H264/H265)
    if (config->template_type == SERVICE_TEMPLATE_ROCKCHIP_MPP_H265 || 
        config->template_type == SERVICE_TEMPLATE_JETSON_HW_H265 || 
        config->template_type == SERVICE_TEMPLATE_SW_H265) {
        g_string_append(pipeline_str, "h265parse ! ");
    } else if (config->template_type == SERVICE_TEMPLATE_JETSON_HW_H264 || 
               config->template_type == SERVICE_TEMPLATE_SW_H264) {
        g_string_append(pipeline_str, "h264parse ! ");
    }

    // 8. Payloader and Sink
    g_string_append_printf(pipeline_str, "%s pt=%d config-interval=1 ! udpsink host=%s port=%d sync=false",
                           payloader, payload_type, config->target_ip, config->port);

    gchar *final_pipeline = g_string_free(pipeline_str, FALSE);
    
    // Free allocated encoder_element strings from g_strdup_printf
    if (strlen(encoder_element) > 0 && 
        (config->template_type != SERVICE_TEMPLATE_MJPEG || (config->template_type == SERVICE_TEMPLATE_MJPEG && config->input_format != INPUT_FORMAT_MJPEG)) && // jpegenc is not strduped if input is mjpeg for mjpeg template
        (strstr(encoder_element, "nvv4l2h264enc") || strstr(encoder_element, "nvv4l2h265enc") || 
         strstr(encoder_element, "x264enc") || strstr(encoder_element, "x265enc") || 
         strstr(encoder_element, "mpph265enc") || strstr(encoder_element, "jpegenc"))) {
        g_free((gchar*)encoder_element);
    }

    g_debug("Generated GStreamer Pipeline: %s", final_pipeline);
    return final_pipeline;
}

gchar* generate_service_script_content(const ServiceConfig *config) {
    if (!config) return NULL;
    
    gchar *pipeline = generate_gstreamer_pipeline(config);
    if (!pipeline) return NULL;
    
    gchar *script_content = g_strdup_printf(
        "#!/bin/bash\n"
        "# Auto-generated camera streaming script for %s\n"
        "set -e\n\n"
        "# Wait for device to be available\n"
        "while [ ! -e \"%s\" ]; do\n"
        "    echo \"Waiting for device %s...\"\n"
        "    sleep 0.5\n"
        "done\n\n"
        "echo \"Starting camera stream: %s\"\n"
        "exec gst-launch-1.0 -e %s\n",
        config->service_name,
        config->device_path,
        config->device_path,
        config->service_name,
        pipeline
    );
    
    g_free(pipeline);
    return script_content;
}

gchar* generate_service_unit_content(const ServiceConfig *config) {
    if (!config) return NULL;
    
    gchar *script_path = g_strdup_printf("/usr/local/bin/run_%s.sh", 
                                        g_strndup(config->service_name, 
                                                 strlen(config->service_name) - 8)); // Remove .service
    
    gchar *unit_content = g_strdup_printf(
        "[Unit]\n"
        "Description=Camera %s stream (%s)\n"
        "After=network-online.target\n"
        "Wants=network-online.target\n\n"
        "[Service]\n"
        "Type=simple\n"
        "ExecStartPre=/bin/sh -c 'until [ -e \"%s\" ]; do sleep 0.5; done'\n"
        "ExecStart=%s\n"
        "Restart=on-failure\n"
        "RestartSec=1\n"
        "User=ubuntu\n\n"
        "[Install]\n"
        "WantedBy=multi-user.target\n",
        config->service_name,
        get_template_name(config->template_type),
        config->device_path,
        script_path
    );
    
    g_free(script_path);
    return unit_content;
}

static gboolean execute_remote_command(const gchar *host, const gchar *user, 
                                      const gchar *command, GError **error) {
    gchar *ssh_command = g_strdup_printf("ssh %s@%s '%s'", user, host, command);
    gint exit_status;
    gboolean success = g_spawn_command_line_sync(ssh_command, NULL, NULL, &exit_status, error);
    g_free(ssh_command);
    
    if (!success || exit_status != 0) {
        if (error && !*error) {
            g_set_error(error, G_SPAWN_ERROR, G_SPAWN_ERROR_FAILED,
                       "Remote command failed with exit status %d", exit_status);
        }
        return FALSE;
    }
    return TRUE;
}

static gboolean write_remote_file(const gchar *host, const gchar *user,
                                 const gchar *content, const gchar *remote_path,
                                 gint mode, GError **error) {
    /* Create temporary local file */
    gchar *temp_file = g_strdup_printf("/tmp/service_temp_%d", getpid());
    
    if (!g_file_set_contents(temp_file, content, -1, error)) {
        g_free(temp_file);
        return FALSE;
    }
    
    /* Copy to remote host */
    gchar *scp_command = g_strdup_printf("scp %s %s@%s:%s", temp_file, user, host, remote_path);
    gint exit_status;
    gboolean success = g_spawn_command_line_sync(scp_command, NULL, NULL, &exit_status, error);
    
    /* Set permissions */
    if (success && exit_status == 0) {
        gchar *chmod_command = g_strdup_printf("chmod %o %s", mode, remote_path);
        success = execute_remote_command(host, user, chmod_command, error);
        g_free(chmod_command);
    }
    
    /* Cleanup */
    unlink(temp_file);
    g_free(temp_file);
    g_free(scp_command);
    
    return success && exit_status == 0;
}

gboolean create_systemd_service(const ServiceConfig *config, GError **error) {
    if (!config || !config->service_name) {
        g_set_error(error, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT,
                   "Invalid service configuration");
        return FALSE;
    }
    
    /* Generate content */
    gchar *script_content = generate_service_script_content(config);
    gchar *unit_content = generate_service_unit_content(config);
    
    if (!script_content || !unit_content) {
        g_set_error(error, G_IO_ERROR, G_IO_ERROR_FAILED,
                   "Failed to generate service content");
        g_free(script_content);
        g_free(unit_content);
        return FALSE;
    }
    
    gboolean success = FALSE;
    
    if (config->is_remote) {
        /* Remote deployment */
        gchar *script_name = g_strndup(config->service_name, strlen(config->service_name) - 8);
        gchar *script_path = g_strdup_printf("/usr/local/bin/run_%s.sh", script_name);
        gchar *unit_path = g_strdup_printf("/etc/systemd/system/%s", config->service_name);
        
        /* Write script file */
        success = write_remote_file(config->remote_host, config->remote_user,
                                   script_content, script_path, 0755, error);
        
        /* Write unit file */
        if (success) {
            success = write_remote_file(config->remote_host, config->remote_user,
                                       unit_content, unit_path, 0644, error);
        }
        
        /* Reload systemd */
        if (success) {
            success = execute_remote_command(config->remote_host, config->remote_user,
                                            "sudo systemctl daemon-reload", error);
        }
        
        g_free(script_name);
        g_free(script_path);
        g_free(unit_path);
    } else {
        /* Local deployment */
        gchar *script_name = g_strndup(config->service_name, strlen(config->service_name) - 8);
        gchar *script_path = g_strdup_printf("/usr/local/bin/run_%s.sh", script_name);
        gchar *unit_path = g_strdup_printf("/etc/systemd/system/%s", config->service_name);
        
        /* Write script file */
        success = g_file_set_contents(script_path, script_content, -1, error);
        if (success) {
            chmod(script_path, 0755);
        }
        
        /* Write unit file */
        if (success) {
            success = g_file_set_contents(unit_path, unit_content, -1, error);
        }
        
        /* Reload systemd */
        if (success) {
            gint exit_status;
            success = g_spawn_command_line_sync("sudo systemctl daemon-reload", 
                                              NULL, NULL, &exit_status, error);
            success = success && (exit_status == 0);
        }
        
        g_free(script_name);
        g_free(script_path);
        g_free(unit_path);
    }
    
    g_free(script_content);
    g_free(unit_content);
    
    return success;
}

gboolean start_systemd_service(const gchar *service_name, gboolean is_remote,
                               const gchar *remote_host, const gchar *remote_user, GError **error) {
    gchar *command = g_strdup_printf("sudo systemctl start %s", service_name);
    gboolean success;
    
    if (is_remote) {
        success = execute_remote_command(remote_host, remote_user, command, error);
    } else {
        gint exit_status;
        success = g_spawn_command_line_sync(command, NULL, NULL, &exit_status, error);
        success = success && (exit_status == 0);
    }
    
    g_free(command);
    return success;
}

gboolean stop_systemd_service(const gchar *service_name, gboolean is_remote,
                             const gchar *remote_host, const gchar *remote_user, GError **error) {
    gchar *command = g_strdup_printf("sudo systemctl stop %s", service_name);
    gboolean success;
    
    if (is_remote) {
        success = execute_remote_command(remote_host, remote_user, command, error);
    } else {
        gint exit_status;
        success = g_spawn_command_line_sync(command, NULL, NULL, &exit_status, error);
        success = success && (exit_status == 0);
    }
    
    g_free(command);
    return success;
}

gboolean remove_systemd_service(const gchar *service_name, gboolean is_remote,
                               const gchar *remote_host, const gchar *remote_user, GError **error) {
    gchar *script_name = g_strndup(service_name, strlen(service_name) - 8);
    gchar *commands = g_strdup_printf(
        "sudo systemctl stop %s; "
        "sudo systemctl disable %s; "
        "sudo rm -f /etc/systemd/system/%s; "
        "sudo rm -f /usr/local/bin/run_%s.sh; "
        "sudo systemctl daemon-reload",
        service_name, service_name, service_name, script_name
    );
    
    gboolean success;
    
    if (is_remote) {
        success = execute_remote_command(remote_host, remote_user, commands, error);
    } else {
        gint exit_status;
        success = g_spawn_command_line_sync(commands, NULL, NULL, &exit_status, error);
        success = success && (exit_status == 0);
    }
    
    g_free(script_name);
    g_free(commands);
    return success;
}

ServiceStatus get_service_status(const gchar *service_name, gboolean is_remote,
                                const gchar *remote_host, const gchar *remote_user) {
    gchar *command = g_strdup_printf("systemctl is-active %s", service_name);
    gchar *output = NULL;
    gint exit_status;
    gboolean success;
    
    if (is_remote) {
        gchar *ssh_command = g_strdup_printf("ssh %s@%s '%s'", remote_user, remote_host, command);
        success = g_spawn_command_line_sync(ssh_command, &output, NULL, &exit_status, NULL);
        g_free(ssh_command);
    } else {
        success = g_spawn_command_line_sync(command, &output, NULL, &exit_status, NULL);
    }
    
    g_free(command);
    
    ServiceStatus status = SERVICE_STATUS_UNKNOWN;
    
    if (success && output) {
        g_strstrip(output);
        if (g_strcmp0(output, "active") == 0) {
            status = SERVICE_STATUS_ACTIVE;
        } else if (g_strcmp0(output, "inactive") == 0) {
            status = SERVICE_STATUS_INACTIVE;
        } else if (g_strcmp0(output, "failed") == 0) {
            status = SERVICE_STATUS_FAILED;
        } else if (g_strcmp0(output, "activating") == 0) {
            status = SERVICE_STATUS_ACTIVATING;
        } else if (g_strcmp0(output, "deactivating") == 0) {
            status = SERVICE_STATUS_DEACTIVATING;
        }
    }
    
    g_free(output);
    return status;
}

ServiceConfig* service_config_new(void) {
    ServiceConfig *config = g_new0(ServiceConfig, 1);
    config->bitrate = 1000000; // Default 1Mbps
    config->template_type = SERVICE_TEMPLATE_SW_H264;
    config->input_format = INPUT_FORMAT_YUYV;
    config->is_remote = FALSE;
    return config;
}

void service_config_free(ServiceConfig *config) {
    if (!config) return;
    
    g_free(config->service_name);
    g_free(config->device_path);
    g_free(config->target_ip);
    g_free(config->remote_host);
    g_free(config->remote_user);
    g_free(config->flip_method);
    g_free(config);
} 