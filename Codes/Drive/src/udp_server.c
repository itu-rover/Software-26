/**
 * @file udp_server.c
 * @brief UDP üzerinden STM32'ye joystick verisi gönderen server uygulaması.
 *
 * Bu uygulama, bir Logitech F710 joystick'ten veri okur, verileri işler ve
 * UDP ile STM32'ye gönderir. Aynı zamanda STM32'den gelen UDP mesajlarını da dinler.
 * Sürüş tank sürüşü olarak çalışmaktadır.
 *
 * @author Kadir/DeanGalaksin
 * @date 2025-04-05
 * @version 1.0
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>

#include "config.h"
#include "constants.h"
#include "network.h"
#include "joystick_handler.h"

// Global variables
char last_message[BUFFER_SIZE];

// OrangePi message sending thread
void* orangepi_message_thread(void* arg) {
    int orangepi_sockfd = create_socket();

    if (orangepi_sockfd < 0) {
        printf("Failed to create OrangePi socket\n");
        return NULL;
    }
    
    // Initialize OrangePi client address
    memset(&orangepi_client_addr, 0, sizeof(orangepi_client_addr));
    orangepi_client_addr.sin_family = AF_INET;
    orangepi_client_addr.sin_addr.s_addr = inet_addr(ORANGEPI_IP);
    orangepi_client_addr.sin_port = htons(ORANGEPI_PORT);
    
    // For timing control
    int message_interval = 10000; // Send message every 100ms
    
    while (running) {
        const char* msg_to_send;
        
        // Check button states
        pthread_mutex_lock(&state_mutex);
        int button6 = js_state.button6;
        int button7 = js_state.button7;
        pthread_mutex_unlock(&state_mutex);
        
        // Determine which message to send
        if (button6 && !button7) {
            msg_to_send = OPEN_GRIPPER;
        } else if (!button6 && button7) {
            msg_to_send = CLOSE_GRIPPER;
        } else {
            msg_to_send = DEFAULT_GRIPPER;
        }
        
        // Send the message to OrangePi
        if (sendto(orangepi_sockfd, msg_to_send, 4, 0, 
                (struct sockaddr*) &orangepi_client_addr, sizeof(orangepi_client_addr)) < 0) {
            perror("Send to OrangePi failed");
        } else {
            // printf("Sent message '%s' to %s:%d\n", 
            //        msg_to_send, ORANGEPI_IP, ORANGEPI_PORT);
        }
        
        usleep(message_interval);
    }
    
    close(orangepi_sockfd);
    return NULL;
}


void* xavier_message_thread(void* arg) {
    int xavier_sockfd = create_xavier_socket();
    if (xavier_sockfd < 0) {
        printf("Failed to create Xavier socket\n");
        return NULL;
    }

    struct sockaddr_in xavier_client_addr;
    memset(&xavier_client_addr, 0, sizeof(xavier_client_addr));
    xavier_client_addr.sin_family = AF_INET;
    xavier_client_addr.sin_addr.s_addr = inet_addr(XAVIER_IP);
    xavier_client_addr.sin_port = htons(XAVIER_PORT);

    while (running) {
        // Send GPS data to Xavier
        if (sendto(xavier_sockfd, gps_msg, 16, 0, (struct sockaddr*)&xavier_client_addr, sizeof(xavier_client_addr)) < 0) {
            perror("Send to Xavier failed");
        } else {
            // Parse as int32_t values and apply scaling
            int32_t* gps_int = (int32_t*)gps_msg;
            double lat = gps_int[0] * 1e-7;
            double lon = gps_int[1] * 1e-7;
            double course = gps_int[2] * 1e-5;
            double soc = gps_int[3] * 1e-3;

            printf("Lat: %.7f\n", lat);
            printf("Long: %.7f\n", lon);
            printf("Course: %.7f\n", course);
            printf("Soc: %.7f\n", soc);
            
            // Only print if any of the values is non-zero (to avoid spamming the console with empty data)
            if (gps_int[0] != 0 || gps_int[1] != 0 || gps_int[2] != 0 || gps_int[3] != 0) {
                 printf("Sent GPS data to Xavier:\n"); 
                 printf("Lat: %.7f\n", gps_int[0] * 1e-7);
                 printf("Long: %.7f\n", gps_int[1] * 1e-7);
                 printf("Course: %.7f\n", gps_int[2] * 1e-7);
                 printf("SoC: %.7f\n", gps_int[3] * 1e-7);
            }
        }
        usleep(100000); // 100 ms sleep - adjust as needed for your desired update rate
    }
    
    close(xavier_sockfd);
    return NULL;
}

double sensitivity_calculation(int rb, int lb) {
    double sensitivity = 0;
        
    if (rb && !lb) {
        sensitivity = 0.3;
    }
    else if (!rb && lb) {
        // yavass
        sensitivity = 0.1;
    }
    else if (rb && lb) {
        // Both LB and RB pressed: Medium speed
        sensitivity = 0.2;
    }

    return sensitivity;
}

// Helper functions for message formatting
static void add_timestamp_to_buffer(uint16_t timestamp) {
    binary_buffer[binary_buffer_length++] = ((timestamp >> 8) & 0xFF);
    binary_buffer[binary_buffer_length++] = (timestamp & 0xFF);
}

static void add_opmode_to_buffer(uint16_t op_mode) {
    binary_buffer[binary_buffer_length++] = (op_mode >> 8) & 0xFF;  
    binary_buffer[binary_buffer_length++] = (op_mode & 0xFF);         // Low byte as ASCII
}

static void add_rotation_to_buffer(int rot_dir, int vel_rot) {
    binary_buffer[binary_buffer_length++] = '0' + (rot_dir & 0xFF);  // ASCII for rot_dir
    binary_buffer[binary_buffer_length++] = '0' + ((vel_rot / 100) % 10);
    binary_buffer[binary_buffer_length++] = '0' + ((vel_rot / 10) % 10);
    binary_buffer[binary_buffer_length++] = '0' + (vel_rot % 10);
}

static void add_linear_to_buffer(int lin_dir, int vel_lin) {
    binary_buffer[binary_buffer_length++] = '0' + (lin_dir & 0xFF);  // ASCII for lin_dir
    binary_buffer[binary_buffer_length++] = '0' + ((vel_lin / 1000) % 10);
    binary_buffer[binary_buffer_length++] = '0' + ((vel_lin / 100) % 10);
    binary_buffer[binary_buffer_length++] = '0' + ((vel_lin / 10) % 10);
    binary_buffer[binary_buffer_length++] = '0' + (vel_lin % 10);
}

static void add_axis_intensities_to_buffer(const int* axis_intensities, int count) {
    for (int i = 0; i < count; i++) {
        binary_buffer[binary_buffer_length++] = '0' + (axis_intensities[i] & 0xFF);
    }
}

static void create_idle_message(uint16_t timestamp, uint16_t op_mode) {
    binary_buffer_length = 0;
    add_timestamp_to_buffer(timestamp);
    add_opmode_to_buffer(op_mode);
    
    // Idle state: no rotation or linear movement
    binary_buffer[binary_buffer_length++] = '0';  // rot_dir
    binary_buffer[binary_buffer_length++] = '0';  // vel_rot hundreds
    binary_buffer[binary_buffer_length++] = '0';  // vel_rot tens  
    binary_buffer[binary_buffer_length++] = '0';  // vel_rot ones
    binary_buffer[binary_buffer_length++] = '1';  // lin_dir (default forward)
    binary_buffer[binary_buffer_length++] = '0';  // vel_lin thousands
    binary_buffer[binary_buffer_length++] = '0';  // vel_lin hundreds
    binary_buffer[binary_buffer_length++] = '0';  // vel_lin tens
    binary_buffer[binary_buffer_length++] = '0';  // vel_lin ones
}

static void create_movement_message(uint16_t timestamp, uint16_t op_mode, 
                                   int rot_dir, int vel_rot, int lin_dir, int vel_lin) {
    binary_buffer_length = 0;
    add_timestamp_to_buffer(timestamp);
    add_opmode_to_buffer(op_mode);
    add_rotation_to_buffer(rot_dir, vel_rot);
    add_linear_to_buffer(lin_dir, vel_lin);
}

static void calculate_movement_values(double sensitivity, int* vel_lin_out, int* vel_rot_out, 
                                     int* lin_dir_out, int* rot_dir_out) {
    pthread_mutex_lock(&state_mutex);
    int left_y = js_state.left_y;
    int right_y = js_state.right_y;
    pthread_mutex_unlock(&state_mutex);

    if (sensitivity > 0) {
        // Use sum of joysticks for linear velocity, not average
        double temp_vel_lin = (left_y + right_y) * (9999.0 / (32768.0 * 2.0)) * sensitivity;
        double temp_vel_rot = ((left_y - right_y) * (999.0 / (32768.0 * 2.0))) * sensitivity;
        
        // Set directions based on calculated velocities
        *lin_dir_out = (temp_vel_lin < 0) ? 1 : 0;  // 1 for reverse, 0 for forward
        *rot_dir_out = (temp_vel_rot > 0) ? 1 : 0;  // 1 for right, 0 for left
        
        // Convert to absolute integer values
        *vel_lin_out = (int)fabs(temp_vel_lin);
        *vel_rot_out = (int)fabs(temp_vel_rot);
        
        // Clamp values to their maximum ranges
        *vel_lin_out = (*vel_lin_out > 3999) ? 3999 : *vel_lin_out;
        *vel_rot_out = (*vel_rot_out > 399) ? 399 : *vel_rot_out;
    } else {
        // When sensitivity is 0, no movement
        *vel_rot_out = 0;
        *vel_lin_out = 0;
        *rot_dir_out = 0;
        *lin_dir_out = 1;  // Default to forward direction when stopped
    }
}

// This is the local static function in udp_server.c to update the axis_intensities array
static void update_local_axis_intensities_array(int* local_axis_intensities_arr) {
    const int default_intensity = 5;

    for (int i = 0; i < 6; i++) {
        if (js_state.active_axes & (1 << i)) { // Check current active_axes from js_state
            switch (i) {
                case 0: // Axis 1 (JS_BUTTON_A)
                    local_axis_intensities_arr[i] = (10 - js_state.x_intensity); // Use current x_intensity
                    break;
                case 5: // Axis 6 (JS_BUTTON_RB_NAIM)
                    local_axis_intensities_arr[i] = js_state.x_intensity; // Use current x_intensity
                    break;
                case 1: 
                case 2: 
                    local_axis_intensities_arr[i] = js_state.y_intensity; // Use current y_intensity
                    break;
                case 3: // Axis 4 (JS_BUTTON_Y)
                     local_axis_intensities_arr[i] = js_state.x_intensity; // Use current x_intensity per original logic
                    break;
                case 4: // Axis 5 (JS_BUTTON_LB_NAIM)
                    local_axis_intensities_arr[i] = (10 - js_state.y_intensity); // Use current y_intensity
                    break;
                default:
                    local_axis_intensities_arr[i] = default_intensity;
                    break;
            }
        } else {
            local_axis_intensities_arr[i] = default_intensity;
        }
    }
}

void* processing_thread(void* arg) {
    int vel_lin = 0, vel_rot = 0;
    int lin_dir = 1, rot_dir = 0;
    uint16_t timestamp = 0;
    double sensitivity = 0.1;
    int axis_intensities[6] = {5, 5, 5, 5, 5, 5}; 
    
    while (running) {
        uint16_t current_js_op_mode;
        int naim_power_save_status; 

        pthread_mutex_lock(&state_mutex);
        current_js_op_mode = js_state.op_mode;
        timestamp += 1;
        sensitivity = sensitivity_calculation(js_state.rb_pressed, js_state.lb_pressed);
        
        pthread_mutex_unlock(&state_mutex);
        
        uint16_t final_op_mode = current_js_op_mode;

        final_op_mode |= OP_MODE_REQUEST_GPS; 
        // final_op_mode |= OP_MODE_SCIENCE_DATA; // Control these via terminal or other input
        final_op_mode |= OP_MODE_LED_DATA;
        
        if (final_op_mode & OP_MODE_DRIVE_ACTIVE) {
            calculate_movement_values(sensitivity, &vel_lin, &vel_rot, &lin_dir, &rot_dir);
        } else {
            vel_lin = 0; vel_rot = 0; lin_dir = 1; rot_dir = 0;
        }
        
        if (final_op_mode & OP_MODE_NAIM_ACTIVE) {
            if (js_state.naim_power_saving_active) { 
                axis_intensities[0] = 5;
                axis_intensities[1] = 0;
                axis_intensities[2] = 0;
                axis_intensities[3] = 0;
                axis_intensities[4] = 0;
                axis_intensities[5] = 0;
            } else {
                pthread_mutex_lock(&state_mutex);
                update_local_axis_intensities_array(axis_intensities);
                pthread_mutex_unlock(&state_mutex);
            }
        } else {
            for (int i = 0; i < 6; i++) axis_intensities[i] = 5;
        }
        
        binary_buffer_length = 0; 

        if (!(final_op_mode & OP_MODE_DRIVE_ACTIVE) && !(final_op_mode & OP_MODE_NAIM_ACTIVE)) {
            create_idle_message(timestamp, final_op_mode);
            add_axis_intensities_to_buffer((int[]){5,5,5,5,5,5}, 6); // Default if no Naim
        } else if ((final_op_mode & OP_MODE_DRIVE_ACTIVE) && sensitivity == 0) {
            create_idle_message(timestamp, final_op_mode);
            if (final_op_mode & OP_MODE_NAIM_ACTIVE) {
                add_axis_intensities_to_buffer(axis_intensities, 6);
            } else {
                add_axis_intensities_to_buffer((int[]){5,5,5,5,5,5}, 6);
            }
        } else if (final_op_mode & OP_MODE_DRIVE_ACTIVE) {
            create_movement_message(timestamp, final_op_mode, rot_dir, vel_rot, lin_dir, vel_lin);
            if (final_op_mode & OP_MODE_NAIM_ACTIVE) {
                add_axis_intensities_to_buffer(axis_intensities, 6);
            } else {
                add_axis_intensities_to_buffer((int[]){5,5,5,5,5,5}, 6);
            }
        } else if (final_op_mode & OP_MODE_NAIM_ACTIVE) {
            create_idle_message(timestamp, final_op_mode); // Send idle for drive part
            add_axis_intensities_to_buffer(axis_intensities, 6);
        } else {
             // Should not be reached if logic above is complete, but as a fallback:
            create_idle_message(timestamp, final_op_mode);
            add_axis_intensities_to_buffer((int[]){5,5,5,5,5,5}, 6);
        }

        if (final_op_mode & OP_MODE_SCIENCE_DATA) {
            for (int i = 0; i < SCIENCE_DATA_SIZE; ++i) { // Use SCIENCE_DATA_SIZE constant
                binary_buffer[binary_buffer_length++] = science_data[i];
            }
        }

        if (final_op_mode & OP_MODE_LED_DATA) {
            for (int i = 0; i < LED_MSG_SIZE; ++i) { 
                binary_buffer[binary_buffer_length++] = led_msg[i];
            }
        }

         printf("Timestamp:%05u Opmode:%04X | Drive: LDir=%d LVel=%04d RDir=%d RVel=%03d | Naim:[%d%d%d%d%d%d] PwrS:%d | BufLen:%d\n", 
                timestamp, final_op_mode,
                lin_dir, vel_lin, rot_dir, vel_rot,
                axis_intensities[0], axis_intensities[1], axis_intensities[2], 
                axis_intensities[3], axis_intensities[4], axis_intensities[5],
                naim_power_save_status,
                binary_buffer_length);

        usleep(1000); 
    }
    return NULL;
}

int main() {
    struct sockaddr_in server_addr;
    pthread_t sender_thread, receiver_thread, drive_thread, naim_thread, proc_thread, orangepi_thread, xavier_thread;

    // Socket oluştur
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // printf("UDP sunucusu %s:%d\n", SERVER_IP, SERVER_PORT);

    /*         GPT Sağ olsun Threading        */
    if (pthread_create(&sender_thread, NULL, send_data, NULL) != 0) {
        perror("Sender thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&receiver_thread, NULL, receive_data, NULL) != 0) {
        perror("Receiver thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    
    // Create separate threads for each joystick
    if (pthread_create(&drive_thread, NULL, drive_joystick_handler, NULL) != 0) {
        perror("Drive joystick thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&naim_thread, NULL, naim_joystick_handler, NULL) != 0) {
        perror("Naim joystick thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    
    // Create processing thread
    if (pthread_create(&proc_thread, NULL, processing_thread, NULL) != 0) {
        perror("Processing thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    
    // Create OrangePi message thread
    if (pthread_create(&orangepi_thread, NULL, orangepi_message_thread, NULL) != 0) {
        perror("OrangePi thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // Create Xavier message thread
    if (pthread_create(&xavier_thread, NULL, xavier_message_thread, NULL) != 0) {
        perror("Xavier thread creation failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }


    while (1) {
        printf("Terminal Test paketi: ");
        fgets(last_message, BUFFER_SIZE, stdin);

        /* Yeni satır karakteri de iletiliyor */
        last_message[strcspn(last_message, "\n")] = 0;
    }

    running = 0;
    pthread_join(sender_thread, NULL);
    pthread_join(receiver_thread, NULL);
    pthread_join(drive_thread, NULL);
    pthread_join(naim_thread, NULL);
    pthread_join(proc_thread, NULL);
    pthread_join(orangepi_thread, NULL);
    pthread_join(xavier_thread, NULL);
    close(sockfd);
    return 0;
}
