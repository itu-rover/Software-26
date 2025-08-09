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
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef SERVER_IP
#define SERVER_IP "192.168.1.6"
#endif

#define ORANGEPI_IP "192.168.1.98"
#define ORANGEPI_PORT 5678

// 6103 is for test only change to 6102
#define SERVER_PORT 6102
#define BUFFER_SIZE 1024

#define JS_DEVICE "/dev/input/js0"
#define JS_DEVICE_NAIM "/dev/input/js1"

volatile int running = 1;
volatile unsigned short int TIMESTAMP;
static double SENSITIVITY = 0.1;

// Global counter for timestamp
volatile uint16_t message_counter = 0;
pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;

/* İlk Client Server'a istek atması gerekmektedir */
struct sockaddr_in last_client_addr;
struct sockaddr_in orangepi_client_addr; // For sending to ORANGEPI
socklen_t last_client_addr_len;

int sockfd;
int orangepi_sockfd; // Socket for ORANGEPI

/* Son mesaj sürekli gönderilecek*/
char last_message[BUFFER_SIZE] = "";
int client_addr_set = 0;

// Binary buffer for sending data
unsigned char binary_buffer[32];
int binary_buffer_length = 0;

// For ORANGEPI messages
char msg_button6[5] = "S71F";
char msg_button7[5] = "S31F";
char msg_default[5] = "S51F";

char gps_msg[16] = {'0'};
char science_msg[4] = {0};

// Science mode control
volatile int science_mode_enabled = 0;
pthread_mutex_t science_mutex = PTHREAD_MUTEX_INITIALIZER;

// Thread synchronization for shared state
pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;

// Shared state between joystick threads
typedef struct {
    int rb_pressed;
    int lb_pressed;
    int left_y;
    int right_y;
    int right_x;
    unsigned short int drive_timestamp;
    
    // For Naim's controls
    int btn_axis1;
    int btn_axis2;
    int btn_axis3;
    int btn_axis4;
    int btn_axis5;
    int btn_axis6;
    int button6;
    int button7;
    int active_axes;
    int x_intensity;
    int y_intensity;
    unsigned short int naim_timestamp;

    bool power_saving_mode;
    
    // Common state
    unsigned short int op_mode;
} JoystickState;

JoystickState js_state = {0};


void* send_data(void* arg) {
    int interval_us = 10000; // 100 Hz (10 ms aralık)

    // Initialize ORANGEPI client address
    memset(&orangepi_client_addr, 0, sizeof(orangepi_client_addr));
    orangepi_client_addr.sin_family = AF_INET;
    orangepi_client_addr.sin_addr.s_addr = inet_addr(ORANGEPI_IP);
    orangepi_client_addr.sin_port = htons(ORANGEPI_PORT);

    while (running) {
        if (binary_buffer_length > 0 && client_addr_set) {
            if (sendto(sockfd, binary_buffer, binary_buffer_length, 0, (struct sockaddr*)&last_client_addr, last_client_addr_len) < 0) {
                perror("Send failed");
            }
        }
        usleep(interval_us);
    }
    return NULL;
}

void* receive_data(void* arg) {
    char buffer[BUFFER_SIZE];
    ssize_t received_bytes;

    while (running) {
        last_client_addr_len = sizeof(last_client_addr);
        received_bytes = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&last_client_addr, &last_client_addr_len);

        if (received_bytes < 0) {
            perror("Receive failed");
            continue;
        }

        /* Process received data */
        buffer[received_bytes] = '\0';
        printf("Received bytes: %ld\n", received_bytes);
        
        // Check if the received data is GPS data with timestamp (20 bytes total)
        if (received_bytes >= 20) {  
            // First 4 bytes are timestamp from STM, skip them
            uint32_t stm_timestamp = *((uint32_t*)(buffer));
            
            // The actual GPS data starts at offset 4
            char* gps_data_start = buffer + 4;
            
            // Copy GPS data to gps_msg (16 bytes)
            memcpy(gps_msg, gps_data_start, 16);
            
            // Extract latitude (bytes 0-3) as signed int
            int32_t latitude_raw = *((int32_t*)(gps_data_start));
            double latitude = latitude_raw * 1e-7; // Convert to actual GPS value
            
            // Extract longitude (bytes 4-7) as signed int
            int32_t longitude_raw = *((int32_t*)(gps_data_start + 4));
            double longitude = longitude_raw * 1e-7; // Convert to actual GPS value
            
            // Extract course (bytes 8-11) as signed int
            int32_t course_raw = *((int32_t*)(gps_data_start + 8));
            double course = course_raw * 1e-5; // Convert to actual GPS value
            
            // Extract altitude (bytes 12-15) as signed int
            int32_t speed_over_ground_raw = *((int32_t*)(gps_data_start + 12));
            double speed_over_ground = speed_over_ground_raw * 1e-3; // Convert to actual GPS value
            
            printf("GPS data received (STM timestamp: %u, Joystick timestamp: %u):\n", 
                   stm_timestamp, TIMESTAMP);
            printf("Lat: %.7f (raw: %d)\n", latitude, latitude_raw);
            printf("Long: %.7f (raw: %d)\n", longitude, longitude_raw);
            printf("Course: %.5f (raw: %d)\n", course, course_raw);
            printf("Speed over ground: %.3f (raw: %d)\n", speed_over_ground, speed_over_ground_raw);
        } else {
            /* Debug message for non-GPS data */
            printf("STM32'den gelen (hex): ");
            for (int i = 0; i < received_bytes; i++) {
                // printf("%02X ", (unsigned char)buffer[i]);
            }
            printf("\n");
        }

        // İlk mesajı alınca istemci adresini kaydet
        if (!client_addr_set) {
            printf("STM32 Bağlandı!!!!!!\n");
            client_addr_set = 1;
        }
    }
    return NULL;
}

// Drive joystick handler thread
void* drive_joystick_handler(void* arg) {
    int js_fd;
    struct js_event js;
    
    // Open Drive joystick
    js_fd = open(JS_DEVICE, O_RDONLY);

    if (js_fd >= 0) {
        fcntl(js_fd, F_SETFL, O_NONBLOCK); // Non-blocking mode
        printf("Drive system joystick connected at %s\n", JS_DEVICE);
        
        pthread_mutex_lock(&state_mutex);
        js_state.op_mode |= 1;  // Set drive joystick bit
        pthread_mutex_unlock(&state_mutex);
    } else {
        printf("Drive joystick not found at %s\n", JS_DEVICE);
    }

    while (running) {
        if (js_fd >= 0) {
            ssize_t bytes = read(js_fd, &js, sizeof(struct js_event));
            
            if (bytes == sizeof(struct js_event)) {
                js.type &= ~JS_EVENT_INIT; // Clear init events
                
                pthread_mutex_lock(&state_mutex);
                
                // Update timestamp
                js_state.drive_timestamp = (unsigned short int)(js.time & 0xFFFF);
                
                if (js.type == JS_EVENT_BUTTON) {
                    if (js.number == 5) { // RB button
                        js_state.rb_pressed = js.value;
                    } 
                    else if (js.number == 4) { // LB button
                        js_state.lb_pressed = js.value;
                    }
                } else if (js.type == JS_EVENT_AXIS) {
                    if (js.number == 1) { // Left Y
                        // Apply deadzone
                        if (abs(js.value) < 3275) { // ~10% deadzone
                            js_state.left_y = 0;
                        } else {
                            js_state.left_y = js.value;
                        }
                    }
                    if (js.number == 4) { // Right Y
                        // Apply deadzone
                        if (abs(js.value) < 3275) { // ~10% deadzone
                            js_state.right_y = 0;
                        } else {
                            js_state.right_y = js.value;
                        }
                    }
                    if (js.number == 3) { // Right X
                        // Apply deadzone
                        if (abs(js.value) < 3275) { // ~10% deadzone
                            js_state.right_x = 0;
                        } else {
                            js_state.right_x = js.value;
                        }
                    }
                }
                
                pthread_mutex_unlock(&state_mutex);
                
            } else if (bytes < 0 && errno != EAGAIN) {
                printf("Drive joystick read error: %s (errno: %d)\n", strerror(errno), errno);
                if (errno == ENODEV || errno == EBADF) {
                    printf("Drive joystick disconnected, closing fd\n");
                    close(js_fd);
                    js_fd = -1;
                    
                    pthread_mutex_lock(&state_mutex);
                    js_state.op_mode &= ~1;  // Clear bit for drive joystick
                    pthread_mutex_unlock(&state_mutex);
                }
            }
        } else {
            // Try to reconnect periodically
            js_fd = open(JS_DEVICE, O_RDONLY);
            if (js_fd >= 0) {
                fcntl(js_fd, F_SETFL, O_NONBLOCK);
                printf("Drive system joystick reconnected at %s\n", JS_DEVICE);
                
                pthread_mutex_lock(&state_mutex);
                js_state.op_mode |= 1;  // Set drive joystick bit
                pthread_mutex_unlock(&state_mutex);
            }
        }
        usleep(1000); // 1 ms sleep
    }
    
    if (js_fd >= 0) {
        close(js_fd);
    }
    return NULL;
}

// Naim joystick handler thread
void* naim_joystick_handler(void* arg) {
    int js_fd_naim;
    struct js_event js_naim;
    

    // Open Naim joystick
    js_fd_naim = open(JS_DEVICE_NAIM, O_RDONLY);

    if (js_fd_naim >= 0) {
        fcntl(js_fd_naim, F_SETFL, O_NONBLOCK); // Non-blocking mode
        printf("Naim joystick connected at %s\n", JS_DEVICE_NAIM);
        
        pthread_mutex_lock(&state_mutex);
        js_state.op_mode |= 2;  // Set Naim joystick bit
        pthread_mutex_unlock(&state_mutex);
    } else {
        printf("Naim joystick not found at %s\n", JS_DEVICE_NAIM);
    }

    while (running) {
        if (js_fd_naim >= 0) {
            ssize_t bytes = read(js_fd_naim, &js_naim, sizeof(struct js_event));
            
            if (bytes == sizeof(struct js_event)) {
                js_naim.type &= ~JS_EVENT_INIT; // Clear init events
                
                pthread_mutex_lock(&state_mutex);
                
                // Update timestamp
                js_state.naim_timestamp = (unsigned short int)(js_naim.time & 0xFFFF);
                
                if (js_naim.type == JS_EVENT_BUTTON) {
                    if (js_naim.number == 0) { // A button - Axis 1 (X-axis control)
                        js_state.btn_axis1 = js_naim.value;
                        if (js_naim.value)
                            js_state.active_axes |= (1 << 0);  // Set bit 0
                        else
                            js_state.active_axes &= ~(1 << 0); // Clear bit 0
                    }
                    else if (js_naim.number == 1) { // B button - Axis 3 (Y-axis control)
                        js_state.btn_axis2 = js_naim.value;
                        if (js_naim.value)
                            js_state.active_axes |= (1 << 1);  // Set bit 1
                        else
                            js_state.active_axes &= ~(1 << 1); // Clear bit 1
                    }
                    else if (js_naim.number == 2) { // X button - Axis 3 (X-axis control)
                        js_state.btn_axis3 = js_naim.value;
                        if (js_naim.value)
                            js_state.active_axes |= (1 << 2);  // Set bit 2
                        else
                            js_state.active_axes &= ~(1 << 2); // Clear bit 2
                    }
                    else if (js_naim.number == 3) { // Y button - Axis 4 (Y-axis control)
                        js_state.btn_axis4 = js_naim.value;
                        if (js_naim.value)
                            js_state.active_axes |= (1 << 3);  // Set bit 3
                        else
                            js_state.active_axes &= ~(1 << 3); // Clear bit 3
                    }
                    else if (js_naim.number == 4) { // LB button - Axis 5 (X-axis control)
                        js_state.btn_axis5 = js_naim.value;
                        if (js_naim.value)
                            js_state.active_axes |= (1 << 4);  // Set bit 4
                        else
                            js_state.active_axes &= ~(1 << 4); // Clear bit 4
                    }
                    else if (js_naim.number == 5) { // RB button - Axis 6 (Y-axis control)
                        js_state.btn_axis6 = js_naim.value;
                        if (js_naim.value)
                            js_state.active_axes |= (1 << 5);  // Set bit 5
                        else
                            js_state.active_axes &= ~(1 << 5); // Clear bit 5
                    }
                    else if (js_naim.number == 6) {
                        js_state.button6 = js_naim.value;
                    }
                    else if (js_naim.number == 7) {
                        js_state.button7 = js_naim.value;
                    }
                    else if (js_naim.number == 8) {
                        js_state.power_saving_mode ^= js_naim.value;
                    }
                    pthread_mutex_unlock(&state_mutex);
                    
                    // OrangePi messages are now handled by the dedicated orangepi_message_thread
                }
                else if (js_naim.type == JS_EVENT_AXIS) {
                    if (js_naim.number == 3) { // Right stick X
                        if (abs(js_naim.value) < 3275) { // ~10% deadzone
                            js_state.x_intensity = 5; // Default value when inactive
                        } else {
                            js_state.x_intensity = (int)(((1 + (js_naim.value / 32760.0)) / 2) * 8 + 1);
                            js_state.x_intensity = js_state.x_intensity < 1 ? 1 : 
                                            js_state.x_intensity > 9 ? 9 : js_state.x_intensity;
                        }
                    }
                    else if (js_naim.number == 4) { // Right stick Y
                        if (abs(js_naim.value) < 3275) { // ~10% deadzone
                            js_state.y_intensity = 5; // Default value when inactive
                        } else {
                            js_state.y_intensity = (int)(((1 + (js_naim.value / 32760.0)) / 2) * 8 + 1);
                            js_state.y_intensity = js_state.y_intensity < 1 ? 1 : 
                                            js_state.y_intensity > 9 ? 9 : js_state.y_intensity;
                        }
                    }
                    pthread_mutex_unlock(&state_mutex);
                }
                else {
                    pthread_mutex_unlock(&state_mutex);
                }
            } else if (bytes < 0 && errno != EAGAIN) {
                printf("Naim joystick read error: %s (errno: %d)\n", strerror(errno), errno);
                if (errno == ENODEV || errno == EBADF) {
                    printf("Naim joystick disconnected, closing fd\n");
                    close(js_fd_naim);
                    js_fd_naim = -1;
                    
                    pthread_mutex_lock(&state_mutex);
                    js_state.op_mode &= ~2;  // Clear bit for Naim joystick
                    pthread_mutex_unlock(&state_mutex);
                }
            }
        } else {
            // Try to reconnect periodically
            js_fd_naim = open(JS_DEVICE_NAIM, O_RDONLY);
            if (js_fd_naim >= 0) {
                fcntl(js_fd_naim, F_SETFL, O_NONBLOCK);
                printf("Naim joystick reconnected at %s\n", JS_DEVICE_NAIM);
                
                pthread_mutex_lock(&state_mutex);
                js_state.op_mode |= 2;  // Set Naim joystick bit
                pthread_mutex_unlock(&state_mutex);
            }
        }
        usleep(1000); // 1 ms sleep
    }
    
    if (js_fd_naim >= 0) {
        close(js_fd_naim);
    }
    return NULL;
}

// Create a socket for OrangePi communication
int create_orangepi_socket() {
    int sock_fd;
    
    // Socket creation
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("OrangePi socket creation failed");
        return -1;
    }
    
    return sock_fd;
}

// OrangePi message sending thread
void* orangepi_message_thread(void* arg) {
    int orangepi_sockfd = create_orangepi_socket();
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
        char* msg_to_send;
        
        // Check button states
        pthread_mutex_lock(&state_mutex);
        int button6 = js_state.button6;
        int button7 = js_state.button7;
        pthread_mutex_unlock(&state_mutex);
        
        // Determine which message to send
        if (button6 && !button7) {
            msg_to_send = msg_button6;
        } else if (!button6 && button7) {
            msg_to_send = msg_button7;
        } else {
            msg_to_send = msg_default;
        }
        
        // Send the message to OrangePi
        if (sendto(orangepi_sockfd, msg_to_send, 4, 0, 
                (struct sockaddr*)&orangepi_client_addr, sizeof(orangepi_client_addr)) < 0) {
            perror("Send to OrangePi failed");
        } else {
            printf("Sent message '%s' to %s:%d\n", 
                    msg_to_send, ORANGEPI_IP, ORANGEPI_PORT);
        }
        
        usleep(message_interval);
    }
    
    close(orangepi_sockfd);
    return NULL;
}

// Processing thread that uses joystick data to create messages
void* processing_thread(void* arg) {
    int vel_lin = 0;
    int vel_rot = 0;
    char lin_dir = 1;
    char rot_dir = 1;
    unsigned short int op_mode = 0;
    double sensitivity = 0.1;
    unsigned short int timestamp = 0;
    int axis_intensities[6] = {5, 5, 5, 5, 5, 5};

    while (running) {
        pthread_mutex_lock(&state_mutex);
        printf("op_mode: %hu\n", js_state.op_mode);
        // Get current state
        op_mode = js_state.op_mode;
        op_mode |= 0x04;
        
        // Update timestamp from most recent event
        if (js_state.drive_timestamp > js_state.naim_timestamp) {
            timestamp = js_state.drive_timestamp;
        } else {
            timestamp = js_state.naim_timestamp;
        }
        TIMESTAMP = timestamp;
        
        // Speed control based on button combinations
        if (!js_state.rb_pressed && !js_state.lb_pressed) {
            // No buttons pressed: No movement
            sensitivity = 0;
        }
        else if (js_state.rb_pressed && !js_state.lb_pressed) {
            // Only RB pressed: Slow speed
            sensitivity = 0.1;
        }
        else if (!js_state.rb_pressed && js_state.lb_pressed) {
            // Only LB pressed: Fast speed
            sensitivity = 0.3;
        }
        else if (js_state.rb_pressed && js_state.lb_pressed) {
            // Both LB and RB pressed: Medium speed
            sensitivity = 0.2;
        }
        
        // Calculate movement values
        if (sensitivity > 0) {
            vel_rot = ((js_state.left_y - js_state.right_y) * (999.0 / (32768.0 * 2.0))) * sensitivity;
            vel_lin = ((js_state.left_y + js_state.right_y) / 2.0) * (9999.0 / 32768.0) * sensitivity;
            
            if (vel_lin < 0) {
                lin_dir = 1;
            } else {
                lin_dir = 0;
            }
            
            if (vel_rot > 0) {
                rot_dir = 1;
            } else {
                rot_dir = 0;
            }
            
            vel_lin = abs((int)vel_lin);
            vel_rot = abs((int)vel_rot);
        } else {
            vel_rot = 0;
            vel_lin = 0;
            rot_dir = 0;
            lin_dir = 1;
        }
        
        // Update axis intensities based on naim joystick
        if (js_state.active_axes & (1 << 0)) axis_intensities[0] = 10 - js_state.x_intensity;
        else axis_intensities[0] = 5;
        
        if (js_state.active_axes & (1 << 1)) axis_intensities[1] = js_state.y_intensity;
        else axis_intensities[1] = 5;
        
        if (js_state.active_axes & (1 << 2)) axis_intensities[2] = js_state.y_intensity;
        else axis_intensities[2] = 5;
        
        if (js_state.active_axes & (1 << 3)) axis_intensities[3] = js_state.x_intensity;
        else axis_intensities[3] = 5;
        
        if (js_state.active_axes & (1 << 4)) axis_intensities[4] = 10 - js_state.y_intensity;
        else axis_intensities[4] = 5;
        
        if (js_state.active_axes & (1 << 5)) axis_intensities[5] = js_state.x_intensity;
        else axis_intensities[5] = 5;
        
        pthread_mutex_unlock(&state_mutex);
        
        // Format binary message based on operational mode
        if (op_mode == 0) {
            // Default idle message
            binary_buffer_length = 0;
            
            // Timestamp (2 bytes) - Big endian (high byte first)
            binary_buffer[binary_buffer_length++] = '0' + ((timestamp >> 8) & 0xFF);  // High byte first (ASCII)
            binary_buffer[binary_buffer_length++] = '0' + (timestamp & 0xFF);         // Low byte second (ASCII)
            
            // Operation mode (2 bytes) - Big endian
            binary_buffer[binary_buffer_length++] = '0';   // High byte first (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '0';   // Low byte second (ASCII for 0)
            
            // Rotation direction (1 byte)
            binary_buffer[binary_buffer_length++] = '0';    // rot_dir (ASCII for 0)
            
            // Rotation velocity (3 bytes) - Big endian
            binary_buffer[binary_buffer_length++] = '0';    // vel_rot high byte (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '0';    // vel_rot middle byte (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '0';    // vel_rot low byte (ASCII for 0)
            
            // Linear direction (1 byte)
            binary_buffer[binary_buffer_length++] = '1';    // lin_dir (ASCII for 1)
            
            // Linear velocity (4 bytes) - Big endian
            binary_buffer[binary_buffer_length++] = '0';    // vel_lin highest byte (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '0';    // vel_lin high middle byte (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '0';    // vel_lin low middle byte (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '0';    // vel_lin lowest byte (ASCII for 0)
            
            // Default axis intensities
            for (int i = 0; i < 6; i++) {
                binary_buffer[binary_buffer_length++] = '5';  // ASCII for 5
            }
        }

        if (op_mode & 0x1) {
           if (sensitivity == 0) {
               // Default idle message for op_mode 1
               binary_buffer_length = 0;
               
               // Timestamp (2 bytes) - Big endian (high byte first)
               binary_buffer[binary_buffer_length++] = '0' + ((timestamp >> 8) & 0xFF);  // High byte first (ASCII)
               binary_buffer[binary_buffer_length++] = '0' + (timestamp & 0xFF);         // Low byte second (ASCII)
               
               // Operation mode (2 bytes) - Big endian
               binary_buffer[binary_buffer_length++] = '0';   // High byte first (ASCII for 0)
               binary_buffer[binary_buffer_length++] = '1';   // Low byte second (ASCII for 1)
               
               // Rotation direction (1 byte)
               binary_buffer[binary_buffer_length++] = '0';    // rot_dir (ASCII for 0)
               
               // Rotation velocity (3 bytes) - Big endian
               binary_buffer[binary_buffer_length++] = '0';    // vel_rot high byte (ASCII for 0)
               binary_buffer[binary_buffer_length++] = '0';    // vel_rot middle byte (ASCII for 0)
               binary_buffer[binary_buffer_length++] = '0';    // vel_rot low byte (ASCII for 0)
               
               // Linear direction (1 byte)
               binary_buffer[binary_buffer_length++] = '1';    // lin_dir (ASCII for 1)
               
               // Linear velocity (4 bytes) - Big endian
               binary_buffer[binary_buffer_length++] = '0';    // vel_lin highest byte (ASCII for 0)
               binary_buffer[binary_buffer_length++] = '0';    // vel_lin high middle byte (ASCII for 0)
               binary_buffer[binary_buffer_length++] = '0';    // vel_lin low middle byte (ASCII for 0)
               binary_buffer[binary_buffer_length++] = '0';    // vel_lin lowest byte (ASCII for 0)
           }
           else {
            // Format binary message for op_mode 1
            binary_buffer_length = 0;
            
            // Timestamp (2 bytes) - Big endian (high byte first)
            binary_buffer[binary_buffer_length++] = '0' + ((timestamp >> 8) & 0x0F);  // High byte first (ASCII)
            binary_buffer[binary_buffer_length++] = '0' + (timestamp & 0x0F);         // Low byte second (ASCII)
            
            // Operation mode (2 bytes) - Big endian
            binary_buffer[binary_buffer_length++] = '0';   // High byte first (ASCII for 0)
            binary_buffer[binary_buffer_length++] = '1';   // Low byte second (ASCII for 1)
            
            // Rotation direction (1 byte)
            binary_buffer[binary_buffer_length++] = '0' + (rot_dir & 0x0F);  // ASCII for rot_dir
            
            // Rotation velocity (3 bytes) - Big endian - Convert to ASCII
            binary_buffer[binary_buffer_length++] = '0' + ((vel_rot / 100) % 10);    // Hundreds place
            binary_buffer[binary_buffer_length++] = '0' + ((vel_rot / 10) % 10);     // Tens place
            binary_buffer[binary_buffer_length++] = '0' + (vel_rot % 10);            // Ones place
            
            // Linear direction (1 byte)
            binary_buffer[binary_buffer_length++] = '0' + (lin_dir & 0x0F);  // ASCII for lin_dir
            
            // Linear velocity (4 bytes) - Big endian - Convert to ASCII
            binary_buffer[binary_buffer_length++] = '0' + ((vel_lin / 1000) % 10);    // Thousands place
            binary_buffer[binary_buffer_length++] = '0' + ((vel_lin / 100) % 10);     // Hundreds place
            binary_buffer[binary_buffer_length++] = '0' + ((vel_lin / 10) % 10);      // Tens place
            binary_buffer[binary_buffer_length++] = '0' + (vel_lin % 10);             // Ones place
            
            printf("TIMESTAMP: %hu, op_mode: %hu, rot_dir: %d, vel_rot: %d, lin_dir: %d, vel_lin: %d\n", 
                   timestamp, op_mode, rot_dir, vel_rot, lin_dir, vel_lin);
           }
        }

        if (op_mode & 0x2) {
           for (int i = 0; i < 6; i++) {
               binary_buffer[binary_buffer_length++] = '0' + (axis_intensities[i] & 0x0F);
           }

            printf("TIMESTAMP: %hu, op_mode: %hu, rot_dir: %d, vel_rot: %d, lin_dir: %d, vel_lin: %d, axes: %d %d %d %d %d %d   ", 
                   timestamp, op_mode, rot_dir, vel_rot, lin_dir, vel_lin, 
                   axis_intensities[0], axis_intensities[1], axis_intensities[2], 
                   axis_intensities[3], axis_intensities[4], axis_intensities[5]);
        }

        if (op_mode & 0x4) {
            // Send the science data
            for (int i = 0; i < 4; i++) {
                binary_buffer[binary_buffer_length++] = '0' + (science_msg[i]);
            }
        }
        printf("opmode: %d\n", op_mode);
        printf("binary_buffer: %s\n", binary_buffer);
        usleep(10000); // 10ms sleep
    }
    return NULL;
}

int main() {
    struct sockaddr_in server_addr;
    pthread_t sender_thread, receiver_thread, drive_thread, naim_thread, proc_thread, orangepi_thread;

    // Socket oluştur
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Sunucu adresini yapılandır
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_port = htons(SERVER_PORT);

    // Soketi belirtilen IP ve port ile bağla
    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("UDP sunucusu %s:%d\n", SERVER_IP, SERVER_PORT);

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

    while (1) {
        printf("Terminal Test paketi: ");
        fgets(last_message, BUFFER_SIZE, stdin);

        /* Yeni satır karakteri de iletiliyor */
        last_message[strcspn(last_message, "\n")] = 0;
        
        // Convert text input to binary for testing
        // Simple conversion: just fill the binary buffer with test values
        binary_buffer_length = 0;
        
        // Add a test timestamp (2 bytes) - Big endian (high byte first)
        binary_buffer[binary_buffer_length++] = '0';  // High byte first (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '1';  // Low byte second (ASCII for 1)
        
        // Add the rest of the message as binary data (simple example)
        binary_buffer[binary_buffer_length++] = '0';  // op_mode high byte (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '7';  // op_mode low byte (ASCII for 0)

        binary_buffer[binary_buffer_length++] = '0';  // rot_dir (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_rot hundreds place (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_rot tens place (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_rot ones place (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // lin_dir (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_lin thousands place (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_lin hundreds place (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_lin tens place (ASCII for 0)
        binary_buffer[binary_buffer_length++] = '0';  // vel_lin ones place (ASCII for 0)

        binary_buffer[binary_buffer_length++] = '1';
        binary_buffer[binary_buffer_length++] = '0';
        binary_buffer[binary_buffer_length++] = '0';
        binary_buffer[binary_buffer_length++] = '1';
    }

    running = 0;
    pthread_join(sender_thread, NULL);
    pthread_join(receiver_thread, NULL);
    pthread_join(drive_thread, NULL);
    pthread_join(naim_thread, NULL);
    pthread_join(proc_thread, NULL);
    pthread_join(orangepi_thread, NULL);
    close(sockfd);
    return 0;
}
