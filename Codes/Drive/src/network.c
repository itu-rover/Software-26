#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

// Project headers
#include "config.h"
#include "network.h"
#include "joystick_handler.h"

// Global variable definitions
unsigned char science_data[4] = {1,1,1,1};
unsigned char led_msg[3] = {49,48,48};

// Global variables
int sockfd;
unsigned char binary_buffer[32];
int binary_buffer_length;
char gps_msg[16];
struct sockaddr_in last_client_addr;
struct sockaddr_in orangepi_client_addr;
socklen_t last_client_addr_len;
int client_addr_set = 0;

// Constants
#define BUFFER_SIZE 1024

// Forward declarations
extern volatile int running;

int create_socket() {
    int sock_fd;
    
    // Socket creation
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return -1;
    }
    return sock_fd;
}

int create_xavier_socket() {
    int sock_fd;
    
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Xavier socket creation failed");
        return -1;
    }
    return sock_fd;
}

void* send_data(void* arg) {
    int interval_us = 10000; // 100 Hz (10 ms aralık)

    memset(&orangepi_client_addr, 0, sizeof(orangepi_client_addr));
    orangepi_client_addr.sin_family = AF_INET;
    orangepi_client_addr.sin_addr.s_addr = inet_addr(ORANGEPI_IP);
    orangepi_client_addr.sin_port = htons(ORANGEPI_PORT);

    while (running) {
        if (binary_buffer_length > 0 && client_addr_set) {
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &last_client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            int client_port = ntohs(last_client_addr.sin_port);

            printf("SEND -> %s:%d Data: ", client_ip, client_port);
            for (int i = 0; i < binary_buffer_length; ++i) {
                printf("%02X ", binary_buffer[i]);
            }
            printf("(Len: %d)", binary_buffer_length);

            if (sendto(sockfd, binary_buffer, binary_buffer_length, 0, (struct sockaddr*)&last_client_addr, last_client_addr_len) < 0) {
                printf(" - FAILED: %s\n", strerror(errno));
            } else {
                printf(" - SENT\n");
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

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &last_client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        int client_port = ntohs(last_client_addr.sin_port);

        if (received_bytes < 0) {
            printf("RECV <- %s:%d Error: %s\n", client_ip, client_port, strerror(errno));
            continue;
        }

        buffer[received_bytes] = '\0';

        printf("RECV <- %s:%d ", client_ip, client_port);

        if (received_bytes >= 20) {  
            uint32_t stm_timestamp = *((uint32_t*)(buffer));
            char* gps_data_start = buffer + 28;
            memcpy(gps_msg, gps_data_start, 16);
            
            int32_t latitude_raw = *((int32_t*)(gps_data_start));
            double latitude = latitude_raw * 1e-7;
            int32_t longitude_raw = *((int32_t*)(gps_data_start + 4));
            double longitude = longitude_raw * 1e-7;
            int32_t course_raw = *((int32_t*)(gps_data_start + 8));
            double course = course_raw * 1e-5;
            int32_t sog_raw = *((int32_t*)(gps_data_start + 12));
            double sog = sog_raw * 1e-3;
            
            printf("GPS: Lat=%.7f, Lon=%.7f, Crs=%.7f, Sog=%.7f (STM_TS: %u)", latitude, longitude, course, sog, stm_timestamp);
        } else {
            printf("Data: ");
            for (int i = 0; i < received_bytes; i++) {
                printf("%02X ", (unsigned char)buffer[i]);
            }
            printf("(Len: %zd)", received_bytes);
        }

        if (!client_addr_set) {
            printf(" (New Client: STM32 Connected!)");
            client_addr_set = 1;
        }
        printf("\n");
    }
    return NULL;
}
