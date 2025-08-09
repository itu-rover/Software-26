#ifndef NETWORK_H
#define NETWORK_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "config.h"

// External variables
extern int sockfd;
extern unsigned char binary_buffer[32];
extern int binary_buffer_length;
extern char gps_msg[16];
extern struct sockaddr_in last_client_addr;
extern struct sockaddr_in orangepi_client_addr;
extern socklen_t last_client_addr_len;
extern int client_addr_set;

// Make science_data and led_msg available to other files
extern unsigned char science_data[4];
extern unsigned char led_msg[3];

// Function declarations
int create_socket(void);
int create_xavier_socket(void);
void* send_data(void* arg);
void* receive_data(void* arg);

#endif /* NETWORK_H */ 