#ifndef MESSAGE_FORMATTER_H
#define MESSAGE_FORMATTER_H

#include <stdint.h>

// Message structure for cleaner handling
typedef struct {
    uint16_t timestamp;
    uint16_t op_mode;
    uint8_t rot_dir;
    char vel_rot[3];
    uint8_t lin_dir;
    uint32_t vel_lin;
    uint8_t axis_intensities[6];

} MessageData;

// Helper functions
void format_message_for_mode(MessageData* msg, unsigned char* buffer, int* buffer_length);
void add_timestamp_to_buffer(uint16_t timestamp, unsigned char* buffer, int* pos);
void add_velocity_to_buffer(uint16_t velocity, int digits, unsigned char* buffer, int* pos);
void add_axis_intensities_to_buffer(uint8_t* intensities, int count, unsigned char* buffer, int* pos);

// Add these lines to define the global variables
// unsigned char science_data[4] = {1}; // Definition moved to network.c
// unsigned char led_msg[3] = {1};    // Definition moved to network.c

#endif 