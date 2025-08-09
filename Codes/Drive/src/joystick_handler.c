#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/joystick.h>
#include <pthread.h>
#include <math.h>

#include "config.h"
#include "constants.h"
#include "joystick_handler.h"
#include "message_formatter.h"

// Global variables
JoystickState js_state = {0};
pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int running = 1;

static int local_naim_power_saving_mode_toggle = 0; 

static void apply_deadzone(int value, int* output) {
    if (abs(value) < DEADZONE_THRESHOLD) {
        *output = 0;
    } else {
        *output = value;
    }
}

static void calculate_intensity(int value, int* output) {
    if (abs(value) < DEADZONE_THRESHOLD) {
        *output = 5; // Default value when inactive
    } else {
        *output = (int)(((1 + (value / 32760.0)) / 2) * 8 + 1);
        *output = *output < 1 ? 1 : (*output > 9 ? 9 : *output);
    }
}

// Helper function to handle joystick connection
static int init_joystick(const char* device_path, int* fd, int mode_bit) {
    *fd = open(device_path, O_RDONLY | O_NONBLOCK);
    
    if (*fd >= 0) {
        printf("Joystick connected at %s\n", device_path);
        
        pthread_mutex_lock(&state_mutex);
        js_state.op_mode |= mode_bit;
        if (mode_bit == OP_MODE_NAIM_ACTIVE) {
            local_naim_power_saving_mode_toggle = 0;
            js_state.naim_power_saving_active = 0;
            js_state.x_intensity = 5;
            js_state.y_intensity = 5;
        }
        pthread_mutex_unlock(&state_mutex);
        return 1;
    }
    
    if (mode_bit == OP_MODE_DRIVE_ACTIVE) {
        printf("Drive joystick not found at %s\n", device_path);
    } else {
        printf("Naim joystick not found at %s\n", device_path);
    }
    return 0;
}

static void handle_joystick_disconnect(int* fd, int mode_bit) {
    if (*fd < 0) return;

    if (mode_bit == OP_MODE_DRIVE_ACTIVE) {
        printf("Drive joystick disconnected\n");
    } else if (mode_bit == OP_MODE_NAIM_ACTIVE) {
        printf("Naim joystick disconnected\n");
    }
    close(*fd);
    *fd = -1;
    
    pthread_mutex_lock(&state_mutex);
    js_state.op_mode &= ~mode_bit;
    if (mode_bit == OP_MODE_DRIVE_ACTIVE) {
        js_state.left_y = 0;
        js_state.right_y = 0;
        js_state.right_x = 0;
        js_state.rb_pressed = 0;
        js_state.lb_pressed = 0;
    } else if (mode_bit == OP_MODE_NAIM_ACTIVE) {
        js_state.active_axes = 0;
        js_state.x_intensity = 5;
        js_state.y_intensity = 5;
        js_state.btn_axis1 = js_state.btn_axis2 = js_state.btn_axis3 = 0;
        js_state.btn_axis4 = js_state.btn_axis5 = js_state.btn_axis6 = 0;
        js_state.button6 = 0;
        js_state.button7 = 0;
        js_state.naim_power_saving_active = 0;
        local_naim_power_saving_mode_toggle = 0;
    }
    pthread_mutex_unlock(&state_mutex);
}

static void update_naim_js_state_for_power_mode(void) {

    if (local_naim_power_saving_mode_toggle) {
        // Power saving mode: 500000 for axis output, 
        js_state.x_intensity = 5; 
        js_state.y_intensity = 0; 
        for (int i = 0; i < 6; i++) { 
            if (js_state.active_axes & (1 << i)) {
                js_state.active_axes &= ~(1 << i);
            }
        }
    } else {
        js_state.x_intensity = 5;
        js_state.y_intensity = 5;
    }
    js_state.naim_power_saving_active = local_naim_power_saving_mode_toggle;
}

void* drive_joystick_handler(void* arg) {
    int js_fd = -1;
    struct js_event js;
    
    // Initialize joystick
    init_joystick(JS_DEVICE, &js_fd, OP_MODE_DRIVE_ACTIVE);
    
    while (running) {
        if (js_fd >= 0) {
            ssize_t bytes = read(js_fd, &js, sizeof(struct js_event));
            
            if (bytes == sizeof(struct js_event)) {
                js.type &= ~JS_EVENT_INIT;
                
                pthread_mutex_lock(&state_mutex);
                js_state.drive_timestamp = (unsigned short int)(js.time & 0xFFFF);
                
                if (js.type == JS_EVENT_BUTTON) {
                    switch (js.number) {
                        case JS_BUTTON_RB:
                            js_state.rb_pressed = js.value;
                            break;
                        case JS_BUTTON_LB:
                            js_state.lb_pressed = js.value;
                            break;
                    }
                } else if (js.type == JS_EVENT_AXIS) {
                    switch (js.number) {
                        case JS_AXIS_LEFT_Y:  // This is axis 1 for left stick Y
                            apply_deadzone(js.value, &js_state.left_y);
                            break;
                        case JS_AXIS_RIGHT_Y: // This is axis 4 for right stick Y
                            apply_deadzone(js.value, &js_state.right_y);
                            break;
                        case JS_AXIS_RIGHT_X: // This is axis 3 for right stick X
                            apply_deadzone(js.value, &js_state.right_x);
                            break;
                    }
                }
                pthread_mutex_unlock(&state_mutex);

                // Print drive joystick state
                printf("Drive JS: LY=%d, RY=%d, RX=%d, RB=%d, LB=%d, T=%hu\n",
                       js_state.left_y, js_state.right_y, js_state.right_x,
                       js_state.rb_pressed, js_state.lb_pressed, js_state.drive_timestamp);
                
                pthread_mutex_unlock(&state_mutex);
                
            } else if (bytes < 0 && errno != EAGAIN) {
                printf("Drive joystick read error: %s (errno: %d)\n", strerror(errno), errno);
                if (errno == ENODEV || errno == EBADF) {
                    handle_joystick_disconnect(&js_fd, OP_MODE_DRIVE_ACTIVE);
                }
            }
        } else {
            // Try to reconnect periodically but don't spam logs
            static int retry_count = 0;
            if (retry_count++ % 100 == 0) { // Try every ~1 second (100 * 10ms)
                init_joystick(JS_DEVICE, &js_fd, OP_MODE_DRIVE_ACTIVE);
            }
        }
        usleep(10000); // 10ms sleep
    }
    
    if (js_fd >= 0) {
        close(js_fd);
    }
    return NULL;
}

// Naim joystick handler thread
void* naim_joystick_handler(void* arg) {
    int js_fd = -1;
    struct js_event js;
    
    init_joystick(JS_DEVICE_NAIM, &js_fd, OP_MODE_NAIM_ACTIVE);
    
    pthread_mutex_lock(&state_mutex);
    local_naim_power_saving_mode_toggle = 0;
    update_naim_js_state_for_power_mode(); 
    pthread_mutex_unlock(&state_mutex);
    
    while (running) {
        if (js_fd >= 0) {
            ssize_t bytes = read(js_fd, &js, sizeof(struct js_event));
            
            if (bytes == sizeof(struct js_event)) {
                js.type &= ~JS_EVENT_INIT;
                
                pthread_mutex_lock(&state_mutex);
                js_state.naim_timestamp = (unsigned short int)(js.time & 0xFFFF);
                
                if (js.type == JS_EVENT_BUTTON) {
                    js_state.button7 = (js.number == 7) ? js.value : js_state.button7; // Store raw state of button 7

                    if (js.number == 8) { 
                        if (js.value == 1) { // Button pressed (not held)
                            local_naim_power_saving_mode_toggle = !local_naim_power_saving_mode_toggle;
                            update_naim_js_state_for_power_mode(); // This updates js_state.naim_power_saving_active too
                            printf("Naim Power Saving Mode: %s\n", local_naim_power_saving_mode_toggle ? "ENABLED" : "DISABLED");
                        }
                    } else { 
                        int axis_bit = -1;
                        int* button_state_ptr = NULL;
                        
                        switch (js.number) {
                            case JS_BUTTON_A: button_state_ptr = &js_state.btn_axis1; axis_bit = 0; break;
                            case JS_BUTTON_B: button_state_ptr = &js_state.btn_axis2; axis_bit = 1; break;
                            case JS_BUTTON_X: button_state_ptr = &js_state.btn_axis3; axis_bit = 2; break;
                            case JS_BUTTON_Y: button_state_ptr = &js_state.btn_axis4; axis_bit = 3; break;
                            case JS_BUTTON_LB: button_state_ptr = &js_state.btn_axis5; axis_bit = 4; break;
                            case JS_BUTTON_RB: button_state_ptr = &js_state.btn_axis6; axis_bit = 5; break;
                            case 6: js_state.button6 = js.value; break;
                        }
                        
                        // Only allow axis button presses to affect active_axes if NOT in power saving mode
                        if (button_state_ptr && axis_bit >= 0 && !local_naim_power_saving_mode_toggle) {
                            *button_state_ptr = js.value;
                            if (js.value) {
                                js_state.active_axes |= (1 << axis_bit);
                            } else {
                                js_state.active_axes &= ~(1 << axis_bit);
                            }
                        }
                    }
                } else if (js.type == JS_EVENT_AXIS && !local_naim_power_saving_mode_toggle) {
                    // Only process axis inputs if NOT in power saving mode
                    switch (js.number) {
                        case JS_AXIS_RIGHT_X: calculate_intensity(js.value, &js_state.x_intensity); break;
                        case JS_AXIS_RIGHT_Y: calculate_intensity(js.value, &js_state.y_intensity); break;
                    }
                }
                // Print Naim joystick state (current js_state.naim_power_saving_active will be accurate)
                printf("Naim JS: A1=%d..A6=[%d%d%d%d%d%d], B6=%d, PwrSave=%d, ActA=0x%X, XI=%d, YI=%d, T=%u\n",
                       js_state.btn_axis1, js_state.btn_axis2, js_state.btn_axis3,
                       js_state.btn_axis4, js_state.btn_axis5, js_state.btn_axis6,
                       js_state.button6, js_state.naim_power_saving_active,
                       js_state.active_axes, js_state.x_intensity, js_state.y_intensity, js_state.naim_timestamp);
                pthread_mutex_unlock(&state_mutex);
                                
            } else if (bytes < 0 && errno != EAGAIN) {
                if (errno == ENODEV || errno == EBADF) {
                    handle_joystick_disconnect(&js_fd, OP_MODE_NAIM_ACTIVE);
                }
            }
        } else {
            static int retry_count = 0;
            if (retry_count++ % 100 == 0) { 
                init_joystick(JS_DEVICE_NAIM, &js_fd, OP_MODE_NAIM_ACTIVE);
            }
        }
        usleep(10000);
    }
    
    if (js_fd >= 0) {
        close(js_fd);
    }
    return NULL;
}
