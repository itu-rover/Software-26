#ifndef CONFIG_H
#define CONFIG_H

// Network configuration
#ifndef SERVER_IP
#define SERVER_IP "192.168.1.6"
#endif

#define ORANGEPI_IP "192.168.1.94"
#define ORANGEPI_PORT 8887

#define XAVIER_IP "192.168.1.5"
#define XAVIER_PORT 2003

// 6103 is for test only change to 6102
#define SERVER_PORT 6102
#define BUFFER_SIZE 1024

// Joystick device paths
#define JS_DEVICE "/dev/input/js0"
#define JS_DEVICE_NAIM "/dev/input/js1"

// Joystick button constants
#define JS_BUTTON_A 0
#define JS_BUTTON_B 1
#define JS_BUTTON_X 2
#define JS_BUTTON_Y 3
#define JS_BUTTON_LB 4
#define JS_BUTTON_RB 5

// Joystick axis constants
#define JS_AXIS_LEFT_X 0
#define JS_AXIS_LEFT_Y 1

#define JS_AXIS_RIGHT_X 3
#define JS_AXIS_RIGHT_Y 4

// Joystick thresholds
#define DEADZONE_THRESHOLD 3275  // ~10% of max value

#endif /* CONFIG_H */
