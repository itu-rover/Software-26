#ifndef JOYSTICK_HANDLER_H
#define JOYSTICK_HANDLER_H

#include <pthread.h>
#include <linux/joystick.h>

/* ───────────────  Shared state  ─────────────── */
typedef struct {
    /* drive joystick */
    int rb_pressed;
    int lb_pressed;
    int left_y;
    int right_y;
    int right_x;
    unsigned short drive_timestamp;

    /* naim joystick */
    int btn_axis1, btn_axis2, btn_axis3, btn_axis4, btn_axis5, btn_axis6;
    int button6, button7;
    int active_axes;
    int x_intensity, y_intensity;
    unsigned short naim_timestamp;
    int naim_power_saving_active;

    /* common */
    unsigned short op_mode;
} JoystickState;

/* Variables that are shared across translation units */
extern JoystickState        js_state;
extern pthread_mutex_t      state_mutex;
extern volatile int         running;

/* Thread entry points */
void *drive_joystick_handler(void *arg);
void *naim_joystick_handler(void *arg);

#endif /* JOYSTICK_HANDLER_H */