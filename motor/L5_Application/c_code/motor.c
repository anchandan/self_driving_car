// @file motor.c
#include <utilities.h>
#include "motor.h"

bool motor_module_init(void)
{
    can_init();
    motor_init();
    carlight_init();
    LCD_init();
    return true;
}

bool can_refresh(void)
{
    if (CAN_is_bus_off(can1)) {
        CAN_bypass_filter_accept_all_msgs();
        CAN_reset_bus(can1);
    }
    return true;
}

#if 0
void switch_motor_decode(void){
    swi_motor_decode();
}

void switch_servo_decode(void){
    swi_servo_decode();
}
#endif

bool receive_motor_commands(void)
{
    uint16_t heading, steer;
    float drive;
    uint8_t dir;

    while (can_rx(&heading, &steer, &drive, &dir)) {
        ;
    }
    set_compass_heading(heading);
    servo_motor_steer(steer);
    dc_motor_drive_speed(drive, dir);
    handle_mia_motor();
    handle_mia_compass();

    return true;
}

void handle_mia_motor(void)
{
    if (handle_mia_motor_command()) {
        servo_motor_steer(0);
        dc_motor_drive_speed(0, 0);
        on_board_7Seg_Display(0); // Indicates that MIA Occurred
    }
}

void handle_mia_compass(void)
{
    if (handle_mia_compass_command()) {
        set_compass_heading(400);
        on_board_7Seg_Display(1); // Indicates that MIA Occurred
    }
}

bool servo_motor_steer(int16_t motor_steer)
{
    switch (motor_steer) {
        case -2:
            steer_full_left();
            break;
        case -1:
            steer_slight_left();
            break;
        case 0:
            steer_straight();
            break;
        case 1:
            steer_slight_right();
            break;
        case 2:
            steer_full_right();
            break;
        default:
            steer_straight();
            break;
    }
    return true;
}

bool dc_motor_drive_speed(float speed, uint8_t dir)
{
#if 0
    if (speed < 0.2f) {
        motor_stop();
    } else if (dir == 0) {
        motor_reverse();
    } else if (dir == 1) {
        motor_forward(speed);
    } else {
        motor_stop();
        return false;
    }
    return true;
#endif
    if (dir == 0 && speed > 0.2f) {
        motor_reverse();
    } else if (dir == 1 && speed < 0.2f) {
        motor_stop();
    } else if (dir == 1 && speed > 0.2f) {
        motor_forward(speed);
    } else {
        motor_stop();
        return false;
    }
    return true;
}

void c_LCD_update(void)
{
   lcd_update();
}
