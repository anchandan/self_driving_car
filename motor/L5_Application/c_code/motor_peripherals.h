// @file motor_peripherals.h
#ifndef MOTOR_CMD_H_
#define MOTOR_CMD_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <utilities.h>
#include "lcd_display.h"

typedef struct motor_pid_s {
    float error;
    float integral;
    float prev_error;
    float derivative;
} motor_pid_t;

extern char * gcvt(double number, int ndigit, char *buf);

bool can_init(void);
void motor_init(void);
void carlight_init(void);
#if 0
void swi_motor_decode(void);
void swi_servo_decode(void);
#endif

void steer_full_left(void);
void steer_slight_left(void);
void steer_straight(void);
void steer_slight_right(void);
void steer_full_right(void);

void motor_forward(float speed);
void motor_stop(void);
void motor_reverse(void);
void control_speed(float reqspeed);
void set_compass_heading(uint16_t heading);
float get_reqd_speed(void);
float get_speed_value(void);
uint8_t get_rpm(void);
//void calc_speed(void);                  // TODO: Write this Function

bool can_rx(uint16_t *heading, uint16_t *steer, float *drive, uint8_t *dir);
bool handle_mia_motor_command(void);
bool handle_mia_compass_command(void);
void send_motor_speed(void);
void send_motor_debug(void);

void on_board_LED(uint8_t ledno);
void on_board_LED_off(uint8_t ledno);
void on_board_7Seg_Display(uint8_t displayno);
void lcd_update(void);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_CMD_H_ */

