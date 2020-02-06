// @file motor.h
#ifndef MOTORS_MODULE_H_
#define MOTORS_MODULE_H_

#include <stdbool.h>
#include <stdint.h>
#include <can.h>
#include "motor_peripherals.h"

bool motor_module_init(void);
bool can_refresh(void);
#if 0
void switch_motor_decode(void);                     // Using Switches to test Motor
void switch_servo_decode(void);                     // Using Switches to test Servo
#endif

bool receive_motor_commands(void);
void handle_mia_motor(void);
void handle_mia_compass(void);
bool servo_motor_steer(int16_t motor_steer);
bool dc_motor_drive_speed(float speed, uint8_t dir);

void c_LCD_update(void);

#endif /* MOTORS_MODULE_H_ */
