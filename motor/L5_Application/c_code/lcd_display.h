// @file lcd_display.h
#ifndef LCD_DISPLAY_H_
#define LCD_DISPLAY_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void LCD_init(void);
void LCD_speed_print(char* sensorspeed, char* requiredspeed);
void LCD_compass_print(char* compass_value);
void LCD_clearscreen(void);
void LCD_pwm_print(char *pwm, char *error);
void LCD_pid_print(char *pi, char *pd);
void LCD_checkpoints_print(char *TotalCheckpoints, char *CheckpointsCompleted);

#ifdef __cplusplus
}
#endif
#endif /* LCD_DISPLAY_H_ */
