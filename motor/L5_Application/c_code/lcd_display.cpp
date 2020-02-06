// @file lcd_display.cpp
#include <utilities.h>
#include <uart3.hpp>
#include "lcd_display.h"

Uart3* uart3;

void LCD_init(void)
{
    uart3 = &Uart3::getInstance();
    uart3->init(38400, 32, 512);
    delay_ms(100);
    uart3->putChar(0xF0);
    delay_ms(100);
    uart3->printf("$CLR_SCR\n"); //clear screen
    uart3->printf("$GOTO:7:0:\n Zeus\n");
    uart3->printf("$GOTO:1:1:\nThe Autonomous Car");
    uart3->printf("$GOTO:1:2:\nPress Start Button");
    uart3->printf("$GOTO:5:3:\non the App");
}

void LCD_speed_print(char* sensorspeed, char* requiredspeed)
{
    uart3->printf("$GOTO:0:0\nCar:%s Req:%s",sensorspeed,requiredspeed);
}

void LCD_compass_print(char* compass_value)
{
    uart3->printf("$GOTO:0:1\nCompass Value:%s",compass_value);
}

void LCD_clearscreen(void)
{
    uart3->printf("$CLR_SCR\n"); //clear screen
}

void LCD_pwm_print(char *pwm, char *error)
{
    uart3->printf("$GOTO:0:2:\nPWM:%s err:%s", pwm, error);
}

void LCD_pid_print(char *pi, char *pd){
    uart3->printf("$GOTO:0:3:\nPi:%s Pd:%s",pi,pd);
}

void LCD_checkpoints_print(char *TotalCheckpoints, char *CheckpointsCompleted){
    uart3->printf("$GOTO:0:3:\nChkpoints = %s/%s", CheckpointsCompleted, TotalCheckpoints);
}
