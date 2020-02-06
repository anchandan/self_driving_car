// @file motor_peripherals.cpp
#include "motor.h"
#include <lpc_pwm.hpp>
#include <io.hpp>
#include <switches.hpp>
#include <gpio.hpp>
#include <eint.h>
#include "_can_dbc/generated_can.h"

const uint32_t          MOTOR_COMMAND__MIA_MS = 3000;
const MOTOR_COMMAND_t   MOTOR_COMMAND__MIA_MSG = { 0 };
const uint32_t   HEADING__MIA_MS = 3000;
const HEADING_t  HEADING__MIA_MSG = { 0 };

DEBUG_MOTOR_t motor_debug = { 0 };
MOTOR_COMMAND_t motor_cmd = { 0 };
HEADING_t heading_cmd = { 0 };
CHECKPOINTS_t total_checkpoints_cmd = { 0 };
DEBUG_CHECKPOINT_t checkpoints_cmd = { 0 };

const uint32_t period_20Hz = 50;

GPIO tail_light(P1_20);
GPIO left_light(P1_19);
GPIO right_light(P1_22);

PWM *SERVO_MOTOR, *DC_MOTOR;
uint8_t reverse_flag = 0;

uint8_t rpm = 0;
int16_t new_drive_value=0, old_drive_value=0;
float pwm=0.0, sensorspeed = 0.0, error = 0.0, requiredspeed = 0.0;
int compass_heading = 0;
const float Kp = 0.01;
const float Ki = 0.001;
const float Kd = 0.001;
motor_pid_t pid = { 0 };

// Speed sensor.
void (*Rfptr) (void);

void read_rising_RPM_fe(void){
    rpm++;
}

/* init */

bool can_init(void)
{
    if (CAN_init(can1, 100, 30, 30, NULL, NULL)) {
        CAN_bypass_filter_accept_all_msgs();
        CAN_reset_bus(can1);
        return true;
    }
    return false;
}

void motor_init(void)
{
    SERVO_MOTOR = new PWM(PWM::pwm1); /* P2.0 and 50Hz */
    DC_MOTOR = new PWM(PWM::pwm2); /* P2.1 and 50Hz */
    pwm = 7.5;
    DC_MOTOR->set(pwm);                 // Default State for Motor
    steer_straight();                   // Default State for Servo

    Rfptr = &read_rising_RPM_fe;
    eint3_enable_port2(2, eint_rising_edge, Rfptr);
}

void carlight_init(void)
{
    tail_light.setAsOutput();
    left_light.setAsOutput();
    right_light.setAsOutput();
}

#if 0
void swi_motor_decode(void){
    const uint8_t switches = SW.getSwitchValues();
    enum {
        sw1 = (1<<0),
        sw2 = (1<<1),
        sw3 = (1<<2),
        sw4 = (1<<3),
    };
    switch (switches)
    {
        case sw1:
            motor_forward(1.0);
            break;
        case sw2:
            motor_forward(2.0);
            break;
        case sw3:
            motor_stop();
            break;
        case sw4:
            motor_reverse();
            break;
    }
}

void swi_servo_decode(void){
    const uint8_t switches = SW.getSwitchValues();
    enum {
        sw1 = (1<<0),
        sw2 = (1<<1),
        sw3 = (1<<2),
        sw4 = (1<<3),
    };
    switch (switches)
    {
        case sw1:
            steer_full_left();
            break;
        case sw2:
            steer_slight_left();
            break;
        case sw3:
            steer_slight_right();
            break;
        case sw4:
            steer_full_right();
            break;
    }
}
#endif

void steer_full_left(void)
{
    on_board_LED(3);
    SERVO_MOTOR->set(5);
    tail_light.set(false);
    left_light.set(true);
    right_light.set(false);
}

void steer_slight_left(void){
    on_board_LED(3);
    SERVO_MOTOR->set(6);
    tail_light.set(false);
    left_light.set(true);
    right_light.set(false);
}

void steer_straight(void){
    on_board_LED_off(3);                    //Switching off the 3rd & 4th
    on_board_LED_off(4);                    //LEDs For Straight Condition
    SERVO_MOTOR->set(7);
    tail_light.set(false);
    left_light.set(false);
    right_light.set(false);
}

void steer_slight_right(void){
    on_board_LED(4);
    SERVO_MOTOR->set(8);
    tail_light.set(false);
    left_light.set(false);
    right_light.set(true);
}

void steer_full_right(void){
    on_board_LED(4);
    SERVO_MOTOR->set(9);
    tail_light.set(false);
    left_light.set(false);
    right_light.set(true);
}

void control_speed(float spd_value)
{
    /* Speed = (2*pi*r*N) m/sec
     * d=2r=10cm
     * Speed = 10*pi*N cm/sec
     * Speed = (10*pi*N)/(100) = 0.31428571*N meters/sec
     *
     * N = no. of revolutions
     * Code running in 20Hz
     * Therefoe, Hz = 20
     *
     * Speed = Hz*0.31428571*N meters/sec
     * Speed = 20*0.31428571*N meters/sec
     */
    const float D = 0.1; /* d = 2*r = 10 cm = 0.1 m */
    const float PI = 3.14159265;
    const float HZ = 20.0;
    sensorspeed = HZ * D * PI * rpm;
    error = spd_value - sensorspeed;
    pid.error = error;
    pid.integral += error;
    pid.derivative = error - pid.prev_error;
    pid.prev_error = error;
#if 0
    if (error > 0.2) {
        //pwm = pwm + 0.1;
        DC_MOTOR->set(8.2);
    } else if (error < -0.065) { // Decrease PWM for Downhill.
        DC_MOTOR->set(7.1);         //Improved Braking from 7.5 to 7.1!!!
    } else if (error < -0.045) {
        pwm = pwm - 0.1;
        if (pwm > 7.9)
            DC_MOTOR->set(pwm);
        else
            DC_MOTOR->set(7.9);             // Min PWM reqd. for motor to run
    }
#else
    pwm += Kp * pid.error;
    //pwm += (Kp * pid.error) + (Kd * pid.derivative);
    //pwm += (Kp * pid.error) + (Ki * pid.integral) + (Kd * pid.derivative);
    if (pwm < 7.5) {
        pwm = 7.8;
    } else if (pwm > 10.0) {
        pwm = 10.0;
    }
    DC_MOTOR->set(pwm);             // Min PWM reqd. for motor to run
#endif
    rpm = 0;
}

void motor_forward(float speed)
{
    reverse_flag = 0;
    on_board_LED(1);
    new_drive_value = speed;
    if (new_drive_value != old_drive_value) {
        if (speed < 1.8){
            pwm = 7.7;
            DC_MOTOR->set(pwm);
            requiredspeed = speed;
        } else {
            pwm = 8.0;
            DC_MOTOR->set(pwm);
            requiredspeed = speed;
        }
        pid = { 0 };
    }
    old_drive_value = new_drive_value;
    control_speed(requiredspeed);
    tail_light.set(false);
    left_light.set(false);
    right_light.set(false);
}

void motor_stop()
{
    reverse_flag = 0;
    on_board_LED_off(1);                            //Switching off the 1st & 2nd
    on_board_LED_off(2);                            //LEDs For Stop Condition
    pwm = 7.5;
    DC_MOTOR->set(pwm);
    old_drive_value = 0;
    tail_light.set(true);
    left_light.set(false);
    right_light.set(false);
}

void motor_reverse()
{
    on_board_LED(2);
    if (reverse_flag == 0) {
        pwm = 6.8;
        reverse_flag++;
    } else if (reverse_flag == 1) {
        pwm = 7.5;
        reverse_flag++;
    } else {
        pwm = 7.0;
    }
    DC_MOTOR->set(pwm);
    old_drive_value = 0;
    tail_light.set(true);
    left_light.set(true);
    right_light.set(true);
}

uint8_t get_rpm()
{
    return rpm;
}

float get_reqd_speed(void)
{
    return requiredspeed;
}

float get_speed_value(void){
    return sensorspeed;
}

void set_compass_heading(uint16_t heading)
{
    compass_heading = heading;
}

/* CAN communication */
bool dbc_app_send_can_msg(uint32_t mid, uint8_t dlc, uint8_t bytes[8])
{
    can_msg_t can_msg = { 0 };
    can_msg.msg_id = mid;
    can_msg.frame_fields.data_len = dlc;
    memcpy(can_msg.data.bytes, bytes, dlc);

    return CAN_tx(can1, &can_msg, 0);
}

bool can_rx(uint16_t *heading, uint16_t *steer, float *drive, uint8_t *dir)
{
    //MOTOR_COMMAND_t motor_cmd = { 0 };
    //HEADING_t heading_cmd = { 0 };
    can_msg_t recv_msg;

    if (CAN_rx(can1, &recv_msg, 0)) {
        dbc_msg_hdr_t can_msg_hdr;

        can_msg_hdr.dlc = recv_msg.frame_fields.data_len;
        can_msg_hdr.mid = recv_msg.msg_id;
        dbc_decode_MOTOR_COMMAND(&motor_cmd, recv_msg.data.bytes, &can_msg_hdr);
        dbc_decode_HEADING(&heading_cmd, recv_msg.data.bytes, &can_msg_hdr);
        dbc_decode_DEBUG_CHECKPOINT(&checkpoints_cmd, recv_msg.data.bytes, &can_msg_hdr);
        dbc_decode_CHECKPOINTS(&total_checkpoints_cmd, recv_msg.data.bytes, &can_msg_hdr);

        *heading = heading_cmd.HEADING_VAL;
        *steer = motor_cmd.STEER_cmd;
        *drive = motor_cmd.SPEED_cmd;
        *dir = motor_cmd.DIRECTION_cmd;
        return true;
    }
    return false;
}

bool handle_mia_motor_command(void)
{
    //MOTOR_COMMAND_t motor_cmd = { 0 };
    return dbc_handle_mia_MOTOR_COMMAND(&motor_cmd, period_20Hz);
}

bool handle_mia_compass_command(void)
{
    //HEADING_t heading_cmd = { 0 };
    //MOTOR_COMMAND_t motor_cmd = { 0 };//TODO
    return dbc_handle_mia_MOTOR_COMMAND(&motor_cmd, period_20Hz);
}

void send_motor_speed(void)
{
    SPEED_MOTOR_t speed_cmd = { 0 };
    speed_cmd.MOTOR_SPEED = get_speed_value();
    dbc_encode_and_send_SPEED_MOTOR(&speed_cmd);
}

void send_motor_debug(void)
{
    motor_debug.RPM_VALUE = get_rpm();
    motor_debug.CURR_SPEED = get_speed_value();
    motor_debug.REQD_SPEED = get_reqd_speed();
    dbc_encode_and_send_DEBUG_MOTOR(&motor_debug);
}

/* display */
void on_board_LED(uint8_t ledno)
{
    on_board_LED_off(ledno);                                // Them on again
    LE.on(ledno); // 1=forward, 2=reverse, 3=left, 4=right
}

void on_board_LED_off(uint8_t ledno)
{
    if (ledno < 3) {                                                 // Reset LEDs
        LE.off(1);
        LE.off(2);                                       // Before Turning
    } else {
        LE.off(3);
    }
    LE.off(4);                                    // Them on again
}

void on_board_7Seg_Display(uint8_t displayno)
{
    LD.setNumber(displayno);
}

void lcd_update(void){
    char cs[6], rs[6], comp[6], PWM[6], err[6], total_c[6], c_comp[6];
    gcvt(sensorspeed,4,cs); gcvt(requiredspeed,4,rs);
    gcvt(compass_heading,3,comp); gcvt(pwm,3,PWM);
    gcvt(error,3,err); gcvt(total_checkpoints_cmd.COUNT,3,total_c);
    gcvt(checkpoints_cmd.INDEX,3,c_comp);
    LCD_clearscreen();
    LCD_speed_print(cs,rs);
    LCD_compass_print(comp);
    //LCD_lat_long_print(lat,longi);
    LCD_pwm_print(PWM,err);
    LCD_checkpoints_print(total_c, c_comp);
}
