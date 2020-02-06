#include "unity.h"
#include "motor.h"
#include "Mockcan.h"
#include "Mockmotor_peripherals.h"
#include "Mocklcd_display.h"

//rake unit single_file=code_test/test_motor_cmd.c

/* void test_motor_decode_func(void)
{
	
}

void test_can_refresh(void)
{
	
} */

 /*
 * motor_cmd.cpp
 *
 *  Created on: Apr 03, 2019
 *      Author: Admin
 */


void test_motor_module_init(void)
{
    TEST_ASSERT_EQUAL(0,0);
    can_init_ExpectAndReturn(true);
    motor_init_Expect();
    LCD_init_Expect();
    TEST_ASSERT_TRUE(motor_module_init());
}

void test_can_refresh(void)
{
    //TEST_ASSERT_EQUAL(0,0);
    CAN_is_bus_off_ExpectAndReturn(can1,true);
  //  CAN_is_bus_off_ExpectAndReturn(can1,false);
    CAN_bypass_filter_accept_all_msgs_Expect();
    CAN_reset_bus_Expect(can1);
    TEST_ASSERT_TRUE(can_refresh());
}

#if 0
void switch_motor_decode(void){
    swi_motor_decode();
}

void switch_servo_decode(void){
    swi_servo_decode();
}
#endif

void test_receive_motor_commands(void)
{
    TEST_ASSERT_EQUAL(0,0);
    /*uint16_t heading = 0;
    uint16_t steer = 0x60;
    float drive = 6.23997e-39;

    can_rx_ExpectAndReturn(&heading, &steer, &drive, true);

    set_compass_heading_Expect(heading);
    TEST_ASSERT_TRUE(servo_motor_steer(steer));
    TEST_ASSERT_TRUE(dc_motor_drive_speed(drive));
  //  handle_mia_motor_Ignore();
  //  handle_mia_compass_Ignore();
    TEST_ASSERT_TRUE(receive_motor_commands());*/
}

void test_handle_mia_motor(void)
{
   // TEST_ASSERT_EQUAL(0,0);
    handle_mia_motor_command_ExpectAndReturn(true);
    steer_straight_Expect();
    motor_stop_Expect();
    servo_motor_steer(0);
    dc_motor_drive_speed(0);
    on_board_7Seg_Display(0);
    //handle_mia_motor();
}

void test_handle_mia_compass(void)
{
    TEST_ASSERT_EQUAL(0,0);
}

void test_servo_motor_steer(void)
{
    TEST_ASSERT_EQUAL(0,0);
}

void test_dc_motor_drive_speed(void)
{
    TEST_ASSERT_EQUAL(0,0);
}

void test_c_LCD_update(void)
{

    lcd_update_Expect();
    c_LCD_update();
}




