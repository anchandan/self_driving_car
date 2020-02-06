#include "unity.h"
#include "motors_module.h"
#include "Mockmotor_cmd.h"

/* void test_motor_decode_func(void)
{
	
}

void test_can_refresh(void)
{
	
} */

 void test_servo_motor_steer(void)
{
	steer_full_left_Expect();
	servo_motor_steer(-2);
	
	steer_slight_left_Expect();
	servo_motor_steer(-1);
	
	steer_straight_Expect();
	servo_motor_steer(0);
	
	steer_slight_right_Expect();
	servo_motor_steer(1);
	
	steer_full_right_Expect();
	servo_motor_steer(2);

	steer_straight_Expect();
	servo_motor_steer(5);
} 

void test_dc_motor_drive_speed(void)
{
	motor_forward_Expect(1);
	dc_motor_drive_speed(1);
	//Forward

	motor_forward_Expect(2);
	dc_motor_drive_speed(2);

	//Forward

	motor_forward_Expect(4);
	dc_motor_drive_speed(4); 

	motor_stop_Expect();
	dc_motor_drive_speed (0); 

	motor_reverse_Expect();
	dc_motor_drive_speed (-1);

	motor_stop_Expect();
	dc_motor_drive_speed (-2);

}

void test_c_motor_init(void)
{
	motor_init_Expect();
	c_motor_init();
}