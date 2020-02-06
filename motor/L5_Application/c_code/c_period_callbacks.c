/**
 * @file
 *
 * The purpose of this "C" callbacks is to provide the code to be able
 * to call pure C functions and unit-test it in C test framework
 */
#include "motor.h"

bool C_period_init(void) {
   motor_module_init();
   return true;
}

bool C_period_reg_tlm(void) {
    return true;
}

void C_period_1Hz(uint32_t count ) {
    can_refresh();
    c_LCD_update();
}

void C_period_10Hz(uint32_t count) {
    (void) count;
    send_motor_speed();
    send_motor_debug();
}

void C_period_100Hz(uint32_t count) {
    (void) count;
    if (count % 5 == 0) {
        receive_motor_commands();
    }
}

void C_period_1000Hz(uint32_t count) {
    (void) count;
}
