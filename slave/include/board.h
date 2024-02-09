#ifndef BOARD_H
#define BOARD_H

#include "clock_accel_stepper.h"
#include "board_config.h"
#include "clock_state.h"

#define INIT_HANDS_ANGLE 360

/**
 * Blink the LED n times
*/
void blinken(uint8_t n);

void setCurrentPos(int i, long p);

/**
 * Zero hand posn when crossing sensor
*/
void zero_hand_with_offset(int index, int offset);

bool get_direction(int index);
void run_clockwise(int index);
void run_counterclockwise(int index);
void set_clock_test(int index, t_clock state);
/**
 * Initializes all motor objects and get the I2C address
*/
void board_begin();

/**
 * Needs to be called on the main loop to move steppers
*/
void board_loop();

void motor_identification();

/**
 * Gets the current I2C address set on the board
 * @return address
*/
uint8_t get_i2c_address();

/**
 * Gets the current clock state
 * @param index     clock index (0 <= index =< 3)
 * @return true if clock index is running, false otherwise
*/
bool clock_is_running(int index);

/**
 * Set the clock state by running motors
 * @param index     clock index (0 <= index =< 3)
 * @param state     clock state
*/
void set_clock(int index, t_clock state);

/**
 * Adjust hour hand
 * @param index     clock index (0 <= index =< 3)
 * @param amount    angle (< 0 clockwise, > 0 counterclockwise)
*/
void adjust_h_hand(int index, signed char amount);

/**
 * Adjust minute hand
 * @param index     clock index (0 <= index =< 3)
 * @param amount    angle (< 0 clockwise, > 0 counterclockwise)
*/
void adjust_m_hand(int index, signed char amount);

bool is_hall_start_set(int index);
bool is_hall_stop_set(int index);
void set_hall_start(int index);
void set_hall_stop(int index);
long get_hall_step_gap(int index);
long get_hall_start_value(int index);
long get_hall_stop_value(int index);
void finish_zero(int index);
void board_loop_setup();

bool isZeroed(int index);

#endif