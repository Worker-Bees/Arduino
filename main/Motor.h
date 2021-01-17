
#ifndef __MOTOR__
#define __MOTOR__

#include <Arduino.h>

#define DIGITAL_MOTOR_1_A 7
#define DIGITAL_MOTOR_1_B 6
#define ANALOG_MOTOR_1 10
#define DIGITAL_MOTOR_2_A 4
#define DIGITAL_MOTOR_2_B 5
#define ANALOG_MOTOR_2 9
#define MAX_SPEED 255 //từ 0-255
#define MIN_SPEED 0

void setup_motors();
void motors_forward(int left_speed, int right_speed);
void motors_backward(int left_speed, int right_speed);
void motors_left(int speed, int angle);
void motors_right(int speed, int angle);
void motors_stop();
void motors_hard_left(int left_speed, int right_speed);
void motors_hard_right(int left_speed, int right_speed);

#endif
