
#ifndef __MOTOR__
#define __MOTOR__

#include <Arduino.h>

#define DIGITAL_MOTOR_1_A 6
#define DIGITAL_MOTOR_1_B 7
#define ANALOG_MOTOR_1 11
#define DIGITAL_MOTOR_2_A 5
#define DIGITAL_MOTOR_2_B 4
#define ANALOG_MOTOR_2 10
#define ENCODER_2 2
#define ENCODER_1 3
#define MAX_SPEED 255 //từ 0-255
#define MIN_SPEED 0

void setup_motors();
void motors_forward(int left_speed, int right_speed);
void motors_backward(int speed);
void motors_left(int speed, int angle);
void motors_right(int speed, int angle);
void motors_stop();
void motors_hard_left(int left_speed, int right_speed);

#endif
