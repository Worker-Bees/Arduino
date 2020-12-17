
#ifndef __MOTOR__
#define __MOTOR__

#include <Arduino.h>

#define DIGITAL_MOTOR_1 4
#define ANALOG_MOTOR_1 5
#define DIGITAL_MOTOR_2 7
#define ANALOG_MOTOR_2 6
#define ENCODER_2 2
#define ENCODER_1 3
#define MAX_SPEED 255 //từ 0-255
#define MIN_SPEED 30

void setup_motors();
void motors_forward(int speed);
void motors_backward(int speed);
void motors_left(int speed, int angle);
void motors_right(int speed, int angle);
void motors_stop();
void motors_hard_left(int left_speed, int right_speed);

#endif
