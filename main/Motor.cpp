#include "Motor.h"

void setup_motors() {
  pinMode(DIGITAL_MOTOR_1, OUTPUT);
  pinMode(ANALOG_MOTOR_1, OUTPUT);
  pinMode(DIGITAL_MOTOR_2, OUTPUT);
  pinMode(ANALOG_MOTOR_2, OUTPUT);
  pinMode(ENCODER_2, INPUT);
  pinMode(ENCODER_1, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void motors_stop() {
  digitalWrite(DIGITAL_MOTOR_1, LOW);
  analogWrite(ANALOG_MOTOR_1, 0);
  digitalWrite(DIGITAL_MOTOR_2, LOW);
  analogWrite(ANALOG_MOTOR_2, 0);
}

void motors_forward(int speed) { 
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1, HIGH);
  analogWrite(ANALOG_MOTOR_1, 255 - speed);
  digitalWrite(DIGITAL_MOTOR_2, HIGH);
  analogWrite(ANALOG_MOTOR_2, 255 - speed);
}
 
void motors_backward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1, LOW);
  analogWrite(ANALOG_MOTOR_1, speed);
  digitalWrite(DIGITAL_MOTOR_2, LOW);
  analogWrite(ANALOG_MOTOR_2, speed);
}

void motors_right(int speed, int angle) {
  int left_speed = constrain(255 - speed, MIN_SPEED, MAX_SPEED);
  int right_speed = constrain(255 - speed + angle, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1, HIGH);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2, HIGH);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}

void motors_left(int speed, int angle) {
  int left_speed = constrain(255 - speed + angle, MIN_SPEED, MAX_SPEED);
  int right_speed = constrain(255 - speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1, HIGH);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2, HIGH);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}

void motors_hard_left(int left_speed, int right_speed) {
  digitalWrite(DIGITAL_MOTOR_1, LOW);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2, HIGH);
  analogWrite(ANALOG_MOTOR_2, 255 - right_speed);
}
