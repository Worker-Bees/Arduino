#include "Motor.h"

void setup_motors() {
  pinMode(DIGITAL_MOTOR_1_A, OUTPUT);
  pinMode(DIGITAL_MOTOR_1_B, OUTPUT);
  pinMode(ANALOG_MOTOR_1, OUTPUT);
  pinMode(DIGITAL_MOTOR_2_A, OUTPUT);
  pinMode(DIGITAL_MOTOR_2_B, OUTPUT);
  pinMode(ANALOG_MOTOR_2, OUTPUT);
  pinMode(ENCODER_2, INPUT);
  pinMode(ENCODER_1, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void motors_stop() {
  analogWrite(ANALOG_MOTOR_1, 0);
  analogWrite(ANALOG_MOTOR_2, 0);
}

void motors_forward(int left_speed, int right_speed) { 
  left_speed = constrain(left_speed, MIN_SPEED, MAX_SPEED);
  right_speed = constrain(right_speed, MIN_SPEED, MAX_SPEED);
  if (left_speed == 40) {
      digitalWrite(DIGITAL_MOTOR_1_A, LOW);
      digitalWrite(DIGITAL_MOTOR_1_B, HIGH);
      analogWrite(ANALOG_MOTOR_1, 60);
  } else {
      digitalWrite(DIGITAL_MOTOR_1_A, HIGH);
      digitalWrite(DIGITAL_MOTOR_1_B, LOW);
      analogWrite(ANALOG_MOTOR_1, left_speed);
  }
  digitalWrite(DIGITAL_MOTOR_2_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_2_B, LOW);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}
 
void motors_backward(int left_speed, int right_speed) {
  left_speed = constrain(left_speed, MIN_SPEED, MAX_SPEED);
  right_speed = constrain(right_speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1_A, LOW);
  digitalWrite(DIGITAL_MOTOR_1_B, HIGH);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2_A, LOW);
  digitalWrite(DIGITAL_MOTOR_2_B, HIGH);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}

void motors_right(int speed, int angle) {
  int left_speed = constrain(speed - angle, 30, MAX_SPEED);
  int right_speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_1_B, LOW);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_2_B, LOW);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}

void motors_left(int speed, int angle) {
  int left_speed = constrain(speed + angle, 30, MAX_SPEED);
  int right_speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(DIGITAL_MOTOR_1_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_1_B, LOW);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_2_B, LOW);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}

void motors_hard_left(int left_speed, int right_speed) {
  digitalWrite(DIGITAL_MOTOR_1_A, LOW);
  digitalWrite(DIGITAL_MOTOR_1_B, HIGH);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_2_B, LOW);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}

void motors_hard_right(int left_speed, int right_speed) {
  digitalWrite(DIGITAL_MOTOR_1_A, HIGH);
  digitalWrite(DIGITAL_MOTOR_1_B, LOW);
  analogWrite(ANALOG_MOTOR_1, left_speed);
  digitalWrite(DIGITAL_MOTOR_2_A, LOW);
  digitalWrite(DIGITAL_MOTOR_2_B, HIGH);
  analogWrite(ANALOG_MOTOR_2, right_speed);
}
