
//
//void setup()
//{
//  setup_motors();
//}
//
//void loop()
//{
////  motors_forward(MAX_SPEED);
////  delay(5000);
////  motors_backward(MAX_SPEED);
//  motors_right(MAX_SPEED, 90);
//  delay(5000);
//  motors_left(MAX_SPEED, 50);
//  delay(5000);
//}

#include <SharpIR.h> 
#include "Motor.h"

#define FRONT_SENSOR A3
#define FRONT_RIGHT_SENSOR A2 
#define RIGHT_TOP_SENSOR A1 
#define RIGHT_BOTTOM_SENSOR A0

#define SENSOR_MODEL 1080 

typedef enum {
  FAST_FORWARD,   //0
  LEFT_FORWARD,   //1
  RIGHT_FORWARD,  //2
  HARD_LEFT,      //3
  HARD_RIGHT,     //4
  SOFT_LEFT,      //5
  SOFT_RIGHT,     //6
  SLOW_FORWARD,   //7
  STOP            //8
} State;

SharpIR front_sensor(FRONT_SENSOR, SENSOR_MODEL); 
SharpIR front_right_sensor(FRONT_RIGHT_SENSOR, SENSOR_MODEL); 
SharpIR right_top_sensor(RIGHT_TOP_SENSOR, SENSOR_MODEL); 
SharpIR right_bottom_sensor(RIGHT_BOTTOM_SENSOR, SENSOR_MODEL); 

volatile int front_dis = 0;
volatile int front_right_dis = 0;
volatile int right_top_dis = 0;
volatile int right_bottom_dis = 0;
volatile State current_state = STOP;
volatile State previous_state = STOP;
volatile unsigned int encoder_2_pulses = 0;
volatile unsigned int encoder_2_start_time = 0;
volatile int encoder_2_avg_time = 0;
volatile unsigned int encoder_1_pulses = 0;
volatile unsigned int encoder_1_start_time = 0;
volatile int encoder_1_avg_time = 0;

void encoder_1_pulses_count() {
    if (encoder_1_pulses++ == 374) {
      encoder_1_pulses = 0;
      encoder_1_avg_time = millis() - encoder_1_start_time;
      encoder_1_start_time = millis();
    }
}

void encoder_2_pulses_count() {
    if (encoder_2_pulses++ == 374) {
      encoder_2_pulses = 0;
      encoder_2_avg_time = millis() - encoder_2_start_time;
      encoder_2_start_time = millis();
    }
}

void setup() { 
  // put your setup code here, to run once:
  setup_motors();
  attachInterrupt(digitalPinToInterrupt(ENCODER_1), encoder_1_pulses_count, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2), encoder_2_pulses_count, RISING);
  Serial.begin(9600);
  delay(3000);
} 

void loop() {
    read_sensors();
    update_state();
    if (current_state != previous_state) {
      update_speed();
      delay(constrain(abs(right_top_dis - right_bottom_dis) * 50 + 300, 300, 1000));
      motors_forward(150);
      delay(250);
      previous_state = current_state;
    }
//    if (encoder_2_avg_time - encoder_1_avg_time > 0) digitalWrite(LED_BUILTIN, LOW);
//    else digitalWrite(LED_BUILTIN, HIGH);
//    Serial.println(encoder_1_avg_time);
//    Serial.println(encoder_2_avg_time);
} 

void update_speed() {
  switch (current_state) {
    case FAST_FORWARD:
      motors_forward(150);
      break;
    case RIGHT_FORWARD:
      motors_right(180, 20);
      break;
    case LEFT_FORWARD:
      motors_left(180, 20);
      break;
    case HARD_LEFT:
      motors_left(MAX_SPEED, 200);
      break;
    case HARD_RIGHT:
      motors_right(MAX_SPEED, 200);
      break;                 
    case SLOW_FORWARD:
      motors_forward(150);
      break;
    case STOP:
      motors_stop();
      break;
  }
}

void update_state() {
  if (front_dis < 30) {
    if (right_bottom_dis > 60) current_state = LEFT_FORWARD;
    else  current_state = STOP;
  } else if (right_top_dis > 70 && right_bottom_dis > 70) {
    if (front_right_dis >70) current_state = HARD_LEFT;
    else current_state = SLOW_FORWARD;
////    if (front_right_dis > 80 || front_dis < 30) current_state = HARD_LEFT;
////    else if (front_right_dis > 30) current_state = RIGHT_FORWARD;
////    else if (front_right_dis < 30) current_state = LEFT_FORWARD;
  } else if (right_top_dis > 22 || right_bottom_dis > 22) {
    current_state = RIGHT_FORWARD;
  } else if (right_top_dis < 20 || right_bottom_dis < 20) {
    current_state = LEFT_FORWARD;
  } else if (right_top_dis - right_bottom_dis < 0) {
    current_state = RIGHT_FORWARD;
  } else if (right_top_dis - right_bottom_dis > 0) {
    current_state = LEFT_FORWARD;
  } 
}

void read_sensors() {
  front_dis = handle_sensors_noise(front_sensor.distance());
  front_right_dis = handle_sensors_noise(front_right_sensor.distance());
  right_top_dis = handle_sensors_noise(right_top_sensor.distance());
  right_bottom_dis = handle_sensors_noise(right_bottom_sensor.distance());
}

int handle_sensors_noise(int val) {
  if (val > 80 || val < 13) return 81;
  else return val;
}
