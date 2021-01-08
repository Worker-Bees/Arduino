#include "Motor.h"
#include <SharpIR.h> 

#define FRONT_SENSOR A0
#define FRONT_RIGHT_SENSOR A1 
#define RIGHT_TOP_SENSOR A2 
#define RIGHT_BOTTOM_SENSOR A3
#define ECHO 12
#define TRIG 13
#define TRIG2 8
#define ECHO2 9

#define SENSOR_MODEL 1080 
SharpIR front_sensor(FRONT_SENSOR, SENSOR_MODEL); 
SharpIR front_right_sensor(FRONT_RIGHT_SENSOR, SENSOR_MODEL); 
SharpIR right_top_sensor(RIGHT_TOP_SENSOR, SENSOR_MODEL); 
SharpIR right_bottom_sensor(RIGHT_BOTTOM_SENSOR, SENSOR_MODEL); 

// State machine implementation
typedef enum
{
  WALL_TRACKING,
  SEE_OBSTACLE,    
  GET_LOST,
  NUM_STATES
} State;

typedef struct
{
  State state;
  void (*function)();
} StateMachine;


int handle_sensors_noise(int val);
void read_sensors();
void encoder_1_pulses_count();
void encoder_2_pulses_count();
void WALL_TRACKING_func();
void SEE_OBSTACLE_func();
void GET_LOST_func();
void MANUAL_CONTROL_func();

volatile int front_ultra_dis = 0;
volatile int front_right_dis = 0;
volatile int right_ultra_dis = 0;

volatile int prev_distance_diff = 0;
volatile int distance_diff = 0;

StateMachine state_machine[] =
{
  { WALL_TRACKING, WALL_TRACKING_func},
  { SEE_OBSTACLE, SEE_OBSTACLE_func},
  { GET_LOST, GET_LOST_func},
};

State state = WALL_TRACKING;
char command;

void setup() { 
  // put your setup code here, to run once:
  setup_motors();
  pinMode(TRIG,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(TRIG2,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO2,INPUT);    // chân echo sẽ nhận tín hiệu
  Serial.begin(9600);
  delay(3000);
//  motors_forward(60, 60);
  read_sensors();
} 

void loop() {
//     if (state < NUM_STATES) {
//        (*state_machine[state].function) ();
//     }
   MANUAL_CONTROL_func();
//  read_sensors();
//  Serial.print(front_ultra_dis);
//  Serial.print(" ");
//  Serial.print(front_right_dis);
//  Serial.print(" ");
//  Serial.println(right_ultra_dis);
} 

void MANUAL_CONTROL_func() {
  if (Serial.available() > 0) {
    command = Serial.read();
    switch (command) {
      case 'w': motors_forward(60,60); break;
      case 'q': motors_forward(41,60); break;
      case 'e': motors_forward(60,41); break;
      case 'a': motors_hard_left(80,80); break;
      case 'd': motors_hard_right(80,80); break;
      case 's': motors_backward(60,60); break;
      case 'z': motors_backward(41,60); break;
      case 'c': motors_backward(60,41); break;
      case 'x': motors_stop(); break;
    }
  }
}

void WALL_TRACKING_func() {
  const int speed = 60;
  do {
    read_sensors();
    if (front_right_dis <= 70) {
      distance_diff = front_right_dis - 18;
      if (prev_distance_diff != distance_diff) {
         motors_forward(constrain(speed + 10 * distance_diff, 40, 120), speed);
         prev_distance_diff = distance_diff;
      }
    } else {
         motors_forward(constrain(speed + 10 * distance_diff, 41, 120), speed);
    }
  } while ((front_right_dis <= 70 || right_ultra_dis <= 90) && front_ultra_dis > 25);
  if (front_ultra_dis <= 25) 
    state = SEE_OBSTACLE;
  else if (right_ultra_dis > 90 && front_right_dis > 70) 
    state = GET_LOST;
  else
    state = WALL_TRACKING;

}
void SEE_OBSTACLE_func() {
  if (right_ultra_dis < 20) {
    motors_stop();
    do {
      read_sensors();
    } while(front_ultra_dis <= 25);
      delay(3000); 
  }
  read_sensors();
  if (front_ultra_dis <= 25) {
    motors_hard_left(120, 100);
    do {
      read_sensors();
    } while (front_ultra_dis < 50 || front_right_dis <= 22);
  }
  state = WALL_TRACKING;
}

void GET_LOST_func() {
  motors_hard_left(120, 100);
  do {
    read_sensors();
  } while (front_right_dis <= 22 || front_ultra_dis < 50 || front_right_dis == 81) ;
  state = WALL_TRACKING;
}

int handle_sensors_noise(int val) {
  if (val > 80 || val < 10) return 81;
  else return val;
}

void read_sensors() {
  front_right_dis = constrain(handle_sensors_noise(front_right_sensor.distance()), 14, 81);
  unsigned long duration; // biến đo thời gian
  int distance;           // biến lưu khoảng cách
  /* Phát xung từ chân trig */
  digitalWrite(TRIG,0);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(TRIG,1);   // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(TRIG,0);   // tắt chân trig
  
  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  duration = pulseIn(ECHO, HIGH);
  // Tính khoảng cách đến vật.
  distance = int(duration/2/29.412);
  front_ultra_dis = constrain(distance, 4, 120);
  delay(3);
    /* Phát xung từ chân trig */
  digitalWrite(TRIG2,0);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(TRIG2,1);   // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(TRIG2,0);   // tắt chân trig
  
  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  duration = pulseIn(ECHO2, HIGH);
  // Tính khoảng cách đến vật.
  distance = int(duration/2/29.412);
  right_ultra_dis = constrain(distance, 4, 120);
}
