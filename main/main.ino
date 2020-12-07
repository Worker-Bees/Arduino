#include "Motor.h"
#include <SharpIR.h> 

#define FRONT_SENSOR A0
#define FRONT_RIGHT_SENSOR A1 
#define RIGHT_TOP_SENSOR A2 
#define RIGHT_BOTTOM_SENSOR A3

#define SENSOR_MODEL 1080 
SharpIR front_sensor(FRONT_SENSOR, SENSOR_MODEL); 
SharpIR front_right_sensor(FRONT_RIGHT_SENSOR, SENSOR_MODEL); 
SharpIR right_top_sensor(RIGHT_TOP_SENSOR, SENSOR_MODEL); 
SharpIR right_bottom_sensor(RIGHT_BOTTOM_SENSOR, SENSOR_MODEL); 

// State machine implementation
typedef enum
{
  START_POINT,
  TOWARD_WALL,  
  AWAY_WALL,     
  PARALLEL_WALL,  
  SEE_OBSTACLE,    
  GET_LOST,
  NUM_STATES
} State;

typedef struct
{
  State state;
  void (*function)();
} StateMachine;

typedef struct {
  State prev_state;
  int front_dis;
  int front_right_dis;
  int right_top_dis;
  int right_bottom_dis;
  int motor_1_speed;
  int motor_2_speed;
} StateInput;

int handle_sensors_noise(int val);
void read_sensors();
void encoder_1_pulses_count();
void encoder_2_pulses_count();
void START_POINT_func();
void TOWARD_WALL_func();
void AWAY_WALL_func();
void PARALLEL_WALL_func();
void SEE_OBSTACLE_func();
void GET_LOST_func();

volatile unsigned int front_dis = 0;
volatile unsigned int front_right_dis = 0;
volatile unsigned int right_top_dis = 0;
volatile unsigned int right_bottom_dis = 0;
volatile unsigned int encoder_2_pulses = 0;
volatile unsigned int encoder_2_start_time = 0;
volatile int encoder_2_avg_time = 0;
volatile unsigned int encoder_1_pulses = 0;
volatile unsigned int encoder_1_start_time = 0;
volatile int encoder_1_avg_time = 0;
volatile int rotation_tune = 1;
volatile int distance_diff = 1;

StateMachine state_machine[] =
{
  { START_POINT, START_POINT_func},
  { TOWARD_WALL, TOWARD_WALL_func},
  { AWAY_WALL, AWAY_WALL_func},
  { PARALLEL_WALL, PARALLEL_WALL_func},
  { SEE_OBSTACLE, SEE_OBSTACLE_func},
  { GET_LOST, GET_LOST_func},
};

State state = START_POINT;
State prev_state = START_POINT;
StateInput state_input = {
    prev_state: START_POINT,
    front_dis: handle_sensors_noise(front_sensor.distance()),
    front_right_dis: handle_sensors_noise(front_right_sensor.distance()),
    right_top_dis: handle_sensors_noise(right_top_sensor.distance()),
    right_bottom_dis: handle_sensors_noise(right_bottom_sensor.distance()),
    motor_1_speed: encoder_1_avg_time,
    motor_2_speed: encoder_2_avg_time,
};

void setup() { 
  // put your setup code here, to run once:
  setup_motors();
  attachInterrupt(digitalPinToInterrupt(ENCODER_1), encoder_1_pulses_count, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2), encoder_2_pulses_count, RISING);
  Serial.begin(9600);
  delay(3000);
  read_sensors();
} 

void loop() {
   if (state < NUM_STATES) {
      (*state_machine[state].function) ();
   }
//  read_sensors();
//  Serial.print(front_dis);
//  Serial.print(" ");
//  Serial.print(front_right_dis);
//  Serial.print(" ");
//  Serial.print(right_top_dis);
//  Serial.print(" ");
//  Serial.println(right_bottom_dis);
} 


void START_POINT_func() {
  motors_left(100, 20);
  do {
    read_sensors();
  } while (right_top_dis > 80);
  prev_state = START_POINT;
  state = AWAY_WALL;
}

void AWAY_WALL_func() {
  rotation_tune = 1;
  do {
    motors_right(80, constrain(5 + 5 * rotation_tune, 10, 20));
    read_sensors();
    if (distance_diff <= right_top_dis - right_bottom_dis) {
      rotation_tune++;
    } else rotation_tune--;
    distance_diff == right_top_dis - right_bottom_dis;
  } while (right_top_dis - right_bottom_dis > 0 && right_top_dis != 81 && front_dis > 20);
  prev_state = AWAY_WALL;
  if (front_dis <= 20) {
    state = SEE_OBSTACLE;
  } else if (right_top_dis == right_bottom_dis) {
    state = PARALLEL_WALL;
  } else {
    state = TOWARD_WALL;
  }
}

void TOWARD_WALL_func() {
  rotation_tune = 1;
  do {
    motors_left(80, constrain(10 + 10 * rotation_tune, 10, 40));
    read_sensors();
    if (distance_diff <= right_bottom_dis - right_top_dis) {
      rotation_tune++;
    } else rotation_tune--;
    distance_diff == right_bottom_dis - right_top_dis;
  } while (right_bottom_dis - right_top_dis > 0 && right_top_dis == 81 && front_dis > 20 && front_right_dis != 81);
  prev_state = TOWARD_WALL;
  if (front_dis <= 20) {
    state = SEE_OBSTACLE;
  } else if (right_top_dis == right_bottom_dis) {
    state = PARALLEL_WALL;
  } else {
    state = AWAY_WALL;
  }
}

void PARALLEL_WALL_func() {
  motors_forward(80);
  do {
    read_sensors();
  } while (right_top_dis == right_bottom_dis && right_top_dis != 81 && right_bottom_dis !=81);
  prev_state = PARALLEL_WALL;
  if (right_top_dis == 81 && front_right_dis == 81) {
    motors_left(120, 60);
    delay(1000);
    if (right_top_dis == 81) state = GET_LOST;
    else state = AWAY_WALL;
  } else if (right_top_dis == 81) {
    state = TOWARD_WALL;
  } else if (right_top_dis > right_bottom_dis) {
    state = AWAY_WALL;
  } else {
    state = TOWARD_WALL;
  } 
}

void SEE_OBSTACLE_func() {
  if (right_bottom_dis == 81 && right_top_dis == 81) {
    motors_left(80, 80);
    do {
      read_sensors();
    } while (front_dis != 81);
  } else {
    motors_stop();
    do {
      read_sensors();
    } while(front_dis < 20);
  }
  
  state = PARALLEL_WALL;
}

void GET_LOST_func() {
  motors_left(80, 80);
  do {
    read_sensors();
  } while (front_right_dis == 81);
  if (front_right_dis < 28) {
    state = TOWARD_WALL;
  } else if (front_right_dis >= 28){
    state = AWAY_WALL;
  } else motors_stop();
}

int handle_sensors_noise(int val) {
  if (val > 80 || val < 11) return 81;
  else return val;
}

void read_sensors() {
  front_dis = handle_sensors_noise(front_sensor.distance());
  front_right_dis = handle_sensors_noise(front_right_sensor.distance());
  right_top_dis = handle_sensors_noise(right_top_sensor.distance());
  right_bottom_dis = handle_sensors_noise(right_bottom_sensor.distance());
}


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
