<<<<<<< HEAD
#include "Motor.h"
#include <SharpIR.h> 
#include <SimpleKalmanFilter.h>

#define FRONT_SENSOR A0
#define FRONT_RIGHT_SENSOR A1 
#define RIGHT_TOP_SENSOR A2 
#define RIGHT_BOTTOM_SENSOR A3
#define ECHO 13
#define TRIG 12

#define SENSOR_MODEL 1080 
SharpIR front_sensor(FRONT_SENSOR, SENSOR_MODEL); 
SharpIR front_right_sensor(FRONT_RIGHT_SENSOR, SENSOR_MODEL); 
SharpIR right_top_sensor(RIGHT_TOP_SENSOR, SENSOR_MODEL); 
SharpIR right_bottom_sensor(RIGHT_BOTTOM_SENSOR, SENSOR_MODEL); 
SimpleKalmanFilter filter(1, 1, 0.001);

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
void filter_sensors_value();
void encoder_1_pulses_count();
void encoder_2_pulses_count();
void START_POINT_func();
void TOWARD_WALL_func();
void AWAY_WALL_func();
void PARALLEL_WALL_func();
void SEE_OBSTACLE_func();
void GET_LOST_func();

volatile unsigned int front_dis = 0;
volatile unsigned int front_ultra_dis = 0;
volatile unsigned int front_right_dis = 0;
volatile unsigned int right_top_dis = 0;
volatile unsigned int right_bottom_dis = 0;
volatile unsigned int encoder_2_pulses = 0;
volatile unsigned int encoder_2_start_time = 0;
volatile int encoder_2_avg_time = 0;
volatile unsigned int encoder_1_pulses = 0;
volatile unsigned int encoder_2_start_time = 0;
volatile int encoder_1_avg_time = 0;
volatile int rotation_tune = 1;
volatile int distance_diff = 1;
volatile int total_val = 5;
volatile int error = 2;

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
  pinMode(TRIG,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO,INPUT);    // chân echo sẽ nhận tín hiệu
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
//  filter_sensors_value();
//  Serial.print(front_ultra_dis);
//  Serial.print(" ");
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
  } while (right_top_dis == 60);
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
  } while (right_top_dis - right_bottom_dis > 0 && front_right_dis >= 16 && right_top_dis != 60 && front_dis > 20 && front_ultra_dis > 10);
  prev_state = AWAY_WALL;
  if (front_dis <= 20 || front_ultra_dis <= 10) {
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
    motors_left(80, constrain(10 + 10 * rotation_tune, 20, 60));
    read_sensors();
    if (distance_diff <= 16 - front_right_dis) {
      rotation_tune++;
    } else rotation_tune--;
    distance_diff == 16 - front_right_dis;
  } while (front_right_dis < 16 && front_dis > 20 && front_ultra_dis > 10 && right_top_dis != 60 );
  prev_state = TOWARD_WALL;
  if (front_dis <= 20 || front_ultra_dis <= 10) {
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
  } while (right_top_dis == right_bottom_dis && front_right_dis < 16 && right_top_dis != 60 && right_bottom_dis !=60 && front_dis > 20 && front_ultra_dis > 10);
  prev_state = PARALLEL_WALL;
  if (front_dis <= 20 || front_ultra_dis <= 10) {
    state = SEE_OBSTACLE;
  } else if (right_top_dis > 45 && front_right_dis > 70) {
    motors_left(120, 100);
    for(int i = 0; i < 10; i++){
       read_sensors();
       if (right_top_dis > 45) break;
    }
    if (right_top_dis > 40) state = GET_LOST;
    else state = AWAY_WALL;
  } else if (right_top_dis == 60 || front_right_dis < 25) {
    state = TOWARD_WALL;
  } else if (right_top_dis > right_bottom_dis) {
    state = AWAY_WALL;
  } else {
    state = TOWARD_WALL;
  } 
}

void SEE_OBSTACLE_func() {
  if (front_ultra_dis <= 15) {
    motors_stop();
    do {
      read_sensors();
    } while(front_ultra_dis <= 10 && right_top_dis != 60);
  } else if (right_bottom_dis == 60 && right_top_dis == 60) {
    motors_hard_left(80, 120);
    do {
      read_sensors();
    } while (front_dis != 60 || front_right_dis < 10);
  } 
  state = PARALLEL_WALL;
}

void GET_LOST_func() {
  motors_hard_left(80, 120);
  do {
    filter_sensors_value();
//    delay(50);
  } while ((front_right_dis > 70 || front_dis < 50) && right_top_dis == 60);
   state = AWAY_WALL;
}

int handle_sensors_noise(int val) {
  if (val > 80 || val < 10) return 81;
  else return val;
}

void read_sensors() {
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
  front_dis = constrain(handle_sensors_noise(front_sensor.distance()), 15, 60);
  front_right_dis = constrain(handle_sensors_noise(front_right_sensor.distance()), 14, 81);
  right_top_dis = constrain(handle_sensors_noise(right_top_sensor.distance()), 10, 60);
  right_bottom_dis = constrain(handle_sensors_noise(right_bottom_sensor.distance()), 10, 60);
}

void filter_sensors_value() {
  int count;
  for(int i = 0; i < total_val; i++) {
    read_sensors();
    if(front_right_dis > 80) {
      count = count + 1;
    }
  }
  if(count > error) {
    front_right_dis = 81;
  }
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
=======
#include "Motor.h"
#include <SharpIR.h> 

#define FRONT_SENSOR A0
#define FRONT_RIGHT_SENSOR A1 
#define RIGHT_TOP_SENSOR A2 
#define RIGHT_BOTTOM_SENSOR A3
#define ECHO 13
#define TRIG 12
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

volatile unsigned int front_ultra_dis = 0;
volatile unsigned int front_right_dis = 0;
volatile unsigned int right_ultra_dis = 0;

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

void setup() { 
  // put your setup code here, to run once:
  setup_motors();
  pinMode(TRIG,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(TRIG2,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO2,INPUT);    // chân echo sẽ nhận tín hiệu
  Serial.begin(9600);
  delay(3000);
  read_sensors();
} 

void loop() {
   if (state < NUM_STATES) {
      (*state_machine[state].function) ();
   }
//  read_sensors();
//  Serial.print(front_ultra_dis);
//  Serial.print(" ");
//  Serial.print(front_right_dis);
//  Serial.print(" ");
//  Serial.println(right_ultra_dis);
} 


void START_POINT_func() {
  motors_left(80, 20);
  do {
    read_sensors();
  } while (front_right_dis == 15);
  state = AWAY_WALL;
}

void AWAY_WALL_func() {
  rotation_tune = 1;
  do {
    motors_right(80, constrain(20 + 15 * rotation_tune, 20, 120));
    read_sensors();
    if (distance_diff <= right_ultra_dis - 9) {
      rotation_tune++;
    } else rotation_tune--;
    distance_diff == right_ultra_dis - 9;
  } while (right_ultra_dis > 9 && front_right_dis >= 18 && front_right_dis != 81 && right_ultra_dis < 90 && front_ultra_dis > 18);
  if (front_ultra_dis <= 18) {
    state = SEE_OBSTACLE;
  } else if (right_ultra_dis > 90 && front_right_dis > 70) {
    state = GET_LOST;
  } else if (right_ultra_dis == 10) {
    state = PARALLEL_WALL;
  } else {
    state = TOWARD_WALL;
  }
}

void TOWARD_WALL_func() {
  rotation_tune = 1;
  do {
    motors_left(80, constrain(10 + 10 * rotation_tune, 20, 60));
    read_sensors();
    if (distance_diff <= 18 - front_right_dis) {
      rotation_tune++;
    } else rotation_tune--;
    distance_diff == 18 - front_right_dis;
  } while (front_right_dis < 18 && front_ultra_dis > 18 && right_ultra_dis < 90);
  if (front_ultra_dis <= 18) {
    state = SEE_OBSTACLE;
  } else if (right_ultra_dis > 90 && front_right_dis > 70) {
    state = GET_LOST;
  } else if (right_ultra_dis == 10) {
    state = PARALLEL_WALL;
  } else {
    state = AWAY_WALL;
  }
}

void PARALLEL_WALL_func() {
  motors_forward(80);
  do {0
    read_sensors();
  } while (right_ultra_dis == 9 && front_right_dis <= 18 && front_ultra_dis > 18);
  if (front_ultra_dis <= 18) {
    state = SEE_OBSTACLE;
  } else if (right_ultra_dis > 20 && front_right_dis > 70) {
    state = GET_LOST;
  } else if (right_ultra_dis <  10) {
    state = TOWARD_WALL;
  } else if (right_ultra_dis > 10) {
    state = AWAY_WALL;
  } else {
    state = TOWARD_WALL;
  } 
}

void SEE_OBSTACLE_func() {
  if (right_ultra_dis < 20) {
    motors_stop();
    do {
      delay(500);
      read_sensors();
    } while(front_ultra_dis < 18);
      delay(500); 
  }
  motors_hard_left(120, 150);
  do {
    read_sensors();
  } while (front_ultra_dis < 90 || front_right_dis < 20);
  state = PARALLEL_WALL;
}

void GET_LOST_func() {
  motors_hard_left(120, 150);
  do {
    read_sensors();
  } while (front_right_dis < 20 || front_ultra_dis < 90 || front_right_dis == 81) ;
  state = AWAY_WALL;
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

//void encoder_1_pulses_count() {
//    if (encoder_1_pulses++ == 374) {
//      encoder_1_pulses = 0;
//      encoder_1_avg_time = millis() - encoder_1_start_time;
//      encoder_1_start_time = millis();
//    }
//}
//
//void encoder_2_pulses_count() {
//    if (encoder_2_pulses++ == 374) {
//      encoder_2_pulses = 0;
//      encoder_2_avg_time = millis() - encoder_2_start_time;
//      encoder_2_start_time = millis();
//    }
//}
>>>>>>> 79d25574267ed4b8d48aa34354142e1fe2b45379
