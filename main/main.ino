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
#define CONTROL_PIN 3
#define SERVO_PIN 2

#define SENSOR_MODEL 1080 
SharpIR front_right_sensor(FRONT_RIGHT_SENSOR, SENSOR_MODEL); 

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
void switch_mode();

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

volatile State state = WALL_TRACKING;
volatile char command;
volatile int command_speed = 0;
int lenMicroSecondsOfPeriod = 25 * 1000; // 25 milliseconds (ms)
int current = 0;
volatile int manual_mode = 0;
void servo_initialize() {
  for(int i = 0; i <= 3700; i+=10){
       
         digitalWrite(SERVO_PIN, HIGH);

         // Delay for the length of the pulse
         delayMicroseconds(i);

         // Turn the voltage low for the remainder of the pulse
         digitalWrite(SERVO_PIN, LOW);

         // Delay this loop for the remainder of the period so we don't
         // send the next signal too soon or too late
         delayMicroseconds(lenMicroSecondsOfPeriod - i); 
  }
}

void servo_up() {
 int end = constrain(current + 300, 700, 3700);
 for(current; current <= end; current+=10){
         // Servos work by sending a 25 ms pulse.  
         // 0.7 ms at the start of the pulse will turn the servo to the 0 degree position
         // 2.2 ms at the start of the pulse will turn the servo to the 90 degree position 
         // 3.7 ms at the start of the pulse will turn the servo to the 180 degree position 
         // Turn voltage high to start the period and pulse
         digitalWrite(SERVO_PIN, HIGH);

         // Delay for the length of the pulse
         delayMicroseconds(current);

         // Turn the voltage low for the remainder of the pulse
         digitalWrite(SERVO_PIN, LOW);

         // Delay this loop for the remainder of the period so we don't
         // send the next signal too soon or too late
         delayMicroseconds(lenMicroSecondsOfPeriod - current); 
  }
}

void servo_down() {
  int first = constrain(current - 300, 700, 3700);
  for(current; current >= first; current-=10){
       // Servos work by sending a 20 ms pulse.
       // 0.7 ms at the start of the pulse will turn the servo to the 0 degree position
       // 2.2 ms at the start of the pulse will turn the servo to the 90 degree position
       // 3.7 ms at the start of the pulse will turn the servo to the 180 degree position
       // Turn voltage high to start the period and pulse
       digitalWrite(SERVO_PIN, HIGH);

       // Delay for the length of the pulse
       delayMicroseconds(current);

       // Turn the voltage low for the remainder of the pulse
       digitalWrite(SERVO_PIN, LOW);

       // Delay this loop for the remainder of the period so we don't
       // send the next signal too soon or too late
       delayMicroseconds(lenMicroSecondsOfPeriod - current);
  }
}


void setup() { 
  // put your setup code here, to run once:
  setup_motors();
  pinMode(TRIG,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(TRIG2,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(ECHO2,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(CONTROL_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(CONTROL_PIN), switch_mode, RISING);
  servo_initialize();
  read_sensors();
} 

void switch_mode() {
  motors_stop();
  if (manual_mode == 1) {
    manual_mode = 0;
  } else {
    manual_mode = 1;
  }
}

void loop() {
   if (manual_mode == 1) { 
      MANUAL_CONTROL_func();
   } else if (state < NUM_STATES) {
      (*state_machine[state].function) ();
   }
//   MANUAL_CONTROL_func();
//  read_sensors();
//  Serial.print(front_ultra_dis);
//  Serial.print(" ");
//  Serial.print(front_right_dis);
//  Serial.print(" ");
//  Serial.println(right_ultra_dis);
} 

void MANUAL_CONTROL_func() {
//  digitalWrite(LED_BUILTIN, LOW);
  motors_stop();
  while(manual_mode == 1){
    if (Serial.available() > 0) {
      command = Serial.read();
      switch (command) {
        case 'w': motors_forward(constrain(60 + command_speed, 41, 255), 60 + command_speed); break;
        case 'q': motors_forward(constrain(41 + command_speed, 41, 255), 80 + command_speed); break;
        case 'e': motors_forward(constrain(80 + command_speed, 41, 255), 41 + command_speed); break;
        case 'a': motors_hard_left(80 + command_speed, 120 + command_speed); break;
        case 'd': motors_hard_right(120 + command_speed, 80 + command_speed); break;
        case 's': motors_backward(60 + command_speed, 60 + command_speed); break;
        case 'z': motors_backward(41 + command_speed, 80 + command_speed); break;
        case 'c': motors_backward(80 + command_speed, 41 + command_speed); break;
        case '+': command_speed += 10; break;
        case '-': command_speed -= 10; break;
        case 'j': servo_up(); break;
        case 'k': servo_down(); break;
        case 'x': motors_stop(); break;
      }
      if (command == '~') break;
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
  } while ((front_right_dis <= 70 || right_ultra_dis <= 90) && front_ultra_dis > 25 && manual_mode == 0);
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
    } while(front_ultra_dis <= 25 && manual_mode == 0);
      delay(3000); 
  }
  read_sensors();
  if (front_ultra_dis <= 25) {
    motors_hard_left(120, 100);
    do {
      read_sensors();
    } while (front_ultra_dis < 50 || front_right_dis <= 22 && manual_mode == 0);
  }
  state = WALL_TRACKING;
}

void GET_LOST_func() {
  motors_hard_left(120, 100);
  do {
    read_sensors();
  } while (front_right_dis <= 22 || front_ultra_dis < 50 || front_right_dis == 81 && manual_mode == 0) ;
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
