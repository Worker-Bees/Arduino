#include "Motor.h"
#include <SharpIR.h> 

#define FRONT_RIGHT_SENSOR A0 
#define ECHO 11
#define TRIG 12
#define TRIG2 8
#define ECHO2 13
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
void read_sensors(int max_index);
void encoder_1_pulses_count();
void encoder_2_pulses_count();
void WALL_TRACKING_func();
void SEE_OBSTACLE_func();
void GET_LOST_func();
void MANUAL_CONTROL_func();
void WAITING_GATE_func();
void OBJECT_DETECTION_func();
void adjust_motors();
void switch_mode(int speed);

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
  read_sensors(5);
} 

void switch_mode() {
  motors_stop();
  manual_mode = 1;
}

void loop() {
   if (manual_mode == 1) {
      WAITING_GATE_func();
      OBJECT_DETECTION_func();
      MANUAL_CONTROL_func();
   } else if (state < NUM_STATES) {
      (*state_machine[state].function) ();
   }
//   MANUAL_CONTROL_func();/
//  read_sensors(20);
//  Serial.print(front_ultra_dis);
//  Serial.print(" ");
//  Serial.print(front_right_dis);
//  Serial.print(" ");
//  Serial.println(right_ultra_dis);
} 

void WAITING_GATE_func() {
  const int speed = 75;
  motors_stop();
  Serial.print("h1");
  do {
    read_sensors(20);
  } while (front_ultra_dis >= 60);
  Serial.print("h2");
  do {
    read_sensors(5);
    if (front_ultra_dis > 30) {
      adjust_motors(speed);
    } else motors_stop();
  } while (front_ultra_dis < 60);
  Serial.print("h3");
  do {
    read_sensors(5);
    adjust_motors(speed);
    if (Serial.available()) {
      command = Serial.read();
      Serial.print("receive---");
    }
  } while (command != 'o');
  Serial.print("h4");
  motors_stop();
}

void OBJECT_DETECTION_func() {
  motors_stop();
  do {
    if (Serial.available() > 0) {
      command = Serial.read();
    }
  } while(command != '!');
}

void MANUAL_CONTROL_func() {
//  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("mmmmm");
  motors_stop();
  while(manual_mode == 1){
    if (Serial.available() > 0) {
      command = Serial.read();
      switch (command) {
        case 'w': motors_forward(constrain(65 + command_speed, 41, 255), 60 + command_speed); break;
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
        case 'm': motors_stop(); manual_mode = 0;
      }
      if (command == '~') break;
    }
  }
}

void adjust_motors(int speed) {
  if (front_right_dis <= 70) {
      distance_diff = front_right_dis - 28;
      if (prev_distance_diff != distance_diff) {
         motors_forward(constrain(speed + 10 * distance_diff, 40, 180), speed);
         prev_distance_diff = distance_diff;
      }
  }else {
         motors_forward(constrain(speed + 10 * prev_distance_diff, 41, 120), speed);
  }
}

void WALL_TRACKING_func() {
  const int speed = 70;
  do {
    read_sensors(5);
    adjust_motors(speed);
  } while ((front_right_dis <= 70 || right_ultra_dis <= 40) && front_ultra_dis > 25 && manual_mode == 0);
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
      read_sensors(5);
    } while(front_ultra_dis <= 25 && manual_mode == 0);
      delay(3000); 
  }
  read_sensors(5);
  if (front_ultra_dis <= 25) {
    motors_hard_left(140, 180);
    do {
      read_sensors(5);
    } while (front_ultra_dis < 50 || front_right_dis <= 30 && manual_mode == 0);
  }
  state = WALL_TRACKING;
}

void GET_LOST_func() {
  motors_hard_left(140, 180);
  do {
    read_sensors(5);
  } while (front_right_dis <= 22 || front_ultra_dis < 50 || front_right_dis == 81 && manual_mode == 0) ;
  state = WALL_TRACKING;
}

int handle_sensors_noise(int val) {
  if (val > 80 || val < 10) return 81;
  else return val;
}

void read_sensors(int max_index) {
  int front_temp = 0, front_right_temp = 0, right_temp = 0;
  for (int i = 0; i < max_index; i++) {
    front_right_temp += constrain(handle_sensors_noise(front_right_sensor.distance()), 14, 81);
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
    front_temp += constrain(distance, 4, 500);
    delay(5);
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
    right_temp += constrain(distance, 4, 500);
  }
  front_ultra_dis = front_temp / max_index;
  front_right_dis = front_right_temp / max_index;
  right_ultra_dis = right_temp / max_index;
}
