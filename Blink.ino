/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
#define IN1 7
#define IN2 6
#define MAX_SPEED 255 //từ 0-255
#define MIN_SPEED 0

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

 
void motor1_forward(int speed) { 
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, HIGH);// chân này không có PWM
  analogWrite(IN2, 255 - speed);
}
 
void motor1_backward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, LOW);
  analogWrite(IN2, speed);
}

 
void loop()
{
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  motor1_forward(MAX_SPEED); // motor 1 tiến
  delay(5000);//tiến 5 s
  motor1_forward(120); // motor 1 tiến
  delay(5000);//tiến 5 s
  motor1_backward(MAX_SPEED); // motor 1 tiến
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  delay(5000);//tiến 5 s
}
