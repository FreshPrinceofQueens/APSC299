
#include <Servo.h>
#include <Arduino_LSM6DS3.h>


#define M1 7
#define E1 6
#define E2 5
#define M2 4
#define wENC 2 //encoder attached to "interupt #0" or pin 2. Pin 2 and 3 are reserved for interupt 0 and 1.
#define wENC2 3
#define pi 3.14
#define K 600


int val3 = LOW; //state of wheel initially low
int preval3 = HIGH, preval4;
int count = 0;
int count2 = 0;

int wSpd; //variable to hold speded value for wheels
int wRot; //variable to hold rotation value for wheels

int wSpd1 = 200; //variable to hold speded value for wheels
int wRot1 = HIGH; //variable to hold rotation value for wheels

int wSpd2 = 200; //variable to hold speded value for wheels
int wRot2 = HIGH; //variable to hold rotation value for wheels

float aSpd1;
float aSpd2;
float dSpd = 0.2;

int initt = 0;

Servo tilt, pan, gripper;

//wheel 1 is right wheel, wheel 2 is left






void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //initializer

  pinMode(wENC, INPUT); //encoder input pin
  pinMode(wENC2, INPUT); //encoder for second wheel
  pinMode(E1, OUTPUT); //speed pin for motor 1
  pinMode(E2, OUTPUT); // speed pin for motor 2
  pinMode(M1, OUTPUT); // direction pin for motor 1
  pinMode(M2, OUTPUT); //direction pin for motor 2
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  attachInterrupt(0, forInter1, CHANGE);  //when any change occurs on interupt 0 (aka pin 2), forInter() is called
  attachInterrupt(1, forInter2, CHANGE);  //pin 3 (left wheel = wheel 2)
  interrupts(); //this function enables interupts

  gripper.attach(9); //associates gripper object with pin 9
  tilt.attach(8); //associates tilt object with pin 8
  pan.attach(10); //associates pan object with pin 10

}

void forInter1() //interrupt for right wheel (wheel 1)
{
  count++;
}





void forInter2() //interrupt for right wheel (wheel 1)
{
  count2++; //increment the count which represents the number of state transitions from high to low
}





void driveMotors() {
  driveMotor1(wSpd, wRot); //right wheel
  driveMotor2(wSpd, wRot); //left wheel
}





void driveMotor1(int spd, int rotation) //allows control of speed and rotation direction for motor one
{
  digitalWrite(M1, rotation);
  analogWrite(E1, spd);
}





void driveMotor2(int spd, int rotation) //allows control of speed and rotation direction for motor two
{
  digitalWrite(M2, rotation);
  analogWrite(E2, spd);
}





void gripAt(int angle)

{
  //past fully open at 0 degrees
  //fully open at 40
  //partly closed at 120
  //fully closed at 180

  if (angle < 20) angle = 20;
  gripper.write(angle);
}





void panTo(int angle)
{
  /*
    fully left at 0
    centered at 90
    fully right at 180

  */

  if (angle < 20) angle = 20;
  pan.write(angle);
}





void tiltTo(int angle)
{
  /*
    hits plate at 10
    horizontal at 70
    vertical at 160
    past vertical at 180

  */

  if (angle < 20) angle = 20;
  tilt.write(angle);
}





int convertToPWM(int value) {
  int result = (value / 0.0325) * (255 / (6 * pi));
  return result;
}





float actSpeed1() //wheel 1 aka right wheel aka pin 2 aka interrupt 0
{
  Serial.println("Getting Right Wheel Speed");
  int time2;
  int time1;

  time1 = millis();
  count = 0;


  while (count <= 16) {
    Serial.print("");
  }

  time2 = millis();
  float diff = time2 - time1;

  //Serial.println((String)diff + "s");


  float dist = 2 * pi * 0.0325;

  //Serial.println((String)dist + "m");
  float wheelSpeed = dist / (diff / 1000);
  //Serial.println((String)wheelSpeed + "m/s");

  return wheelSpeed;
}





float actSpeed2() //wheel 2 aka left wheel aka pin 3 aka interrupt 1
{
  Serial.println("Getting left wheel speed");
  int time2;
  int time1;

  time1 = millis();
  count2 = 0;


  while (count2 <= 16) {
    Serial.print("");
  }

  time2 = millis();
  float diff = time2 - time1;

  //Serial.println((String)diff + "s");


  float dist = 2 * pi * 0.0325;

  //Serial.println((String)dist + "m");
  float wheelSpeed = dist / (diff / 1000);
  //Serial.println((String)wheelSpeed + "m/s");

  return wheelSpeed;
}





void adaptToSpeed()
{
  Serial.println("Adapting");
  adaptwheel1(dSpd);
  adaptwheel2(dSpd);
  Serial.println("Adapted");
  driveMotor1(wSpd1, wRot1);
  driveMotor2(wSpd2, wRot2);
}





void adaptwheel1(float desired)
{

  aSpd1 = actSpeed1();
  Serial.println((String)"Wheel 1 PWM BEFORE "+wSpd1);
  
  float diff = desired - aSpd1;
  //
  
  
  int r_wheel_change = (int) (K * (diff)); //returns change for pulse width modulations
  wSpd1 += r_wheel_change;
 
  
  
  Serial.println((String)"Wheel 1 PWM AFTER "+wSpd1);
}





void adaptwheel2(float desired)
{
  aSpd2 = actSpeed2();
  Serial.println((String)"Wheel 2 PWM BEFORE "+wSpd2);

  float diff = desired - aSpd2;
  
  int l_wheel_change = (int)(K * (diff)); //returns change for pulse width modulations
  wSpd2 += l_wheel_change;
  
  
  Serial.println((String)"Wheel 2 PWM After "+wSpd2);
}





void loop()
{


  driveMotor1(wSpd1, wRot1);
  driveMotor2(wSpd2, wRot2);

  //Serial.println((String)"Count: "+count);
  //Serial.println((String)"Count2: "+count2);


  aSpd1 = actSpeed1();
  aSpd2 = actSpeed2();
  Serial.println("");
  Serial.println((String)"Right wheel is" + aSpd1 + " m/s");
  Serial.println((String)"Left wheel is" + aSpd2 + " m/s");
 // delay(2000);

  adaptToSpeed();

 // delay(2000);

  aSpd1 = actSpeed1();
  aSpd2 = actSpeed2();
  Serial.println("");
  Serial.println((String)"Right wheel is" + aSpd1 + " m/s");
  Serial.println((String)"Left wheel is" + aSpd2 + " m/s");
 // delay(4000);


  Serial.println("Sequence Completed");
}
