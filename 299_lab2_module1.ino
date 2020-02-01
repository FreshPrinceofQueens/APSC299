
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

float distance = 0.0;
float curVeloc = 0.0;
float curTime = 0.0;
float preVeloc = 0.0;
float preTime = 0.0;
//LSM6DS3Class myIMU;

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
//;
  attachInterrupt(digitalPinToInterrupt(2), forInter1, CHANGE);  //when any change occurs on interupt 0 (aka pin 2), forInter() is called
  attachInterrupt(digitalPinToInterrupt(3), forInter2, CHANGE);  //pin 3 (left wheel = wheel 2)
  //;
  interrupts(); //this function enables interupts
//;
  gripper.attach(9); //associates gripper object with pin 9
  tilt.attach(8); //associates tilt object with pin 8
  pan.attach(10); //associates pan object with pin 10
  //;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void forInter1() //interrupt for right wheel (wheel 1)
{
  count++;
  
//  float ttrf = (float)ttr;
//  distance = ((aSpd2 * (ttrf/1000))+(aSpd1*(ttrf/1000))) /2;

//  float rot = count/16;
//  float totl = 2 * pi * 0.0325 *rot;
//  float ttr = millis();
//  sppddd = totl/(ttr/1000);
  

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





float getSpeed1() //wheel 1 aka right wheel aka pin 2 aka interrupt 0
{
  //Serial.println("Getting Right Wheel Speed");
  int time2;
  int time1;

  time1 = millis();
  count = 0;


  while (count <= 16) {
    Serial.print("");
  }

  Serial.println("");
  time2 = millis();
  float diff = time2 - time1;

  //Serial.println((String)diff + "s");


  float dist = 2 * pi * 0.0325;

  //Serial.println((String)dist + "m");
  float wheelSpeed = dist / (diff / 1000);
  //Serial.println((String)wheelSpeed + "m/s");

  return wheelSpeed;
}





float getSpeed2() //wheel 2 aka left wheel aka pin 3 aka interrupt 1
{
  //Serial.println("Getting left wheel speed");
  int time2;
  int time1;

  time1 = millis();
  count2 = 0;


  while (count2 <= 16) {
    Serial.print("");
  }

  Serial.println("");
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
  adaptwheel2(dSpd);//used to be dSpd
  Serial.println("Adapted");
  driveMotor1(wSpd1, wRot1);
  driveMotor2(wSpd2, wRot2);
}





void adaptwheel1(float desired)
{

  //aSpd1 = getSpeed1();
  //Serial.println((String)"Wheel 1 PWM BEFORE "+wSpd1);
  
  float diff = desired - aSpd1;
  //
  int r_wheel_change = 0;
  
  if(diff >= 0.01 || diff <= -0.01) r_wheel_change = (int) (K * (diff)); //returns change for pulse width modulations
  else r_wheel_change = 0;
  wSpd1 += r_wheel_change;
  if(wSpd1 >= 255) wSpd1 = 250;
 
  
  
  //Serial.println((String)"Wheel 1 PWM AFTER "+wSpd1);
}





void adaptwheel2(float desired)
{
  //aSpd2 = getSpeed2();
  //Serial.println((String)"Wheel 2 PWM BEFORE "+wSpd2);

  float diff = desired - aSpd2;
  int l_wheel_change = 0;
  
  if(diff >= 0.01 || diff <= -0.01) l_wheel_change = (int)(K * (diff)); //returns change for pulse width modulations
  else l_wheel_change = 0;
  wSpd2 += l_wheel_change;
  if(wSpd2 >= 255) wSpd2 = 250;
  
  
  //Serial.println((String)"Wheel 2 PWM After "+wSpd2);
}





float getIMUAccel(){
  float accelx = 0;
  float accely = 0;
  float accelz = 0;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(accelx, accely, accelz);

    accelx = accelx * 9.81;
    accely = accely * 9.81;

    float groundAccel = sqrt((accelx*accelx)+(accely*accely));

    if(groundAccel < 0.30) groundAccel = 0;

    return groundAccel;
  }
  else return -1;
}




float getWDistance(){
  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();

  curVeloc = (aSpd1+aSpd2)/2;
  curTime = millis()/1000;
  float timediff = curTime - preTime;

  float addeddistance = ((curVeloc + preVeloc)/2) *timediff;
  distance = distance + addeddistance;

  preTime = curTime;
  preVeloc = curVeloc;
  
  return distance;
}




float getWAccel()
{
  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();

  curVeloc = (aSpd1+aSpd2)/2;
  curTime = millis()/1000;
  float timediff = curTime - preTime;

  float accelNow = ((curVeloc - preVeloc) / timediff);
  return accelNow;
}




void printInfo(){
  float accel = getWAccel();
  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();
  float dists = getWDistance();

  Serial.println("Robot Movement Information: ");
  Serial.print("Right Wheel Velocity: ");
  Serial.print(aSpd1);
  Serial.println(" m/s");
  Serial.print("Left Wheel Velocity: ");
  Serial.print(aSpd2);
  Serial.println(" m/s");
  Serial.print("Current Aceelaration: ");
  Serial.print(accel);
  Serial.println(" m/s2");
  Serial.print("Total Distance Travelled: ");
  Serial.print(dists);
  Serial.println(" m");
}





void loop()
{

  Serial.println("----------------------------Sequence Begin----------------------------");
  wSpd1 = wSpd2 = 128;
  wRot1 = wRot2 = HIGH;
  driveMotor1(wSpd1, wRot1);
  driveMotor2(wSpd2, wRot2);

  printInfo();
  delay(1000);

  /*
  
  Serial.println("----------------------------Sequence Begin----------------------------");
  
  
  driveMotor1(wSpd1, wRot1);
  driveMotor2(wSpd2, wRot2);

//  Serial.println(sppddd);
  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();
  Serial.println("");
  Serial.println((String)"Right wheel is" + aSpd1 + " m/s");
  Serial.println((String)"Left wheel is" + aSpd2 + " m/s");
  Serial.println((String)"Travelled "+distance+" m");
  adaptToSpeed();

  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();
  Serial.println("");
  Serial.println((String)"Right wheel is" + aSpd1 + " m/s");
  Serial.println((String)"Left wheel is" + aSpd2 + " m/s");
 
//  float myacc = curAccel();
//  Serial.print("Current Acceleration is ");
//  Serial.print(myacc);
//  Serial.println(" m/s");

  Serial.println("----------------------------Sequence Completed----------------------------");
  delay(6000);
*/
}
