
#include <Servo.h>
#include <Arduino_LSM6DS3.h>

#define M1 7
#define E1 6
#define E2 5
#define M2 4
#define wENC 2 //encoder attached to "interupt #0" or pin 2. Pin 2 and 3 are reserved for interupt 0 and 1.
#define wENC2 3
#define SENSECOLOUR 3
#define pi 3.14
#define K 600
#define IMUK 2150
#define DISTK 475
#define LEFTBUMPER 11
#define RIGHTBUMPER 12
#define IRSENSOR 5


#define LEFT 0
#define RIGHT 1
#define FORWS 2 //for different directions, allows us to just use variable name instead.
#define BACKWS 3

#define NORTH 0
#define SOUTH 180
#define WEST 270
#define EAST 90


float wallLength =  1.7

float wheelSpeed = 0.15;

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
float aSpd2;    //variables used to hold sped of individual wheels
float dSpd = 0.2;

int initt = 0;

float distance = 0.0;
float curVeloc = 0.0;
float curTime = 0.0;
float preVeloc = 0.0; //variables for tracking movement and distance of robot
float preTime = 0.0;
int stopSignal = 0;
int colorSignal = 0;
int orientation = NORTH;
int pre_orientation = -1;


float cordX = 0.0, cordY = 0.0;
//for coordinate system

int invertCSG = 0;
//0 means checkColorSignal will return true if colorsignal > 1000
//1 means checkColorSignal will return false if colorsignal > 1000

boolean onBase = false;
//used to say if  the robot is on base;

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
  pinMode(LEFTBUMPER, INPUT);
  pinMode(RIGHTBUMPER, INPUT);
  pinMode(IRSENSOR, INPUT);
  pinMode(SENSECOLOUR, INPUT);
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
  // you're connected now, so print out the data:
}

//--------------------------------------------FUNCTION BEGIN----------------------------------
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

//-------------------------------------------------------------------------------------------
void forInter2() //interrupt for right wheel (wheel 1)
{
  count2++; //increment the count which represents the number of state transitions from high to low
}

//-------------------------------------------------------------------------------------------
void driveMotors() {
  driveMotor1(wSpd, wRot); //right wheel
  driveMotor2(wSpd, wRot); //left wheel
}

//-------------------------------------------------------------------------------------------
void driveMotor1(int spd, int rotation) //allows control of speed and rotation direction for motor one
{

  digitalWrite(M1, rotation);
  analogWrite(E1, spd);

}

//-------------------------------------------------------------------------------------------
void driveMotor2(int spd, int rotation) //allows control of speed and rotation direction for motor two
{
  digitalWrite(M2, rotation);
  analogWrite(E2, spd);
}
//-------------------------------------------------------------------------------------------
void forwards() {
  driveMotor1(128, HIGH);
  driveMotor2(128, HIGH);
}
//-------------------------------------------------------------------------------------------
void backwards() {
  driveMotor1(128, LOW);
  driveMotor2(128, LOW);
}
//-------------------------------------------------------------------------------------------
void backwards(float timing) { //possibly recall
  driveMotor1(128, LOW);
  driveMotor2(128, LOW);
  delay(timing);
}
//-------------------------------------------------------------------------------------------
void forwards(float timing) { //possibly recall
  driveMotor1(128, HIGH);
  driveMotor2(128, HIGH);
  delay(timing);
}
//-------------------------------------------------------------------------------------------
void stopWheels(float timing) {
  driveMotor1(0, LOW);
  driveMotor2(0, LOW);
  delay(timing);
}

//-------------------------------------------------------------------------------------------
void stopWheels() {
  driveMotor1(0, LOW);
  driveMotor2(0, LOW);
}
//-------------------------------------------------------------------------------------------
int forwardsBy(float distance) { //possibly recall
  //at 128PWM speedis approx 0.18m/s

  float time1 = 0.0, time2 = 0.0, diff = 0.0, dist = 0.0;
  float timing = (distance / wheelSpeed) * 1000;
  int exitstatus = 0;
  time1 = millis();
  forwards();
  while (true) {
    time2 = millis();
    diff = time2 - time1;

    if (toReact() == 1) {
      exitstatus = 2;
      break;
    }

    if (analogRead(IRSENSOR) >= 400) {
      IR_Avoid();
      exitstatus = 1;
      break;
    }

    if (diff >= timing) {
      exitstatus = 0;
      break;
    }
  }

  stopWheels();

  if (exitstatus == 2) {
    //barrier error, recall fucntion
    reactBumpers();
    dist = (diff * wheelSpeed * 1000);
    forwardsBy(distance - dist);
  }

}
//-------------------------------------------------------------------------------------------
boolean checkColorSignal(){
  boolean result = false;
  if(colorSignal >= 1000)return = true;//colorsignal greater than 1000 when on base
  else result = false;

  if(invertCSG == 1){
    if(result == true) result = false;
    if(result == false) result = true;
  }

  return result;
}
//-------------------------------------------------------------------------------------------
int go(float distance, int dir) { //go function for driving the wheels
  /*
   * exit status key:
   * -1: not asserted (shouldn't happen)
   * 0: clear run
   * 1: IR sensor sensed incoming object
   * 2: Bumper came in contact with object
   * 4: for hitting homebase
   * 5: for asserted stop signal.
   */

 
  if(distance <0){ //support for negative distances
    distance = distance * -1;
    if(dir == RIGHT) dir = LEFT;
    if(dir == LEFT) dir = RIGHT;
    if(dir == FORWS) dir = BACKWS;
    if(dir == BACKWS) dir = FORWS;
  }

   //switch case for the direction
  //different directions require the wheels to be turned first before 
  //moving forward.

   switch (dir){
    case LEFT:
      turnWheels(90, LEFT);
      break;

    case RIGHT:
      turnWheels(90, RIGHT);
      break;

    case BACKWS:
      turnWheels(180, RIGHT);
      break;

    case FORWS:
      break;
   }
  
    int exitstatus = -1;
    //exit status
    float addtoX = 0.0, addtoY = 0.0;
    float time1 = 0.0, time2 = 0.0, diff = 0.0, curdist = 0.0; //initial variables
  
    float timing = (distance / wheelSpeed) * 1000; //converting given distance to amount of time
    
    time1 = millis(); //takes time before while loop begins
    forwards(); //makes robot go forward
    while (true) { //for driving
      time2 = millis();
      diff = time2 - time1;
      curdist = diff * wheelSpeed;

      //essential idea is that the time2 is updated for each while loop run
      //the diff is then checked with timing, if it equal to or greater than the timing
      //then the robot has travelled the required distance and it will break out of the loop
      //curdist is used to track robot movement so as to properly update coordinates
      //if there is an error that caused robot to stop traversing/

      if(stopSignal == 1){
        //have been told to stop, need to break out of both while loops
        exitstatus = 5;
        break;
      }
      if (analogRead(IRSENSOR) >= 400) {
        //ir sensor senses something close
      exitstatus = 1;
      break;
      }
      if (toReact() == 1) {
        //bumpers are hit
        exitstatus = 2;
        break;
      }

      colorSignal = analogRead(SENSECOLOUR); //read from ir sensor
      if(checkColorSignal()){
        exitstatus = 4;
        break;
      }
      
      if (diff >= timing) {
        //drove full length
        exitstatus = 0;
        break;
      }

    }
    stopWheels();

  //designed to properly update coordinates properly based on the orientation of the robot
  if(pre_orientation == -1) pre_orientation = NORTH;
   switch(pre_orientation){
    case NORTH:
    if(dir == FORWS) addtoY = curdist;
    else if(dir == BACKWS) addtoY = curdist * -1;
    else if(dir == RIGHT) addtoX = curdist;
    else if (dir == LEFT) addtoX = curdist * -1;
    break;
    
    case EAST:
    if(dir == FORWS) addtoX = curdist;
    else if(dir == BACKWS) addtoX = curdist * -1;
    else if(dir == RIGHT) addtoY = curdist * -1;
    else if (dir == LEFT) addtoY = curdist;
    break;

    case SOUTH:
    if(dir == FORWS) addtoY = curdist * -1;
    else if(dir == BACKWS) addtoY = curdist;
    else if(dir == RIGHT) addtoX = curdist * -1;
    else if (dir == LEFT) addtoX = curdist;
    break;

    case WEST:
    if(dir == FORWS) addtoX = curdist * -1;
    else if(dir == BACKWS) addtoX = curdist;
    else if(dir == RIGHT) addtoY = curdist;
    else if (dir == LEFT) addtoY = curdist * -1;
    break;
   }

   cordX = cordX + addtoX;
   cordY = cordY + addtoY;

    switch(exitstatus){
      //exitstatus info
      case 0:
      Serial.println("Exit Status 0: Clear Run.");
      break;
      case 1:
      Serial.println("Exit Status 1: IR Error.");
      break;
      case 2:
      Serial.println("Exit Status 2: Bumper Barrier Error.");
      break;
      case 4:
      Serial.println("Exit Status 4: Color signal asserted.");
      break;
      case 5:
      Serial.println("Exit Status 5: Stop signal asserted.");
      break;
    }

    if (exitstatus == 2) {
      //barrier error, recall fucntion
      Serial.println("Exit Status 2");
      float totravel = distance - curdist;
      go(totravel, FORWS);
    }
    return exitstatus;
}
//-------------------------------------------------------------------------------------------
void gripAt(int angle) //gripping robot

{
  //past fully open at 0 degrees
  //fully open at 40
  //partly closed at 120
  //fully closed at 180

  if (angle < 20) angle = 20;
  gripper.write(angle);
}
//-------------------------------------------------------------------------------------------
void panTo(int angle) //panning robot
{
  /*
    fully left at 0
    centered at 90
    fully right at 180

  */

  if (angle < 20) angle = 20;
  pan.write(angle);
}
//-------------------------------------------------------------------------------------------
void tiltTo(int angle) //tilting robot
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
//-------------------------------------------------------------------------------------------
int convertToPWM(int value) { //converts a given speed (m/s) to PWM value.
  int result = (value / 0.0325) * (255 / (6 * pi));
  return result;
}
//-------------------------------------------------------------------------------------------
float getSpeed1() //wheel 1 aka right wheel aka pin 2 aka interrupt 0
{
  //Serial.println("Getting Right Wheel Speed");
  int time2;
  int time1;

  time1 = millis();
  count = 0; //wheel 1 counter


  while (count <= 16) {
    Serial.print("");
  }

  Serial.println("");
  time2 = millis();
  float diff = time2 - time1;

  //uses the wheel encoders to get the time for one wheel rotation
  //(one rotation is 16 encoder counts)
  //encoder count is run through the interrupt functions above


  float dist = 2 * pi * 0.0325;
  //one wheel rotation is dist m.

  float wheelSpeed = dist / (diff / 1000);
  //calculates the speed.
  
  return wheelSpeed;
}
//-------------------------------------------------------------------------------------------
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

  //uses the wheel encoders to get the time for one wheel rotation
  //(one rotation is 16 encoder counts)
  //encoder count is run through the interrupt functions above


  float dist = 2 * pi * 0.0325;
  //one wheel rotation is dist m.

  float wheelSpeed = dist / (diff / 1000);
  //calculates the speed.
  
  return wheelSpeed;
}
//-------------------------------------------------------------------------------------------
void adaptToSpeed() //adapting wheels of robot to drive straight
{
  Serial.println("Adapting");
  adaptwheel1(dSpd);
  adaptwheel2(dSpd);//used to be dSpd
  Serial.println("Adapted");
  driveMotor1(wSpd1, wRot1);
  driveMotor2(wSpd2, wRot2);
}
//-------------------------------------------------------------------------------------------
void adaptwheel1(float desired) //adapting wheel 1 to the speed given through method paramters
{

  //aSpd1 = getSpeed1();
  //Serial.println((String)"Wheel 1 PWM BEFORE "+wSpd1);

  float diff = desired - aSpd1;
  //
  int r_wheel_change = 0;

  if (diff >= 0.01 || diff <= -0.01) r_wheel_change = (int) (K * (diff)); //returns change for pulse width modulations
  else r_wheel_change = 0;
  wSpd1 += r_wheel_change;
  if (wSpd1 >= 255) wSpd1 = 250;



  //Serial.println((String)"Wheel 1 PWM AFTER "+wSpd1);
}
//-------------------------------------------------------------------------------------------
void adaptwheel2(float desired) //adapting wheel 2 to the speed given through method paramters
{
  //aSpd2 = getSpeed2();
  //Serial.println((String)"Wheel 2 PWM BEFORE "+wSpd2);

  float diff = desired - aSpd2;
  int l_wheel_change = 0;

  if (diff >= 0.01 || diff <= -0.01) l_wheel_change = (int)(K * (diff)); //returns change for pulse width modulations
  else l_wheel_change = 0;
  wSpd2 += l_wheel_change;
  if (wSpd2 >= 255) wSpd2 = 250;


  //Serial.println((String)"Wheel 2 PWM After "+wSpd2);
}
//-------------------------------------------------------------------------------------------
float getIMUAccel() { //gets acceleration of robot from IMU
  float accelx = 0;
  float accely = 0;
  float accelz = 0;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(accelx, accely, accelz);

    accelx = accelx * 9.81;
    accely = accely * 9.81;

    float groundAccel = sqrt((accelx * accelx) + (accely * accely));
    //combines them using pythagoras

    if (groundAccel < 0.30) groundAccel = 0;
    //threshold for some acceleration.

    return groundAccel;
  }
  else return -1;
}
//-------------------------------------------------------------------------------------------
float getIMUGyroX() { //gets x component from gyroscope, in degrees/s
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    return x;
  }
  else return -1;
}
//-------------------------------------------------------------------------------------------
float getIMUGyroY() { //gets y component from gyroscope, in degrees/s
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    return z;
  }
  else return -1;
}
//-------------------------------------------------------------------------------------------
float getIMUGyroZ() { //gets z component from gyroscope, in degrees/s

  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    return y;
  }
  else return -1;
}
//-------------------------------------------------------------------------------------------
float getVertAccel() { //gets vertical acceleration from IMU

  float accelx = 0;
  float accely = 0;
  float accelz = 0;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(accelx, accely, accelz);

    accelz = accelz * 9.81;

    float groundAccel = accelz;

    return groundAccel;
  }
  else return -1;
}
//-------------------------------------------------------------------------------------------
float getWDistance() { //gets distance travelled based on wheel encoders
  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();

  curVeloc = (aSpd1 + aSpd2) / 2;
  curTime = millis() / 1000;
  float timediff = curTime - preTime;

  float addeddistance = ((curVeloc + preVeloc) / 2) * timediff;
  distance = distance + addeddistance;

  preTime = curTime;
  preVeloc = curVeloc;

  return distance;
}
//-------------------------------------------------------------------------------------------
float getWAccel() //gets robot acceleration based on wheel speeds
{
  aSpd1 = getSpeed1();
  aSpd2 = getSpeed2();

  curVeloc = (aSpd1 + aSpd2) / 2; //average speed of robot
  curTime = millis() / 1000;
  float timediff = curTime - preTime;

  float accelNow = ((curVeloc - preVeloc) / timediff); //kinematics functions
  return accelNow;
}
//-------------------------------------------------------------------------------------------
void printInfo() {//possibly recall
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
//-------------------------------------------------------------------------------------------
void turnWheels(float angle, int dir) {
  //turnwheels with the specified angle in the specified direction
  //either RIGHT or LEFT

  if(angle<0){
    angle = angle * -1;
    if(dir == RIGHT) dir = LEFT;
    if(dir == LEFT) dir = RIGHT;
  }
  
  angle = angle / 180 * 1400;
  //conversion from given angle to specific millisecond delay for wheels
  pre_orientation = orientation;
  
  stopWheels(200);
  Serial.println("In angle");
  Serial.println(dir);
  Serial.println(angle);

  if (dir == RIGHT) {
    driveMotor1(128, LOW );
    driveMotor2(128, HIGH);
    delay(angle); //driving to right angle

    orientation = orientation + angle;
  }

  if (dir == LEFT) {
    driveMotor1(128, HIGH );
    driveMotor2(128, LOW);
    delay(angle);

    orientation = orientation + (360 -angle);
  }

  if(orientation == 360) orientation = 0;
  
  stopWheels(200);
}
//-------------------------------------------------------------------------------------------
void resetOrientation(){
  turnWheels(orientation, LEFT);
  pre_orientation = -1;
}
//-------------------------------------------------------------------------------------------
int toReact() { //to determine if bumpers should react
  int r_value = digitalRead(RIGHTBUMPER);
  int l_value = digitalRead(LEFTBUMPER);
  if (l_value == LOW || r_value == LOW) return 1;
  else return 0;
}
//-------------------------------------------------------------------------------------------
void reactBumpers() {//react bumpers

  delay(200);
  int r_value = digitalRead(RIGHTBUMPER);
  int l_value = digitalRead(LEFTBUMPER);

  if (l_value == LOW && r_value == LOW) {
    //back up and pivot 180
    Serial.println("BOTH");
    delay(500);
    backwards(1000);
    turnWheels(180, RIGHT);
  }

  if (l_value == LOW && r_value == HIGH) {
    Serial.println("LEFT PUSHED RIGHT UNPUSHED");
    delay(500);
    backwards(1000);
    turnWheels(45, RIGHT);
  }
  if (l_value == HIGH && r_value == LOW) {
    //back up and pivot 45 degrees to the left
    Serial.println("RIGHT PUSHED, LEFT UNPUSHED");
    delay(500);
    backwards(1000);
    turnWheels(45, LEFT);
  }

}
//-------------------------------------------------------------------------------------------
void readIRSensors() {
  float value = analogRead(IRSENSOR);
  Serial.print("Value is ");
  Serial.println(value);
  delay(500);
  while (value >= 150) {
    value = analogRead(IRSENSOR);
    driveMotor1(50, HIGH);
    driveMotor2(50, HIGH);

    if (value >= 300) {
      backwards(1000);

      stopWheels(500);

      turnWheels(180, RIGHT);
      forwards();
      break;
    }
  }

}
//-------------------------------------------------------------------------------------------
void IR_Avoid() {

//called when IR sensor is hit
  backwards(300);
  stopWheels(500);
  turnWheels(180, RIGHT);

}
//-------------------------------------------------------------------------------------------

void search() {
  //ran when searching the enclosure for the base

  int exitstatus = -1;
  //variable for exit status
  
  ending = -1;
  //used to determine what side of the enclosure we are on, necessary for proper turns
  //if at beginning, it is -1, when it reaches another end it is multiplied negative 1.
  //beginning -> -1
  //end -> 1;
  
  float width = 0.0;
  //width of enclosure traversed.
  
  float preCordX = -1, preCordY = -1;
  
  invertCSG = 0;
  //needs to stop when reached base, therefore when colorSignal is over 1000
  //this means that we do not invertCSG
  
  while (true) { 
    
    exitstatus = go(wallLength, FORWS);
    //first begins with traveling the length of the enclosure
    
    if (exitstatus == 0) {
      //traversed the whole length
      //at new point of enclosure therefore change ending.
      ending = ending * -1;
      
      if (width <= 0.60) {
        //individual sections are 60cm each therefore if wehave reached 60cm in width
        //we can no longer traverse unless we will collide with another robot
        
        if (ending == 1) {
          //we are at end of enclosure
          //offset to the right to search another part of our section
          go(0.15, RIGHT);
          turnWheels(90, RIGHT);
        }
        else
        {
          //at the beginning therefore need to turn left to search more of the section
          go(0.15,LEFT);
          turnWheels(90, LEFT);
        }

        width = width +0.15;
        //increment the width
      }
      else
      {
        stopWheels();
        //individual sections are 60cm each therefore if wehave reached 60cm in width
        //we can no longer traverse unless we will collide with another robot
      }
    }
    else if(exitstatus == 4){
    //have hit homebase need to run homebase code, break out of while loop
    break;
   }
   else if(exitstatus == 5){
    //have encountered stop signal therefore need to break out of while loop
    break;
   }

  } //end while loop

  if(exitstatus == 4){
    //need to go to base, run drive to base function
    driveOntoBase();
  }

  if(exitstatus == 5){
    //drive to given location from robot.
    //given location is in x and y coordinates,
    //compare using our current coordinates and subtract.
    driveToLocation();
    //should be at base now, therefore need to call driveontobase function
  driveOntoBase();
  }


}
//--------------------------------------------------------------------------------------------
void driveOntoBase() {
  int exitstatus = 0;
  int turnangle = 90;
  invertCSG = 1; //inverts the values of checkColorSignal();
  //now go function will exit if the colorsignal < 1000 (therefore will exit if robot is not
  //not on base
  
  while(exitstatus != 0){
    //if exitstatus, it was a clear run
    //therefore  robot drove onto base properly
    
  exitstatus = go(0.25, FORWS);; //go forwards onto the base the robot length

  if(exitstatus == 4) {
    //drove off of base, needs to recalibrate and find direction of base.
    while(colorSignal<1000){
      turnWheels(turnangle, RIGHT);
      colorSignal = analogRead(SENSECOLOUR);
      turnangle = turnangle+90;

      //continues turning and reading the colorsignal, thatis
      //turns 90, reads from colorsignal, if it is not a greater than a 1000 then turn again
    }

    //will exit while loop if color signal is greater than 1000
    //therefore on base and should drive up on it
  }
  }

}
//--------------------------------------------------------------------------------------------
void drivetoLocation(float rec_x, float rec_y){
  float distx = rec_x - cordX;
  float disty = rec_y - cordY;

  //passed in coordinates are based on North orientation, therefore robot must be set
  //to north orientation before being told to drive.
  resetOrientation();

  if(distx < 0){
    //needs to go in -x direction, therefore to the left
    distx = distx * -1;
    go(distx, LEFT);  
  }
  else
  {
    //positive x direction therefore go right
    go(distx, RIGHT);
  }
  
  resetOrientation();
  if(disty < 0){
    //needs to go -y direction, therefore backwards
    disty = disty * -1;
    go(disty, BACKWS);
  }
  else
  {
    //positive y direction, therefore go forwards
    go(disty, FORWS);
  }
}

//---------------------------------------------FUNCTION END--------------------------------------
void loop() {
  //square
  search();
}
