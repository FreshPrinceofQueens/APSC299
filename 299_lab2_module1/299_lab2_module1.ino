#include <Servo.h>
#include <Arduino_LSM6DS3.h>
#include "Comms.h"



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

const int idle= 0;
const int SEARCHING = 1;
const int FOUND = 2;
const int DRIVING = 3;


float wallLength =  1.7;

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
float rec_cordX = -1, rec_cordY = -1;
int colorSignal = 0;
int orientation = NORTH;
int pre_orientation = -1;


float cordX = 0.0, cordY = 0.0;
//for coordinate system

int invertCSG = 0;
//0 means checkColorSignal will return true if colorsignal > 1000
//1 means checkColorSignal will return false if colorsignal > 1000
int STATE = -1;

boolean taskComplete = false;
boolean awaitingInstructions = false;
//used to say if  the robot is on base;

Servo tilt, pan, gripper;

//wheel 1 is right wheel, wheel 2 is left

CommsLink link{"Network", "Password", IPAddress(1, 2, 3, 4)};






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

  link.init();
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
boolean checkColorSignal() {
  boolean result = false;
  if (colorSignal >= 1000)result = true; //colorsignal greater than 1000 when on base
  else result = false;

  if (invertCSG == 1) {
    if (result == true) result = false;
    if (result == false) result = true;
  }

  return result;
}
//-------------------------------------------------------------------------------------------
int go(float distance, int dir) { //go function for driving the wheels
  /*
     exit status key:
     -1: not asserted (shouldn't happen)
     0: clear run
     1: IR sensor sensed incoming object
     2: Bumper came in contact with object
     4: for hitting homebase
     5: for asserted stop signal.
  */


  if (distance < 0) { //support for negative distances
    distance = distance * -1;
    if (dir == RIGHT) dir = LEFT;
    if (dir == LEFT) dir = RIGHT;
    if (dir == FORWS) dir = BACKWS;
    if (dir == BACKWS) dir = FORWS;
  }

  //switch case for the direction
  //different directions require the wheels to be turned first before
  //moving forward.

  int tempangle = 0;
  float angle = 0.0;
  int dire = -1;
  //Serial.println("support for negative distances checked");
  switch (dir) {
    case LEFT:
      Serial.println("going left");
      angle = 90;
      dire = LEFT;

      tempangle = (int)angle;
      angle = angle / 180 * 1400;
      //conversion from given angle to specific millisecond delay for wheels
      pre_orientation = orientation;
      driveMotor1(0, LOW);
      driveMotor2(0, LOW);
      delay(200);
      if (dire == RIGHT) {
        driveMotor1(128, LOW );
        driveMotor2(128, HIGH);
        delay(angle); //driving to right angle
        orientation = orientation + tempangle;
      }
      if (dire == LEFT) {
        driveMotor1(128, HIGH );
        driveMotor2(128, LOW);
        delay(angle);
        orientation = orientation + (360 - tempangle);
      }
      if (orientation == 360) orientation = 0;
      driveMotor1(0, LOW);
      driveMotor2(0, LOW);
      delay(200);
      break;

    case RIGHT:
      Serial.println("going right");
      angle = 90;
      dire = RIGHT;
      tempangle = (int)angle;
      angle = angle / 180 * 1400;
      //conversion from given angle to specific millisecond delay for wheels
      pre_orientation = orientation;
      driveMotor1(0, LOW);
      driveMotor2(0, LOW);
      delay(200);
      if (dire == RIGHT) {
        driveMotor1(128, LOW );
        driveMotor2(128, HIGH);
        delay(angle); //driving to right angle
        orientation = orientation + tempangle;
      }
      if (dire == LEFT) {
        driveMotor1(128, HIGH );
        driveMotor2(128, LOW);
        delay(angle);
        orientation = orientation + (360 - tempangle);
      }
      if (orientation == 360) orientation = 0;
      driveMotor1(0, LOW);
      driveMotor2(0, LOW);
      delay(200);

      break;

    case BACKWS:
      //turnWheels(180, RIGHT);
      Serial.println("going back");
      angle = 180;
      dire = RIGHT;
      tempangle = (int)angle;
      angle = angle / 180 * 1400;
      //conversion from given angle to specific millisecond delay for wheels
      pre_orientation = orientation;
      driveMotor1(0, LOW);
      driveMotor2(0, LOW);
      delay(200);
      if (dire == RIGHT) {
        driveMotor1(128, LOW );
        driveMotor2(128, HIGH);
        delay(angle); //driving to right angle
        orientation = orientation + tempangle;
      }
      if (dire == LEFT) {
        driveMotor1(128, HIGH );
        driveMotor2(128, LOW);
        delay(angle);
        orientation = orientation + (360 - tempangle);
      }
      if (orientation == 360) orientation = 0;
      driveMotor1(0, LOW);
      driveMotor2(0, LOW);
      delay(200);

      break;

    case FORWS:
      Serial.println("going forw");
      pre_orientation = orientation;
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
    curdist = diff * wheelSpeed / 1000;

    //essential idea is that the time2 is updated for each while loop run
    //the diff is then checked with timing, if it equal to or greater than the timing
    //then the robot has travelled the required distance and it will break out of the loop
    //curdist is used to track robot movement so as to properly update coordinates
    //if there is an error that caused robot to stop traversing/

    checkStopSignal();
    if (stopSignal == 1) {
      //have been told to stop, need to break out of both while loops
      exitstatus = 5;
      break;
    }
    if (analogRead(IRSENSOR) >= 400) {
      //ir sensor senses something close
      exitstatus = 1;
      break;
    }
    // Serial.print(toReact());
    if (toReact() == 1) {

      //bumpers are hit
      exitstatus = 2;
      break;
    }

    colorSignal = analogRead(SENSECOLOUR); //read from ir sensor
    if (checkColorSignal()) {
      exitstatus = 4;
      break;
    }

    if (diff >= timing) {
      //drove full length
      exitstatus = 0;
      break;
    }

  }

  driveMotor1(0, LOW);
  driveMotor2(0, LOW);
  delay(200);

  //designed to properly update coordinates properly based on the orientation of the robot
  if (pre_orientation == -1) pre_orientation = NORTH;
  switch (pre_orientation) {
    case NORTH:
      if (dir == FORWS) addtoY = curdist;
      else if (dir == BACKWS) addtoY = curdist * -1;
      else if (dir == RIGHT) addtoX = curdist;
      else if (dir == LEFT) addtoX = curdist * -1;
      break;

    case EAST:
      if (dir == FORWS) addtoX = curdist;
      else if (dir == BACKWS) addtoX = curdist * -1;
      else if (dir == RIGHT) addtoY = curdist * -1;
      else if (dir == LEFT) addtoY = curdist;
      break;

    case SOUTH:
      if (dir == FORWS) addtoY = curdist * -1;
      else if (dir == BACKWS) addtoY = curdist;
      else if (dir == RIGHT) addtoX = curdist * -1;
      else if (dir == LEFT) addtoX = curdist;
      break;

    case WEST:
      if (dir == FORWS) addtoX = curdist * -1;
      else if (dir == BACKWS) addtoX = curdist;
      else if (dir == RIGHT) addtoY = curdist;
      else if (dir == LEFT) addtoY = curdist * -1;
      break;
  }

  cordX = cordX + addtoX;
  cordY = cordY + addtoY;

  switch (exitstatus) {
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

  printCoordinates();
//
//  if (exitstatus == 2) {
//    //barrier error, recall fucntion
//    float totravel = distance - curdist;
//    exit;
//    //go(totravel, FORWS);
//  }


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
void turnWheels(float angle, int dir) {
  if(angle != 0){
  //turnwheels with the specified angle in the specified direction
  //either RIGHT or LEFT
  String direct = " ";
  //Serial.println("in turn wheels");
  if (angle < 0) {
    angle = angle * -1;
    if (dir == RIGHT) dir = LEFT;
    if (dir == LEFT) dir = RIGHT;
  }
  //Serial.println("passed negative support");

  if (dir == RIGHT) direct = " degrees to the RIGHT";
  else direct == " degrees to the LEFT";

  Serial.print("Turning wheels ");
  Serial.print(dir);
  Serial.print(" + ");
  Serial.println(angle);

  int tempangle = (int)angle;
  angle = angle / 180 * 1400;
  //conversion from given angle to specific millisecond delay for wheels
  pre_orientation = orientation;

  stopWheels(200);

  if (dir == RIGHT) {
    driveMotor1(128, LOW );
    driveMotor2(128, HIGH);
    delay(angle); //driving to right angle

    orientation = orientation + tempangle;
  }

  if (dir == LEFT) {
    driveMotor1(128, HIGH );
    driveMotor2(128, LOW);
    delay(angle);

    orientation = orientation + (360 - tempangle);
  }

  if (orientation >= 360) orientation = orientation = orientation - 360;

  //orientationInfo();

  stopWheels(200);
  }
}
//-------------------------------------------------------------------------------------------
void setOrientation(int desiredOrientation) {
  int diff = desiredOrientation - orientation;
  if (diff < 0) diff = 360 + diff;
  turnWheels(diff, RIGHT);
  pre_orientation = orientation;
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
void forceSTATE(int stateIn) {
  switch (stateIn){
    case idle:
    link.force_idle();
    STATE = idle;
    break;
    case SEARCHING:
    STATE= SEARCHING;
    break;
    case FOUND:
    link.notify_found();
    STATE = FOUND;
    break;
    case DRIVING:
    link.force_driving(12.7, 8.5);
    STATE = DRIVING;
    break;
  }

}
//-------------------------------------------------------------------------------------------
void getSTATE(){
  switch (link.update()) {
    case CommsLink::IDLE:
    forceSTATE(idle);
      break;
    case CommsLink::SEARCHING:
    forceSTATE(SEARCHING);
      break;
    case CommsLink::FOUND:
    forceSTATE(FOUND);
      break;
    case CommsLink::DRIVING:
    forceSTATE(DRIVING);
      break;
  }
}
//-------------------------------------------------------------------------------------------
void checkStopSignal() {
  getSTATE();
  if (STATE ==  FOUND || STATE == DRIVING) stopSignal = 1;
  //should stop if we have been notified that it is found
  //or have been given a location to drive to.
}
//-------------------------------------------------------------------------------------------
void sendLocation() {
  //takes current coordinate of robot and sends it to server.
  link.notify_target(3, cordX + 0.25, cordY + 0.25);
  link.notify_target(2, cordX + 0.25, cordY + 0.25);
  //offsetted location for other robots.
}
//-------------------------------------------------------------------------------------------

void search() {
  //ran when searching the enclosure for the base
  int state = -1;
  if (!taskComplete && !awaitingInstructions) {
    //variable for exit status

    int ending = -1;
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

     state = go(wallLength, FORWS);
      //first begins with traveling the length of the enclosure

      if (state == 0) {
        //traversed the whole length
        //at new point of enclosure therefore change ending.
        ending = ending * -1;

        if (width < 0.60) {
          //individual sections are 60cm each therefore if wehave reached 60cm in width
          //we can no longer traverse unless we will collide with another robot

          if (ending == 1) {
            //we are at end of enclosure
            //offset to the right to search another part of our section
            turnWheels(90, RIGHT);
            go(0.15, FORWS);
            turnWheels(90, RIGHT);
          }
          else
          {
            //at the beginning therefore need to turn left to search more of the section
            turnWheels(90, LEFT);
            go(0.15, FORWS);
            turnWheels(90, LEFT);
          }

          width = width + 0.15;
          //increment the width
        }
        else
        {
          //individual sections are 60cm each therefore if wehave reached 60cm in width
          //we can no longer traverse unless we will collide with another robot

          stopWheels();
          forceSTATE(idle);
          awaitingInstructions = true;
          break;
        }
      }
      else   if (state == 2) {
        //barrier error, recall fucntion
        //call react bumpers to realign
        
      } 
      else if (state == 4) {
        //have hit homebase need to run homebase code, break out of while loop
        break;
      }
      else if (state == 5) {
        //have encountered stop signal therefore need to break out of while loop
        break;
      }

    } //end while loop
  }
  
    if(awaitingInstructions && !taskComplete){
      //needs to be told what state to be put in next 
      while(true){
        checkStopSignal();
        if(stopSignal == 1){
          awaitingInstructions = false;
          state = 5;
          break;
        }
      }
    }

    if (state== 4 && !taskComplete) {
      //need to go to base, run drive to base function
      //sendStopSignal();
      forceSTATE(FOUND);//changes our state and other robots to found
      driveOntoBase(); //drive onto the base
      sendLocation(); //send our location to other robots
      forceSTATE(idle); //change our state to idle since now on board
      taskComplete = true;
    }

    if (state == 5 && !taskComplete) {
      while (STATE != DRIVING){
        getSTATE();
      }

      //once in driving state, get coordinates and drive to location.
      rec_cordX = link.get_target_x();
      rec_cordY = link.get_target_y();
      driveToLocation(rec_cordX, rec_cordY);

      forceSTATE(idle); //let other memebers know we have reached the base
      stopWheels(); //stop the wheels of the robot
      taskComplete = true; //set taks complete to true so robot no longer conitnues to traverse

    }

    if(taskComplete && (awaitingInstructions == false)){
      Serial.println("Task Complete");
    }
}
//--------------------------------------------------------------------------------------------
void driveOntoBase() {
  int exitstatus = -1;
  int turnangle = 90;
  invertCSG = 1; //inverts the values of checkColorSignal();
  //now go function will exit if the colorsignal < 1000 (therefore will exit if robot is not
  //not on base

  while (exitstatus != 0) {
    //if exitstatus, it was a clear run
    //therefore  robot drove onto base properly

    exitstatus = go(0.25, FORWS);; //go forwards onto the base the robot length

    if (exitstatus == 4) {
      //drove off of base, needs to recalibrate and find direction of base.
      while (colorSignal < 1000) {
        turnWheels(turnangle, RIGHT);
        colorSignal = analogRead(SENSECOLOUR);
        turnangle = turnangle + 90;

        //continues turning and reading the colorsignal, thatis
        //turns 90, reads from colorsignal, if it is not a greater than a 1000 then turn again
      }

      //will exit while loop if color signal is greater than 1000
      //therefore on base and should drive up on it
    }
  }

  //got out of while loop so exit status is 0, completely on base
  //let other memebers know we have reached the base
  stopWheels();
  //stop the wheels of the robot

}
//--------------------------------------------------------------------------------------------
void driveToLocation(float rec_x, float rec_y) {
  float distx = rec_x - cordX;
  float disty = rec_y - cordY;

  //passed in coordinates are based on North orientation, therefore robot must be set
  //to north orientation before being told to drive.
  setOrientation(NORTH);

  if (distx < 0) {
    //needs to go in -x direction, therefore to the left
    distx = distx * -1;
    go(distx, LEFT);
  }
  else
  {
    //positive x direction therefore go right
    go(distx, RIGHT);
  }

  setOrientation(NORTH);
  if (disty < 0) {
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
//-------------------------------------------------------------------------------------------
String printCoordinates() {
  Serial.print("(x: ");
  Serial.print(cordX);
  Serial.print(",y: ");
  Serial.print(cordY);
  Serial.println(")");
}
//-------------------------------------------------------------------------------------------
void oriToString(int input) {
  String tobereturned = "POOJA WHAT IS THIS BEHAVIOUR???";
  switch (input) {
    case NORTH:
      tobereturned = "NORTH";
      break;

    case SOUTH:
      tobereturned = "SOUTH";
      break;

    case EAST:
      tobereturned = "EAST";
      break;

    case WEST:
      tobereturned = "WEST";
      break;

    case -1:
      tobereturned = "UNINITIALIZED";
      break;

    default:
      tobereturned = "UNKNOWNN";
      break;
  }
  Serial.println(tobereturned);
}
//-------------------------------------------------------------------------------------------
void orientationInfo() {
  Serial.print("Previous Orientation is ");
  oriToString(pre_orientation);
  Serial.print("Orientation is ");
  oriToString(orientation);
}
//---------------------------------------------FUNCTION END--------------------------------------
int poop = -1;

void loop() {
  go(0.25, FORWS);
  delay(2000);
  stopWheels();
  delay(2000);
  go(0.25, RIGHT);
  delay(5000);

  //  int r_value = digitalRead(RIGHTBUMPER);
  //  int l_value = digitalRead(LEFTBUMPER);
  //
  //  Serial.println(toReact());
}
