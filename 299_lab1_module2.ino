#define M1 7
#define E1 6
#define E2 5
#define M2 4
#define BUTTON 2 //encoder attached to "interupt #0" or pin 2. Pin 2 and 3 are reserved for interupt 0 and 1.
#define P3 10
#define P4 11
#define pi 3.14


int val3 = LOW; //state of wheel initially low
int preval3 = HIGH, preval4;
int count = 0;
int wSpd; //variable to hold speded value for wheels
int wRot; //variable to hold rotation value for wheels


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //initializer
  
  pinMode(BUTTON, INPUT); //encoder input pin
  pinMode(E1, OUTPUT); //speed pin for motor 1 
  pinMode(E2, OUTPUT); // speed pin for motor 2
  pinMode(M1, OUTPUT); // direction pin for motor 1
  pinMode(M2, OUTPUT); //direction pin for motor 2
  pinMode(P3, INPUT); //wheel encoder for motor 2
  pinMode(P4, INPUT);
  attachInterrupt(0, forInter, CHANGE);  //when any change occurs on interupt 0 (aka pin 2), forInter() is called
  interrupts(); //this function enables interupts
}

void forInter()
{
  count++; //increment the count which represents the number of state transitions from high to low
  //MATH FOR SPEED
  int veloc = ((6*pi)/255) * wSpd;
  //String strvel = (String) "Speed is "+ veloc + " rad/s";
  Serial.println(count);
  Serial.print(veloc);
  Serial.print(" rad/s");
}

int countEncoder2()
{
  val3 = digitalRead(P3);
  if(val3 != preval3) // change (1 rotation =20 counts).
  {
    count++;
    Serial.println(count);
    preval3 = val3;
  }

  
}

void driveMotors(){
  driveMotor1(wSpd,wRot);
  driveMotor2(wSpd, wRot);
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




void loop() 
{
  // put your main code here, to run repeatedly:

while(count <=60) //1 rotation = approx 20 transitions, therefore 3 is 60
//3 rotations forward
{
wSpd = 128;
wRot = HIGH; //setting states for wheels
driveMotors(); //calling drive, no input needed as wheel states 
                //are global variables
}

wSpd =0;
driveMotors(); //stopping for one second
delay(1000);

while(count > 60 && count <= 110)
//2 rotations backward
{
wSpd = 128;
wRot = LOW;
driveMotors();
}

wSpd =0;
driveMotors(); //stop for one second.
delay(1000);










/*
  if(val == HIGH)
  {
    Serial.println("Button Pressed");
    preval=val;
  }
 
  
  if(val == LOW && preval == HIGH)
  {
   Serial.println("IN if");
  digitalWrite(M2, HIGH);
analogWrite(E2, 128);
digitalWrite(M1, LOW);
analogWrite(E1, 128);
delay(6000);
digitalWrite(M2, LOW);
analogWrite(E2, 128);
digitalWrite(M1, HIGH);
analogWrite(E1, 128);
delay(6000);

  }

*/
}
