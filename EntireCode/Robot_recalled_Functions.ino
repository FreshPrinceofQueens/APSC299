
void forwardsBy(float distance) { //possibly recall
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
      reactBumpers();
      exitstatus = 2;
      break;
    }

    if (analogRead(IRSENSOR) >= 400) {
      IR_Avoid();
      exitstatus = 2;
      break;
    }

    if (diff >= timing) {
      exitstatus = 1;
      break;
    }

    Serial.println(diff * 0.15);
  }

  stopWheels();

  if (exitstatus == 2) {
    //barrier error, recall fucntion
    dist = (diff * wheelSpeed * 1000);
    forwardsBy(distance - dist);
  }

}
//--------------------------------------------------------------------------------------------
forwards(timing){
 float time1 = 0.0,time2=0.0,diff=0.0;
 float curaccel= 0.0;
 float accel1 = 0.0, accel2 = 0.0;
 float vo=0.0, vf=0.0;
 float dist = 0.0, curdist = 0.0;
 float IRValue = 0;
 int exitstatus = 0;

  forwards();
 while(true){
  time1 =millis();
  accel1 = getIMUAccel();
  time2= millis();
  accel2 = getIMUAccel();
  diff = time2-time1;
  curaccel = (accel1+accel2) * 0.5;
  curdist = ((vo*(diff/1000))+(0.5 * curaccel * pow(diff/1000,2)))*DISTK;
  if(curdist < 0) curdist = curdist * -1;
  dist = dist +curdist;
  vf = vo + (curaccel * (diff/1000));
  vo = vf;
  if(toReact() == 1){
    reactBumpers();
    exitstatus=2;
    break;
  }
  if(analogRead(IRSENSOR) >= 400) {
    IR_Avoid();
    exitstatus = 2;
    break;
  }
  Serial.println(dist);
  if(dist >= distance) {
    Serial.println("Got here");
    exitstatus = 1;
    break;
  }

 }
 stopWheels();

 if(exitstatus == 2){
  //barrier error, recall fucntion
  forwardsBy(distance - dist);
 }

}
//-------------------------------------------------------------------------------------------
void backwardsBy(float distance) //possibly recall
{
  turnWheels(180, RIGHT);
  forwardsBy(distance);
}
//-------------------------------------------------------------------------------------------
void turnWheels2(float angle, int dir) //possibly recall
{
  stopWheels(500);
  float intdegrees = 0.0;
  float gyrox1 = 0.0, gyrox2 = 0.0, gyroy1 = 0.0, gyroy2 = 0.0, gyroz = 0.0;
  float curdegrees = 0.0;
  float time1 = 0.0, time2 = 0.0, diff = 0.0;
  int countie = 0;
  Serial.println("Variables initialized");
  if (dir == RIGHT) {
    driveMotor1(128, LOW );
    driveMotor2(128, HIGH);
    Serial.println("Turning wheels");
  }
  if (dir == LEFT) {
    driveMotor1(128, HIGH );
    driveMotor2(128, LOW);
  }

  while (intdegrees <= angle)
  {
    Serial.println("in");
    time1 = millis();
    // Serial.println("init time");
    gyrox1 = getIMUGyroZ();
    //Serial.println("initgyro");
    //gyroy1 = getIMUGyroY();
    time2 = millis();
    //Serial.println("fin time");
    gyrox2 = getIMUGyroZ();
    //Serial.println("fin gyro");
    //gyroy2 = getIMUGyroY();
    //float sumgyro1 = pow(  pow(gyrox1,2) + pow(gyroy1,2),0.5);
    //float sumgyro2 = pow(  pow(gyrox2,2) + pow(gyroy2,2),0.5);
    diff = time1 - time2;
    curdegrees = ((gyrox1 + gyrox2) * 0.5) * (diff / 1000) * IMUK;
    //Serial.print("curdegrees");
    //Serial.println(curdegrees);
    if (curdegrees < 0) curdegrees = curdegrees * -1;
    intdegrees = intdegrees + curdegrees;
    //Serial.print("fulldegrees:");
    Serial.println(intdegrees);

    // String info1 = ("Gryox1: "+(String)gyrox1+"\n");
    // String info2 = ("Gryox2: "+(String)gyrox2+"\n");
    // String info3 = ("Curdegrees :"+(String)curdegrees+"\t totaldegrees: "+(String)intdegrees+"\n");
    //
    // Serial.println(info1+info2+info3);
    // Serial.println("dafuq");
  }
  stopWheels(200);


}
