#define LEDPIN 7
#define BUTTON 11



int val = 0;
int delayValue;
bool isPressed = false;
//int pressed = 0;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
pinMode(LEDPIN, OUTPUT);
pinMode(BUTTON, INPUT);
pinMode(A0, INPUT);


}

void loop() {
 
  val = analogRead(A0);
  float voltage = val * (5.0/1023.0);

  if(voltage < 2.5)
    delayValue = 500;
  else if(voltage > 2.5)
    delayValue = 1500;
  
  digitalWrite(LEDPIN,HIGH);
  delay(delayValue);
  digitalWrite(LEDPIN, LOW);
    delay(delayValue);
  while(!isPressed)
    break;
  if(val == LOW)
    isPressed = true;
  if(val == HIGH && isPressed) {
  digitalWrite(LEDPIN,HIGH);
delay(1000);
     digitalWrite(LEDPIN,LOW);
delay(1000);
  
//}


}
