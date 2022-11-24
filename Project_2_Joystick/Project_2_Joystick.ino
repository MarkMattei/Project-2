#include <PID_v1.h>
#define enA 9
#define in1 5
#define in2 13
#define enB 10
#define in3 4
#define in4 7

double Setpoint, Input, Output;
int joyx,joyy;
double Kp = 50, Ki = 0.01, Kd = 0.0001;


void setup() {
pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
Serial.begin(9600);

}

void loop() {

  joyx = analogRead(A0);
  joyy = analogRead(A1);

//Deadzone for joystick
  if((joyx>450) && (joyx<550) && (joyy>450)&& (joyy<550)){
    analogWrite(enA,0);
    analogWrite(enB,0);
  }
//Forward Movement and Speed control
 if((joyy<450)){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  int speed_ = map(joyy,450,0,30,255);
  analogWrite(enA,speed_);
  analogWrite(enB,speed_);
  Serial.println(joyy);
 }
//Backward Movement and Speed control
 if((joyy>550)){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  int speed_ = map(joyy,550,1023,30,255);
  analogWrite(enA,speed_);
  analogWrite(enB,speed_);
  Serial.println(joyy);
  
 }
//Turning Movement right
 if(joyx>550){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  int speed_ = map(joyx,550,1023,30,255);
  analogWrite(enA,speed_);
  analogWrite(enB,speed_);
  Serial.println(joyx);
 }
  //Turning Movement left
 if(joyx<450){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  int speed_ = map(joyx,450,0,30,255);
  analogWrite(enA,speed_);
  analogWrite(enB,speed_);
  Serial.println(joyx);
 }
 
  







}
