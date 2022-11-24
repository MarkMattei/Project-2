#include <PID_v1.h>
#define enA 9
#define in1 5
#define in2 13
#define enB 10
#define in3 4
#define in4 7
#define CLK 2
#define DT 6
#define CLK2 3
#define DT2 8
int counter = 0,counter2 = 0;
int currentStateCLK,currentStateCLK2;
int lastStateCLK,lastStateCLK2;
String currentDir="";
const float pi = 3.14;
const float R = 3.25;
const int N = 20;
float distance = 0,distance2 = 0;
int finaldistance = 100;
int motoroutput;

void setup() {
pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
pinMode(CLK,INPUT);
pinMode(DT,INPUT);
pinMode(CLK2,INPUT);
pinMode(DT2,INPUT);
Serial.begin(115200);

attachInterrupt(digitalPinToInterrupt(2), encoder, RISING);  
attachInterrupt(digitalPinToInterrupt(3), encoder2, RISING);  
Point1();
delay(1000);
Turn1();
delay(1000);
Point2();
delay(1000);
Turn2();
delay(1000);
Point3();

}
void loop() {



}

void Point1(){
while(distance<finaldistance && distance2<finaldistance){
if(distance>distance2+2){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
if(distance2>distance+2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
else{
  encoder();
  encoder2();
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
}

encoder();
encoder2();
analogWrite(enA,0);
analogWrite(enB,0);
distance=0;
distance2=0;
Serial.println("pt1reached");
}

void Turn1(){
counter=0;
counter2=0;
distance=0;
distance2=0;

while(distance<11 && distance>-7){
  encoder();
  encoder2();
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
  Serial.println(distance);
  Serial.println(distance2);
}
encoder();
encoder2();
analogWrite(enA,0);
analogWrite(enB,0);
distance=0;
distance2=0;
Serial.println("finished turning");
}

void Point2(){
counter=0;
counter2=0;
distance=0;
distance2=0;
while(distance<finaldistance && distance2<finaldistance){
if(distance>distance2+2){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
if(distance2>distance+2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
else{
  encoder();
  encoder2();
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
}

encoder();
encoder2();
analogWrite(enA,0);
analogWrite(enB,0);
distance=0;
distance2=0;
Serial.println("pt2reached");
}
void Turn2(){
counter=0;
counter2=0;
distance=0;
distance2=0;

while(distance>-17 && distance2<13){
  encoder();
  encoder2();
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA,100);
  analogWrite(enB,100);
  Serial.println(distance);
  Serial.println(distance2);
}
encoder();
encoder2();
analogWrite(enA,0);
analogWrite(enB,0);
distance=0;
distance2=0;
Serial.println("finished turning");
}


void Point3(){
counter=0;
counter2=0;
distance=0;
distance2=0;
while(distance<finaldistance && distance2<finaldistance){
if(distance>distance2+2){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
if(distance2>distance+2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
else{
  encoder();
  encoder2();
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA,100);
  analogWrite(enB,100);
}
}

encoder();
encoder2();
analogWrite(enA,0);
analogWrite(enB,0);
distance=0;
distance2=0;
Serial.println("pt3reached");
}


int encoder(){
// Read the current state of CLK
currentStateCLK = digitalRead(CLK);
// If last and current state of CLK are different, then pulse occurred
// React to only 1 state change to avoid double count
if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
// If the DT state is different than the CLK state then
// the encoder is rotating CCW so decrement
if (digitalRead(DT) != currentStateCLK) {
counter --;
} else {
// Encoder is rotating CW so increment
counter ++;
}
}
// Remember last CLK state
lastStateCLK = currentStateCLK;
distance = ((2*pi*R)/N) * counter ;
delay(1);
}




int encoder2(){
// Read the current state of CLK
currentStateCLK2 = digitalRead(CLK2);
// If last and current state of CLK are different, then pulse occurred
// React to only 1 state change to avoid double count
if (currentStateCLK2 != lastStateCLK2  && currentStateCLK2 == 1){
// If the DT state is different than the CLK state then
// the encoder is rotating CCW so decrement
if (digitalRead(DT2) != currentStateCLK2) {
counter2 ++;
} else {
// Encoder is rotating CW so increment
counter2 --;
}
}
// Remember last CLK state
lastStateCLK2 = currentStateCLK2;
distance2 = ((2*pi*R)/N) * counter2 ;
delay(1);
}
