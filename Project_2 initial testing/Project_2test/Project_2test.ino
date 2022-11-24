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
#include <PID_v1.h>
double Setpoint, Input, Output,Input2, Output2,Inputc,Outputc,Setpointc,Kpc=0.2,Kic=0,Kdc=0;
double kp = 2, ki = 5, kd = 1;
double Kp = 1, Ki = 1, Kd = 0;
int counter = 0,counter2 = 0;
int currentStateCLK,currentStateCLK2;
int lastStateCLK,lastStateCLK2;
String currentDir="";
const float pi = 3.14;
const float R = 3.25;
const int N = 20;
float distance = 0,distance2 = 0;
int finaldistance = 120;
int motoroutput;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint,output2;
double cumError, rateError;
unsigned long previousMillis = 0;
const long interval = 1000;
double refangle;



PID PID_control1(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID PID_control2(&Input2, &Output2, &Setpoint, Kp, Ki, Kd, DIRECT);


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
PID_control1.SetMode(AUTOMATIC);
PID_control1.SetTunings(Kp, Ki, Kd);
PID_control2.SetMode(AUTOMATIC);
PID_control2.SetTunings(Kp, Ki, Kd);
PID_controlcompass.SetMode(AUTOMATIC);
PID_controlcompass.SetTunings(Kpc, Kic, Kdc);
attachInterrupt(digitalPinToInterrupt(2), encoder, RISING);  
attachInterrupt(digitalPinToInterrupt(3), encoder2, RISING);  
Turn1();
delay(1000);
Turn2();
}
void loop() {

//----------------------TASK 1----------------------//

}
void Turn2(){
counter=0;
counter2=0;
distance=0;
distance2=0;

while(distance>-19 && distance2<15){
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
void Turn1(){
counter=0;
counter2=0;
distance=0;
distance2=0;

while(distance<15 && distance>-11){
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


int Task_1p1(){

if(distance<finaldistance && distance2<finaldistance){
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
Setpoint = finaldistance;
Input = distance;
Input2 = distance2;
PID_control1.Compute();
PID_control2.Compute();
analogWrite(enA,Output2);
analogWrite(enB,Output);
Serial.println("----");
Serial.println(distance);
Serial.println(distance2);
Serial.println("----");

}




}











double computePID(double Input){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = Setpoint - Input;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
