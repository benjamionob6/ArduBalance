/*****************************
Main Ardubalance Source

Ben O'Brien - 12-Nov-2012
*****************************/
#include "Ardudefine.h"
#include "IRSensor.h"
#include "MotorDriver.h"

IRSensor ForwardSensor(A0);
IRSensor BackwardSensor(A1);

//MotorDriver(int pwm_a, int a1, int a2, int pwm_b, int b1, int b2)
MotorDriver Motor(10,3,2,11,4,5);

void setup(){
  Serial.begin(115200);
  
  //Enable FAST ACD
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
}

void loop(){
  //ForwardSensor.takeReading();
  //Serial.println(ForwardSensor.getLastReading());
  //for ( int i = -255; i < 255; i++){
  //  Motor.SetMotors(i,i);
  //  delay(50);
  //}
}
