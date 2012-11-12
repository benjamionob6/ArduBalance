/*****************************
Main Ardubalance Source

Ben O'Brien - 12-Nov-2012
*****************************/
#include "Ardudefine.h"
#include "IRSensor.h"

IRSensor ForwardSensor(A0);
IRSensor BackwardSensor(A1);

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
}
