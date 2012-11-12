#include "IRSensor.h"

IRSensor::IRSensor(int port){
  //Assign Port
  port = port;
  pinMode(port,INPUT);
  
  //Zero out readings
  for ( int i = 0; i < 8; i++){
   readings[i] = (short)0; 
  }
  
  //Initialize ring buffer indexes
  buffstop = -1;
  buffstart = 0;  
  num_readings = 0;
}

IRSensor::~IRSensor(){}

void IRSensor::takeReading(){
 short val = analogRead(port);
 if ( num_readings < 8 ){
   readings[num_readings] = linearizemm[val];
   num_readings++;
   buffstop++;   
 } else {
  readings[buffstart] = linearizemm[val];
  buffstop = buffstart;
  buffstart = (buffstart+1) % 8;
 }
}

short IRSensor::getLastReading(){
 return readings[buffstop]; 
}
