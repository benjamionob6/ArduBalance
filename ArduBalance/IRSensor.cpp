#include "IRSensor.h"

IRSensor::IRSensor(int fwd_inport, int bwd_inport) {
  //Assign Port
  fwd_port = fwd_inport;
  bwd_port = bwd_inport;
  pinMode(fwd_port,INPUT);
  pinMode(bwd_port,INPUT);
  
  
  //Zero out readings
  for ( int i = 0; i < 4; i++){
   fwd_readings[i] = (short)0; 
   bwd_readings[i] = (short)0; 
  }
}

void IRSensor::setBalance(){
  short fwd_average, bwd_average = 0;
  for ( int i = 0; i < 8; i++){
   takeReading();
   fwd_average += getLastReading(0);
   bwd_average += getLastReading(1);
  } 
  fwd_average /= 8;
  bwd_average /= 8;
  fwd_balanced_reading = fwd_average;
  bwd_balanced_reading = bwd_average;
}

short IRSensor::getBalanceError(int which){
 return (getLastReading(which) - (which == 0 ? fwd_balanced_reading : bwd_balanced_reading)) ; 
}

IRSensor::~IRSensor(){}

void IRSensor::takeReading(){
 
  int new_fwd = analogRead(fwd_port);
  int new_bwd = analogRead(bwd_port);
  int fwd_lin = 0;
  int bwd_lin = 0;
  
  int maxSize = sizeof(linearizemm) / sizeof(linearizemm[0]);
  if ( new_fwd < maxSize ){
    fwd_lin = linearizemm[new_fwd];    
  }
  if ( new_bwd < maxSize ){
    bwd_lin = linearizemm[new_bwd]; 
  }

  if ( fwd_lin + bwd_lin < 270 && fwd_lin + bwd_lin > 220 ){
     fwd_last = (fwd_lin * 0.75) + ((250-bwd_lin) * 0.25);
     bwd_last = (bwd_lin * 0.75) + ((250-fwd_lin) * 0.25);
  }
}


short IRSensor::getLastReading(int which){
 return ( which == 0 ? fwd_last : bwd_last );
}
