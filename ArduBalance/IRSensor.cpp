#include "IRSensor.h"

IRSensor::IRSensor(int fwd_inport, int bwd_inport) : Filter1(), Filter2() {
  //Assign Port
  fwd_port = fwd_inport;
  bwd_port = bwd_inport;
  pinMode(fwd_port,INPUT);
  pinMode(bwd_port,INPUT);
  
  Filter1.begin();
  Filter1.setFilter('c');
  Filter1.setOrder(1);
  
  Filter2.begin();
  Filter2.setFilter('c');
  Filter2.setOrder(1);
  
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

//void quickSort(short *arr, int elements) {
//  short  piv, beg[10], end[10], i=0, L, R, swap ;
//
//  beg[0]=0; end[0]=elements;
//  while (i>=0) {
//    L=beg[i]; R=end[i]-1;
//    if (L<R) {
//      piv=arr[L];
//      while (L<R) {
//        while (arr[R]>=piv && L<R) R--; if (L<R) arr[L++]=arr[R];
//        while (arr[L]<=piv && L<R) L++; if (L<R) arr[R--]=arr[L]; 
//      }
//      arr[L]=piv; beg[i+1]=L+1; end[i+1]=end[i]; end[i++]=L;
//      if (end[i]-beg[i]>end[i-1]-beg[i-1]) {
//        swap=beg[i]; beg[i]=beg[i-1]; beg[i-1]=swap;
//        swap=end[i]; end[i]=end[i-1]; end[i-1]=swap; 
//      }
//   } else {
//      i--; 
//   }
//  }
//}

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
     fwd_last = (fwd_lin * 0.75) + ((250-bwd_lin) * 0.25); //Filter1.run(fwd_lin);
     bwd_last = (bwd_lin * 0.75) + ((250-fwd_lin) * 0.25);
  }
}


short IRSensor::getLastReading(int which){
 return ( which == 0 ? fwd_last : bwd_last );
}
