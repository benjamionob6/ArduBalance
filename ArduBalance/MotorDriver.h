#include "Arduino.h"

class MotorDriver{
  int port_pwm_A, port_pwm_B, port_A_Dir, port_A_Break, port_B_Dir, port_B_Break;
  public:
  MotorDriver(int,int,int,int,int,int);
  void SetMotors(int,int);
  ~MotorDriver();
};

MotorDriver::MotorDriver(int pwm_a, int a1, int a2, int pwm_b, int b1, int b2){
  port_pwm_A = pwm_a;
  port_pwm_B = pwm_b;
  port_A_Dir = a1;
  port_A_Break = a2;
  port_B_Dir = b1;
  port_B_Break = b2;
 
  pinMode(port_pwm_A,OUTPUT); 
  pinMode(port_pwm_B,INPUT); 
  pinMode(port_A_Dir,OUTPUT); 
  pinMode(port_A_Break,OUTPUT); 
  pinMode(port_B_Dir,INPUT); 
  pinMode(port_B_Break,INPUT);
 
  //Break mode with 0% duty
  digitalWrite(port_A_Dir,HIGH);
  digitalWrite(port_A_Break,HIGH);
  analogWrite(port_pwm_A,0);
 
  digitalWrite(port_B_Dir,LOW);
  digitalWrite(port_B_Break,HIGH);
  digitalWrite(port_pwm_B,LOW);//analogWrite(port_pwm_B,0);
}

MotorDriver::~MotorDriver(){}

void MotorDriver::SetMotors(int pwm_A, int pwm_B){
  
  if ( abs(pwm_A) <= 0 ){
    digitalWrite(port_A_Dir,HIGH);
    digitalWrite(port_A_Break,HIGH);
    analogWrite(port_pwm_A,0);
  } else {
    digitalWrite(port_A_Dir,LOW);
    digitalWrite(port_A_Break,LOW);
    
    digitalWrite(port_A_Dir, pwm_A <= 0 ? LOW : HIGH);
    digitalWrite(port_B_Dir, pwm_A >= 0 ? HIGH : LOW);
    
    analogWrite(port_pwm_A, min(abs(pwm_A),255)); 
 }
}
