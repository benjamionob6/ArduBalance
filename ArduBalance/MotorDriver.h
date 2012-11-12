#include "Arduino.h"

class MotorDriver{
  int port_pwm_A, port_pwm_B, port_A1, port_A2, port_B1, port_B2;
  public:
  MotorDriver(int,int,int,int,int,int);
  void SetMotors(int,int);
  ~MotorDriver();
};

MotorDriver::MotorDriver(int pwm_a, int a1, int a2, int pwm_b, int b1, int b2){
  port_pwm_A = pwm_a;
  port_pwm_B = pwm_b;
  port_A1 = a1;
  port_A2 = a2;
  port_B1 = b1;
  port_B2 = b2;
 
  pinMode(port_pwm_A,OUTPUT); 
  pinMode(port_pwm_B,OUTPUT); 
  pinMode(port_A1,OUTPUT); 
  pinMode(port_A2,OUTPUT); 
  pinMode(port_B1,OUTPUT); 
  pinMode(port_B2,OUTPUT);
 
  //Break mode with 0% duty
  digitalWrite(port_A1,HIGH);
  digitalWrite(port_A2,HIGH);
  analogWrite(port_pwm_A,0);
 
  digitalWrite(port_B1,HIGH);
  digitalWrite(port_B2,HIGH);
  analogWrite(port_pwm_B,0);
}

MotorDriver::~MotorDriver(){}

void MotorDriver::SetMotors(int pwm_A, int pwm_B){
  digitalWrite(port_A1, pwm_A >= 0 ? HIGH : LOW);
  digitalWrite(port_A2, pwm_A >= 0 ? LOW : HIGH);
  analogWrite(port_pwm_A, min(abs(pwm_A),255));
  
  digitalWrite(port_B1, pwm_B < 0 ? HIGH : LOW);
  digitalWrite(port_B2, pwm_B < 0 ? LOW : HIGH);
  analogWrite(port_pwm_B, min(abs(pwm_B),255));
}
