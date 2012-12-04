#include <Wire.h>
#include <Encoder.h>

// Gyro
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D
int gyroX;
double gyroXrate;
unsigned long lastGyroTime;
double currentAngle;

double OscillationTarget = 10.0;
unsigned long lastOscTarget = 0;
unsigned long oscArray[20];
int oscindex = 0;

/*****************************
Main Ardubalance Source

Ben O'Brien - 12-Nov-2012
*****************************/
#include "Ardudefine.h"
#include "IRSensor.h"
#include "MotorDriver.h"
#include "SignalFilter.h"

double SensorInput, MotorOutput, SetPoint, pre_error;

IRSensor Sensor(A2,A3);
Encoder myEncoder(2,3);

//PID myPID(&SensorInput,&MotorOutput,&SetPoint,3,1,2,REVERSE);
SignalFilter Filter;
int fwd_lastError, bwd_lastError;
//MotorDriver(int pwm_a, int a1, int a2, int pwm_b, int b1, int b2)
MotorDriver Motor(3,12,9,11,13,8);
double lastMotor;

long lastEncoder;
unsigned long lastEncoderTimeStamp;
unsigned long lastMicros;
unsigned long lastBiasFix = 0;
double fErr, bErr, dErr = 0;
int barLength;
double diff;
double angle_off;
double fwd_lastlast;

double targetAngle;
int eepromAdd;

double P,I,D;
float kP,kI,kD;
  
void setup(){
  
  Serial.begin(19200);
  Wire.begin();
  setupL3G4200D(500);
  currentAngle = 0;
  SetPoint = 0;
  SensorInput = 0;
  //myPID.SetOutputLimits(-100,100);
  //myPID.SetSampleTime(40);
  //myPID.SetMode(AUTOMATIC);
  fwd_lastlast = 0;
  fwd_lastError = 0;
  lastMotor = 0;
  P = I = D = 0;
  pre_error = 0;
 
  
  //Enable FAST ACD  
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
  
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);
  myEncoder.write(0);

  Filter.begin();
  Filter.setFilter('c');
  Filter.setOrder(1);

  barLength = 200.0;
  diff = 0.0;
  angle_off = 0.0;
  
  
  delay(2000);
  Sensor.setBalance();
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  delay(2000);
}


void setmotor(int o){
 if ( o < -225 ){  o = -255; }
 if ( o > 255 ) { o = 255; }
 o = map(o,-255,255,-90,90);
 
 //motor1.write(90-o);
 //motor2.write(90-o); 
}

void loop(){
  long currentEncoder = 0;
  long currentEncoderTimeStamp = 0;
  double xdot = 0;
  
  //Our sensors don't update any faster than 10ms
  
  if (micros() > lastMicros + 10000){
  
  //Read from the IR Sensors  
  Sensor.takeReading();
  
  //Save off the previous gyro readings
  double lastGyro = gyroXrate;  
  unsigned long dt = micros() - lastGyroTime;

  //calculate the angle change degrees 
  double angleChange = (lastGyro) * (dt / 1000000.0); 
  
  //Get new values
  getGyroValues();
  
  //Take the difference in the last two gyro readings to get the angular acceleration...
  double gyroAccel = (lastGyro - gyroXrate);
  
  // Update the current angle based on calculations
  currentAngle += angleChange;
   
  
  //Check for significantly small error from the IRSensors and reset the angle
  //to 0 no more than twice a second
  fErr = Sensor.getBalanceError(0);
  bErr = Sensor.getBalanceError(1);
  if ( abs(fErr) <= 2 && abs(bErr) <= 2 && (micros() - lastBiasFix) > 500000){
    currentAngle = 0;
    lastBiasFix = micros();
  }

  
//  currentEncoder = (myEncoder.read());
//  if ( currentEncoder != lastEncoder ){
//    currentEncoderTimeStamp = micros();
//    xdot = (6594)/((double)currentEncoderTimeStamp - lastEncoderTimeStamp);
//   
//    lastEncoderTimeStamp = currentEncoderTimeStamp;
//    lastEncoder = currentEncoder; 
//  }

//  int NewMotorOutput = 0;
//  if ( fErr != fwd_lastError  && bErr != bwd_lastError ){   
//    if ( (fErr > 0 && bErr > 0) || (fErr < 0 && bErr < 0) ){
//      //MotorOutput = 0;
//    } else {
//      if ( max(abs(fErr),abs(bErr)) < 20 )
//        NewMotorOutput = ( abs(fErr) > abs(bErr) ? fErr * 4.255 : bErr * -4.255 );
//      else
//        NewMotorOutput = max(min( abs(fErr) > abs(bErr) ? fErr * 5.255 : bErr * -5.255, 100 ),-100);
//
//      MotorOutput = Filter.run(NewMotorOutput);
//


      unsigned long currentMicros = micros();
      lastMicros = currentMicros;

      //c (z-1) +2.562e(z) -2.17e(z-1) +0.2485e(z-2)
//      MotorOutput = lastMotor + 2.562*fErr - 2.17* fwd_lastError + 0.2485 * fwd_lastlast;
//      MotorOutput /= 3;
    
      //PI 
      //MotorOutput = lastMotor + 1.866*fErr - 1.484*fwd_lastError;

      //PID
      //MotorOutput = lastMotor + 7.252*fErr - 5.315*fwd_lastError + 0.3866 * fwd_lastlast;

//      MotorOutput = lastMotor - 0.4091*fErr + 0.3854*fwd_lastError;

//      P = error * kP;
//      if ( error > 2 ){
//        I = I + ( P * dt ) * kI / 1000000;
//      } else { I = 0; }
//      
//      D = kD * 1000 * (error - pre_error) / dt;
//      if ( I > 90 ){ I = 90;}
//      if ( I < -90 ) { I = -90 ;}
//      
//      double pid = P + I + D;
//      pre_error = error;
//      MotorOutput = pid;
      
//      MotorOutput = fErr * 4;
//      Serial.print(micros());
//      Serial.print(",");
//      //Serial.print(Sensor.getLastReading(0));
//      //Serial.print(",");
//      Serial.print(/*Sensor.getLastReading(1)*/ fErr);
//      Serial.print(",");
//      Serial.println(MotorOutput);
//    }  
    //MotorOutput = fErr * 3;
    
//    
//    Serial.print(micros());
//    Serial.print(",");
//    Serial.print(fErr);
//    Serial.print(",");
//    Serial.println(MotorOutput); 0.54
    
    
//    MotorOutput = 13 * currentAngle - 0.042 * gyroXrate - 0.45 * gyroAccel;
//    if ( abs(MotorOutput) > 15 ){
//      Motor.SetMotors(MotorOutput,MotorOutput);
//    } else {
//      Motor.SetMotors(1,1); 
//    }
      MotorOutput = 18.5 * currentAngle + 1.24 * gyroXrate + 0.6885 * gyroAccel;
      
      //MotorOutput = 20 * currentAngle + 0.64 * gyroXrate + 0.70 * gyroAccel;
     
      Motor.SetMotors(MotorOutput,MotorOutput);
      //setmotor(MotorOutput);      

    lastMotor = MotorOutput;
//    Serial.print(currentAngle);
//    Serial.print("\t");
//    Serial.println(MotorOutput);
//  }

  fwd_lastlast = fwd_lastError;
  fwd_lastError = fErr;
  bwd_lastError = bErr;
  
 } 
  
}














int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001100);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte
    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

void getGyroValues(){
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  
  
  
  short tempGyro = -1*((xMSB << 8) | xLSB);
  if (tempGyro != gyroX){
    gyroX = tempGyro; 
    gyroXrate = gyroX  * 17.50 * 0.001;
    lastGyroTime = micros();
  }
}
