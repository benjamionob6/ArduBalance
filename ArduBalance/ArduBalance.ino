/*****************************
Main Ardubalance Source

Ben O'Brien - 12-Nov-2012
*****************************/
#include <Wire.h>
#include "Ardudefine.h"
#include "IRSensor.h"
#include "MotorDriver.h"

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
double MotorOutput;
IRSensor Sensor(A2,A3);
MotorDriver Motor(3,12,9,11,13,8);
unsigned long lastMicros;
unsigned long lastBiasFix = 0;
double fErr, bErr = 0;
  
void setup(){
  Serial.begin(19200);
  Wire.begin();
  setupL3G4200D(500);
  currentAngle = 0;
    
  //Enable FAST ACD  
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
  
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);
 
  delay(2000);
  Sensor.setBalance();
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

void loop(){
  
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
  if ( abs(fErr) <= 2 && abs(bErr) <= 2 && (micros() - lastBiasFix) > 100000){
    currentAngle = 0;
    lastBiasFix = micros();
  }
  
  unsigned long currentMicros = micros();
  lastMicros = currentMicros;

  //Allow 2 seconds for setting down the robot before we start balancing
  if ( currentMicros < 4000000 )
    return;

  MotorOutput = 18.5 * currentAngle + 1.24 * gyroXrate + 0.6885 * gyroAccel;
  Motor.SetMotors(MotorOutput,MotorOutput);
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
