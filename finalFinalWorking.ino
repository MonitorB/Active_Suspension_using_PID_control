
#include <Wire.h>
#include <Servo.h>
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
///////////////////////////////////////////////////calibration
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
/////////////////////////////////////////////////////////////
Servo servoFR;
Servo servoFL;
Servo servoRR;
Servo servoRL;
float initialServoFR=130;//INITIAL ANGLES
float initialServoFL=45;
float initialServoRR=70;
float initialServoRL=90;

float rollTarget=0;
float rollError=0;//e1
float rollErrorOld;//e2
float rollErrorChange;//e2-e1=de
float rollErrorSlope=0;//de/dt
float rollErrorArea=0;

//Correction = K1Error + K2Slope + K3Area
//Correction = KpError + Ki(area+error*time) +Kd(de/dt)  
 
float pitchTarget=0;
float pitchError=0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope=0;
float pitchErrorArea=0;

float pitchCorrectionDiff;
float pitchCorrectionSum;
float rollCorrectionDiff;
float rollCorrectionSum;

float Kp=0.250;
float Ki=0.001;
float Kd=50;

int milliOld;//t2
int milliNew;//t1 
int dt;//t2 - t1
//////////////////////////////////////////////////////////////

int esc=7;// # define esc 9;
Servo bldc;
//////////////////////////////////////////////////////////////
void gyro_signals(void) {
  Wire.beginTransmission(0x68);//LPF activation
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);//For accelerometer o/p same as in gyro where us et LDB 65.5
  Wire.write(0x1C);//Register 1C used to configure accelerometer output
  Wire.write(0x10);//8g or 4096 LSB/g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);//For accelerometer measurements
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);//6 bits
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);//gyro configuartion
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();  
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;//gyro config end
  AccX=(float)AccXLSB/4096;//convertinf LSB to g //configure here by adding 0.01 or subtracting all three DIRECTION TO BE 1G
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);//radians to degree
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void setup() {
  /////////////////////////////////////////////////////////////////////////////////////
  bldc.attach(esc,  1000, 2000);//bldc attached to esc with min and max pulse 1000 and 200
  bldc.write(180);//intial position set to 180 degrees full throttle
  delay(5000);
  bldc.write(0);//then set to 0 degrees no throttle
  delay(2000);
  bldc.write(25);
  /////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  /////////////////////////////////////////////////
  servoFR.attach(10);
  servoFL.attach(9);
  servoRR.attach(4);
  servoRL.attach(5);
  servoFR.write(initialServoFR);
  servoFL.write(initialServoFL);
  servoRR.write(initialServoRR);
  servoRL.write(initialServoRL);
    milliNew=millis();//for first time measurement 
  
    for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}

void loop() {
  gyro_signals();
/*  Serial.print("Acceleration X [g]= ");
  Serial.print(AccX);
  Serial.print(" Acceleration Y [g]= ");
  Serial.print(AccY);
  Serial.print(" Acceleration Z [g]= ");
  Serial.println(AccZ);*/
  Serial.print(" Angle roll [degree/s]= ");
  Serial.print(AngleRoll);
  Serial.print(" Angle pitch [degree/s]= ");
  Serial.println(AnglePitch);
/////////////////////////////////////////////////////////////////////////////////// 
  Serial.print("RollError[]=");
  Serial.print(rollError);
  Serial.print(" PitchError[]=");
  Serial.println(pitchError);
/*  Serial.print(" pitchCorrectionDiff= ");
  Serial.print(pitchCorrectionDiff);
  Serial.print(" pitchCorrectionSum= ");
  Serial.println(pitchCorrectionSum);*/
  delay(100);/////////////////////////most imp
///////////////////////////////////////////////////////////////////////////////

  
  milliOld=milliNew;
  milliNew=millis();
  dt=milliNew-milliOld;//delta T

  pitchErrorOld=pitchError; /////////////////////////pitchDerivative////  
  pitchError=pitchTarget-AnglePitch;
  pitchErrorChange=pitchError-pitchErrorOld; 
  pitchErrorSlope=pitchErrorChange/dt;
  pitchErrorArea=pitchErrorArea+pitchError*dt;//////////////////rollIntegral//////
  
  rollErrorOld=rollError; /////////////////////////rollDerivative////  
  rollError=rollTarget-AngleRoll;
  rollErrorChange=rollError-rollErrorOld; 
  rollErrorSlope=rollErrorChange/dt;
  rollErrorArea=rollErrorArea+rollError*dt;//////////////////rollIntegral//////

  pitchCorrectionDiff=pitchCorrectionDiff-(Kp*pitchError+Kd*pitchErrorSlope+Ki*pitchErrorArea);
  pitchCorrectionSum=pitchCorrectionSum+(Kp*pitchError+Kd*pitchErrorSlope+Ki*pitchErrorArea);
  
  rollCorrectionDiff=rollCorrectionDiff-(Kp*rollError+Kd*rollErrorSlope+Ki*rollErrorArea);
  rollCorrectionSum=rollCorrectionSum+(Kp*rollError+Kd*rollErrorSlope+Ki*rollErrorArea);

if( 2>rollError>0 && 1>pitchError>-5){//&& -2>rollError<3//-5>pitchError<5
  servoFR.write(130);
  servoFL.write(45);
  servoRR.write(90);
  servoRL.write(70);
  }
  
if(pitchError>1 ){//pitchup

    servoFR.write(constrain(initialServoFR+pitchCorrectionSum, 85, 180));
    servoFL.write(constrain(initialServoFL+pitchCorrectionDiff, 0, 90));
    servoRR.write(constrain(initialServoRR+pitchCorrectionSum, 50, 130));
    servoRL.write(constrain(initialServoRL+pitchCorrectionDiff, 25, 115));
}

if(pitchError<-5){//pitch down

    servoFR.write(constrain(initialServoFR+pitchCorrectionDiff, 85, 180));
    servoFL.write(constrain(initialServoFL+pitchCorrectionSum, 0, 90));
    servoRR.write(constrain(initialServoRR+pitchCorrectionDiff, 50, 130));
    servoRL.write(constrain(initialServoRL+pitchCorrectionSum, 25, 115));
} 

/*
if(rollError>2){//rollright

    servoFR.write(constrain(initialServoFR+rollCorrectionDiff, 85, 180));
    servoFL.write(constrain(initialServoFL+rollCorrectionDiff, 0, 90));
    servoRR.write(constrain(initialServoRR+rollCorrectionSum, 50, 130));
    servoRL.write(constrain(initialServoRL+rollCorrectionSum, 25, 115));
}


if(rollError<0){//rollleft

    servoFR.write(constrain(initialServoFR+rollCorrectionSum, 85, 180));
    servoFL.write(constrain(initialServoFL+rollCorrectionSum, 0, 90));
    servoRR.write(constrain(initialServoRR+rollCorrectionDiff, 50, 130));
    servoRL.write(constrain(initialServoRL+rollCorrectionDiff, 25, 115));
}
*/
}     