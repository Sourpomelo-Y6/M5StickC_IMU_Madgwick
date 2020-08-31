#include <M5StickC.h>
#include <Preferences.h>


#include "MadgwickAHRS.h"

Preferences preferences;
Madgwick *filter = new Madgwick();

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

double sum_accX = 0.0;
double sum_accY = 0.0;
double sum_accZ = 0.0;

double sum_gyroX = 0.0;
double sum_gyroY = 0.0;
double sum_gyroZ = 0.0;

float calb_accX = 0.0;
float calb_accY = 0.0;
float calb_accZ = 0.0;

float calb_gyroX = 0.0;
float calb_gyroY = 0.0;
float calb_gyroZ = 0.0;

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.IMU.Init();
  //unsigned char regdata = 0x09;
  //I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1,&regdata);
  //delay(1);
  Serial.begin(115200);
  
  preferences.begin("imu_calb_data", true);
  calb_accX = preferences.getFloat("ax", 0);
  calb_accY = preferences.getFloat("ay", 0);
  calb_accZ = preferences.getFloat("az", 0);
  calb_gyroX = preferences.getFloat("gx", 0);
  calb_gyroY = preferences.getFloat("gy", 0);
  calb_gyroZ = preferences.getFloat("gz", 0);
  Serial.println(calb_accX);
  Serial.println(calb_accY);
  Serial.println(calb_accZ);
  Serial.println(calb_gyroX);
  Serial.println(calb_gyroY);
  Serial.println(calb_gyroZ);
  preferences.end();
  
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(40, 0);
  M5.Lcd.println("IMU TEST");
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.println("  Pitch   Roll    Yaw");
}

unsigned int pre_time =0;
float temp = 0;
/*****************************************
M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
M5.IMU.getAccelData(&accX,&accY,&accZ);
M5.IMU.getAhrsData(&pitch,&roll,&yaw);
M5.IMU.getTempData(&temp);
*****************************************/
void loop() {
  M5.update();
  
  // put your main code here, to run repeatedly:
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  //M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  //M5.IMU.getTempData(&temp);

  Serial.print(filter->q0);
  Serial.print(',');
  Serial.print(filter->q1);
  Serial.print(',');
  Serial.print(filter->q2);
  Serial.print(',');
  Serial.println(filter->q3);

  filter->MadgwickAHRSupdateIMU(
    PI/180.0F*(gyroX - calb_gyroX), 
    PI/180.0F*(gyroY - calb_gyroY), 
    PI/180.0F*(gyroZ - calb_gyroZ),
    accX - calb_accX, 
    accY - calb_accY, 
    accZ - calb_accZ
    );

  pitch = filter->getPitch()*180.0F/PI;
  roll = filter->getRoll()*180.0F/PI;
  yaw = filter->getYaw()*180.0F/PI;
  
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(140, 20);
  M5.Lcd.print("o/s");
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(140, 30);
  M5.Lcd.print("G");
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("Temperature : %.2f C", temp);
  //delay(10);
  while(millis() - pre_time < 10)
  {
    delay(1);
  }
  pre_time = millis();
  
  if(M5.BtnA.isPressed()){
    Serial.println("Calibration Start");
    getCalibrationVal();
  }
}

void getCalibrationVal()
{
    const int CalbNum = 1000;
    sum_clear();
    
    for(int i = 4 ; i >= 0 ; i--){
      M5.Lcd.setCursor(0, 70);
      M5.Lcd.printf("Start Calb %d sec",i);
      delay(1000);
    }
      
    for(int i = 0 ; i < CalbNum ; i++){
        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
        M5.IMU.getAccelData(&accX,&accY,&accZ);

        sum_accX += accX;
        sum_accY += accY;
        sum_accZ += accZ;

        sum_gyroX += gyroX;
        sum_gyroY += gyroY;
        sum_gyroZ += gyroZ;
        delay(10);
    }

    calb_accX = (float)(sum_accX/CalbNum);
    calb_accY = (float)(sum_accY/CalbNum);
    calb_accZ = (float)(sum_accZ/CalbNum) - 1.0F;

    calb_gyroX = (float)(sum_gyroX/CalbNum);
    calb_gyroY = (float)(sum_gyroY/CalbNum);
    calb_gyroZ = (float)(sum_gyroZ/CalbNum);

    preferences.begin("imu_calb_data", false);
    preferences.putFloat("ax", calb_accX);
    preferences.putFloat("ay", calb_accY);
    preferences.putFloat("az", calb_accX);
    preferences.putFloat("gx", calb_gyroX);
    preferences.putFloat("gy", calb_gyroY);
    preferences.putFloat("gz", calb_gyroZ);
    preferences.end();
    
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.printf("End Calb");
}

void sum_clear()
{
  sum_accX = 0.0;
  sum_accY = 0.0;
  sum_accZ = 0.0;

  sum_gyroX = 0.0;
  sum_gyroY = 0.0;
  sum_gyroZ = 0.0;  
}
