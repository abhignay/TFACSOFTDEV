// TFAC Fight Software Dev Version
// Author - Abhigna Y

#include <Arduino.h>  
#include <Wire.h>
#include "BMI088.h"
#include <MS56XX.h>
#include <LIS3MDL.h>
#include <Kalman.h>
#include <SPI.h>
#include <memory.h>
#include <LED.h>
#include <State_Machine.h>
#include <GNSS.h>
#include <Telemetry.h>
#include <Dev.h>

// declaring sensor objects
Bmi088Accel accel(Wire,0x18); // accel
Bmi088Gyro gyro(Wire,0x68);  //gyro
MS56XX MSXX(MS56XX_ADDR_LOW, MS5607); // baro
LIS3MDL mag;
SFE_UBLOX_GNSS gnss;

// declaring pyro pins
int pyro1 = 23;
int pyro2 = 22;

void setup() 
{
  // connecting to serial monitor and wire lib
  Serial.begin(9600);
  Wire.begin();

  // starting sensors
  accel.begin();
  gyro.begin();
  MSXX.begin();
  MSXX.configBaro(BARO_PRESS_D1_OSR_4096, BARO_TEMP_D2_OSR_4096);
  mag.init();
  mag.enableDefault();

  // starting GNSS
  gnss.begin();
  gnss.setI2COutput(COM_TYPE_UBX); 
  gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); 
  gnss.setNavigationFrequency(4); //Produce four solutions per second
  gnss.setAutoPVTcallbackPtr(&getPVTData); 
  gnss.setAutoHNRINScallbackPtr(&getHNRINSdata);

  // starting led and buzzer
  pinMode (pyro1, OUTPUT);
  pinMode (pyro2, OUTPUT);

  // starting memory devices
  // SD.begin(sd_cs);
  // SerialFlash.begin(flash_cs);

  // // starting LoRa radio
  // ESerial.begin(9600);
  // Transceiver.init();
  // myTransfer.begin(ESerial);

  // openFile();
  // openFlashFile();

  // writeHeader();
}

bool baro_DRDY;
float alt_offset;
uint32_t millis2 = millis();
bool baro_offsetflag = false;
uint32_t millistime;

void offset_baro(){
  millistime = millis();
  if (!baro_offsetflag){
    if (millistime > 10000){
      alt_offset += rawalt;
      baro_offsetflag = true;
    }
  }
}

void dataUpdate(){
  
  // reading data from sensors
  accel.readSensor();
  gyro.readSensor();
  baro_DRDY = MSXX.doBaro(true);
  mag.read();

  // raw  sensor data

  // accel variables
  // mss = meters per second squared
  rawaxb = accel.getAccelX_mss() * -1;
  rawaxg = accel.getAccelX_mss() + 9.8; // vertical acceleration in meters per second squared
  raway = accel.getAccelY_mss();
  rawaz = accel.getAccelZ_mss(); 

  gaxRaw = rawaxb / 9.8; // vertical acceleration in G

  // gyro variables
  gx_rads = gyro.getGyroX_rads(); // rads = radian
  gy_rads = gyro.getGyroY_rads();
  gz_rads = gyro.getGyroZ_rads();

  imutemp = accel.getTemperature_C(); // celsius, join the club 'merica

  millistime = millis();  

  if (baro_DRDY){
    // baro variables
    pres = MSXX.pressure; // pascals
    barotemp = MSXX.temperature; // celsius
    rawalt = MSXX.altitude - alt_offset; // meters above sea level
  }

  temp_C = (barotemp + imutemp) /2; // average of temperature between the temp sensors onboard

  // mag variables
  magx = mag.m.x;
  magy = mag.m.y;
  magz = mag.m.z;

  //gnss variables
  gnss.checkUblox();
  gnss.checkCallbacks();

  // timing
  ontime = (float) millis() / 1000.0;

  if (state == INIT or state == IDLE){
    flightTime = 0;
  }
  else{
    flightTime = (float) (millis() - launchtime) /1000.0; 
  }

  intontme = int(ontime);
  intFltTme = int(flightTime);

  currTime = millis();
  delT = (currTime - prevTime) / 1000.0;
 
  // pyro states
  py1_state = digitalRead(pyro1);
  py2_state = digitalRead(pyro2);

  // kalman filtered data
  predict(rawaxg);

  update(rawalt, rawaxg); // updates our position velocity and acceleration (vertical on 'x' axis for TFAC)
  update_AccelY(rawalt, raway); // updates only acceleration on y axis
  update_AccelZ(rawalt, rawaz); // updates only acceleration on z axis

  kalmanaxb = kalmanaxg + 9.81;
  gax = kalmanaxb / 9.807; // vertical acceleration in G

  // get previous altitude to check if we have reached apogee
  if(millis() - millis2 >= 1500){
    millis2 = millis();
    prev_alt = kalmanalt;
  }
  prevTime = currTime;

}

uint32_t GCSms;
void sendGCSData(){
  if (millis() - GCSms >= 350){
    GCSms = millis();

    Serial.print(kalmanaxb);
    Serial.print(", ");
    Serial.print(kalmanvelx);
    Serial.print(", ");
    Serial.print(int(ontime));
    Serial.print(", ");
    Serial.print(int(flightTime));
    Serial.print(", ");
    Serial.print(kalmanalt);
    Serial.print(", ");
    Serial.print(temp_C);
    Serial.print(", ");
    Serial.print(state);
    Serial.print(", ");
    Serial.print(gnssSiv);
    Serial.print(", ");
    Serial.print(pDOP);
    Serial.print(", ");
    Serial.print(gnssFix);
    Serial.print(", ");
    Serial.print(latitude);
    Serial.print(", ");
    Serial.print(longitude);
    Serial.print(", ");
    Serial.println(gnssAlt);
  }
}

uint32_t millisSense;
int FiredONCE = 3;
void loop() 
{  
  if (millis() - millisSense >= 20){
    millisSense = millis();
    dataUpdate(); // creates and updates the data variables
  }

  // sendTLMData();
  // sendGCSData();
  
  //state machine
  if (state == INIT){
    led_init();
    offset_baro();
    pyro_low(pyro1, pyro2); 
    initialize(millistime);
  
  }
  if (state == IDLE){ // waiting for launch 
    led_pad_idle(); 
    checkFor10(kalmanalt, state);
    // SlowDataLog(state);
  } 
  else if (state == DEVAlt){
    ledRed();
    if (FiredONCE != 0){
      if (millis < Pre10meterMillis){
        depChutes(pyro1);
      }
      FiredONCE = 1;
    }
  }


  // else if (state == POW_FLIGHT){ // in flight, logging data at 50 hertz, checks if computer has reached Apogee
  //   led_pow_flight();
  //   dataLog(state);
  //   apogee(kalmanalt, prev_alt);

  // }
  // else if (state == DESCENT){ // we have reached apogee deploy chutes and check if we are under 5 meters
  //   led_descent();
  //   dataLog(state); 
  //   depChutes(pyro1);
  //   landed_check(kalmanalt);
  // }
  // else if (state == LANDED){ // we have landed transferring data from flash to micro sd
  //   led_landed();
  //   dumpData();

  //   party_switch();
  // }
  // else if (state == PARTY){ // nothing to do flight is done happy buzzer tones and led
  //   led_party();
  //   eraseFlash();
  // }

}  