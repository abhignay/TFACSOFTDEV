#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>

// SD stuff
File file;

const int sd_cs = 21;
int dataDumpflag = 0;

// flash stuff
#define FILE_SIZE_4M 4194304
#define FILE_SIZE_512K 524288

SerialFlashFile flashFile; 

// defining interval durations
const uint32_t slowDataInterval = 2000;
const uint32_t fastDataInterval = 20;

// cs pin
const byte flash_cs = 10;

//char filename[16];
uint8_t isRecording = false;

struct flashDataType{

  // number of records because SerialFlash doesn't know when the file ends
  uint32_t dataNumber;

  // times
  float fltTime;
  float onTime;
  int state;

  //accelerometer
  float accxB;
  float accxG;
  float accy;
  float accz;
  float gax;
  float gaxkal;

  //gnss data
  float lat;
  float lon;
  int gnssSats;
  float gnssAccelX;
  float gnssAccelY;
  float gnssAccelZ;
  float gnssAltitude;
  float gnssVX;
  float gnssVY;
  float gnssVZ;
  float gpsgndSpeed;
  float UTCYear;
  int UTCDay;
  int UTCMinute;
  int UTCHour;
  int UTCSec;
  int UTCMonth;
  int PDOP;
  int gnssTypeFix;

  // gyro
  float gx;
  float gy;
  float gz;

  // filtered data
  float kalalt;

  float kalaxb;
  float kalaxg;
  float kalay;
  float kalaz;

  float kalvelx;
  float kalvely;
  float kalvelz;

  //kalman covariances
  float covalt;
  float covax;
  float covay;
  float covaz;
  float covvel;
  float batVolt;

  // baro
  float rawalt;
  float pres;

  //temp
  float imutemp;
  float barotemp;
  float avgtemp;

  //mag
  float magx;
  float magy;
  float magz;

  //pyro
  int py1_fire;
  int py2_fire;

} flashData;

/*SD Functions*/
char filename[] = "FLT00.CSV"; //File name

void openFile(){
  for (uint8_t i = 0; i < 100; i++) { //The SD card can store up to 100 files
    filename[3] = i/10 + '0';
    filename[4] = i%10 + '0';

    if (! SD.exists(filename)) {
      file = SD.open(filename, O_CREAT | O_WRITE); //Only open a new file if it doesn't exist
      break;
    }
  }
}

void writeHeader() {
  // time
  //file.print("Data Number");
  file.print(F("On Time"));
  file.print(F(",Flight Time"));
  file.print(F(",State"));

  //accelerometer
  file.print(F(",Acc_xb_mss"));
  file.print(F(",Acc_xg_mss"));
  file.print(F(",Acc_y_mss"));
  file.print(F(",Acc_z_mss"));
  file.print(F(",Acc_x_G"));
  file.print(F(",Kal_Acc_x_G"));

  // gyro
  file.print(F(",Gyro_x_rps"));
  file.print(F(",Gyro_y_rps"));
  file.print(F(",Gyro_z_rps"));

  // filtered data
  file.print(F(",Kal_Alt_m"));

  file.print(F(",Kal_Acc_x_mss_bf"));
  file.print(F(",Kal_Acc_x_mss_gf"));
  file.print(F(",Kal_Acc_y_mss"));
  file.print(F(",Kal_Acc_z_mss"));

  file.print(F(",Kal_Vel_x_ms")); // from kalman filter
  file.print(F(",Kal_Vel_y_ms")); // from kalman filter
  file.print(F(",Kal_Vel_z_ms")); // from kalman filter

  //gnss data
  file.print(F(",Latitude"));
  file.print(F(",Longitude"));
  file.print(F(",GNSS_Sats"));
  file.print(F(",GNSS_AccX_mss"));
  file.print(F(",GNSS_AccY_mss"));
  file.print(F(",GNSS_AccZ_mss"));
  file.print(F(",GNSS_Alt_m"));
  file.print(F(",GNSS_VelX_ms"));
  file.print(F(",GNSS_VelY_ms"));
  file.print(F(",GNSS_VelZ_ms"));
  file.print(F(",GNSS_GroundSpeed_ms"));
  file.print(F(",GNSS_Year_UTC"));
  file.print(F(",GNSS_Month_UTC"));
  file.print(F(",GNSS_Day_UTC"));
  file.print(F(",GNSS_Hour_UTC"));
  file.print(F(",GNSS_Minute_UTC"));
  file.print(F(",GNSS_Second_UTC"));
  file.print(F(",GNSS_pDOP"));
  file.print(F(",GNSS_Fix"));

  //kalman covariances
  file.print(F(",Covariance_Alt"));
  file.print(F(",Covariance_AX"));
  file.print(F(",Covariance_AY"));
  file.print(F(",Covariance_AZ"));
  file.print(F(",Covariance_Vel"));

  // baro
  file.print(F(",Baro_alt_m"));
  file.print(F(",Baro_Pres_kpa"));

  //temp
  file.print(F(",IMU_Temp_C"));
  file.print(F(",Baro_Temp_C"));
  file.print(F(",Temp_c"));

  //mag
  file.print(F(",Mag_X_gauss"));
  file.print(F(",Mag_Y_gauss"));
  file.print(F(",Mag_Z_gauss"));

  //pyro
  file.print(F(",Py1_Fire"));
  file.print(F(",Py2_Fire"));

  file.println();
  file.flush();
}


void dumpData(){
  flashFile = SerialFlash.open( "dataFile.txt" );

  if (flashFile) {
    do {
      flashFile.read( (uint8_t *)&flashData, sizeof(flashData) );

      // library doesn't know where end of actual written data is so we have
      // to look for it ourselves!
      if ( flashData.dataNumber != 0xFFFFFFFF ) {
        // file.print(flashData.dataNumber);
        // file.print( "," );
        file.print( flashData.onTime );
        file.print( "," );
        file.print( flashData.fltTime );
        file.print( "," );
        file.print( flashData.state );
        file.print( "," );
        file.print( flashData.accxB );
        file.print( "," );
        file.print( flashData.accxG  );
        file.print( "," );
        file.print( flashData.accy );
        file.print( "," );
        file.print( flashData.accz );
        file.print( "," );
        file.print( flashData.gax );
        file.print( "," );
        file.print( flashData.gaxkal );
        file.print( "," );
        file.print( flashData.gx );
        file.print( "," );
        file.print( flashData.gy );
        file.print( "," );
        file.print( flashData.gz );        
        file.print( "," );
        file.print( flashData.kalalt );
        file.print( "," );
        file.print( flashData.kalaxb );
        file.print( "," );
        file.print( flashData.kalaxg );
        file.print( "," );
        file.print( flashData.kalay );
        file.print( "," );
        file.print( flashData.kalaz );
        file.print( "," );
        file.print( flashData.kalvelx );
        file.print( "," );
        file.print( flashData.kalvely );
        file.print( "," );
        file.print( flashData.kalvelz );
        file.print( "," );
        file.print( flashData.lat, 7 );
        file.print( "," );
        file.print( flashData.lon, 7) ;
        file.print( "," );
        file.print( flashData.gnssSats );
        file.print( "," );
        file.print( flashData.gnssAccelX );
        file.print( "," );
        file.print( flashData.gnssAccelY );
        file.print( "," );
        file.print( flashData.gnssAccelZ );
        file.print( "," );
        file.print( flashData.gnssAltitude );
        file.print( "," );
        file.print( flashData.gnssVX );
        file.print( "," );
        file.print( flashData.gnssVY );
        file.print( "," );
        file.print( flashData.gnssVZ );
        file.print( "," );
        file.print( flashData.gpsgndSpeed );
        file.print( "," );
        file.print( flashData.UTCYear );
        file.print( "," );
        file.print( flashData.UTCMonth );
        file.print( "," );
        file.print( flashData.UTCDay );
        file.print( "," );
        file.print( flashData.UTCHour );
        file.print( "," );
        file.print( flashData.UTCMinute );
        file.print( "," );
        file.print( flashData.UTCSec );
        file.print( "," );
        file.print( flashData.PDOP );
        file.print( "," );
        file.print( flashData.gnssTypeFix );
        file.print( "," );
        file.print( flashData.covalt );
        file.print( "," );
        file.print( flashData.covax );
        file.print( "," );
        file.print( flashData.covay );
        file.print( "," );
        file.print( flashData.covaz );
        file.print( "," );
        file.print( flashData.covvel );
        file.print( "," );
        file.print( flashData.rawalt );
        file.print( "," );
        file.print( flashData.pres );
        file.print( "," );
        file.print( flashData.imutemp );
        file.print( "," );
        file.print( flashData.barotemp );
        file.print( "," );
        file.print( flashData.avgtemp );
        file.print( "," );
        file.print( flashData.magx );
        file.print( "," );
        file.print( flashData.magy );
        file.print( "," );
        file.print( flashData.magz );
        file.print( "," );
        file.print( flashData.py1_fire );
        file.print( "," );
        file.println( flashData.py2_fire );
      }
    } while ( flashData.dataNumber != 0xFFFFFFFF );
    file.close();
    flashFile.close();
  }
  else {
    Serial.println(F("No more files."));
  }
  dataDumpflag = 1;
}

void openFlashFile(){
  SerialFlash.create( "dataFile.txt", FILE_SIZE_4M );
  flashFile = SerialFlash.open( "dataFile.txt" );
}

unsigned long slowmillis;
unsigned long fastmillis;

void updateFlashData(int state){
  flashData.dataNumber++;

  // times
  flashData.fltTime = flightTime;
  flashData.onTime = ontime;
  flashData.state = state;

  //accelerometer
  flashData.accxB = rawaxb;
  flashData.accxG = rawaxg;
  flashData.accy = raway;
  flashData.accz = rawaz;
  flashData.gax = gaxRaw;
  flashData.gaxkal = gax;

  // gyro
  flashData.gx = gx_rads;
  flashData.gy = gy_rads;
  flashData.gz = gz_rads;

  // filtered data
  flashData.kalalt = kalmanalt;

  flashData.kalaxb = kalmanaxb;
  flashData.kalaxg = kalmanaxg;
  flashData.kalay = kalmanay;
  flashData.kalaz = kalmanaz;

  flashData.kalvelx = kalmanvelx;
  flashData.kalvely = kalmanvely;
  flashData.kalvelz = kalmanvelz;

  //gnss data
  flashData.lat = latitude;
  flashData.lon = longitude;
  flashData.gnssSats = gnssSiv;
  flashData.gnssAccelX = gnssAccX;
  flashData.gnssAccelY = gnssAccY;
  flashData.gnssAccelZ = gnssAccZ;
  flashData.gnssAltitude = gnssAlt;
  flashData.gnssVX = gnssVelX;
  flashData.gnssVY = gnssVelY;
  flashData.gnssVZ = gnssVelZ;
  flashData.gpsgndSpeed = GroundSpeed;
  flashData.UTCYear = gnssYear;
  flashData.UTCMonth = gnssMonth;
  flashData.UTCDay = gnssDay;
  flashData.UTCHour = gnssHour;
  flashData.UTCMinute = gnssMinute;
  flashData.UTCSec = gnssSec;
  flashData.PDOP = pDOP;
  flashData.gnssTypeFix = gnssFix;


  //kalman covariances
  flashData.covalt = covariancealt;
  flashData.covax = covarianceax;
  flashData.covay = covarianceay;
  flashData.covaz = covarianceaz;
  flashData.covvel = covariancevel;

  // baro
  flashData.rawalt = rawalt;
  flashData.pres = pres;

  //temp
  flashData.imutemp = imutemp;
  flashData.barotemp = barotemp;
  flashData.avgtemp = temp_C;

  //mag
  flashData.magx = magx;
  flashData.magy = magy;
  flashData.magz = magz;

  //pyro
  flashData.py1_fire = py1_state;
  flashData.py2_fire = py2_state;

}

void SlowDataLog(int State){
  if (millis() - slowmillis >= 2000){
    slowmillis = millis();

    updateFlashData(State);
    flashFile.write( (uint8_t *)&flashData, sizeof(flashData) );

  }
}

void dataLog(int State){
  if (millis() - fastmillis >= 20){
    fastmillis = millis();

    updateFlashData(State);
    flashFile.write( (uint8_t *)&flashData, sizeof(flashData) );
  }
}

int erasedOnce = 0;
void eraseFlash(){
  if (erasedOnce == 0){
    Serial.println(F("ERASING FLASH CHIP"));
    Serial.println("Approximate wait time is around 2 minutes more specifically 72 seconds :) ");
    SerialFlash.eraseAll();
  }
  erasedOnce = 1;
}

int erasedOnce1 = 0;
void eraseFlash1(){
  if (erasedOnce == 0){
    Serial.println(F("ERASING FLASH CHIP"));
    Serial.println("Approximate wait time is around 2 minutes more specifically 72 seconds :) ");
    SerialFlash.eraseAll();
  }
  erasedOnce1 = 1;
}