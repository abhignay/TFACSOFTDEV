#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

void getPVTData(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  latitude = ubxDataStruct->lat; // Print the latitude
  latitude = latitude/10000000;
  longitude = ubxDataStruct->lon; // Print the longitude
  longitude = longitude/10000000;
  gnssSec = ubxDataStruct->sec;
  gnssSiv = ubxDataStruct->numSV;
  gnssAlt = ubxDataStruct->hMSL;
  gnssAlt = gnssAlt/1000;
  gnssVelX = ubxDataStruct->velN;
  gnssVelY = ubxDataStruct->velE;
  gnssVelZ = ubxDataStruct->velD;
  gnssYear = ubxDataStruct->year;
  gnssMonth = ubxDataStruct->month;
  gnssDay = ubxDataStruct->day;
  gnssHour = ubxDataStruct->hour;
  gnssMinute = ubxDataStruct->min;
  gnssSec = ubxDataStruct->sec;
  pDOP = ubxDataStruct->pDOP;
  pDOP = pDOP / 100.0;
  gnssFix = ubxDataStruct->fixType;
  GroundSpeed = ubxDataStruct->gSpeed;
  GroundSpeed = GroundSpeed / 1000;
}

void getHNRINSdata(UBX_HNR_INS_data_t *ubxDataStruct)
{
  gnssAccX = ubxDataStruct->xAccel;
  gnssAccY = ubxDataStruct->yAccel;
  gnssAccZ = ubxDataStruct->zAccel;
}
