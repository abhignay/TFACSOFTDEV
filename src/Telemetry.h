#include "EBYTE.h"
#include "SerialTransfer.h"

#define ESerial Serial1 // connecting to one of the teensy's serial port

// defining aux and the 2 mode pins
#define PIN_M0 4 // Z+
#define PIN_M1 3 // Z-
#define PIN_AX 8 // CS Pin

EBYTE Transceiver(&ESerial, PIN_M0, PIN_M1, PIN_AX); // creating the ebyte object
SerialTransfer myTransfer;

byte packetID = 0;

struct Data_1{
    // acceleration and velocity
    float kaccx;
    float kalvelx;

    // times
    int onTME;
    int fltTME;

    // baro
    float kalt;
    float baroT;

    // misc
    int sysState;

} 
Data1;

struct Data_2{
    // gnss misc
    int sats;
    float pdop;
    int gpsFix;

    // gnss important
    float lat;
    float lon;
    float gpsAlt;

} 
Data2;


uint32_t tlmMillis;

void updateData(){
    
    uint16_t sendSize = 0;

    switch (packetID) { //selector byte chooses the variable(s) to send.
    case 1:
        sendSize = myTransfer.txObj(Data1, sendSize);
        break;
    case 2:
        sendSize = myTransfer.txObj(Data2, sendSize);
        break;
    }

    myTransfer.sendData(sendSize, packetID);
}

void sendTLMData(){
    if (millis() - tlmMillis >= 350){
        tlmMillis = millis();

        // update the structs with tfac data
        Data1 = {kalmanaxb, kalmanvelx, intontme, intFltTme, kalmanalt, barotemp, state};
        Data2 = {gnssSiv, pDOP, gnssFix, latitude,longitude, gnssAlt};

        packetID++;
        updateData();

        if (packetID == 2){
            packetID = 0;
        }
    }
}