#include <Settings.h>

/* States :-
  State0 - INIT - Initialize all sensors

  State1 - IDLE - Beep buzzer at certain tone + green light blinking. 
  We are waiting for the rocket to liftoff, we know that we have lifted off 
  if accel x reading filtered passes 2G.

  State2 - POW_FLIGHT - Read data from Accel, Gyro, Baro, Mag after 
  it goes through a kalman filter and log it to flash chip at 50 hz. 
  Buzzer tone beeping + white color blinking. checks if apogee reached

  State3 - DESCENT  - if apogee is reached then deploy chutes, checks if altitude under 5m. blue flashing light + buzzer

  State4 - LANDED - move all data from flash chip to sd card. 

  State5 - PART - cycle through an rgb with happy buzzer tones!
*/

// state machine
enum FlightState{
  INIT,
  IDLE,
  DEVAlt,
  POW_FLIGHT,
  DESCENT,
  LANDED,
  PARTY
};

FlightState state = INIT; // by default the rocket will be in init to initialize the sensors

// state machine functions

// INIT state func
int initonce = 0;
void initialize(uint32_t millistime){
  if (initonce == 0){
    if (millistime > 15000){
      state = IDLE;
      initonce = 1;
    }     
  }
}

void zeroLaunchTime(){
  if (state == INIT or state == IDLE){
    flightTime = 0;
    intFltTme = 0;
  }
}

// IDLE state funcs
unsigned long launchtime;
void launchdetect(float accG){
  if (accG > launchthresh){ // if we are accelerating above 2Gs for over 0.5 seconds then only we have launched 
    state = POW_FLIGHT;
    launchtime = millis();

    Serial.println("Launch Detected!");
    Serial.println("Now switching to powered flight...");
    Serial.println(); // dummy print
  }
}

int py_low_once = 0;
void pyro_low(int pyro1, int pyro2){
  if (py_low_once == 0){
    digitalWrite(pyro1, LOW);
    digitalWrite(pyro2, LOW);
  }
  py_low_once = 1; // we only need to run this once
}

// POW_FLIGHT state func
void apogee (float alt, float prev_alt){
  if (prev_alt - alt > descentthresh){
    state = DESCENT;
  }
}

// DESCENT state func
bool firingStatus = false;
unsigned long fireTime;

void depChutes(int py1){
  if (firingStatus == false && fireTime == 0)
  {
    //Fire pyro charge
    fireTime = millis();
    firingStatus = true;
    digitalWrite(py1, HIGH);
  }

  if (firingStatus == true && millis() - fireTime >= fire_length)
  {
    //Stop pyro charge
    digitalWrite(py1, LOW);
    firingStatus = false;
  }
}

unsigned long landedtimecheck;

void landed_check(float kalmanalt){
  if (kalmanalt < landedthresh){ 

    landedtimecheck = millis();
    while(millis() < landedtimecheck + 5000){
      //wait approx 5000 ms
    }

    if (kalmanalt < landedthresh){
      state = LANDED;
    }
  }
}

/*No functions for landed because the only one is dumpData(); and that is in memory.h*/

// PARTY state func
void party_switch(){
  if (dataDumpflag == 1){
    state = PARTY;
  }
}