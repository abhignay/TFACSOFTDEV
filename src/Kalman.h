#include <BasicLinearAlgebra.h> 

// defining raw variables
float rawaxb; // body frame basically raw
float rawaxg; // global frame is body frame + 9.81
float raway;
float rawaz;

float gx_rads;
float gy_rads;
float gz_rads;

float imutemp;
float rawalt;
float prev_alt;
float pres;
float barotemp;
float temp_C;

float magx;
float magy;
float magz;

float gax;
float gaxRaw;

float latitude;
float longitude;
float GroundSpeed;
byte gnssSiv;
float gnssAccX;
float gnssAccY;
float gnssAccZ;
float gnssAlt;
float gnssVelX;
float gnssVelY;
float gnssVelZ;
int gnssYear;
int gnssDay;
int gnssMinute;
int gnssHour;
int gnssSec;
int gnssMonth;
float pDOP;
int gnssFix;

int py1_state;
int py2_state;

float ontime; 
float flightTime;
int intontme;
int intFltTme;

float delT;
unsigned long currTime;
unsigned long prevTime;

// defining the filtered variables
float kalmanaxb; // body frame
float kalmanaxg; // global frame
float kalmanay;
float kalmanaz;
float kalmanalt;
float kalmanvelx;  
float kalmanvely; 
float kalmanvelz; 
// covariance variables for testing
float covarianceax;
float covarianceay;
float covarianceaz;
float covariancealt;
float covariancevel;

// setting up matrices for kalman filter
using namespace BLA;

// prediction matrices

BLA::Matrix<3, 1> X = { // x is not technically a prediction only matrix
  0, // position or altitude
  0, // velocity
  0  // acceleration
}; // state vector

BLA::Matrix<3, 3> F; // represents prediction step/ in math terms it is called system matrix

BLA::Matrix<3, 3> P = { // also not just a prediction matrix
  1, 0, 0, // pp, pv, pa
  0, 1, 0, // vp, vv, va
  0, 0, 1 // ap, av, aa (i think)
}; // covariance matrix of our best estimate

BLA::Matrix<3, 1> B; // control matrix, can be omitted for simple systems with no external influence  
BLA::Matrix<1, 1> U; // control vector same as B but a vector instead of a matrix

BLA::Matrix<3, 3> Q = {
  0.00000003125, 0, 0,
  0, 0.0000125, 0,
  0, 0, 0.001
}; // covariance uncertainity

// update matrices
BLA::Matrix<3, 3> H = {
  1, 0, 0,
  0, 0, 0,
  0, 0, 1
}; // sensor matrix

BLA::Matrix<3, 1> Z; // sensor measurement vector 

BLA::Matrix<3, 3> R{
  0.01, 0, 0,
  0, 0.01, 0,
  0, 0, 0.01
}; // measurement uncertainity

BLA::Matrix<3, 3> K; // kalman gain

// prediction step

// time variables
uint32_t startTime;
uint32_t currentTime;
float dT;

bool firstreading = true;

void predict(float accel){
  currentTime = millis();
  dT = (currentTime - startTime) / 1000.0f;

  // defining some of our matrices, these have variables that were not previously defined so we're doing it here
  if (!firstreading)
  {
    F = {
      1, dT, (dT * dT)/2,
      0, 1, dT,
      0, 0, 1
    };

    B = {
      (dT * dT)/2,
      dT,
      0
    };

    U = {accel};

    // actual prediction equations
    X = F * X + B * U;
    P = F * P * ~F + Q;

    startTime = currentTime;
  }
  firstreading = false;
}

// the update functions
void update(float alt, float ax){
  Z = {alt, 0, ax};

  K = P * ~H * Inverse(H * P *  ~H + R);

  // update our original estimates with sensor readings
  X = X + K * (Z - H * X);
  P = P - K * H * P;

  kalmanalt = X(0, 0); // filtered alt
  kalmanvelx = X(1, 0); // filtered vertical velocity 
  kalmanaxg = X(0, 2); // filtered vertical accel

  covariancealt = P(0, 0);
  covariancevel = P(0, 1);
  covarianceax = P(0, 2);
}

void update_AccelY(float alt, float ax){
  Z = {alt, 0, ax};

  K = P * ~H * Inverse(H * P *  ~H + R);

  // update our original estimates with sensor readings
  X = X + K * (Z - H * X);
  P = P - K * H * P;

  kalmanay = X(0, 2);
  kalmanvely = X(1, 0); // filtered vertical velocity
  covarianceay = P(0, 2);
}

void update_AccelZ(float alt, float ax){
  Z = {alt, 0, ax};

  K = P * ~H * Inverse(H * P *  ~H + R);

  // update our original estimates with sensor readings
  X = X + K * (Z - H * X);
  P = P - K * H * P;

  kalmanaz = X(0, 2);
  kalmanvelz = X(1, 0); // filtered vertical velocity
  covarianceaz = P(0, 2);
}