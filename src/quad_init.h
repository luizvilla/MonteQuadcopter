

#include "Arduino.h"

// // RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__BLE_NANO

#include <RemoteXY.h>

// Initial includes
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include "VL53L0X.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file


#define ADDR_PITCH_OFFSET 0
#define ADDR_ROLL_OFFSET 1
#define FL_MOTOR 3
#define FR_MOTOR 9
#define BR_MOTOR 10
#define BL_MOTOR 11
// // RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__BLE_NANO

#include <RemoteXY.h>


//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
//#define REMOTEXY__DEBUGLOGS Serial


// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600


// ----------------------------------------------------------------------------

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,4,0,0,0,21,0,11,13,0,
  5,32,6,9,39,39,2,26,31,5,
  32,55,9,39,39,2,26,31 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  int8_t joystick_2_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_2_y; // =-100..100 y-coordinate joystick position 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop) 



/////////////////////////////////////////////
//           END RemoteXY include          //
///////////////////////////////////////////// 



//---------------------------------PID------------------------------------
//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;
double yawSetpoint, yawInput, yawOutput;

//Define the aggressive and conservative Tuning Parameters
double consKp = 0.5, consKi = 0.05, consKd = 0.05;

PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);
//------------------------------------------------------------------------


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
VL53L0X sensor;   // altitude measurement object
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_cal[3] = {-0.55, -0.03, 0.01}; // initilizes the calibration (done by hand. This should be automated)
uint16_t altitude = 0; // holds the value of the altitude for the QuadCopter

// motor control variables
int pwm_out = 0; //holds the value of the pwm

// battery level variables
float batt_level = 0; //holds the value of the battery level
float motorBattery = 0; //holds the maximum value of the battery





// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#define ALPHA 0.1
#define MULTIPLIER 6.67

int targetSpeed[4];


//Declaration of base functions

float smoothBattery (float prevEntry, float newEntry, float alpha);
void setSpeed(int val);
void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff);
void checkIndividual (int motor, int* actSpeed);
void runIndividual (int* actSpeed);

void quadInit();

void get_ypr();

void measurement_routine();

