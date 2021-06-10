//---------------------------------------
// QUADCOPTER PROGRAM
// V1.0
// PROGRAMMER - Luiz Villa
//---------------------------------------

#include "quad_init.h"




// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  RemoteXY_Init ();  

  quadInit(); // Function to initialize the quadCopter
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

int delay_count = 0; //count the number of delays - used to increment the motor speed slowly


void loop() {

  RemoteXY_Handler ();    // Handler of the graphic user interface. De-comment to trigger communication with the GUI
  
  // MEASUREMENT SUB-ROUTINE -------------------------------------------------
  measurement_routine();

  // DECISION SUB-ROUTINE ----------------------------------------------------
  pwm_out = 5;    // sets a fixed pwm_out value

  // ACTION SUB-ROUTINE ------------------------------------------------------

  setSpeed(pwm_out);      // sets the speed to test the motors
}


// ---------------- OTHER FUNCTIONS --------------- 


/** Calculates the battery usage.
 * @param prevEntry Previous battery voltage value
 * @param newEntry New battery voltage value
 * @param alpha Smooth factor
 * @return Smoothed value
 */
float smoothBattery (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}


/** Calls the measurement routine and stores the values in global variables
 */
void measurement_routine (){
  mpuInterrupt = false;                 // reset interrupt flag 
  mpuIntStatus = mpu.getIntStatus();    // get INT_STATUS byte  
  fifoCount = mpu.getFIFOCount(); // get current FIFO count

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {    // check for overflow (this should never happen unless our code is too inefficient) 
    mpu.resetFIFO(); // reset so we can continue cleanly    
  } else if (mpuIntStatus & 0x02) { // otherwise, check for DMP data ready interrupt (this should happen frequently)
    
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // wait for correct available data length, should be a VERY short wait    
    
    mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
    fifoCount -= packetSize;                  // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);   //Gets the Quaternion
    mpu.dmpGetGravity(&gravity, &q);        //Gets the gravity vector
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //Gets the Yaw, Pitch and Roll

    yawInput   = (ypr[0]-ypr_cal[0]) * 180 / M_PI;  //compensates for the offset and converts in degrees
    pitchInput = (ypr[1]-ypr_cal[1]) * 180 / M_PI;
    rollInput  = (ypr[2]-ypr_cal[2]) * 180 / M_PI;

    batt_level = analogRead(A0) / 1023.0 * MULTIPLIER; //reads and calculates battery level

    altitude = sensor.readRangeSingleMillimeters(); //reads the altitude
  }
}


/** Sets a single pwm value for all motors.
 * @param val Motor PWM value between 0 and 256
 */
void setSpeed(int val) {
  analogWrite(FL_MOTOR, val);
  analogWrite(FR_MOTOR, val);
  analogWrite(BR_MOTOR, val);
  analogWrite(BL_MOTOR, val);
}

/** Sets a single pwm value for all motors.
 * @param currSpeed Currently set motor speed
 * @param actSpeed Actual speed to be set
 * @param rollDiff Changes in value due to roll
 * @param pitchDiff Changes in value due to the pitch
 */
void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  actSpeed[0] = (int) currSpeed[0] + (rollDiff) - (pitchDiff);  //each motor has actual Speed and speed at which we want them to fly...
  actSpeed[1] = (int) currSpeed[1] + (rollDiff) + (pitchDiff);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff) + (pitchDiff);  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[3] = (int) currSpeed[3] - (rollDiff) - (pitchDiff);

  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0 )
      actSpeed[i] = 0;
  }
}

/** Checks the value for a single motor.
 * @param motor ID of the motor to be checked FL = 0, FR = 1, BR = 2, BL = 3
 * @param actSpeed Actual speed to be set
 */
void checkIndividual (int motor, int* actSpeed) {
  analogWrite(FL_MOTOR, 0);
  analogWrite(FR_MOTOR, 0);
  analogWrite(BR_MOTOR, 0);
  analogWrite(BL_MOTOR, 0);

  if (motor == 0)
    analogWrite(FL_MOTOR, actSpeed[0]);
  if (motor == 1)
    analogWrite(FR_MOTOR, actSpeed[1]);
  if (motor == 2)
    analogWrite(BR_MOTOR, actSpeed[2]);
  if (motor == 3)
    analogWrite(BL_MOTOR, actSpeed[3]);
}

/** Updates the actual speed for each motor.
 * @param actSpeed Actual speed to be set
 */
void runIndividual (int* actSpeed) {
  analogWrite(FL_MOTOR, actSpeed[0]);
  analogWrite(FR_MOTOR, actSpeed[1]);
  analogWrite(BR_MOTOR, actSpeed[2]);
  analogWrite(BL_MOTOR, actSpeed[3]);
}

/** Initializes the QuadCopter setups.
 */
void quadInit(){

  Serial.begin(9600);               // initializes the Serial
  Serial.println("INITIALIZING");   // sends a message indicating the system is initializing
  

  //Enable internal reference of 1.1V
  //initialise battery level array with current battery level value
  pinMode(A0, INPUT);                                 //sets the A0 pin as input
  analogReference(INTERNAL);                          //sets the analog reference as internal
  batt_level = analogRead(A0) / 1023.0 * MULTIPLIER;  // reads and calculates battery level

  motorBattery = 3.7;                   //This is the expected high battery voltage
  Serial.print("Battery level");        //Prints the battery level
  Serial.println(batt_level);

  //------------------------------PID (NOT USED) ----------------------------------
  //initialize the variables we're linked to
  // pitchInput = 0.0;
  // rollInput = 0.0;

  // pitchSetpoint = 0.0;
  // rollSetpoint = 0.0;

  //turn the PID on
  // pitchPID.SetMode(AUTOMATIC);
  // rollPID.SetMode(AUTOMATIC);

  // pitchPID.SetOutputLimits(-20, 20);
  // rollPID.SetOutputLimits(-20, 20);
  //-------------------------------------------------------------------
  
  // Resets the target speeds
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  }

  //initializes the motors and their pin mode
  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println(" ");
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(" ");
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // CODE TO INITIALIZE THE DISTANCE SENSOR
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
  }

  #if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
  #endif


}



