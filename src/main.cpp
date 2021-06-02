//---------------------------------------
// QUADCOPTER PROGRAM
// V1.0
// PROGRAMMER - Luiz Villa
//---------------------------------------

#include "quad_init.h"  // Header that includes all that is needed to start the quadCopter

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  //RemoteXY_Init ();  

  quadInit(); // Function to initialize the quadCopter
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int myReading = 0;
int pwm_out = 0;
int delay_count = 0; //count the number of delays - used to increment the motor speed slowly

void loop() {

  //RemoteXY_Handler (); 

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //        mySerial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //compensates for the offset and converts in degrees
    yawInput   = (ypr[0]-ypr_cal[0]) * 180 / M_PI;  //compensates for the offset and converts in degrees
    pitchInput = (ypr[1]-ypr_cal[1]) * 180 / M_PI;
    rollInput  = (ypr[2]-ypr_cal[2]) * 180 / M_PI;

    batt_level = analogRead(A0) / 1023.0 * MULTIPLIER;

    RemoteXY.motor_speed = pwm_out;
    RemoteXY.show_pitch = pitchInput;
    RemoteXY.show_roll = rollInput;
    RemoteXY.show_yaw = yawInput;


      // pwm_out = 5;
      setSpeed(pwm_out);
      // runIndividual(targetSpeed);
      // Serial.print(F("speed="));
      // Serial.println(pwm_out);
      if(delay_count==2000) pwm_out++;
      if(pwm_out > 50) pwm_out = 1 ;
  }
//  delay(100);
  delay_count++;
  if(delay_count > 2000) delay_count = 1 ;
}


// ---------------- OTHER FUNCTIONS --------------- 

float smoothBattery (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}

void setSpeed(int val) {
  analogWrite(FL_MOTOR, val);
  analogWrite(FR_MOTOR, val);
  analogWrite(BR_MOTOR, val);
  analogWrite(BL_MOTOR, val);
}

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

void runIndividual (int* actSpeed) {
  analogWrite(FL_MOTOR, actSpeed[0]);
  analogWrite(FR_MOTOR, actSpeed[1]);
  analogWrite(BR_MOTOR, actSpeed[2]);
  analogWrite(BL_MOTOR, actSpeed[3]);
}

void quadInit(){
Serial.begin(9600);
  Serial.println("INITIALIZING");
  

  //Enable internal reference of 1.1V
  //initialise battery level array with current battery level value
  pinMode(A0, INPUT);
  analogReference(INTERNAL);
  batt_level = analogRead(A0) / 1023.0 * MULTIPLIER;

  motorBattery = 3.7;
  Serial.print("Battery level");
  Serial.println(batt_level);

  //------------------------------PID----------------------------------
  //initialize the variables we're linked to
  pitchInput = 0.0;
  rollInput = 0.0;

  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;

  //turn the PID on
  // pitchPID.SetMode(AUTOMATIC);
  // rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
  //-------------------------------------------------------------------
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

  // initialize mySerial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  //mySerial.begin(9600);
  // Serial.println("Initializing the arduino");
  // while (!mySerial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  // Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  // Serial.println(" ");
  // Serial.println("Testing device connections...");
  // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // mySerial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (mySerial.available() && mySerial.read()); // empty buffer
  // while (!mySerial.available());                 // wait for data
  // while (mySerial.available() && mySerial.read()); // empty buffer again

  // load and configure the DMP
  // Serial.println(" ");
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devStatus);
    // Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);


}



//UNUSED CODE

    // yawInput   = ypr[0];  //compensates for the offset and converts in degrees
    // pitchInput = ypr[1];
    // rollInput  = ypr[2];

    // Serial.print("Yaw");
    // Serial.print(" ");
    // Serial.print("Pitch");
    // Serial.print(" ");
    // Serial.print("Roll");
    // Serial.print(" ");
    // Serial.print("Speed");
    // Serial.print(" ");
    // Serial.println("Battery level");

    // Serial.print(yawInput);
    // Serial.print(" ");
    // Serial.print(pitchInput);
    // Serial.print(" ");
    // Serial.print(rollInput);
    // Serial.print(" ");
    // Serial.print(pwm_out);
    // Serial.print(" ");
    // Serial.println(batt_level);

    // Serial.print(gravity.x);
    // Serial.print(gravity.y);
    // Serial.println(gravity.z);
    // Serial.println("");

    // Serial.print(q.x);//compensates for the offset and converts in degrees
    // Serial.print(q.y);
    // Serial.println(q.z);
    // Serial.println("");


    //----------------------------------------PID-----------------------------------------
    // if (myReading == 0) {
    //   Serial.println(F("CALIBRATING"));
    //   ypr_cal[0] = ypr[0] * 180 / M_PI;
    //   ypr_cal[1] = ypr[1] * 180 / M_PI;
    //   ypr_cal[2] = ypr[2] * 180 / M_PI;

    //   //              ypr_cal[1] = 1.26;
    //   //              ypr_cal[2] = -0.55;
    // }

    // pitchInput = ypr[1] * 180 / M_PI - ypr_cal[1];
    // rollInput = ypr[2] * 180 / M_PI - ypr_cal[2];





    //            pitchInput /= 2.0;
    //            rollInput /= 2.0;

//     pitchPID.Compute();
//     rollPID.Compute();

//     int actSpeed[4];
//     stabilise (targetSpeed, actSpeed, rollOutput, pitchOutput);
// //    targetSpeed = actSpeed; // should this behere or not?

//     Serial.print(F("pitchInput="));
//     Serial.print(pitchInput);
//     Serial.print(F("   pitchOutput="));
//     Serial.print(pitchOutput);

//     Serial.print(F("   rollInput="));
//     Serial.print(rollInput);
//     Serial.print(F("   rollOutput="));
//     Serial.print(rollOutput);

//     Serial.print(F("   mot[0]="));
//     Serial.print(actSpeed[0]);
//     Serial.print(F("   mot[1]="));
//     Serial.print(actSpeed[1]);
//     Serial.print(F("   mot[2]="));
//     Serial.print(actSpeed[2]);
//     Serial.print(F("   mot[3]="));
//     Serial.println(actSpeed[3]);

//     runIndividual (actSpeed);
//                checkIndividual(myReading, actSpeed);
//     ------------------------------------------------------------------------------------
//     motorBattery = smoothBattery(motorBattery, analogRead(A0) / 1023.0 * MULTIPLIER, ALPHA);
//     if (motorBattery < 2.0){
//       mySerial.println (F("WARNING! LOW BATTERY!"));
//     }
//     mySerial.print(motorBattery);
//     mySerial.print(F("   ypr   "));
//     mySerial.print(ypr[0] * 180 / M_PI);
//     mySerial.print("   ");
//     mySerial.print(ypr[1] * 180 / M_PI);
//     mySerial.print("   ");
//     mySerial.println(ypr[2] * 180 / M_PI);


  //   // blink LED to indicate activity
  //   blinkState = 1;
  //   digitalWrite(LED_PIN, blinkState);
