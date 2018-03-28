/*
   _____________   __    _____ ___________  _____ _   _ _____ _____  ____________ _____  ___   _   __ ___________  _____
  /  ___| ___ \ \ / /   /  __ \_   _| ___ \/  __ \ | | |_   _|_   _| | ___ \ ___ \  ___|/ _ \ | | / /|  ___| ___ \/  ___|
  \ `--.| |_/ /\ V /    | /  \/ | | | |_/ /| /  \/ | | | | |   | |   | |_/ / |_/ / |__ / /_\ \| |/ / | |__ | |_/ /\ `--.
   `--. \  __/ /   \    | |     | | |    / | |   | | | | | |   | |   | ___ \    /|  __||  _  ||    \ |  __||    /  `--. \
  /\__/ / |   / /^\ \   | \__/\_| |_| |\ \ | \__/\ |_| |_| |_  | |   | |_/ / |\ \| |___| | | || |\  \| |___| |\ \ /\__/ /
  \____/\_|   \/   \/    \____/\___/\_| \_| \____/\___/ \___/  \_/   \____/\_| \_\____/\_| |_/\_| \_/\____/\_| \_|\____/
  Author: @RKouchoo
  Emotional support: @BBlkBoi
  
  Last major edit:
    March 28, 2018
  Written for SPX robotics
  Written in sublime with deviot.

  Robot motor layout diagram
  
  1, 2, 3, 4 are the corresponding motor names in the code.
  
    3   back  4
  right      left
       / - \
    2  mouth  1

  FYI colour = color.. ;) (American libraries...)

  TODO List:
   - Implement gyro code.
   - Implement RF communication.
   - field is spelt as feild (auto fillout)
  
  RF204L01 module guide:
  https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
*/

// "Stock" libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// "Custom" libraries
#include <Pixy.h>
//#include <Adafruit.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_VL53L0X.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#define SERIAL_BANDWIDTH 9600
#define NEO_PIXEL_PER_ROBOT 16
#define DUAL_NEO_PIN 12
#define RF_PIN 0

#define GYRO_LOW_ADDRES = 0x68 // depends on how I configure the robot.
#define GYRO_HIGH_ADDRES = 0x69


Pixy pixy; // Create a pixy object
Adafruit_NeoPixel dualStrip = Adafruit_NeoPixel(NEO_PIXEL_PER_ROBOT, DUAL_NEO_PIN, NEO_GRB + NEO_KHZ800); // create the object for interfacing both of the LED bars.
Adafruit_VL53L0X timeOfFlight = Adafruit_VL53L0X(); // Create a time of flight sensor object.
MPU6050 gyro = MPU6050();

/*
 * Gyro variables
 */
// accelerometer values
int accel_offset = 200;
float accel_scale = 1; // set to 0.01

// gyro values
int gyro_offset = 151; // 151
float gyro_scale = 0.02; // 0.02 by default - tweak as required
float angle = 0.00; // value to hold final calculated gyro angle

// time stamp variables
float loop_time = 0.05; // 50ms loop
int last_update;
int cycle_time;
long last_cycle = 0;

/*
* Pixy variables
*/
int objectChoiceSignature = 0;
unsigned int objectXPos = 0;           //positon x axis
unsigned int objectYPos = 0;           //position y axis
unsigned int object_width = 0;         //object's width
unsigned int object_height = 0;        //object's height
unsigned int object_area = 0;
unsigned int object_newArea = 0;
unsigned int Xmin = 70;                //min x position
unsigned int Xmax = 200;               //max x position
unsigned int maxArea = 0;
unsigned int minArea = 0;

/*
 *  Hardware variables
 *  NEED TO BE UPDATED AS SOON AS HARDWARE FINISHED
 */
static int MOTOR_ONE[2] = {1, 2}; // forward and backwards controller channels `.
static int MOTOR_TWO[2] = {1, 2};
static int MOTOR_THREE[2] = {1, 2};
static int MOTOR_FOUR[2] = {1, 2};

static int MOTOR_ONE_PWM = 1;
static int MOTOR_TWO_PWM = 1;
static int MOTOR_THREE_PWM = 1;
static int MOTOR_FOUR_PWM = 1;

static int LIGHT_SENSOR_LEFT = 0;
static int LIGHT_SENSOR_RIGHT = 0;
static int LIGHT_SENSOR_FRONT = 0;
static int LIGHT_SENSOR_BACK = 0;

static int lightSensors[4] = {LIGHT_SENSOR_LEFT, LIGHT_SENSOR_RIGHT, LIGHT_SENSOR_FRONT, LIGHT_SENSOR_BACK};

static int NOT_FOUND_COLOR_DATA[3] = {255, 0, 0}; // dark red
static int FOUND_COLOR_DATA[3] = {255, 105, 180}; // hot pink
static int REFLECTIVE_COLOR_DATA[3] = {250, 250, 210}; // At the moment this is bright yellow.
static int LOCAL_ROBOT_ERROR_COLOR_DATA[3] = {237, 148, 90}; // error orange, should be changed later so the robot does not break the rules lol.

/*
 * The arrays that collect the data for automated setup routines.
 */
static int pwms[4] = {MOTOR_ONE_PWM, MOTOR_TWO_PWM, MOTOR_THREE_PWM, MOTOR_FOUR_PWM};
static int motors[2][4] = {
  {MOTOR_ONE[0], MOTOR_TWO[0], MOTOR_THREE[0], MOTOR_FOUR[0]},  // forward channels
  {MOTOR_ONE[1], MOTOR_TWO[1], MOTOR_THREE[1], MOTOR_FOUR[1]}  // backward channels
};

/*
 * Camera object ID's to track
 */
#define CMYK_CYAN_GOAL_ID 2
#define CMYK_YELLOW_GOAL_ID 3
#define CMYK_ORGANGE_BALL_ID 1

#define FEILD_BLACK_LINE_RATIO 50
#define FEILD_WHITE_LINE_RATIO 50
#define FEILD_GREEN_SURFACE_RATIO 50

enum cameraTrackingObject {
  CMYK_CYAN_GOAL,
  CMYK_YELLOW_GOAL,
  CMYK_ORANGE_BALL
};

enum thisMotorDirection {
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_STOP
};

enum thisRobotDirection {
  // Basic movements
  ROBOT_FORWARD,
  ROBOT_BACKWARD,

  // Left movements
  ROBOT_CRAB_LEFT,
  ROBOT_STRAFE_LEFT,
  ROBOT_STRAFE_LEFT_BACKWARD,
  ROBOT_ROTATE_LEFT,

  // Right movements
  ROBOT_CRAB_RIGHT,
  ROBOT_STRAFE_RIGHT,
  ROBOT_STRAFE_RIGHT_BACKWARD,
  ROBOT_ROTATE_RIGHT,
  
  ROBOT_STOP
};

enum thisFoundFeildObject {
  FEILD_BLACK_LINE,
  FEILD_WHITE_LINE,
  FEILD_GREEN_SURFACE
};

enum dualStripColor {
  NOT_FOUND_COLOR,
  FOUND_COLOR,
  REFLECTIVE_COLOR,
  LOCAL_ROBOT_ERROR_COLOR
};

/*
 * Takes an array of motor IDS and then creates all of the motor outputs
 */
void initMotorConfig(int motorList[2][4]) {
  int firstLength = 2;
  int secondLength = 4;

  for (int i = 0; i < firstLength; i++) {
    for (int j = 0; j < secondLength; j++) {
      if (!motorList[i][j]) {
        pinMode(OUTPUT, motorList[i][j]);
        Serial.println("Registered motors: " + motorList[i][j]);
      }
    }
  }
}

/*
 * configuration routine for pwm channels
 */
void initMotorPwmConfig(int pwmChannel[4]) {
  for (int i = 0; i < 4; i ++) {
    pinMode(OUTPUT, pwmChannel[i]);
  }
}

/*
 * routine to set up the sensor pins as an input.
 */
void initLightSensorConfig(int sensors[4]) {
  for (int i = 0; i < 4; i ++) {
    pinMode(INPUT, sensors[i]);
  }
}

/*
 * returns the current value of the selected sensor.
 */
double getLightSensorValue(int sensor) {
  return analogRead(sensor);
}

/*
 * returns the found object below the robot.
 * Will not 
 */
thisFoundFeildObject getCurrentFeildObject(double sensorValue) {
  if (sensorValue <= FEILD_BLACK_LINE_RATIO) {
    return FEILD_BLACK_LINE;
  } if (sensorValue <= FEILD_WHITE_LINE_RATIO) {
    return FEILD_WHITE_LINE;
  } if (sensorValue <= FEILD_GREEN_SURFACE_RATIO) {
    return FEILD_GREEN_SURFACE;
  } else {
    return FEILD_GREEN_SURFACE;
  }
}

/*
 * Routine for setting the speed for all of the motors on the robot.
 */
void setRobotSpeed(double motorSpeed, int pwmChannel[4]) {
  for (int i = 0; i < 4; i ++) {
    analogWrite(pwmChannel[i], motorSpeed);
  }
}

/*
 * hardware ish level of code development
 */
void setMotorDirection(thisMotorDirection motorDirection, int motorId[2], double motorSpeed) {
  setRobotSpeed(motorSpeed, pwms);

  switch (motorDirection) {
    case MOTOR_FORWARD:
      digitalWrite(motorId[1], LOW);
      digitalWrite(motorId[0], HIGH);
    break;

    case MOTOR_BACKWARD:
      digitalWrite(motorId[0], LOW);
      digitalWrite(motorId[1], HIGH);
    break;

    case MOTOR_STOP:
      digitalWrite(motorId[0], LOW);
      digitalWrite(motorId[1], LOW);
    break;

    default:
      setMotorDirection(MOTOR_STOP, motorId, motorSpeed); // The default command is motor stopped.
  }
}

/*
 * routine that sets the robot direction and speed.
 * contains the logic that tells the motor exactly what to do.
 */
void setRobotDirection(thisRobotDirection robotDirection, double robotSpeed) {

  switch (robotDirection) {
    case ROBOT_FORWARD:
      setMotorDirection(MOTOR_FORWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_BACKWARD:
      setMotorDirection(MOTOR_BACKWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_FOUR, robotSpeed);
    break;

    /*
     * left movements
     */
    case ROBOT_CRAB_LEFT:
      setMotorDirection(MOTOR_BACKWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_STRAFE_LEFT:
      setMotorDirection(MOTOR_STOP, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_STRAFE_LEFT_BACKWARD:
      setMotorDirection(MOTOR_STOP, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_ROTATE_LEFT:
      setMotorDirection(MOTOR_FORWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_FOUR, robotSpeed);
    break;

    /*
     * right movements
     */
    case ROBOT_CRAB_RIGHT:
      setMotorDirection(MOTOR_FORWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_STRAFE_RIGHT:
      setMotorDirection(MOTOR_FORWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_STRAFE_RIGHT_BACKWARD:
      setMotorDirection(MOTOR_BACKWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_ROTATE_RIGHT:
      setMotorDirection(MOTOR_BACKWARD, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_BACKWARD, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_FORWARD, MOTOR_FOUR, robotSpeed);
    break;

    case ROBOT_STOP:
      setMotorDirection(MOTOR_STOP, MOTOR_ONE, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_TWO, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_THREE, robotSpeed);
      setMotorDirection(MOTOR_STOP, MOTOR_FOUR, robotSpeed);
    break;
  }

}

/*
 * gets the pixy to update what objects are around it.
 */
void scanObjects(cameraTrackingObject object) {
int objectId;

// Choose the object to track based on what is fed in from the loop.
switch (object) {
  case CMYK_CYAN_GOAL:
    objectId = CMYK_CYAN_GOAL_ID;
  break;

  case CMYK_YELLOW_GOAL:
    objectId = CMYK_YELLOW_GOAL_ID;
  break;

  case CMYK_ORANGE_BALL:
    objectId = CMYK_ORGANGE_BALL_ID;
  break;

  default:
    objectId = CMYK_ORGANGE_BALL_ID;
  break;

}
  // CLANG: why is ths an error!? v
  uint16_t blocks = pixy.getBlocks();                       // get the data from the pixy.
  objectChoiceSignature = pixy.blocks[objectId].signature;  // get object's signature
  objectXPos = pixy.blocks[objectId].x;                     // get x position
  objectYPos = pixy.blocks[objectId].y;                     // get y position
  object_width = pixy.blocks[objectId].width;               // get width
  object_height = pixy.blocks[objectId].height;             // get height
}

/*
 * Scales the speed of the motors based on object Distance
 * This needs to be fixed! should not work the first time!.
 */
double calculateRobotSpeed(int objectDistance, int maxObjectDistance) {
  int div = objectDistance / maxObjectDistance; // stupid. need to get the actual distance of the ball from the robot.
  return map(div, 0, 1000, 0, 255);
}

 /*
  * Updated the led strips
  */
void updateDualStrip() {
  dualStrip.show();
}

 /*
  * Sets the led strip color before it is flushed.
  */
void setDualStripColor(int r, int g, int b) {
  for(int i = 0; i < NEO_PIXEL_PER_ROBOT; i ++) {
    dualStrip.setPixelColor(i, r, g, b);
    dualStrip.setBrightness(255); // Sets the brightness to full
  }
}


void setDualStripColor(dualStripColor color) {
  switch(color) {
    case NOT_FOUND_COLOR:
      setDualStripColor(NOT_FOUND_COLOR_DATA[0], NOT_FOUND_COLOR_DATA[1], NOT_FOUND_COLOR_DATA[2]);
    break;

    case FOUND_COLOR:
      setDualStripColor(FOUND_COLOR_DATA[0], FOUND_COLOR_DATA[1], FOUND_COLOR_DATA[2]);
    break;

    case REFLECTIVE_COLOR:
      setDualStripColor(REFLECTIVE_COLOR_DATA[0], REFLECTIVE_COLOR_DATA[1], REFLECTIVE_COLOR_DATA[2]);
    break;

    case LOCAL_ROBOT_ERROR_COLOR:
      setDualStripColor(LOCAL_ROBOT_ERROR_COLOR_DATA[0], LOCAL_ROBOT_ERROR_COLOR_DATA[1], LOCAL_ROBOT_ERROR_COLOR_DATA[2]);
    break;
  }

  updateDualStrip(); // push the new values to the strip's after the values have been set.
}

VL53L0X_RangingMeasurementData_t measureTOFDistance() { // gets the measurement object from the adafruit library.
  VL53L0X_RangingMeasurementData_t measurementObject;
  timeOfFlight.rangingTest(&measurementObject, false);  
  return measurementObject;
}

double getTOFDistanceMilli(VL53L0X_RangingMeasurementData_t measurementObject) { // gets the distance from the measured object to the robot.
  if (measurementObject.RangeStatus  == 4) {
    Serial.println("12C TOF SENSOR ERROR");
    setDualStripColor(LOCAL_ROBOT_ERROR_COLOR);
    return 0;
  } else {
    return measurementObject.RangeMilliMeter;
  }
}

float getGyroAngle() { // gets the angle that the gyro is facing
  
  int gyro_corrected;
  int gyro_reading;
  float gyro_rate;
  float accel_angle;
  float gyro_angle;
  int accel_reading;
  int accel_corrected;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accel_reading = ax;
  accel_corrected = accel_reading - accel_offset;
  accel_corrected = map(accel_corrected, -16800, 16800, -90, 90);
  accel_corrected = constrain(accel_corrected, -90, 90);
  accel_angle = (float)(accel_corrected * accel_scale);

  gyro_reading = gy;
  gyro_corrected = (float)((gyro_reading/131) - gyro_offset);  // 131 is sensivity of gyro from data sheet
  gyro_rate = (gyro_corrected * gyro_scale) * -loop_time;      // loop_time = 0.05 ie 50ms        
  gyro_angle = angle + gyro_rate;

  return gyro_angle;
}

void timeStamp() {
  while ((millis() - last_cycle) < 50) {
    delay(1);
  }
  // once loop cycle reaches 50ms, reset timer value and continue
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}

void threadRunner() {
    scanObjects(CMYK_ORANGE_BALL);

    object_area = object_width * object_height; //calculate the object area
    maxArea = object_area + 1000;
    minArea = object_area - 1000;

    // Check if the pixy has the proper object lock.
    if (objectChoiceSignature == CMYK_ORANGE_BALL) {
      object_newArea = object_width * object_height;  //calculate the object area

      // calculate the speed for the robot
      double speed = calculateRobotSpeed(object_newArea, maxArea);

      if (objectXPos < Xmin) {                        // turn left if x position < max x position
        setRobotDirection(ROBOT_STRAFE_LEFT, speed);
      } else if (objectXPos > Xmax) {                 // turn right if x position > max x position
        setRobotDirection(ROBOT_STRAFE_RIGHT, speed);
      } else if (object_newArea < minArea) {          // go forward if object too small
        setRobotDirection(ROBOT_FORWARD, speed);
      } else if (object_newArea > maxArea) {          // go backward if object too big
        setRobotDirection(ROBOT_BACKWARD, speed);
      } else {
        setRobotDirection(ROBOT_STOP, LOW);           // stop the robot if it has lost sight of the ball.
      }
    } else {
        // Code to tell the other robot to step in, for now just look for the ball.
        scanObjects(CMYK_ORANGE_BALL);
    }
}

/////////////////////////////////////////////SETUP/////////////////////////////////////////////
void setup() {
  // Start serial communication
  Serial.begin(SERIAL_BANDWIDTH);

  Wire.begin();

  // set up hardware
  pixy.init();
  gyro.initialize();
  initMotorPwmConfig(pwms);
  initMotorConfig(motors);
  initLightSensorConfig(lightSensors);

  dualStrip.begin();
  dualStrip.show(); // Initialize all pixels to 'off'

  if (!timeOfFlight.begin()) { // only complain if its not working.
    Serial.println(F("Failed to boot VL53L0X"));
    setDualStripColor(LOCAL_ROBOT_ERROR_COLOR);
  }

  if (!gyro.testConnection()) { // only complain if its not working.
    Serial.println(F("Failed to boot MPU6050"));
    setDualStripColor(LOCAL_ROBOT_ERROR_COLOR); 
  }

  // make sure the robot is not moving until loop runs
  setRobotDirection(ROBOT_STOP, 0);
}

/////////////////////////////////////////////LOOP/////////////////////////////////////////////
void loop() {
  // run the thread
  threadRunner();

  // calc the time
  timeStamp();
}
