/*
   _____________   __    _____ ___________  _____ _   _ _____ _____  ____________ _____  ___   _   __ ___________  _____
  /  ___| ___ \ \ / /   /  __ \_   _| ___ \/  __ \ | | |_   _|_   _| | ___ \ ___ \  ___|/ _ \ | | / /|  ___| ___ \/  ___|
  \ `--.| |_/ /\ V /    | /  \/ | | | |_/ /| /  \/ | | | | |   | |   | |_/ / |_/ / |__ / /_\ \| |/ / | |__ | |_/ /\ `--.
   `--. \  __/ /   \    | |     | | |    / | |   | | | | | |   | |   | ___ \    /|  __||  _  ||    \ |  __||    /  `--. \
  /\__/ / |   / /^\ \   | \__/\_| |_| |\ \ | \__/\ |_| |_| |_  | |   | |_/ / |\ \| |___| | | || |\  \| |___| |\ \ /\__/ /
  \____/\_|   \/   \/    \____/\___/\_| \_| \____/\___/ \___/  \_/   \____/\_| \_\____/\_| |_/\_| \_/\____/\_| \_|\____/
  Author: @RKouchoo
  SubAuthor: @Oliver
  Emotional support: @BBlkBoi (James)

  Last major edit:
    August 8, 2018
  Written for SPX robotics
  Written in sublime with deviot and a mix an mash of other IDE's....

  Robot motor layout diagram

  1, 2, 3, 4 are the corresponding motor names in the code.

    3   back  4
  right      left
       / - \
    2  mouth  1

  FYI colour = color.. ;) (American libraries...)

  TODO List:
   - Implement gyro code.
   - Implement RF communication. (Being worked on)
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
#include <Adafruit_VL53L0X.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <I2Cdev.h>
#include <QMC5883L.h>
#include <LiquidCrystal_I2C.h>

#define ROBOT_IS_MASTER true ///////////////////////// IMPORTANT FOR RF MASTER/SLAVE, INCLUDE MENU SCREEN or JUMPER???

#define SERIAL_BANDWIDTH 9600
#define RF_PIN 22
#define RF_CE 7 ///////////////////////////////////// PLEASE CHECK WITH PINS ON MODULE AND MEGA
#define RF_CSN 8 ///////////////////////////////////// PLEASE CHECK WITH PINS ON MODULE AND MEGA

#define FRAME_SKIP 1 // frames to skip in the next loop.

#define BALL_TRACKING_NUM 0

Pixy pixy; // Create a pixy object
Adafruit_VL53L0X timeOfFlight = Adafruit_VL53L0X(); // Create a time of flight sensor object.
LiquidCrystal_I2C display(0x3F, 16, 2);
QMC5883L compass;

static int frameCount = 0;
int cameraWatchDogCount = 0;
static int cameraWatchDogCountMax = 25; // 1000 failed tries of connecting to the camera or sensing the ball.
boolean isCameraFlipped = true; // the camera is mounted upside down on the robots.
uint16_t currentObjectSignature = 0;

// TODO flip these if the camera is flipped!
static int xMaxCoord = 319; // 1 is the respective minimum for both coordinates
static int xMidCoord = 159; // almost half way
static int xCentreZone = 30; // 30 pixels where the centre zone of the robot is
// keft and right threshold cutoff values
static int xLeft = xMaxCoord - xCentreZone / 2;
static int xRight = xMaxCoord - xCentreZone / 2;

// not needed at the moment as we can base movement off the x axis
static int yMaxCoord = 199;
static int yMidCoord = 100;

// time stamp variables
float loop_time = 0.05; // 50ms loop
int last_update;
int cycle_time;
long last_cycle = 0;

/**
 *  Hardware variables
 *  NEED TO BE UPDATED AS SOON AS HARDWARE FINISHED
 */
static int MOTOR_ONE[2] = {1, 2}; // forward and backwards controller channels `.
static int MOTOR_TWO[2] = {1, 2};
static int MOTOR_THREE[2] = {1, 2};
static int MOTOR_FOUR[2] = {1, 2};

static int MOTOR_ONE_PWM = 8;
static int MOTOR_TWO_PWM = 9;
static int MOTOR_THREE_PWM = 10;
static int MOTOR_FOUR_PWM = 11;

/**
 * The arrays that collect the data for automated setup routines.
 */
static int pwms[4] = {MOTOR_ONE_PWM, MOTOR_TWO_PWM, MOTOR_THREE_PWM, MOTOR_FOUR_PWM};
static int motors[2][4] = {
 {MOTOR_ONE[0], MOTOR_TWO[0], MOTOR_THREE[0], MOTOR_FOUR[0]},  // forward channels
 {MOTOR_ONE[1], MOTOR_TWO[1], MOTOR_THREE[1], MOTOR_FOUR[1]}  // backward channels
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

/**
 * Takes an array of motor IDS and then creates all of the motor outputs
 */
void initMotorConfig(int motorList[2][4]) {
  int firstLength = 2;
  int secondLength = 4;

  for (int i = 0; i < firstLength; i++) {
    for (int j = 0; j < secondLength; j++) {
      if (!motorList[i][j]) {
        pinMode(OUTPUT, motorList[i][j]);
        Serial.println("Registered motor: " + motorList[i][j]);
      }
    }
  }
}

/**
 * configuration routine for pwm channels
 */
void initMotorPwmConfig(int pwmChannel[4]) {
  for (int i = 0; i < 4; i++) {
    pinMode(OUTPUT, pwmChannel[i]);
  }
}

/**
 * Routine for setting the speed for all of the motors on the robot.
 */
void setRobotSpeed(double motorSpeed, int pwmChannel[4]) {
  for (int i = 0; i < 4; i++) {
    analogWrite(pwmChannel[i], motorSpeed);
  }
}

/**
 * ardunio native calls for motor movement.
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

/**
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

VL53L0X_RangingMeasurementData_t measureTOFDistance() { // gets the measurement object from the adafruit library.
  VL53L0X_RangingMeasurementData_t measurementObject;
  timeOfFlight.rangingTest(&measurementObject, false);
  return measurementObject;
}

double getTOFDistanceMilli(VL53L0X_RangingMeasurementData_t measurementObject) { // gets the distance from the measured object to the robot.
  if (measurementObject.RangeStatus == 4) {
    Serial.println("12C TOF SENSOR ERROR");
    return 0;
  } else {
    return measurementObject.RangeMilliMeter;
  }
}

// DOES NOT WORK!
int calcRobotSpeed() {
  return 200; // medium to high speed, will work and we dont need to modulate speed anyway
}

// TODO prams need to be tuned at home and on the feild !
boolean processCoords(int width, int height, int x, int y) {
  int maxWidth = 70;
  int maxHeight = 70;
  int minWidth = 30;
  int minHeight = 30;

  if (width < maxWidth && width > minWidth && height < maxHeight && height > minHeight) {
    return true;
  } else {
    return false;
  }
}

boolean handleRobotMovement(int speed, uint16_t signature) {
  currentObjectSignature = signature;

  if (currentObjectSignature != BALL_TRACKING_NUM) {
    Serial.println("Invalid target found..");
    return;
  }

  int x = pixy.blocks[BALL_TRACKING_NUM].x;
  int y = pixy.blocks[BALL_TRACKING_NUM].y;
  // dont really need to be used
  int width = pixy.blocks[BALL_TRACKING_NUM].width;
  int height = pixy.blocks[BALL_TRACKING_NUM].height;

  // check if the pixy has locked to a valid object
  // can cause issues!
  if (processCoords(width, height, x, y)) {
    // the ball is on the left side
    if (x < xLeft) {
      if (x < xLeft / 2) { // if the ball is in the lower half of the left fov then crab
        // crab left
        setRobotDirection(ROBOT_CRAB_LEFT, speed);
      } else {
        // forward diag left till the ball goes into the middle dead zone.
        setRobotDirection(ROBOT_STRAFE_LEFT, speed);
      }

    }

    // if the ball is in the right size
    if (x < xRight) { // if the ball is on the right side of the dead zone
      if (x < xRight / 2) { // check if its in the lower right side of the dead zone
        // crab right
        setRobotDirection(ROBOT_CRAB_RIGHT, speed);
      } else {
        // forward diag right towards the ball into the dead zone
        setRobotDirection(ROBOT_STRAFE_RIGHT, speed);
      }
    }

    // if the ball is in the centre deadzone
    if (x > xLeft && x < xRight) {
      // before moving forward, amybe check if the robot has the correct heading and has the ball with the TOF sensor.
      setRobotDirection(ROBOT_FORWARD, speed);
    }
  } else {
    setRobotDirection(ROBOT_STOP, speed);
    return false;
  }
}

// after getting the compass data, convert it to a heading
int getCompassHeading() {
  return compass.readHeading();
}

void threadRunner() {
  uint16_t pixyBlocks = pixy.getBlocks(); // get the object that the pixy can see.
  int speed = calcRobotSpeed();

  float compassHeading = getCompassHeading();

  if (!handleRobotMovement(speed, pixyBlocks)) {
      cameraWatchDogCount ++;
  }

  if (cameraWatchDogCount == cameraWatchDogCountMax) {
    Serial.println("Pixy has no blocks !");
  }
}

/////////////////////////////////////////////RADIO/////////////////////////////////////////////

// Basic protocol for Bi-Directional Communications

unsigned int cbRFPulse = 0;

typedef struct cbRFPacket {
  boolean robotIsMaster = ROBOT_IS_MASTER;
  int orientation;
  boolean ballVisible;
  boolean hasBall;
};

RF24 radio(RF_CE, RF_CSN);
const uint64_t rfPipes[2] = {0xF0F0F0F0F011, 0xF0F0F0F022};

cbRFPacket txPacket;
cbRFPacket rxPacket;

void initCBRF() {
  radio.begin();
  //radio.setPALevel(RF24_PA_LOW);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(0x2A);
  if (ROBOT_IS_MASTER) {
    radio.openWritingPipe(rfPipes[0]);
    radio.openReadingPipe(1, rfPipes[1]);
  } else {
    radio.openWritingPipe(rfPipes[1]);
    radio.openReadingPipe(1, rfPipes[0]);
  }
  radio.powerUp();
  radio.startListening();
};

void cbRFTransmit() {
  radio.stopListening();
  // examples for txPacket information sending
  // txPacket.orientation = readOrientation()
  // txPacket.ballVisible = canPixySeeBall()
  // txPacket.hasBall = doIOwnTheBall()
  radio.write(&txPacket, sizeof(txPacket));
  radio.startListening();
}

void cbRFReceive() {
  radio.read(&rxPacket, sizeof(rxPacket));
  //Serial.println(rxPacket.robotIsMaster);
};

void cbRFThread() {
  // every 100ms assuming timestamp pulse produces program 'clock' of 50ms
  if (ROBOT_IS_MASTER) {
      if (cbRFPulse >= 2) {
        // >=2 means every second timestamp() pulse provided cbRFThread is called before timestamp(); else use >2 if after timestamp()
        cbRFPulse = 0;
        cbRFTransmit();
      }
      if (radio.available()) {
        cbRFReceive();
      }
  } else {
      if (radio.available()) {
        cbRFReceive();
        cbRFTransmit();
      }
  }
};

void timeStamp() {
  while ((millis() - last_cycle) < 50) {
    delay(1);
  }
  // once loop cycle reaches 50ms, reset timer value and continue
  cycle_time = millis() - last_cycle;
  last_cycle = millis();

  //call information transmission
  if (ROBOT_IS_MASTER) {
    cbRFPulse = cbRFPulse + 1;
  }
}

/////////////////////////////////////////////SETUP/////////////////////////////////////////////
void setup() {
  // Start communication interfaces
  Serial.begin(SERIAL_BANDWIDTH);
  Wire.begin();

  display.begin();
  pixy.init();
  timeOfFlight.begin();
  compass.init();

  // init "factory" devices
  initMotorPwmConfig(pwms);
  initMotorConfig(motors);
  initCBRF(); // Initialize Radios, dependant on ROBOT_IS_MASTER constant

  if (!timeOfFlight.begin()) { // only complain if its not working.
    Serial.println(F("Failed to boot VL53L0X"));
  }

  // make sure the robot is not moving until  loop runs
  setRobotDirection(ROBOT_STOP, 0);
}

/////////////////////////////////////////////LOOP/////////////////////////////////////////////
void loop() {
    display.print(getCompassHeading());

// run the thread
  threadRunner();

  // run the radio functions and listen/transmit data
  //cbRFThread();

  // calculate the time
  // every 2 times timeStamp() is called, the MASTER robot transmits to SLAVE robot and gets a response for CBRFPackets
  //timeStamp();
}
