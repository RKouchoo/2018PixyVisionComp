/*
   _____________   __    _____ ___________  _____ _   _ _____ _____  ____________ _____  ___   _   __ ___________  _____
  /  ___| ___ \ \ / /   /  __ \_   _| ___ \/  __ \ | | |_   _|_   _| | ___ \ ___ \  ___|/ _ \ | | / /|  ___| ___ \/  ___|
  \ `--.| |_/ /\ V /    | /  \/ | | | |_/ /| /  \/ | | | | |   | |   | |_/ / |_/ / |__ / /_\ \| |/ / | |__ | |_/ /\ `--.
   `--. \  __/ /   \    | |     | | |    / | |   | | | | | |   | |   | ___ \    /|  __||  _  ||    \ |  __||    /  `--. \
  /\__/ / |   / /^\ \   | \__/\_| |_| |\ \ | \__/\ |_| |_| |_  | |   | |_/ / |\ \| |___| | | || |\  \| |___| |\ \ /\__/ /
  \____/\_|   \/   \/    \____/\___/\_| \_| \____/\___/ \___/  \_/   \____/\_| \_\____/\_| |_/\_| \_/\____/\_| \_|\____/
  Author: @RKouchoo

  Last major edit:
    Feb 28, 2018

  Written for SPX robotics

*/

// Robot motor layout diagram
// 1, 2, 3, 4 are the corresponding motor names in the code.

/*
    3   back  4

  right      left
       / - \
    2  mouth  1
*/

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

#define SERIAL_BANDWIDTH 9600

Pixy pixy; // Create a pixy object

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
static int LIGHT_SENSOR_BACK = 0;

static int lightSensors[3] = {LIGHT_SENSOR_LEFT, LIGHT_SENSOR_RIGHT, LIGHT_SENSOR_BACK};

/*
 * The arrays that collect the data for automated setup routines.
 */
static int pwms[4] = {MOTOR_ONE_PWM, MOTOR_TWO_PWM, MOTOR_THREE_PWM, MOTOR_FOUR_PWM};
static int motors[2][4] = { {MOTOR_ONE[0], MOTOR_TWO[0], MOTOR_THREE[0], MOTOR_FOUR[0]}, {MOTOR_ONE[1], MOTOR_TWO[1], MOTOR_THREE[1], MOTOR_FOUR[1]} };

/*
 * Camera object ID's to track
 */
#define CMYK_CYAN_GOAL_ID 2
#define CMYK_YELLOW_GOAL_ID 3
#define CMYK_ORGANGE_BALL_ID 1

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
void initLightSensorConfig(int sensors[3]) {
  for (int i = 0; i < 3; i ++) {
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
* hardware ish level of code development
*/
void setMotorDirection(thisMotorDirection motorDirection, int motorId[2], double motorSpeed) {
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
      digitalWrite(motorId[0], LOW);
      digitalWrite(motorId[1], LOW);
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
  // CLANG: why is ths an error!?
  uint16_t blocks = pixy.getBlocks();                       // Get the data from the pixy.
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
* The setup routine for the the robot code.
*/
void setup() {
  // Start serial communication
  Serial.begin(SERIAL_BANDWIDTH);

  // set up hardware
  pixy.init();
  initMotorPwmConfig(pwms);
  initMotorConfig(motors);
  initLightSensorConfig(lightSensors);

  // make sure the robot is not moving until loop runs
  setRobotDirection(ROBOT_STOP, 0);
}


// The main loop that the robot runs at ~100Hz
void loop() {

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
