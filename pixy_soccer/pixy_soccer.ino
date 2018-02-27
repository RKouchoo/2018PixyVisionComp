/*
   _____________   __    _____ ___________  _____ _   _ _____ _____  ____________ _____  ___   _   __ ___________  _____
  /  ___| ___ \ \ / /   /  __ \_   _| ___ \/  __ \ | | |_   _|_   _| | ___ \ ___ \  ___|/ _ \ | | / /|  ___| ___ \/  ___|
  \ `--.| |_/ /\ V /    | /  \/ | | | |_/ /| /  \/ | | | | |   | |   | |_/ / |_/ / |__ / /_\ \| |/ / | |__ | |_/ /\ `--.
  `--. \  __/ /   \    | |     | | |    / | |   | | | | | |   | |   | ___ \    /|  __||  _  ||    \ |  __||    /  `--. \
  /\__/ / |   / /^\ \   | \__/\_| |_| |\ \ | \__/\ |_| |_| |_  | |   | |_/ / |\ \| |___| | | || |\  \| |___| |\ \ /\__/ /
  \____/\_|   \/   \/    \____/\___/\_| \_| \____/\___/ \___/  \_/   \____/\_| \_\____/\_| |_/\_| \_/\____/\_| \_|\____/
  Author: @RKouchoo

*/

#include <SPI.h>
#include <Pixy.h>

Pixy pixy; // Create a pixy object

/**
   Pixy variables
*/
int object_signature = 0;
int object_x = 0;                      //positon x axis
int object_y = 0;                      //position y axis
unsigned int object_width = 0;         //object's width
unsigned int object_height = 0;        //object's height
unsigned int object_area = 0;
unsigned int object_newArea = 0;
int Xmin = 70;                  //min x position
int Xmax = 200;                 //max x position
int maxArea = 0;
int minArea = 0;

/**
   Hardware variables
*/
int motor1 = 4;                 //motor1 on Pin D4
int enable1 = 5;                //enable1 on Pin D5
int motor2 = 7;                 //motor2 on Pin D7
int enable2 = 6;                //enable2 on Pin D6
int Speed = 70;                 //speed for motor

static unsigned int MOTOR_ONE[2] = {1, 2}; // forward and backwards controller channels `.
static unsigned int MOTOR_TWO[2] = {1, 2};
static unsigned int MOTOR_THREE[2] = {1, 2};
static unsigned int MOTOR_FOUR[2] = {1, 2};

static unsigned int MOTOR_ONE_PWM = 1;
static unsigned int MOTOR_TWO_PWM = 1;
static unsigned int MOTOR_THREE_PWM = 1;
static unsigned int MOTOR_FOUR_PWM = 1;

/*
 * The arrays that handle the
 * 
  */

static int pwms[4] = {MOTOR_ONE_PWM, MOTOR_TWO_PWM, MOTOR_THREE_PWM, MOTOR_FOUR_PWM};
static int motors[2][4] = { {MOTOR_ONE[0], MOTOR_TWO[0], MOTOR_THREE[0], MOTOR_FOUR[0]}, {MOTOR_ONE[1], MOTOR_TWO[1], MOTOR_THREE[1], MOTOR_FOUR[1]} };

/**
  Camera object ID's to track
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
  ROBOT_FORWARD,
  ROBOT_BACKWARD,
  ROBOT_LEFT,
  ROBOT_STRAFE_LEFT,
  ROBOT_RIGHT,
  ROBOT_STRAFE_RIGHT,
  ROBOT_STOP
};

/*
 * Takes an array of motor IDS and then creates all of the motor outputs
 */
void initMotorConfig(int motorList[2][4]) {
  static int firstLength = 2;
  static int secondLength = 4;
  for (int i = 0; i < firstLength; i++) {
    for (int j = 0; j < secondLength; j++) {
      if (!motorList[i][j]) {
        pinMode(OUTPUT, motorList[i][j]);
      }
    }
  }
}

void initMotorPwmConfig(int pwmChannel[4]) {
  for (int i = 0; i < 4; i ++) {
    pinMode(OUTPUT, pwmChannel[i]);
  }
}

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
void setRobotSpeed(double motorSpeed, int pwmChannel[4]) {
  for (int i = 0; i < 4; i ++) {
    analogWrite(pwmChannel[i], motorSpeed);
  }
}

/*
 * Sets the robot direction and speed
 */
void setRobotDirection(thisRobotDirection robotDirection, double robotSpeed) {
 
  switch (robotDirection) {
    case ROBOT_FORWARD:
      // Move forward
    break;

    case ROBOT_BACKWARD:
      // Move backward
    break;

   case ROBOT_LEFT:
      // move left
    break;

    case ROBOT_STRAFE_LEFT:
      // Strafe left
    break;
  
    case ROBOT_RIGHT:
      // move right
    break;
    
    case ROBOT_STRAFE_RIGHT:
      // Strafe right
    break;

    case ROBOT_STOP:
      // Stop the robot
    break;
  }
  
}

void scanObjects() {
  int i = 1;
  uint16_t blocks;
  blocks = pixy.getBlocks();  //receive data from pixy
  object_signature = pixy.blocks[i].signature;    //get object's signature
  object_x = pixy.blocks[i].x;                    //get x position
  object_y = pixy.blocks[i].y;                    //get y position
  object_width = pixy.blocks[i].width;            //get width
  object_height = pixy.blocks[i].height;          //get height
}


void setup() {
  Serial.begin(9600);
  initMotorConfig(motors);
  initMotorPwmConfig(pwms);
  pixy.init();
  setRobotDirection(ROBOT_STOP, LOW);
}

void loop() {
  
  scanObjects();
  
  object_area = object_width * object_height; //calculate the object area
  maxArea = object_area + 1000;
  minArea = object_area - 1000;
  
  if (object_signature == CMYK_ORANGE_BALL) {
    object_newArea = object_width * object_height; //calculate the object area
    if (object_x < Xmin) { //turn left if x position < max x position
      setRobotDirection(ROBOT_STRAFE_LEFT, 200);
    } else if (object_x > Xmax) { //turn right if x position > max x position
      setRobotDirection(ROBOT_STRAFE_RIGHT, 200);
    } else if (object_newArea < minArea) { //go forward if object too small
      setRobotDirection(ROBOT_FORWARD, 200);
    } else if (object_newArea > maxArea) { //go backward if object too big
      setRobotDirection(ROBOT_BACKWARD, 200);
    } else {
      setRobotDirection(ROBOT_STOP, LOW);
    }
  } else {
      setRobotDirection(ROBOT_STOP, LOW);
  }
}
