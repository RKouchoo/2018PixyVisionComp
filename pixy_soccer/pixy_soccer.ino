/*
   _____________   __    _____ ___________  _____ _   _ _____ _____  ____________ _____  ___   _   __ ___________  _____
  /  ___| ___ \ \ / /   /  __ \_   _| ___ \/  __ \ | | |_   _|_   _| | ___ \ ___ \  ___|/ _ \ | | / /|  ___| ___ \/  ___|
  \ `--.| |_/ /\ V /    | /  \/ | | | |_/ /| /  \/ | | | | |   | |   | |_/ / |_/ / |__ / /_\ \| |/ / | |__ | |_/ /\ `--.
  `--. \  __/ /   \    | |     | | |    / | |   | | | | | |   | |   | ___ \    /|  __||  _  ||    \ |  __||    /  `--. \
  /\__/ / |   / /^\ \   | \__/\_| |_| |\ \ | \__/\ |_| |_| |_  | |   | |_/ / |\ \| |___| | | || |\  \| |___| |\ \ /\__/ /
  \____/\_|   \/   \/    \____/\___/\_| \_| \____/\___/ \___/  \_/   \____/\_| \_\____/\_| |_/\_| \_/\____/\_| \_|\____/

  Author: @RKouchoo
  

*/

#include <Ardunio.h>
#include <SPI.h>
#include <Pixy.h>

Pixy pixy; // Create a pixy object

/**
 * Pixy variables
 */
int signature = 0;
int x = 0;                      //positon x axis
int y = 0;                      //position y axis
unsigned int width = 0;         //object's width
unsigned int height = 0;        //object's height
unsigned int area = 0;
unsigned int newarea = 0;
int Xmin = 70;                  //min x position
int Xmax = 200;                 //max x position
int maxArea = 0;
int minArea = 0;

/**
 * Hardware variables
 */
int motor1 = 4;                 //motor1 on Pin D4
int enable1 = 5;                //enable1 on Pin D5
int motor2 = 7;                 //motor2 on Pin D7
int enable2 = 6;                //enable2 on Pin D6
int Speed = 70;                 //speed for motor

final static int MOTOR_ONE[2] = {1, 2};
final static int MOTOR_TWO[2] = {1, 2};
final static int MOTOR_THREE[2] = {1, 2};
final static int MOTOR_FOUR[2] = {1, 2};

final static int MOTOR_INDEX_LIST[MOTOR_ONE][MOTOR_TWO][MOTOR_THREE][MOTOR_FOUR]; // No way this works..

#define i 0 // useless variable that needs to be changed.

/**
* Camera object ID's to track
*/

#define CMYK_CYAN_GOAL 2
#define CMYK_YELLOW_GOAL 3
#define CMYK_ORGANGE_BALL 1

static enum cameraObject {
   CMYK_CYAN_GOAL,
   CMYK_YELLOW_GOAL,
   CMYK_ORANGE_BALL
}

enum thisMotorDirection {
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_STRAFE_LEFT,
  MOTOR_STRAFE_RIGHT,
  MOTOR_STOPPED
};

void setMotorDirection(thisMotorDirection thisDirection) {
  
}

void initMotorConfig(int motorList[][][][]) {
   
}

void backward()//backward
{
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  analogWrite(enable1, Speed);
  analogWrite(enable2, Speed);
}

void forward()//forward
{
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, HIGH);
  analogWrite(enable1, Speed);
  analogWrite(enable2, Speed);
}

void right()//turn right
{
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, LOW);
  analogWrite(enable1, Speed);
  analogWrite(enable2, Speed);
}

void left()//turn left
{
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, HIGH);
  analogWrite(enable1, Speed);
  analogWrite(enable2, Speed);
}

void Stop() {
  digitalWrite(enable1, LOW);
  digitalWrite(enable2, LOW);
}

void scanObjects() {
  uint16_t blocks;
  blocks = pixy.getBlocks();  //receive data from pixy
  signature = pixy.blocks[i].signature;    //get object's signature
  x = pixy.blocks[i].x;                    //get x position
  y = pixy.blocks[i].y;                    //get y position
  width = pixy.blocks[i].width;            //get width
  height = pixy.blocks[i].height;          //get height
}


void setup()
{
  Serial.begin(9600);
  Stop();
  pixy.init();
}

void loop()
{
  while (millis() < 5000)
  {
    scanObjects();
    area = width * height; //calculate the object area
    maxArea = area + 1000;
    minArea = area - 1000;
  }

  scanObjects();

  if (signature == 2) //looking for signature 2
  {
    newarea = width * height; //calculate the object area

    if (x < Xmin)//turn left if x position < max x position
    {
      left();
    }
    else if (x > Xmax) //turn right if x position > max x position
    {
      right();
    }
    else if (newarea < minArea) //go forward if object too small
    {
      forward();
    }
    else if (newarea > maxArea) //go backward if object too big
    {
      backward();
    }

    //else stop
    else
    {
      Stop();
    }
  }
  else
  {
    Stop();
  }
}


