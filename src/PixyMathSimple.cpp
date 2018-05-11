/* simple math and movement code that works for the arduino and pixy TODO: tweak code to work with new fish eye lenses */

// WARN: Much of this may need to be changed based on how the camera is mounted to the robot.

#include <SPI.h>  
#include <Pixy.h>

#define DEADZONE 5 //deadzone in pixels
#define DEADZONE_STRAFE 10 // deadzone to strafe.

#define FRAME_SKIP 1 // wait one frame to run the next loop

int TurnLeft = 13; 
int TurnRight = 12;


// This is the main Pixy object 
Pixy pixy;

int watchDog = 0;
int watchDogMax = 1000; // 1000 failed tries of connecting to the camera or sensing the robot.

void setup(){
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pinMode(TurnLeft, OUTPUT);
  pinMode(TurnRight, OUTPUT);

  pixy.init();
}

void loop() {
  static int i = 0;
  uint16_t blocks;
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  if (blocks){
    i++;
    watchDog = 0;
  } else {
    watchDog++;
  }

  // wait for %frameskip frames
  if (i%FRAME_SKIP == 0){
    if (blocks == 2){ // goal or ball.
      //Serial.print(distance());
       turnRobot();
      i = 1;
    } else {
      i = 1;
      // stop the robot     

      }
  }

  if (watchDog == watchDogMax){
    Serial.println("No Blocks");
    
    // do rf communication to the other robot here.

    // stop the robot or move backward slowly.
    // possibly turn around to locate the ball and then rotate back and go to it,
    // setting an 'imaginary' averageX() value that the robot can head to.
  }
}

int average(int a, int b){
  return (a + b) / 2;
}

int averageX() {
  return 160-average(pixy.blocks[0].x, pixy.blocks[1].x);
}

void turnRobot() {
    Serial.println(averageX());
    if (averageX() > DEADZONE){
      Serial.println("turn left");
      if (averageX() > DEADZONE_STRAFE) {
          // move backward a little then strafe left (90deg) 
        } else {
          // diag left
        }

    } else if (averageX() < DEADZONE*-1) {
      Serial.println("turn right");
      if (averageX() > DEADZONE_STRAFE*-1) {
        // move backward a little then strafe right (90deg) 
        } else {
        // diag right
        }
      
    } else {
      Serial.println("go straight");
      
      // ball should be in the centre of the dead zone pixels, 
      // so the robot should be able to move straight.

    }
}