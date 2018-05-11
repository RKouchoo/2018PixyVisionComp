#include <SPI.h>  
#include <Pixy.h>

#define DEADZONE 5 //deadzone in pixels

int TurnLeft = 13;
int TurnRight = 12;


// This is the main Pixy object 
Pixy pixy;

int watchDog = 0;
int watchDogMax = 1000;

void setup(){
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pinMode(TurnLeft, OUTPUT);
  pinMode(TurnRight, OUTPUT);

  pixy.init();
}

void loop() {
  static int i = 0;
  int j;
  uint16_t blocks;
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  if (blocks){
    i++;
    watchDog = 0;
  } else {
    watchDog++;
  }

  //every 50 frames
  if (i%50 == 0){
    if (blocks == 2){ //gear or shoot
      Serial.print(distance());
      Serial.println(" ft");
      turnRobot();
      i = 1;
    } else {
      Serial.println("Rope??");
      i = 1;
      digitalWrite(TurnLeft, LOW);
      digitalWrite(TurnRight, LOW);
    }
  }

  if (watchDog == watchDogMax){
    Serial.println("No Blocks");
    digitalWrite(TurnLeft, LOW);
    digitalWrite(TurnRight, LOW);
  }
}

int average(int a, int b){
  return (a+b)/2;
}
int averageX(){
  return 160-average(pixy.blocks[0].x, pixy.blocks[1].x);
}
int averageY(){
  return 100-average(pixy.blocks[0].y, pixy.blocks[1].y);
}
int wholeWidth(){
  if (pixy.blocks[0].x > pixy.blocks[1].x){
    return (pixy.blocks[0].x-(pixy.blocks[0].width/2))-(pixy.blocks[1].x+(pixy.blocks[1].width/2));
  } else {
    return (pixy.blocks[1].x-(pixy.blocks[1].width/2))-(pixy.blocks[0].x+(pixy.blocks[0].width/2));
  }
}
double distance(){
  return 1/(((8.006*pow(10,-3))*wholeWidth())+(8.664*pow(10,-4)));
}

void turnRobot(){
    Serial.println(averageX());
    if (averageX() > DEADZONE){
      Serial.println("turn left");
      digitalWrite(TurnLeft, HIGH);
      digitalWrite(TurnRight, LOW);
    } else if (averageX() < DEADZONE*-1) {
      Serial.println("turn right");
      digitalWrite(TurnLeft, LOW);
      digitalWrite(TurnRight, HIGH);
    } else {
      Serial.println("go straight");
      digitalWrite(TurnLeft, HIGH);
      digitalWrite(TurnRight, HIGH);
    }
}