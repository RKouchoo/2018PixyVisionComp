
// A class that holds extra methods that we arent using atm


int averageY() {
  return 100-average(pixy.blocks[0].y, pixy.blocks[1].y);
}

int wholeWidth() {
  if (pixy.blocks[0].x > pixy.blocks[1].x){
    return (pixy.blocks[0].x-(pixy.blocks[0].width/2))-(pixy.blocks[1].x+(pixy.blocks[1].width/2));
  } else {
    return (pixy.blocks[1].x-(pixy.blocks[1].width/2))-(pixy.blocks[0].x+(pixy.blocks[0].width/2));
  }
}

double distance() { // in feet
  return 1 / (((8.006 * pow(10,-3)) * wholeWidth()) + (8.664 * pow(10,-4)));
}