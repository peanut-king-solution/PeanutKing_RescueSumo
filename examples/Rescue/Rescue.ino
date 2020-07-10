#include <PeanutKing_Rescue.h>
static PeanutKing_Rescue robot = PeanutKing_Rescue();

long timer = 0;
byte data[4];
byte cmd = 0x55;
int i = 0;

#define dirPin  8
#define stepPin 9
#define stepsPerRevolution 25600


void setup() {
  robot.init();
}

void dispDist(uint8_t i) {
  Serial.print("Distance: ");
  Serial.print(robot.readLaserSensor(i));
  Serial.print(" mm ");
  if (robot.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}

void dispColor(uint8_t i) {
  colorSensor_t s = robot.readcolorSensor(i);

  Serial.print("Color Temp: "); Serial.print(s.k, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(s.l, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(s.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(s.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(s.b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(s.c, DEC); Serial.print(" ");
  Serial.println(" ");
}

void loop() {
  /*
  for (int i=0; i<5; i++)
    dispColor(i);
  for (int i=0; i<3; i++)
    dispDist(i);
  delay(1000);
  */


  color_t colorMid = robot.readAdvColor(2);

  switch( colorMid ) {
    case black:   Serial.print("black  ");  break;
    case white:   Serial.print("white  ");  break;
    case grey:    Serial.print("grey   ");  break;
    case red:     Serial.print("red    ");  break;
    case green:   Serial.print("green  ");  break;
    case blue:    Serial.print("blue   ");  break;
    case yellow:  Serial.print("yellow ");  break;
    case cyan:    Serial.print("cyan   ");  break;
    case magenta: Serial.print("magenta");  break;
  }

  
  // ------------------------------------------

  robot.moveForward(1);
  delay(2000);
  robot.moveForward(0);
  delay(2000);
  robot.stop();
  delay(1000);
  robot.moveBackward(1);
  delay(2000);
  robot.moveBackward(0);
  delay(2000);
  robot.stop();
  delay(1000);

  robot.stepperMove(0, 25600);
  delay(500);
  robot.stepperMove(0, -25600);
  delay(500);
  

}
