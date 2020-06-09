#include <PeanutKing_RescueSumo.h>
static PeanutKing_RescueSumo robot = PeanutKing_RescueSumo();

long timer = 0;
byte data[4];
byte cmd = 0x55;
int i=0;

void setup() {
  robot.init();
  robot.colorSensorInit(1);
  robot.laserSensorInit(7);
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;
  
  Serial.print("Distance: ");
  Serial.print(robot.readLaserSensor(7));
  Serial.print(" mm ");
  if (robot.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  // ------------------------------------------

  colorSensor_t s = robot.readcolorSensor(1);

  Serial.print("Color Temp: "); Serial.print(s.colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(s.lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(s.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(s.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(s.b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(s.c, DEC); Serial.print(" ");
  Serial.println(" ");

//  delay(500);

  robot.servoMove(0, 10);
  robot.stepperMove(0 , 10);
}
