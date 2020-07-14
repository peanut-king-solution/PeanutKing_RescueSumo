#include "PeanutKing_Sumo.h"


PeanutKing_Sumo::PeanutKing_Sumo(void)
{
}

void PeanutKing_Sumo::init(void) {
  Wire.begin();
  Serial.begin(115200);
  //Serial1.begin(9600);
  
  digitalWrite(tcanRstPin, HIGH);

  ledSetup(1, tcsblPin, 1);
  ledShow(1, 255, 255, 255, 255, 1);
  ledUpdate(1);
  delay(10);
  // change this to the number of steps on your motor

  colorSensorInit(0);
  colorSensorInit(2);
  colorSensorInit(4);
  colorSensorInit(5);
  laserSensorInit(1);
  laserSensorInit(3);
  laserSensorInit(7);
  colorSensorInit(1);
  colorSensorInit(2);
  colorSensorInit(4);
  colorSensorInit(5);
  colorSensorInit(6);
  laserSensorInit(0);
  laserSensorInit(3);
  laserSensorInit(7);
  stepperSpeed = 100;
  
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to

  /*
  cli();    //disable interrupts
  // Timer 1
  TCCR1A = 0x00;            // Normal mode, just as a Timer
  TCCR1B = 0;               // same for TCCR0B
  TCNT1 = 0;
  
  OCR1A = 624;       // =(16*10^6) / (125*256) -1 (must be <65536)
  
  TCCR1B |= (1 << WGM12);   // CTC mode; Clear Timer on Compare
  TCCR1B |= (1 << CS12);    // prescaler = 256
  
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei();    //allow interrupts
  */

  //while ( compassRead() == 400 );
  delay(1);
}




uint16_t PeanutKing_Sumo::readLaserSensor(uint8_t i) {
  uint8_t idx = 0;
  switch(i) {
    case 0: idx = 1;  break;
    case 1: idx = 3;  break;
    case 2: idx = 7;  break;
  }

  tcaselect(idx);
  return readRangeSingleMillimeters();
}

colorSensor_t PeanutKing_Sumo::readcolorSensor(uint8_t i) {
  uint16_t r, g, b, c, colorTemp, lux;
  uint8_t idx = 0;

  switch(i) {
    case 0: idx = 2;  break;
    case 1: idx = 4;  break;
    case 2: idx = 5;  break;
    case 3: idx = 0;  break;
  }

  tcaselect(idx);

  getRawData(&r, &g, &b, &c);
  colorTemp = calculateColorTemperature_dn40(r, g, b, c);
  lux = calculateLux(r, g, b);
  colorSensor_t s = {r, g, b, c, colorTemp, lux};
  return s;
}

color_t PeanutKing_Sumo::readAdvColor(uint8_t i) {
  colorSensor_t cs = readcolorSensor(i);

  rgb_t in = {cs.r, cs.g, cs.b};

  hsv_t op = rgb2hsv(in);
  Serial.print("h:");
  Serial.print( op.h );
  Serial.print("  S:");
  Serial.print( op.s );
  Serial.print("  V:");
  Serial.println( op.v );

  if ( op.v < 60 && op.s < 50  )        return black;
  else if ( op.h < 80 )                 return yellow;
  else if ( op.s < 50 && op.v > 60 )    return white;
  else if ( op.h < 15 || op.h > 315 )   return red;
  
  else if ( op.h < 150 )                return green;
  else                                  return blue;

}

/* =============================================================================
 *                                  Motors
 * ============================================================================= */

// simple motor turn, motor_no cannot add, one by one 
void PeanutKing_Sumo::motorSet(uint8_t motor_no, int16_t speed) {
  if      ( speed>0 && speed<256 ) {
    digitalWrite(dirAPin[motor_no], LOW);
    digitalWrite(dirBPin[motor_no], HIGH);
    analogWrite(pwmPin[motor_no], speed);
  }
  else if ( speed<0 && speed>-256 ) {
    digitalWrite(dirAPin[motor_no], HIGH);
    digitalWrite(dirBPin[motor_no], LOW);
    analogWrite(pwmPin[motor_no], -speed);
  }
  else{
    digitalWrite(dirAPin[motor_no], HIGH);
    digitalWrite(dirBPin[motor_no], HIGH);
    digitalWrite(pwmPin[motor_no], HIGH);
  }
}

