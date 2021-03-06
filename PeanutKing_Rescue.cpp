#include "PeanutKing_Rescue.h"


PeanutKing_Rescue::PeanutKing_Rescue(void)
{
}

void PeanutKing_Rescue::init(void) {
  Serial.begin(115200);

  digitalWrite(tcanRstPin, HIGH);

  ledSetup(1, tcsblPin, 1);
  ledShow(1, 255, 255, 255, 255, 1);
  ledUpdate(1);
  delay(10);
  // change this to the number of steps on your motor

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
  servoMotor[0].s.attach(10);    // attaches the servo on pin 9 to the servo object
  servoMotor[1].s.attach(11);    // attaches the servo on pin 9 to the servo object

  setStepperSpeed(stepperSpeed);

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


void PeanutKing_Rescue::moveForward(uint8_t spd) {
  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  switch(spd) {
    case 0:
      TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
      TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
      break;
    case 1:
      TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
      TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
      break;
  }
  // Set the spinning direction clockwise:
  digitalWrite(dirPin[0], HIGH);
  analogWrite(stepPin[0], 128);

  digitalWrite(dirPin[1], LOW);
  analogWrite(stepPin[1], 128);
}

void PeanutKing_Rescue::moveBackward(uint8_t spd) {
  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  switch(spd) {
    case 0:
      TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
      TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
      break;
    case 1:
      TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
      TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
      break;
  }
  // Set the spinning direction clockwise:
  digitalWrite(dirPin[0], LOW);
  analogWrite(stepPin[0], 128);

  digitalWrite(dirPin[1], HIGH);
  analogWrite(stepPin[1], 128);
}
void PeanutKing_Rescue::moveRight(uint8_t spd) {
  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  switch(spd) {		//rotation
    case 0:
      TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
	    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
	    //TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
      //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
      break;
    case 1:    //turning
		  TCCR2B = TCCR2B & B11111000 | B00000100; 	// set timer 2 divisor to    64 for PWM frequency of   490.20 Hz//rightmotor
      //  TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
      TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz //leftmotor
      break;
  }
  // Set the spinning direction clockwise:
  digitalWrite(dirPin[0], LOW);
  analogWrite(stepPin[0], 128);

  digitalWrite(dirPin[1], LOW);
  analogWrite(stepPin[1], 128);
}
void PeanutKing_Rescue::moveLeft(uint8_t spd) {
  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  switch(spd) {
    case 0:
      TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
      TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
      //TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
      //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
      break;
    case 1:
		 
      TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
      //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz 
	  TCCR1B = TCCR1B & B11111000 | B00000011;		// set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
      break;
  }
  // Set the spinning direction clockwise:
  digitalWrite(dirPin[0], HIGH);
  analogWrite(stepPin[0], 128);

  digitalWrite(dirPin[1], HIGH);
  analogWrite(stepPin[1], 128);
}

void PeanutKing_Rescue::stop(void) {
  // Set the spinning direction clockwise:
  digitalWrite(dirPin[0], LOW);
  analogWrite(stepPin[0],0);

  digitalWrite(dirPin[1], HIGH);
  analogWrite(stepPin[1],0);
}


uint16_t PeanutKing_Rescue::readLaserSensor(uint8_t i) {
  uint8_t idx = 0;
  switch(i) {
    case 0: idx = 0;  break;
    case 1: idx = 3;  break;
    case 2: idx = 7;  break;
  }

  tcaselect(idx);
  return readRangeSingleMillimeters();
}

colorSensor_t PeanutKing_Rescue::readcolorSensor(uint8_t i) {
  static uint32_t colorTimer[5] = {0};

  uint16_t r, g, b, c, colorTemp, lux;
  uint8_t idx = 0;

  // 0 3
  // 1 2
  switch(i) {
    case 0: idx = 1;  break;
    case 1: idx = 2;  break;
    case 2: idx = 4;  break;
    case 3: idx = 5;  break;
    case 4: idx = 6;  break;
  }

  tcaselect(idx);

  if ( millis() - colorTimer[i] > 24 ) {
    getRawData(&r, &g, &b, &c);
    colorTemp = calculateColorTemperature_dn40(r, g, b, c);
    lux = calculateLux(r, g, b);
    colorSensor_t s = {r, g, b, c, colorTemp, lux};
    return s;
  }
}

color_t PeanutKing_Rescue::readAdvColor(uint8_t i) {
  colorSensor_t cs = readcolorSensor(i);

  rgb_t in = {cs.r, cs.g, cs.b};

  hsv_t op = rgb2hsv(in);
  Serial.print("h:");
  Serial.print( op.h );
  Serial.print("  S:");
  Serial.print( op.s );
  Serial.print("  V:");
  Serial.println( op.v );

  if ( op.v < 60 && op.s < 50  )     return black;
  else if ( op.h < 80 )                return yellow;
  else if ( op.s < 50 && op.v > 60 )  return white;
  else if ( op.h < 15 || op.h > 315 )  return red;
  
  else if ( op.h < 150 )               return green;
  else                                 return blue;

}

hsv_t PeanutKing_Rescue::readbrightless(uint8_t i) {
  colorSensor_t cs = readcolorSensor(i);
  rgb_t in = {cs.r, cs.g, cs.b};
  hsv_t op = rgb2hsv(in);
  return op ;
}



