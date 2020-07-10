#include "PeanutKing_Rescue.h"


PeanutKing_Rescue::PeanutKing_Rescue(void)
{
}

void PeanutKing_Rescue::init(void) {
  Wire.begin();
  Serial.begin(9600);
  //Serial1.begin(9600);
  
  digitalWrite(tcanRstPin, HIGH);

  ledSetup(1, tscblPin, 1);
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
  delay(10);
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


void PeanutKing_Rescue::stop(void) {
  // Set the spinning direction clockwise:
  digitalWrite(dirPin[0], LOW);
  analogWrite(stepPin[0],0);

  digitalWrite(dirPin[1], HIGH);
  analogWrite(stepPin[1],0);
}

