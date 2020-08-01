#include "PeanutKing_Sumo.h"


PeanutKing_Sumo::PeanutKing_Sumo(void)
{
}





void PeanutKing_Sumo::init(void) {
  Serial.begin(115200);
  //Serial1.begin(9600);
  
  for (uint8_t i=0; i<2; i++) {
    pinMode(pwmPin[i],  OUTPUT);
    pinMode(dirAPin[i],  OUTPUT);
    pinMode(dirBPin[i], OUTPUT);
    pinMode(diagPin[i], OUTPUT);
    digitalWrite(diagPin[i], HIGH);
  }

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
  
  pinMode (encoderPin[0][0],INPUT_PULLUP); 
  pinMode (encoderPin[0][1],INPUT_PULLUP);
  pinMode (encoderPin[1][0],INPUT_PULLUP); 
  pinMode (encoderPin[1][1],INPUT_PULLUP);
  
  // aLastState1 = digitalRead(encoderPin[0][0]);
  // aLastState2 = digitalRead(encoderPin[1][0]);
  attachInterrupt(digitalPinToInterrupt(encoderPin[0][0]), encode1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin[1][0]), encode2, CHANGE);

  delay(10);
  
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
  static uint32_t colorTimer[4] = {0};
  static colorSensor_t s[4];
  uint16_t r, g, b, c, colorTemp, lux;
  uint8_t idx = 0;

  switch(i) {
    case 0: idx = 2;  break;
    case 1: idx = 4;  break;
    case 2: idx = 5;  break;
    case 3: idx = 0;  break;
  }

  tcaselect(idx);

  if ( millis() - colorTimer[i] > 24 ) {
    colorTimer[i] = millis();
    getRawData(&r, &g, &b, &c);
    colorTemp = calculateColorTemperature_dn40(r, g, b, c);
    lux = calculateLux(r, g, b);

    s[i] = {r, g, b, c, colorTemp, lux};
  }
  return s[i];
}

color_t PeanutKing_Sumo::readAdvColor(uint8_t i) {
  colorSensor_t cs = readcolorSensor(i);

  rgb_t in = {cs.r, cs.g, cs.b};

  hsv_t op = rgb2hsv(in);
  // Serial.print("h:");
  // Serial.print( op.h );
  // Serial.print("  S:");
  // Serial.print( op.s );
  // Serial.print("  V:");
  // Serial.println( op.v );

  if ( op.v < 60 && op.s < 50  )        return black;
  else if ( op.h < 80 )                 return yellow;
  else if ( op.s < 50 && op.v > 60 )    return white;
  else if ( op.h < 15 || op.h > 315 )   return red;
  
  else if ( op.h < 180 )                return green;
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
    digitalWrite(diagPin[motor_no], HIGH);
  }
  else if ( speed<0 && speed>-256 ) {
    digitalWrite(dirAPin[motor_no], HIGH);
    digitalWrite(dirBPin[motor_no], LOW);
    analogWrite(pwmPin[motor_no], -speed);
    digitalWrite(diagPin[motor_no], HIGH);
  }
  else{
    digitalWrite(dirAPin[motor_no], HIGH);
    digitalWrite(dirBPin[motor_no], HIGH);
    digitalWrite(pwmPin[motor_no], HIGH);
  }
}

void PeanutKing_Sumo::encoderMotor(int16_t speedL, int16_t speedR) {
  targetSpeed[0] = speedL;
  targetSpeed[1] = speedR;

  // static uint32_t timer = 0, timeDiff = 0;
  // static int32_t encoderLast[2] = {0}, encoderDiff[2] = {0};

  // float spd[2] = {0.0};
  // int16_t targetSpeed[2] = {speedL, speedR};

  // timeDiff = micros() - timer;

  // if ( timeDiff > interval ) {
  //   timer = micros();
  //   for (uint8_t i=0; i<2; i++) {
  //     encoderDiff[i] = encoderCounter[i] - encoderLast[i];
  //     spd[i] = interval * scale * encoderDiff[i] / timeDiff;

  //     int16_t realSpd = Incremental_PID(i, spd[i], targetSpeed[i]);
  //     motorSet(i, realSpd);
      
  //     encoderLast[i] = encoderCounter[i];
  //   }
  // }
}


int32_t PeanutKing_Sumo::encoderCounter[2]  = {0, 0};
int16_t PeanutKing_Sumo::targetSpeed[2]     = {0, 0};
float   PeanutKing_Sumo::currentSpeed[2]   = {0.0, 0.0};

void PeanutKing_Sumo::encode1(void) { 
  static uint32_t timer = 0;
  static int32_t encoderLast = 0;
  static float bias[3] = {0}, pwm = 0;
  uint32_t timeDiff = micros() - timer;
  
  uint8_t aState1 = digitalRead(encoderPin[0][0]);
  if (digitalRead(encoderPin[0][1]) != aState1) {
    encoderCounter[0] ++;
  } else {
    encoderCounter[0] --;
  }

  if ( timeDiff > ENCODER_INTERVAL ) {
    int32_t encoderDiff = encoderCounter[0] - encoderLast;
    currentSpeed[0] = (float) ENCODER_INTERVAL * ENCODER_SCALE * encoderDiff / timeDiff;

    bias[0] =  currentSpeed[0] - targetSpeed[0];
  
    pwm -= (
      kp * (bias[0]-bias[1]) + 
      ki *  bias[0] +
      kd * (bias[0]-2*bias[1]+bias[2])
    );   // fomula of PID
    
    bias[2]=bias[1];    // last last
    bias[1]=bias[0];    // last

    if ( pwm > 250 )  pwm = 250;
    if ( pwm <-250 )  pwm =-250;

    if      ( pwm>0 && pwm<256 ) {
      digitalWrite(8, LOW);
      digitalWrite(9, HIGH);
      analogWrite(10, (int)pwm);
    }
    else if ( pwm<0 && pwm>-256 ) {
      digitalWrite(8, HIGH);
      digitalWrite(9, LOW);
      analogWrite(10, (int)-pwm);
    }
    else{
      digitalWrite(8, HIGH);
      digitalWrite(9, HIGH);
      digitalWrite(10, HIGH);
    }

    timer = micros();
    encoderLast = encoderCounter[0];
  }
}

void PeanutKing_Sumo::encode2(void) { 
  static uint32_t timer = 0;
  static int32_t encoderLast = 0;
  static float bias[3] = {0}, pwm = 0;
  uint32_t timeDiff = micros() - timer;
  
  uint8_t aState2 = digitalRead(encoderPin[1][0]);
  if (digitalRead(encoderPin[1][1]) != aState2) {
    encoderCounter[1] ++;
  } else {
    encoderCounter[1] --;
  }

  if ( timeDiff > ENCODER_INTERVAL ) {
    int32_t encoderDiff = encoderCounter[1] - encoderLast;
    currentSpeed[1] = (float) ENCODER_INTERVAL * ENCODER_SCALE * encoderDiff / timeDiff;

    bias[0] =  currentSpeed[1] - targetSpeed[1];

    pwm -= (
      kp * (bias[0]-bias[1]) + 
      ki *  bias[0] +
      kd * (bias[0]-2*bias[1]+bias[2])
    );   // fomula of PID
    
    bias[2]=bias[1];    // last last
    bias[1]=bias[0];    // last

    if ( pwm > 250 )  pwm = 250;
    if ( pwm <-250 )  pwm =-250;

    if      ( pwm>0 && pwm<256 ) {
      digitalWrite(12, LOW);
      digitalWrite(11, HIGH);
      analogWrite(13, (int)pwm);
    }
    else if ( pwm<0 && pwm>-256 ) {
      digitalWrite(12, HIGH);
      digitalWrite(11, LOW);
      analogWrite(13, (int)-pwm);
    }
    else{
      digitalWrite(12, HIGH);
      digitalWrite(11, HIGH);
      digitalWrite(13, HIGH);
    }

    timer = micros();
    encoderLast = encoderCounter[1];
  }
}

