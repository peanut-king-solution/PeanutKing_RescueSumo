#include <PeanutKing_Sumo.h>
static PeanutKing_Sumo robot = PeanutKing_Sumo();

byte m1c1 = 19;
byte m1c2 = 18;
byte m2c1 = 3;
byte m2c2 = 2;

const float interval = 24000.0, scale = 1;

int32_t counter[2] = {0};
int aState1, aState2;
int aLastState1, aLastState2;

float kp=2,ki=0.6,kd=2.5;  // PID

uint32_t timer = 0, timeDiff = 0, dispTimer = 0;

int32_t encoderLast[2] = {0}, encoderDiff[2] = {0};

float spd[2] = {0.0};


int defaultSpeed = 70;


void setup() {
  robot.init();
  
  robot.ledShow(1, 0, 0, 0, 0, 1);
  robot.ledUpdate(1);
  
  pinMode (m1c1,INPUT_PULLUP); 
  pinMode (m1c2,INPUT_PULLUP);
  pinMode (m2c1,INPUT_PULLUP); 
  pinMode (m2c2,INPUT_PULLUP);
  
  aLastState1 = digitalRead(m1c1);
  aLastState2 = digitalRead(m2c1);
  attachInterrupt(digitalPinToInterrupt(m1c1), encode1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2c1), encode2, CHANGE);
  
  Serial.print("Position1: "); 
  Serial.print(counter[0]);
  Serial.print("   Position2: "); 
  Serial.println(counter[1]);
  robot.motorSet(0, defaultSpeed);
  robot.motorSet(1, defaultSpeed);

  delay(500);
  
}

void loop() {
  
  timeDiff = micros() - timer;
  
  if ( timeDiff > interval ) {
    timer = micros();
    for (uint8_t i=0; i<2; i++) {
      encoderDiff[i] = counter[i] - encoderLast[i];
      spd[i] = interval * scale * encoderDiff[i] / timeDiff;

      int realSpd = Incremental_PID(i, spd[i], defaultSpeed );
      robot.motorSet(i, realSpd);
      
      encoderLast[i] = counter[i];
    }
  }


//
//  if ( millis() - dispTimer > 48 ) {
//    Serial.print("spd: "); 
//    Serial.print(spd[0]);
//    Serial.print("   spd: "); 
//    Serial.println(spd[1]);
//    dispTimer = millis();
//  }
  
  delay(1);

//  Serial.print("Position1: "); 
//  Serial.print(counter[0]);
//  Serial.print("   Position2: "); 
//  Serial.println(counter[1]);
//  robot.motorSet(0, -50);
//  robot.motorSet(1, -50);
//  delay(1500);
  
}


int Incremental_PID(uint8_t idx, int current_speed, int target_speed){
  static float 
    pwm[2] = {0},
    last_bias[2],
    prev_bias[2];
  
  float bias = current_speed - target_speed;            
  
  pwm[idx] -= (
    kp * (bias-last_bias[idx]) + 
    ki * bias +
    kd * (bias-2*last_bias[idx]+prev_bias[idx])
  );   //
  
  prev_bias[idx]=last_bias[idx];  //
  last_bias[idx]=bias;       //
  
  if (pwm[idx]<-250) {
    pwm[idx]=250;     
  }
  if (pwm[idx]>250) {
    pwm[idx]=250;  
  }
  //Serial.println(pwm[idx]);
  return pwm[idx];           //

}


void encode1() { 
  aState1 = digitalRead(m1c1);
  if (digitalRead(m1c2) != aState1) {
    counter[0] ++;
  } else {
    counter[0] --;
  }
  aLastState1 = aState1;
}

void encode2() { 
  aState2 = digitalRead(m2c1);
  if (digitalRead(m2c2) != aState2) {
    counter[1] ++;
  } else {
    counter[1] --;
  }
  aLastState2 = aState2;
}
