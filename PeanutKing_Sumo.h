#ifndef PeanutKing_Sumo_H
#define PeanutKing_Sumo_H

#include "PeanutKing_RescueSumo.h"




#define ENCODER_INTERVAL 10000.0
#define ENCODER_SCALE    4.7
// PID
#define kp 2
#define ki 0.6
#define kd 2.5  


class PeanutKing_Sumo : public PeanutKing_RescueSumo {
 public:
  // Constructor 
  PeanutKing_Sumo(void);


/* =============================================================================
 *                              Functions
 * ============================================================================= */
  void
    init(void),
    stop(void);

  color_t readAdvColor(uint8_t i);
  colorSensor_t readcolorSensor(uint8_t i);

  uint16_t readLaserSensor(uint8_t i);

  //static int16_t Incremental_PID(uint8_t idx, int16_t currentSpeed, int16_t targetSpeed);
  
  void
    motorSet(uint8_t motor_no, int16_t speed),
    encoderMotor(int16_t speedL, int16_t speedR);

  static void encode1(void);
  static void encode2(void);
  // Constant  ===========================================================

  const uint8_t
    tcanRstPin  = 40,
    tcsblPin    = 41,

    dirAPin[2]  = {8, 12},
    dirBPin[2]  = {9, 11},
    pwmPin[2]   = {10, 13},
    diagPin[2]  = {52, 53};

  static const uint8_t encoderPin[2][2] = { {19, 18}, {3, 2} };

  static int32_t encoderCounter[2];
  static int16_t targetSpeed[2];
  static float currentSpeed[2];

};

  // #define MY_PIN    8

  // // do this once at setup
  // uint8_t myPin_mask = digitalPinToBitMask(MY_PIN);
  // volatile uint8_t *myPin_port = portInputRegister(digitalPinToPort(MY_PIN));

  // // read the pin
  // uint8_t pinValue = (*myPin_port & myPin_mask) != 0;

  // volatile uint8_t *myPin_port = portOutputRegister(digitalPinToPort(MY_PIN));



#endif

