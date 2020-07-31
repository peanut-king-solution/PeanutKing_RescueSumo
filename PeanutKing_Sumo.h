#ifndef PeanutKing_Sumo_H
#define PeanutKing_Sumo_H

#include "PeanutKing_RescueSumo.h"

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

  int16_t Incremental_PID(uint8_t idx, int16_t currentSpeed, int16_t targetSpeed);
  
  void
    motorSet(uint8_t motor_no, int16_t speed),
    encoderMotor(int16_t speedL, int16_t speedR);

  static void encode1(void);
  static void encode2(void);
  // Constant  ===========================================================

  const float interval = 24000.0, scale = 1;

  const uint8_t
    tcanRstPin  = 40,
    tcsblPin    = 41,

    dirAPin[2]  = {8, 12},
    dirBPin[2]  = {9, 11},
    pwmPin[2]   = {10, 13},
    diagPin[2]  = {52, 53};

  static const uint8_t encoderPin[2][2] = { {19, 18}, {2, 3} };

  static int32_t encoderCounter[2];

};


#endif
