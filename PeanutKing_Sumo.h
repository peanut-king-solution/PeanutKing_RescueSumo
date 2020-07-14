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

  void motorSet(uint8_t motor_no, int16_t speed);

  // Constant  ===========================================================
  const int8_t  compass_address = 8;

  const uint8_t
    GET_READING = 0x55,
    SET_HOME    = 0x54,
    tcanRstPin  = 40,
    tcsblPin    = 41,

    dirAPin[2]  = {9, 12},
    dirBPin[2]  = {8, 11},
    pwmPin[2]   = {10, 13},
    diagPin[2]  = {52, 53};

};


#endif
