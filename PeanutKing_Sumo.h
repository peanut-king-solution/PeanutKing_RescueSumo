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
    stop(void),
    moveForward(uint8_t spd = 1),
    moveBackward(uint8_t spd = 1),
    moveRight(uint8_t spd = 1),
    moveLeft(uint8_t spd = 1);

  // Constant  ===========================================================
  const int8_t  compass_address = 8;

  const uint8_t
    GET_READING = 0x55,
    SET_HOME    = 0x54,
    tcanRstPin  = 40,
    tcsblPin    = 41,

    dirAPin[2]  = {9, 12},
    dirBPin[2]  = {8, 11},
    pwmPin[2]   = {10, 13};

};


#endif
