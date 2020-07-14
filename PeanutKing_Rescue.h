#ifndef PeanutKing_Rescue_H
#define PeanutKing_Rescue_H

#include "PeanutKing_RescueSumo.h"

class PeanutKing_Rescue : public PeanutKing_RescueSumo {
 public:
  // Constructor 
  PeanutKing_Rescue(void);


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
    tcanRstPin  = 53,
    tscblPin    = 51,

    dirPin[2]   = {8, 13},
    stepPin[2]  = {9, 12},
    servoPin[2] = {11, 10};

    


  // Variables ===========================================================
  uint16_t
    systemTime,      //a reference 100Hz clock, 0-100 every second
    autoScanSensors;
  uint32_t
    sysTicks = 0;
  
  colorSensor_t color[5];



};


#endif
