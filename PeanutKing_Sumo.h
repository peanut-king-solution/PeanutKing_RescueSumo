#ifndef PeanutKing_Sumo_H
#define PeanutKing_Sumo_H

#include "PeanutKing_RescueSumo.h"

class PeanutKing_Sumo : public PeanutKing_RescueSumo {
 public:
  // Constructor 
  PeanutKing_Sumo(void);


  // Constant  ===========================================================
  const int8_t  compass_address = 8;

  const uint8_t
    GET_READING = 0x55,
    SET_HOME    = 0x54,
    tcanRstPin  = 40,
    tcsblPin = 32;
    
};


#endif
