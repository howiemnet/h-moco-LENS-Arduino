#ifndef ___ControlLoopLENS___
#define ___ControlLoopLENS___

//#include "Config.h"


#include <Arduino.h>
#include <stdint.h>
//#include "Motor.h"
//#include <Encoder.h>
#define MANUAL 0
#define AUTOMATIC 1


class ControlLoopLENS {
   
    int8_t  _currentIris;
    
    int32_t   nextDelta;
    int16_t  _lensScale;
   
    void      waitForCameraNotBusy();
    uint8_t   doByte(uint8_t byteToSend);
    bool      lensNotBusy();
    
public:    
    // lens moves
     int16_t  _lensOffset;
    bool lensMoving();
    void waitForLensToStop();
        int16_t  _currentPosition;
        int16_t  _requestedSetpoint;

     int16_t  _currentFocus;
    void     lensMoveNearest();
    void     lensMoveFurthest();
    int16_t  lensReadPosition();
    int16_t  lensGetPosition();
    void     lensHome();
    
    
    // iris moves
    void lensApertureOpen();
    void lensApertureChange(int8_t amount);


    void lensInit();
    void moveLens(int16_t delta);
  
  
  
  
//public:

             ControlLoopLENS();
             
    void     init();

    void     update(void);

    void     setSetpoint(int16_t);
    int16_t  getSetpoint();

    void nudge(int32_t distance);
    void setZero();
    
    
 

  
  
};

//ControlLoopSTEPPER controller;


#endif


