
//#ifndef TEENSYDUINO

#include "ControlCanonLens.h"
#include <Arduino.h>
//#ifdef CONTROL_TYPE_LENS


//bidirectional clock PF4 - A3 on micro
#define CLK_OUTPORT  PORTC
#define CLK_INPORT    PINC
#define CLK_DIR     DDRC
#define CLK_BIT   0
#define clkOut()    CLK_DIR |= (1 << CLK_BIT)
#define clkIn()   CLK_DIR &= ~(1 << CLK_BIT)
#define clockIsHigh() (CLK_INPORT & (1 << CLK_BIT))
#define clkHigh()   CLK_OUTPORT |= (1 << CLK_BIT)
#define clkLow()    CLK_OUTPORT &= ~(1 << CLK_BIT)

//output to lens (Arduino Digital I/O 12 = PB6)
#define DCO_PORT  PORTC
#define DCO_DIR   DDRC
#define DCO_BIT 1

//input from lens (Arduino Digital I/O 9 = PH6)
#define DCI_OUTPORT PORTC
#define DCI_INPORT  PINC
#define DCI_DIR   DDRC
#define DCI_BIT 2




#define BIT_LEN 10

//iterations to hold after receiving end of lens busy, before next byte
#define POST_BUSY_WAIT 40

//how long to wait for lens busy (to set or clear) before giving up entirely.
// NB: The lens will take forever to respond to motor commands if there is no motor voltage!
#define BUSY_TIMEOUT  3000

// Storage buffer
#define STORAGE_SIZE  256
uint8_t storage[STORAGE_SIZE];
int storagePos = 0;

extern int32_t getNextDelta();

ControlLoopLENS::ControlLoopLENS() {
}

void ControlLoopLENS::init() {
  clkOut(); // clock as output to start
  DCO_DIR |= (1 << DCO_BIT);  // DCO as output
  DCI_DIR &= ~(1 << DCI_BIT); // DCI as input
  DCI_OUTPORT |= (1 << DCI_BIT);  //   with pull-up enabled
  //  DBG_DIR |= (1 << DBG_BIT);  // debug as output

  clkHigh();

  //targetVelocity = 0;

  //  motor = new Motor(pmot_G, pmot_E, pmot_L, pmot_R);
  //_loopCount = 0;
  lensInit();
 // lensHome();
 // lensApertureChange(0x79);
 // delay(200);
 // lensApertureChange(0x80);
}


int16_t ControlLoopLENS::getSetpoint() {
  return _requestedSetpoint;
}


// --------------------- DIFFERENT LENS stuff!

void ControlLoopLENS::setSetpoint(int16_t newVal) {
  _requestedSetpoint = newVal;
}

bool ControlLoopLENS::lensMoving() {
  waitForCameraNotBusy();
  storagePos = 0;
  doByte (0x90);
  doByte (0xB9);
  doByte (0x00);
  if ((storage[5] == 4) || (storage[5] == 36)) {
    digitalWrite(13,HIGH);
    return true;
  } else {
    digitalWrite(13,LOW);
    return false;
  }

}

void ControlLoopLENS::waitForLensToStop() {
  int timeout = 1000;
  while (lensMoving()) {
    timeout --;
    if (timeout < 1) break;
  }
}

// -----------------------------------------


void ControlLoopLENS::setZero() {
  _lensOffset = lensGetPosition();
  _requestedSetpoint = 0;
  _currentPosition = 0;
  _currentFocus = _lensOffset;
  _currentIris = 0;
  //  _currentVelocity = 0;
}


void ControlLoopLENS::nudge(int32_t distance) {
  _requestedSetpoint += distance;
  moveLens((int16_t) distance);
}

void ControlLoopLENS::moveLens(int16_t delta) {

 //   digitalWrite(3,(abs(delta) > 10));
 //   digitalWrite(4,(abs(delta) > 50));
    int8_t lowB = 0xFF & delta;
    delta >>= 8;
    storagePos = 0;
    doByte(0x44);
 
    doByte(0xFF & delta);
    doByte(lowB);

}



void ControlLoopLENS::update(void) {

  if (!lensMoving()) {
    _currentPosition = lensGetPosition() - _lensOffset;
    if (_currentPosition != _requestedSetpoint) {
      moveLens(_requestedSetpoint - _currentPosition);
    }

  }
  //      int32_t theDelta = _currentPosition - _requestedSetpoint;
  //    _currentPosition += theDelta;

  //      moveLens(theDelta); // - _currentFocus);


}


uint8_t ControlLoopLENS::doByte(uint8_t byteToSend) {
  if (storagePos >= STORAGE_SIZE) {
    storagePos = 0;
  }
  clkHigh(); //clock should be high already
  clkOut();

  noInterrupts(); //timing is critical and Arduino keeps doing things

  uint8_t byteIn;
  uint8_t byteOut = byteToSend;
  for (int i = 0; i < 8; i++) { //for each bit

    //set the output bit
    DCO_PORT = (DCO_PORT & ~(1 << DCO_BIT)) | ((byteOut & 0x80) >> (7 - DCO_BIT));
    byteOut = (byteOut << 1); //move to next bit

    clkLow(); //drop the clock, the lens sets our DCI after this edge.
    // dbgLow();

    for (int j = 0; j < BIT_LEN; j++); //bit delay on low clock
    __asm__("nop\n\t");

    clkHigh(); //raise the clock. The lens reads our DCO after this edge.
    uint8_t dciVal = DCI_INPORT;
    byteIn = (byteIn << 1) | ((dciVal >> DCI_BIT) & 0x01); //store the DCI bit in byteIn and shift
    // dbgHigh();

    for (int j = 0; j < BIT_LEN; j++); //bit delay on high clock
    __asm__("nop\n\t");

  }


  clkHigh(); //enable pull-up for clock
  clkIn();   //and release the clock line output

  //  dbgLow(); //debug signature for end of byte
  // dbgHigh();
  // dbgLow();

  //Delay a bit to give the lens time to pull the clock low to signify it is busy
  // (we don't assume it always does, although it does appear to)
  for (int j = 0; j < BIT_LEN; j++);
  __asm__("nop\n\t");
  //  dbgHigh(); //dbg signature for 'busy received'
  //
  //wait for lens busy to end. This can take a long time, or forever
  int timeout = 0;
  while (!clockIsHigh()) {
    timeout++;
    if (timeout > BUSY_TIMEOUT)
      goto busyTimeout;
  };
  //dbgLow(); //debug signature: busy complete
  // dbgHigh();
  //  dbgLow();

  // wait a little bit longer before proceeding with the next byte
  for (int j = 0; j < POST_BUSY_WAIT; j++);
  __asm__("nop\n\t");

  //  dbgHigh();

  clkOut();
  interrupts(); //end of critical timing

  //store and return
  storage[storagePos++] = byteToSend;
  storage[storagePos++] = byteIn;
  delay(1);
  // digitalWrite(13,LOW);
  return byteIn;

busyTimeout:
  /*  dbgLow(); //debug error signature
    dbgHigh();
    dbgLow();
    dbgHigh();
    dbgLow();
    dbgHigh();
    dbgLow(); */
  storage[storagePos++] = 0xDE; //mark command failure (maybe)
  storage[storagePos++] = 0xAD;
  // digitalWrite(13,HIGH);
  return 0xFF;
}


void ControlLoopLENS::waitForCameraNotBusy() {
  //Serial.print("Busy? ");

  
  bool cameraBusy = true;
     //digitalWrite(13,HIGH);
  while (cameraBusy) {
    storagePos = 0;
    doByte(0x0A);
    doByte(0x00);
    if (storage[3] == 0xAA) {
      cameraBusy = false;
    } else {
  //    Serial.print(".");
      delay(5);
    }

  }
  //digitalWrite(13,LOW);
  //Serial.println("OK");
}

bool ControlLoopLENS::lensNotBusy() {
  storagePos = 0;
  doByte(0x0A);
  doByte(0x00);
  return (storage[3] == 0xAA);
}


void ControlLoopLENS::lensMoveNearest() {
  doByte(0x06);
  //waitForCameraNotBusy();
}

void ControlLoopLENS::lensMoveFurthest() {
  doByte(0x05);
  //waitForCameraNotBusy();
}

int16_t ControlLoopLENS::lensGetPosition() {
  waitForCameraNotBusy();
  storagePos = 0;
  doByte(0xC0);
  doByte(0x00);
  doByte(0x00);
  int16_t theFoc = ((int16_t) storage[3] << 8) | storage[5];
  //theFoc -= _lensOffset;
  if (abs(theFoc - _lensOffset) > 10) {
    //digitalWrite(5,HIGH);
  } else {
    //digitalWrite(5,LOW);
 
  }
  return (theFoc);
}






void ControlLoopLENS::lensHome() {
  storagePos = 0;
  doByte(0x05);
  waitForLensToStop();
  //delay(1200);
  int16_t lensMax = lensGetPosition();

  doByte(0x06);
  
  //delay(1200);
  waitForLensToStop();

  _lensOffset = lensGetPosition();
  _lensScale = lensMax - _lensOffset;


  //    doByte(0x05);
  _currentPosition = 0;
  _requestedSetpoint = 0;

}


void ControlLoopLENS::lensApertureOpen() {
  storagePos = 0;
  doByte (0x13);
  doByte (0x80);
  waitForCameraNotBusy();
  _currentIris = 0;
}

void ControlLoopLENS::lensApertureChange(int8_t amount) {
  waitForCameraNotBusy();
  storagePos = 0;
  //  doByte (0x07);
  doByte (0x13);
  doByte (amount);
  _currentIris += amount;

}


void ControlLoopLENS::lensInit() {
  waitForCameraNotBusy();
  storagePos = 0;
  delayMicroseconds(100);

  doByte(0x90); // ???
  doByte(0x8E);
  doByte(0x00);

  delayMicroseconds(100);

  doByte(0xB0); // request aperture data
  doByte(0x00);
  doByte(0x00);
  doByte(0x00);
  storagePos = 0;
  delayMicroseconds(100);

  doByte(0x80); // request lens data
  doByte(0x00); //? lens code
  doByte(0x00); // lens code
  doByte(0x00);
  doByte(0x00); // focal len in mm
  doByte(0x00);
  doByte(0x00); // focal len in mm (maybe they are min/max?)
  doByte(0x00);
  storagePos = 0;

  //dumpStorage();
  lensHome();
  nextDelta = 0;

}


//#endif
