#include "ControlCanonLens.h"

#define THIS_INTERFACE_ID 12

/*#include <Wire.h>          // *** I2C Mode
#define OLED_Address 0x3c
#define OLED_Command_Mode 0x80
#define OLED_Data_Mode 0x40
*/
ControlLoopLENS controller;

//IntervalTimer innerTimer;

byte outgoingBuffer[8];
byte incomingBuffer[8];


// -------------------------------------------
//
//   STATES
//
// -------------------------------------------

//#define DEBUG_ENABLED

#define STATE_MOTOR_OFF 0
#define STATE_MOTOR_ON_IDLE 1
#define STATE_MOTOR_ON_PLAYING_FRAME 2
#define STATE_MOTOR_OFF_OUT_OF_BOUNDS 3
#define STATE_MOTOR_OFF_RAN_OUT_OF_FRAMES 4
#define STATE_DOING_STICTION_CALIBRATION 5
#define BUFFER_STATE_NEED_NEXT_FRAME 1
#define BUFFER_STATE_OK 0

uint8_t state;
uint8_t buffer_state;
long lastMillis = 0;

void stepperIntHandler() {
  controller.update();
}




#pragma pack(push,1)

union __attribute__((packed)) {
  int16_t output;
  uint8_t input[2];
}
buffer;

#pragma pack(pop)


int16_t nextPosition;

#ifdef DEBUG_ENABLED

#include <Adafruit_CharacterOLED.h>
Adafruit_CharacterOLED lcd(9, 8, 10, 7, 6, 5, 4);

#endif

uint8_t addressOffset = 0;

void setup() {

   // sort out address
   pinMode(2,INPUT);
   pinMode(3,INPUT);
   digitalWrite(2,HIGH);
   digitalWrite(3,HIGH);

   
   if (digitalRead(3)) { addressOffset += 1; }
   if (digitalRead(2)) { addressOffset += 2; }
   


  //  Wire.begin();
  // OLED_start();
#ifdef DEBUG_ENABLED

lcd.begin(16, 2);
lcd.print("Starting up");
#endif


  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial2.begin(115200);
  pinMode(13, OUTPUT);
 // pinMode(23, INPUT);
 // digitalWrite(23, HIGH);
 // delay(500);
  while (!Serial) {
  ;// wait
  }
  
  while (Serial.available() > 0) {
    Serial.read();
  }
  // 
 
  
  controller.init();
  // innerTimer.begin(stepperIntHandler, 16);
  // innerTimer.priority(20);

 state = STATE_MOTOR_ON_IDLE;
 lastMillis = millis();

#ifdef DEBUG_ENABLED

lcd.clear();
#endif

}



uint8_t charsExpected = 0;
bool  needNextData = true;
bool flasher = false;

void loop() {

  if ((millis() - lastMillis) > 40) {
    lastMillis += 40;
    //    controller.update();
    
    controller.setSetpoint(getNextDelta());
    controller.update();

    // OLED_numprint(controller._currentFocus, 0,  1);
    // updateScreen();
  }
  checkForInput();
  //controller.update();

}




int16_t getNextDelta() {
  if (state == STATE_MOTOR_ON_PLAYING_FRAME) {
    if (buffer_state == BUFFER_STATE_OK) {
      buffer_state = BUFFER_STATE_NEED_NEXT_FRAME;
      doSendUpdate();
    } else {
      //state = STATE_MOTOR_OFF_RAN_OUT_OF_FRAMES;
      doSendUpdate();
      //state = STATE_MOTOR_ON_IDLE;
      
     // doSendUpdate();
      
    }
  }
  return nextPosition;
 
}

int16_t charsReceived = 0;
long lastInputMillis = 0;




void checkForInput() {
      

  if (Serial.available() > 0) {
      

    if ((millis() - lastInputMillis) > 1000) {
      charsReceived = 0;

    }
    lastInputMillis = millis();

    incomingBuffer[charsReceived] = Serial.read();
    charsReceived++;
    if (charsReceived >= 8) {
      charsReceived = 0;
      handleInput();
    }
 
  }
}



void handleInput() {

  switch (incomingBuffer[0]) {
    case 104: 
      // 'h'
     // digitalWrite(6,HIGH);
      controller.lensHome();
     // digitalWrite(6,LOW);
      break;
    case 105:
      // identify!
      sendIdentificationPacket();
      break;
    case 102:
      // reset!
      controller.lensMoveFurthest();
      break;
    case 110:
      // reset!
      controller.lensMoveNearest();
      break;
    case 114:
      // reset!
      controller.init();
      break;
    case 0:
      // manual control
      //controller.disableMotor();
      state = STATE_MOTOR_OFF;
      break;
    case 1:
      //controller.enableMotor();
      state = STATE_MOTOR_ON_IDLE;
      break;
    case 2:
      receiveFrame();
      break;
    case 16:
      doSendUpdate();
      break;
  }

}





int32_t getLongFromIncomingBufferAtPosition(uint8_t bufferPosition) {
  int32_t theVal = ((int32_t) incomingBuffer[bufferPosition] & 0xFF);
  theVal |= (((int32_t) incomingBuffer[bufferPosition + 1] & 0xFF)) << 8;
  theVal |= (((int32_t) incomingBuffer[bufferPosition + 2] & 0xFF)) << 16;
  theVal |= (((int32_t) incomingBuffer[bufferPosition + 3] & 0xFF)) << 24;
  return theVal;
}

void receiveFrame() {
  nextPosition = (int16_t) getLongFromIncomingBufferAtPosition(1);
  buffer_state = BUFFER_STATE_OK;
  // if it's the first frame, set it, reset timers
  if (state == STATE_MOTOR_ON_IDLE) {
    state = STATE_MOTOR_ON_PLAYING_FRAME;
    controller.setSetpoint(nextPosition);
    lastMillis = millis();
    
    getNextDelta();
    //   controller.resetAndStartAnim();
  } else {
    //buffer_state = BUFFER_STATE_NEED_FRAME;
  }
//buffer_state = BUFFER_STATE_OK;

#ifdef DEBUG_ENABLED

//lcd.clear();
lcd.setCursor(8,0);
lcd.print(nextPosition);
lcd.print (" ");

#endif
}




void sendIdentificationPacket() {
  outgoingBuffer[0] = 0;
  outgoingBuffer[1] = 1;
  outgoingBuffer[2] = 1;
  outgoingBuffer[3] = THIS_INTERFACE_ID + addressOffset;
  outgoingBuffer[4] = 'H';
  sendBuffer();
}

void doSendUpdate() {
  outgoingBuffer[0] = 1; // live data
  outgoingBuffer[1] = 1;
  outgoingBuffer[2] =  state;
  outgoingBuffer[3] = buffer_state;
  putIntInOutgoingBufferAtPosition(controller._requestedSetpoint, 4);
  putIntInOutgoingBufferAtPosition(controller._lensOffset, 6);
  //  putLongInOutgoingBufferAtPosition(controller.homeSensorPosition, 44);
#ifdef DEBUG_ENABLED

//lcd.clear();
lcd.setCursor(0,0);
lcd.print(controller._requestedSetpoint);
lcd.print (" ");
lcd.setCursor(0,1);
lcd.print(controller._lensOffset);
lcd.print (" ");

#endif

  sendBuffer();
}



void sendBuffer() {
//  for (int i = 0; i < 8; i++) {
//Serial.flush();
//delay(10);
    Serial.write(outgoingBuffer,8);
    Serial.flush();
//  }
}

void putIntInOutgoingBufferAtPosition(int16_t theValue, uint8_t bufferPosition) {
  outgoingBuffer[bufferPosition]     = theValue & 0xFF;
  outgoingBuffer[bufferPosition + 1] = (theValue >> 8)  & 0xFF;
}


void putLongInOutgoingBufferAtPosition(long theValue, uint8_t bufferPosition) {
  outgoingBuffer[bufferPosition]     = theValue & 0xFF;
  outgoingBuffer[bufferPosition + 1] = (theValue >> 8)  & 0xFF;
  outgoingBuffer[bufferPosition + 2] = (theValue >> 16) & 0xFF;
  outgoingBuffer[bufferPosition + 3] = (theValue >> 24) & 0xFF;
}

