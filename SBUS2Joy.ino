// SBUS to Joystick interface
// (c) 2023 Pablo Montoreano

/*********************************************************
  @file       SBUS2Joy.ino
  @brief      SBUS to PC Joystick interface
  @author     Pablo Montoreano
  @copyright  2023 Pablo Montoreano

  3rd party library: Joystick (2.1.1) by Matthew Heironimus

  for Arduino Pro Micro (ATMega32U4) or Arduino Leonardo
  does not work with Arduino Nano because it cannot emulate PC Joystick interface
*********************************************************/

static const unsigned int maxChan= 8;         // number of channels in joystick
static const unsigned int signalLost= 100;    // if channel[3] below this value -> failsafe
static const unsigned long timeOutMs= 1000;   // timeout if no packet received after 1 sec
static const unsigned long fsResetTime= 250;  // flight sim reset time

static bool oldFsMode, newFsMode;
static unsigned long lastReception;
static unsigned int sbusByte, byteNmbr;
static byte frame[25];  // 25 bytes per frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int lastReported[9];  // 8 channels plus array[0]
static unsigned int i; // a counter

#include <Joystick.h>

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,  // do not use JOYSTICK_TYPE_MULTI_AXIS because it is not recognized by Win10
  5,  // button count 5
  0,  // hat switch count
  true, // X axis Aileron
  true, // Y axis Elevator
  true, // Z axis throttle
  true,    // x rotation CH6
  false,   // y rotation
  false,   // z rotation
  true,    // rudder
  false,   // throttle
  false, false, false);  // no accelerator, no brake, no steering

void decodeChannels() {
int bitPtr;   // bit pointer in SBUS byte being decoded
int bytePtr;  // byte pointer in SBUS frame
int chan;     // channel number being decoded
int chanBit;  // current channel bit being proccessed

  channel[0]= frame[23];
  bytePtr= 1;
  bitPtr= 0;
  for (chan= 1; chan <= 16; chan++){
    channel[chan]= 0;
    for (chanBit= 0; chanBit < 11; chanBit++) {
      channel[chan] |= ((frame[bytePtr] >> bitPtr) & 1) << chanBit;
      if (++bitPtr > 7) {
        bitPtr=0;
        bytePtr++;
      }
    }
  }
}

bool getFrame() {
  while (Serial1.available()) {
    sbusByte= Serial1.read();
    if (sbusByte == 0x0F) byteNmbr= 0;
    if (byteNmbr <= 24) {
      frame[byteNmbr]= sbusByte;
      byteNmbr++;
      if ((byteNmbr == 25) && (sbusByte == 0) && (frame[0] == 0x0F)) return true;  // byteNmbr is now invalid, ready for next frame
    }
  }
  return false;
}

void setup() {
  Serial1.begin(100000, SERIAL_8E2);  // SBUS baud rate
  byteNmbr= 255;  // invalidate current frame byte
  Joystick.setXAxisRange(0, 2047);    // Aileron
  Joystick.setYAxisRange(0, 2047);    // Elevator
  Joystick.setZAxisRange(0, 2047);
  Joystick.setRxAxisRange(0, 2047);   // Channel 6
//  Joystick.setRyAxisRange(0, 2047);
  Joystick.setRudderRange(0, 2047);
//  Joystick.setThrottleRange(0, 2047);
  for (i= 1; i <= maxChan; i++) lastReported[i]= 0;
  lastReception= 0;
  Joystick.begin();
}

void loop() {
  if (getFrame()) {
    lastReception= millis();
    decodeChannels();
    newFsMode= ((channel[3] < signalLost) || (channel[0] & 8)); // if signal lost prepare to doReset
    for (i= 1; i <= maxChan; i++) if (lastReported[i] != channel[i]) {
      switch (i) {
        case 1: // Aileron
          Joystick.setXAxis(channel[i]);
          break;
        case 2: // Elevator
          Joystick.setYAxis(channel[i]);
          break;
        case 3: // throttle
          Joystick.setZAxis(channel[i]);
          break;
        case 4: // Rudder
          Joystick.setRudder(channel[i]);
          break;
        case 5:
          if (channel[i] <= 1023)
            Joystick.releaseButton(0);
          else
            Joystick.pressButton(0);
          break;
        case 6:
          Joystick.setRxAxis(channel[i]);
          break;
        case 7:
          if (channel[i] <= 1023)
            Joystick.releaseButton(1);
          else
            Joystick.pressButton(1);
          break;
        case 8:
          if (channel[i] <= 500) {
            Joystick.releaseButton(4);
            Joystick.pressButton(3);
          }
          else if (channel[i] > 1500) {
            Joystick.releaseButton(3);
            Joystick.pressButton(4);
          }
          else {
            Joystick.releaseButton(3);
            Joystick.releaseButton(4);
          }
          break;
      }
      lastReported[i]= channel[i];
    }
  }
  if (lastReception != 0) if ((millis() - lastReception) > timeOutMs) {
    newFsMode= true;
    lastReception= 0;
  }
  if (oldFsMode != newFsMode) { // detected signal lost
    oldFsMode= newFsMode;
    if (!newFsMode) { // exit failsafe mode
    // some flight sims require joystick button 3 pressed as "reset"
      Joystick.pressButton(2);
      delay(fsResetTime);
      Joystick.releaseButton(2);
    }
  }
}