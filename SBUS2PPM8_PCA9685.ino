// SBUS to PPM and PCA9685 (with 16 servos) converter
// (c) 2023 Pablo Montoreano

/*********************************************************
  @file       SBUS2PPM8_PCA9685.ino
  @brief      SBUS to PPM protocol converter and PCA9685 Servo driver
  @author     Pablo Montoreano
  @copyright  2023 Pablo Montoreano
  @version    1.1 - 05/oct/23 - bug fix (0x0F is a valid SBUS value)

  3rd party library: Adafruit_PWM_Servo_Driver_Library (3.0.0) - PCA9685 library

  for Arduino Pro Micro (ATMega32U4) or Arduino Leonardo (I needed at least 2 serial ports for debugging)
  added support for Arduino Nano or UNO (no debugging possible)
  Compile as Arduino Leonardo if using a Pro Micro
*********************************************************/

// define PPM output, LED and config ports in constants below

// PPM signal is idle high, low 0.5 ms (start), 0.400 to 1.600 milliseconds channel pulse, 0.5 ms channel separation
// pulse train separation is 10.5 ms

// 16 servo outputs are sent to PCA9685. See the Adafruit library for explanation and calibration

// connect PCA9685 to:
// Pro Micro    -> SDA pin   2, SCL pin 3
// Leonardo     -> SDA pin  D2, SCL pin D3
// Arduino Nano -> SDA pin  A4, SCL pin A5
// Arduino UNO  -> SDA pin D18, SCL pin D19

// if you want precision and resolution, the PCA9585 is *NOT* a good solution. Not only does the PCA9685 require
// calibration because of low quality crystals, but we are using 12% of the SBUS' 11 bits.
// Explanation:
// we are using the PCA9685 at 50Hz because we need 20mS (1/50 sec= 20mS) for the servos pulses and pulse separation.
// the PCA9685 has 12 bits of resolution which gives us 4095 values (0 is not valid). Dividing the 20mS by 4095 we get
// about 4.88 microseconds per PCA9685 tick. Since servos operate in the range of 900 to 2100 uS, we have 2100-900= 1200 uS
// bandwidth, which in turn gives us 1200/4.88= 245 valid positions out of the SBUS 2048 possible values for each servo.

// PCA9685 lib declarations
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// end PCA9685

#define SHOWSIGNAL
#define LEDON HIGH
#define LEDOFF LOW
static const unsigned int PPM_out= 9;  // PPM output port (Pro Micro port A9)
static const unsigned int portCfg= 10; // resolution configuration (Jumper= 10 bits)
static const unsigned int LED_noSignal= 8;  // low speed indicator LED
static const unsigned int maxChan= 8;  // number of PPM channels in pulse train
static const unsigned int maxServos= 16; // number of Servos in PCA9685
static const unsigned int trainSepStart= (maxChan+1)<<1;  // start of pulse train separation
static const unsigned int trainSep= 21000; // 10500 uS * 2 MHz (0.5 uS per tick using timer prescaler= 8)
static const unsigned int chanSep= 1000;   // 500 uS * 2 MHz
// failsafe in channel 3 when value below signalLost
// for chinese receivers that do not report failsafe in SBUS (i.e. Microzone MC9002)
static const unsigned int signalLost= 100;
static const unsigned long timeOutMs= 1000; // timeout if no packet received after 1 sec
static const unsigned long SBUSbaudRate= 100000; // SBUS baudrate 100K

static unsigned int sbusByte, byteNmbr;
static byte frame[25];  // SBUS frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int ppm1[maxChan+1], ppm2[maxChan+1];  // double buffering for interrupt data
static volatile unsigned int pTrain;  // pulse train pointer
static bool lock1;  // use pulse train 2 when 1 locked
static bool resol1024;  // low resolution. lose a bit for a steadier output using 10 bits instead of 11
static unsigned long lastReception;  // millis of last reception
static unsigned int i; // general counter
static bool newFrame;

// Timer1 interrupt
ISR (TIMER1_COMPA_vect) {
  TIMSK1= 0;  // disable timer
  if (pTrain == 0)
    digitalWrite(PPM_out, LOW);  // disable PPM output
  else  {
    pTrain++;
    if (pTrain < trainSepStart) { // still processing channels
      if (pTrain & 1) { // now odd, process channel separation
        digitalWrite(PPM_out, LOW);
        OCR1A= chanSep; // channel separation pulse= 0.5mS
      }
      else {
        digitalWrite(PPM_out, HIGH); // start of channel output (0.4 to 1.6 mS)
        // pTrain counts both the pulse separations and the channel pulses, that's why we have to divide
        // by two the ppm pulses index with the >> 1
        if (lock1) OCR1A= ppm2[pTrain >> 1];  // if ppm1 array is being written to use ppm2
        else OCR1A= ppm1[pTrain >> 1];
      }
    }
    else if (pTrain > trainSepStart) {  // end of 10.5mS pulse train separation, start a channel separation pulse
      digitalWrite(PPM_out, LOW);
      OCR1A= chanSep; // channel separation pulse= 0.5mS
      pTrain= 1; // start new pulse train
    }
    else {  // pTrain == trainSepStart (start pulse train separation)
      digitalWrite(PPM_out, HIGH);
      OCR1A= trainSep;
    }
    TIMSK1= bit(OCIE1A);  // start timer
  }
}

void initTimer1() {
// setup timer prescaler to 8. This is equivalent to a 2MHz timer (16MHz/8), equal to 0.5uS per tick
  cli();  // clear interrupts
  TCCR1A= 0;  // disable compare mode (for timer 3 use TCCR3A)
  TCCR1B= 0;  // no clock source (timer stopped)
  TCNT1= 0;
  OCR1A= trainSep; // defaults to pulse separation
  TCCR1B= bit(WGM12) | bit(CS11);  // WGM12 -> CTC(Clear Timer on Compare Match), only CS11 -> prescaler = 8
  //  CS12    CS11    CS10
  //  0       0       0     no clock source, timer stopped
  //  0       0       1     no prescale
  //  0       1       0     clk/8
  //  0       1       1     clk/64
  //  1       0       0     clk/256
  //  1       0       1     clk/1024
  //  1       1       0     extclk falling
  //  1       1       1     extclk rising
  TIMSK1= 0;  // disable timer
  sei();  // enable interrupts
}

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
      if (++bitPtr > 7) { // change byte every 8 bits
        bitPtr=0;
        bytePtr++;
      }
    }
  }
}

bool getFrame() {
#ifdef __AVR_ATmega32U4__ // Arduino Pro Micro expected
  while (Serial1.available()) {
    sbusByte= Serial1.read();
#endif
#ifdef __AVR_ATmega328P__ // Arduino nano detected, use Serial only
  while (Serial.available()) {
    sbusByte= Serial.read();
#endif
// Bug fix: 0x0F is a valid value in the SBUS stream
// so we use a flag to detect the end of a packet (0) before enabling the capture of next frame
    if ((sbusByte == 0x0F) && newFrame) { // if this byte is SBUS start byte start counting bytes
      newFrame= false;
      byteNmbr= 0;
    }
    else if (sbusByte == 0) newFrame= true; // end of frame, enable start of next frame (to distinguish from 0x0F channel values)
    if (byteNmbr <= 24) { // 25 bytes total
      frame[byteNmbr]= sbusByte;  // save a byte
      byteNmbr++;
// if a valid frame is complete (check pointer position, start byte 0F and end byte 0)
      if ((byteNmbr == 25) && (sbusByte == 0) && (frame[0] == 0x0F)) return true;  // byteNmbr is now > 24, so this routine will now wait for next frame
    }
  }
  return false; // keep buffering
}

void setup() {
  pinMode(PPM_out, OUTPUT);
  digitalWrite(PPM_out, LOW);  // disable PPM output
  pinMode(LED_noSignal, OUTPUT);
#ifdef SHOWSIGNAL
  digitalWrite(LED_noSignal, LEDON);
#else
  digitalWrite(LED_noSignal, LEDOFF);
#endif
  pinMode(portCfg, INPUT_PULLUP);
// PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos updated @59 Hz (about 20mS between pulses)
// end PCA9685
#ifdef __AVR_ATmega32U4__  // Arduino Pro Micro
  Serial1.begin(SBUSbaudRate, SERIAL_8E2);
#endif
#ifdef __AVR_ATmega328P__ // Arduino Nano
  Serial.begin(SBUSbaudRate, SERIAL_8E2);
#endif
  byteNmbr= 255; // invalidate SBUS byte number
  lastReception= 0;
  newFrame= false;
  pTrain= 0;  // idle
  lock1= true;
  initTimer1(); // initialize timer registers
}

void loop() {
  if (getFrame()) {
    lastReception= millis();
    decodeChannels(); // decode channel bitstream into 0 - 2047 channel values
    resol1024= (digitalRead(portCfg) == LOW); // can change resolution while running
#ifndef SHOWSIGNAL    
    digitalWrite(LED_noSignal, (resol1024) ? LEDON : LEDOFF); // reflect in LED
#endif
    for (i= 1; i <= maxChan; i++) {
      // each PPM channel pulse is 400 to 1600 uS while SBUS data is 0 to 2047, ajust values to timer
      // ((int) (channel[i] * 1200.0 / 2047.0) + 400) << 1; simplified with a single float operation
      // We multiply by 2 with the << 1 because we have 2 ticks per uS in the timer and we are working in uS
      // as an alternative we can use map(channel[i],0,2047,800,3200);
      // or map(channel[i] >> 1,0,1023,800,3200); for half resolution
      if (resol1024) {
        if (lock1) ppm1[i]= 800 + (int) ((channel[i] >> 1) * 2.344895);  // low (1024) resolution
        else ppm2[i]= 800 + (int) ((channel[i] >> 1) * 2.344895);
      }
      else {
        if (lock1) ppm1[i]= 800 + (int) (channel[i] * 1.172447);  // full 2048 resolution
        else ppm2[i]= 800 + (int) (channel[i] * 1.172447);
      }
    }
    lock1= !lock1;  // new stream is now valid, switch it. Timer interrupt will output next channel from new reading
// PCA9685
// map the SBUS 11 bits to 900 to 2100 uS servo pulses
    for (i=1 ; i <= maxServos; i++) pwm.writeMicroseconds(i-1, map(channel[i],0,2047,900,2100));
// end PCA9685
    if ((channel[3] < signalLost) || (channel[0] & 8)) { // if signal lost disable PPM out
      pTrain= 0;
      TIMSK1= 0;
      digitalWrite(PPM_out, LOW);
#ifdef SHOWSIGNAL
      digitalWrite(LED_noSignal, LEDON);
#endif
    }
    else if (pTrain == 0) { // if we got signal and PPM is idle
      OCR1A= trainSep;  // start train separation pulse= 10.5mS (1/3)
      pTrain= trainSepStart; // start new channel separation (PPM output was LOW)
      digitalWrite(PPM_out, HIGH); // start pulse for channel sep
      TCNT1= 0; // reset timer counter to 0
      TIMSK1= bit(OCIE1A);  // start timer
#ifdef SHOWSIGNAL
      digitalWrite(LED_noSignal, LEDOFF);
#endif
    }
  }
  if (lastReception != 0) if ((millis() - lastReception) > timeOutMs) lastReception= 0;
  if (lastReception == 0) {
    pTrain= 0;
    TIMSK1= 0;
    digitalWrite(PPM_out, LOW);
#ifdef SHOWSIGNAL
    digitalWrite(LED_noSignal, LEDON);
#endif
  }
}
