// SBUS to PPM and PWM decoder (with interrupt driven PPM)
// (c) 2023 Pablo Montoreano
// for Arduino Nano or UNO (no debugging possible).
// Arduino Pro Micro supported but limited to 9 PWM servo channels (debugging possible).

/*********************************************************
  @file       SBUS2PPM8PWM12INT.ino
  @brief      SBUS to PPM protocol converter (interrupt driven) and PWM servo driver
  @author     Pablo Montoreano
  @copyright  2023 Pablo Montoreano

  This version generates a flawless PPM stream using interrupts with double buffering for PPM data

  no 3rd party libraries used
  no additional hardware used (PCA9685 not necessary)

  Arduino Pro Micro with 1 PPM output and 9 PWM servo channels (debugging possible).
  Arduino Nano or UNO supported with 1 PPM and 12 PWM servo outputs but no debugging possible.
*********************************************************/

// 8 channel timer interrupt driven PPM output on Nano pin A0, Pro Micro pin 14
// 12 PWM Servo outputs on Nano pins D2 to D13
// or 9 PWM Servo outputs on Pro Micro pins 2 to 10
// signal LED (ON= no signal) on Nano or Pro Micro pin A3.
// failsafe LED (ON= failsafe) on Nano pin A4 or Pro Micro A0.
// low resolution LED on Nano pin A5 or Pro Micro pin A1.
// config pin A2 (Nano) or 16/MOSI (Pro Micro). Use 10K pullup, and jumper to ground

// Use 470 Ohm resistors for all LEDs

// **** PPM ****
// PPM signal is idle high, low 0.5 ms (start), 0.400 to 1.600 milliseconds channel pulse, 0.5 ms channel separation
// PPM output will be driven LOW when there is no signal or SBUS reports failsafe
// PPM pulse train separation is 10.5 ms. The whole PPM stream is continuous and driven by timer interrupts.
// Max time for an 8 channel PPM train where all pulses are 1.6mS is 27.8mS
// PPM High/Low resolution (2048/1024) config jumper (1024 when pulled LOW)
// this config is used to reduce PPM flight simulator shaking

// **** Servos PWM ***
// use suitable external power supply for Servos ***do NOT use USB power***
// PWM servo pulses are 900 to 2100uS, 20mS minimum pulse separation
// concurrent output of the 12 servos is done during PPM train separation using a max of 2.1mS of the 10.5mS train sep
// servo pulse separation is not critical so the 20mS pulse separation is measured in the main loop using millis()
// the servo pulses are delayed until the start of the PPM train separation if necessary

#include <avr/wdt.h>  // we need to disable watchdog during servo output

#define LEDON HIGH
#define LEDOFF LOW
static const unsigned int PPM_out= 14; // PPM output port (A0)
static const unsigned int portCfg= 16; // resolution configuration (A2), jumper= 10 bits
static const unsigned int LED_failsafe= 18; // falsafe LED (A4)
static const unsigned int LED_lowRes= 19;   // low resolution LED (A5)
#ifdef __AVR_ATmega32U4__
static const unsigned int LED_noSignal= 21; // no signal LED (A2)
static const unsigned int maxServos= 9; // only 9 servos (ports 2 to 10) if using Pro Micro
#endif
#ifdef __AVR_ATmega328P__
static const unsigned int LED_noSignal= 17; // no signal LED (A3)
static const unsigned int maxServos= 12;  // number of servos. We will output up to 12 servos in ports 2 to 13
#endif
static const unsigned int maxChan= 8;  // number of PPM channels in pulse train
static const unsigned int trainSepStart= (maxChan+1)<<1;  // 2*(maxChan+1) start of train separation pulse pointer
static const unsigned int trainSep= 21000;  // 10500 uS * 2 MHz (0.5 uS per tick using timer prescaler= 8)
static const unsigned int chanSep= 1000;    // 500 uS * 2 MHz
static const unsigned long servoSep= 20;     // servo pluse separation in millis
// failsafe in channel 3 when value below signalLost
// for chinese receivers that do not report failsafe in SBUS (i.e. Microzone MC9002)
static const unsigned int signalLost= 100;
static const unsigned int localFailsafe= (signalLost << 1)+1800; // 0.5uS per tick starting from 900uS
static const unsigned long timeOutMs= 1000;    // timeout if no packet received after 1 sec
static const unsigned long SBUSbaudRate= 100000; // SBUS baudrate 100K

static unsigned int sbusByte, byteNmbr;
static byte frame[25];  // SBUS frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int ppm1[maxChan+1], ppm2[maxChan+1];  // double buffering for interrupt data
static bool lock1;  // use pulse train 2 when 1 locked
static unsigned long lastReception;  // millis of last reception
static unsigned long lastServoPulse;
static unsigned int tempPulse;
static volatile unsigned int pTrain;  // pulse train pointer
static volatile bool inPause; // true during PPM train separation
static bool resol1024;  // low (1024) resolution flag
static bool gotData, updData;
static bool failsafeMode;
static bool noSignalMode;
static unsigned int i,j; // general counters

struct pulseArray {
  unsigned int pulseWidth;  // pulse width in uS
  unsigned int outputPin;   // pin number
};

static pulseArray pulses[maxServos+1];  // 10 pulses for servos + 1 for [0]
static unsigned int newPulseWithds[maxServos+1];

// Timer1 interrupt
ISR (TIMER1_COMPA_vect) {
  TIMSK1= 0;  // disable timer1 interrupt
  if (pTrain == 0)
    digitalWrite(PPM_out, LOW);  // disable PPM output (just in case)
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
      inPause= false;
    }
    else {  // pTrain == trainSepStart (start pulse train separation)
      digitalWrite(PPM_out, HIGH);
      OCR1A= trainSep;
      inPause= true;
    }
    TIMSK1= bit(OCIE1A);  // start timer interrupt
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
  TIMSK1= 0;  // disable timer1 interrupt
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
    if (sbusByte == 0x0F) byteNmbr= 0;  // if this byte is SBUS start byte start counting bytes
    if (byteNmbr <= 24) { // 25 bytes total
      frame[byteNmbr]= sbusByte;  // save a byte
      byteNmbr++;
// if a valid frame is complete (check pointer position, start byte 0F and end byte 0)
      if ((byteNmbr == 25) && (sbusByte == 0) && (frame[0] == 0x0F)) return true;
// byteNmbr is now > 24, so this routine will now wait for next frame
    }
  }
  return false; // keep buffering
}

void sortPulses() {
  for (i= 1; i < maxServos; i++) {
    for (j= i+1; j <= maxServos; j++) {
      if (pulses[j].pulseWidth < pulses[i].pulseWidth) {
        tempPulse= pulses[i].pulseWidth;
        pulses[i].pulseWidth= pulses[j].pulseWidth;
        pulses[j].pulseWidth= tempPulse;
        tempPulse= pulses[i].outputPin;
        pulses[i].outputPin= pulses[j].outputPin;
        pulses[j].outputPin= tempPulse;
      }
    }
  }
}

void setup() {
  pinMode(PPM_out, OUTPUT);
  digitalWrite(PPM_out, LOW);  // disable PPM output
  pinMode(LED_noSignal, OUTPUT);
  pinMode(LED_failsafe, OUTPUT);
  pinMode(LED_lowRes, OUTPUT);
  for (i= 2; i <= (maxServos + 1); i++) { // all servos to idle (LOW)
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  digitalWrite(LED_noSignal, LEDON);
  digitalWrite(LED_failsafe, LEDOFF);
  digitalWrite(LED_lowRes, LEDOFF);
  pinMode(portCfg, INPUT_PULLUP);
// Arduino Pro Micro pin definitions
#ifdef __AVR_ATmega32U4__
  Serial1.begin(SBUSbaudRate, SERIAL_8E2);
#endif
// Arduino nano pin definitions
#ifdef __AVR_ATmega328P__
  Serial.begin(SBUSbaudRate, SERIAL_8E2);
#endif
  byteNmbr= 255; // invalidate SBUS byte number
  lastReception= 0;
  failsafeMode= false;
  noSignalMode= true;
  pTrain= 0;  // idle
  lock1= true;
  inPause= false;
  gotData= false; // if we ever received data this is true
  updData= false; // true if data changed in the loop
  lastServoPulse= millis();
  initTimer1(); // initialize timer registers
}

void loop() {
  if (getFrame()) {
    lastReception= millis();
    decodeChannels(); // decode channel bitstream into 0 - 2047 channel values
    resol1024= (digitalRead(portCfg) == LOW); // can change resolution while running
    digitalWrite(LED_lowRes, resol1024 ? LEDON : LEDOFF); // reflect in LED if we are not showing signal
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
    noSignalMode= false;
    digitalWrite(LED_noSignal, LEDOFF);
    if ((channel[3] < signalLost) || (channel[0] & 8)) { // if signal lost disable PPM out
      lastReception= 0; // we use lastReception as a no signal flag
      digitalWrite(LED_failsafe, LEDON);
      failsafeMode= true;
    }
    else if (pTrain == 0) { // if we got signal and PPM is idle
      OCR1A= trainSep;  // start train separation pulse= 10.5mS (1/3)
      pTrain= trainSepStart; // start new channel separation (PPM output was LOW)
      digitalWrite(PPM_out, HIGH); // start pulse for channel sep
      TCNT1= 0; // reset timer counter to 0
      TIMSK1= bit(OCIE1A);  // enable Timer1 interrupts
      digitalWrite(LED_failsafe, LEDOFF);
      failsafeMode= false;
    }
    gotData= true;
    updData= true;
  }
  if (lastReception != 0) if ((millis() - lastReception) > timeOutMs) {  // SBUS cable disconnected
    lastReception= 0;
    digitalWrite(LED_noSignal, LEDON);
    noSignalMode= true;
    updData= true;
  }
  if (noSignalMode || failsafeMode) {
    TIMSK1= 0;  // disable timer1 interrupts
    pTrain= 0;  // PPM pulse train to idle
    digitalWrite(PPM_out, LOW); // invalidate PPM output
    TCNT1= 0; // start from 0. Timer interrupts are disabled (TIMSK1= 0 above)
    OCR1A= trainSep;  // timer has to keep running for servo pulses
    inPause= true;
  }
  if (updData)   {
    updData= false;
// prepare pulses for the servos
    for (i= 1; i <= maxServos; i++){
      if (resol1024)
        pulses[i].pulseWidth= map(channel[i] >> 1,0,1023,1800,4200);
      else
        pulses[i].pulseWidth= map(channel[i],0,2047,1800,4200);
      pulses[i].outputPin= i+1;
    }
    if (noSignalMode) pulses[3].pulseWidth= localFailsafe;  // if no signal we can't use SBUS failsafe
    sortPulses();
  }
// even if there is no signal we drive the servos (in case failsafe is active channel 3 should have changed)
  if ((millis() - lastServoPulse) < servoSep)
    inPause= false; // invalidate pause mode if servoSep has not yet elapsed
  else if (inPause && gotData) { // if at least 20mS elapsed and PPM is pausing output servo pulses
    // servo pulses (concurrent)
    inPause= false;
    wdt_reset();  // lets make sure the watchdog does not interrupt this
    for (i= 1; i <= maxServos; i++) {
// we can't touch the timer but we can use it. Add current timer counter to pulse width. We have a bit less than 10.5mS to work here
// also we can't modify pulses[i] because we don't know if it was updated in this loop or not
// we must not accumulate TCNT1 in pulseWidth, it has to be preserved for next loop in case we do not receive another frame
      newPulseWithds[i]= pulses[i].pulseWidth+TCNT1;
      digitalWrite(i+1, HIGH); // start pulse for all servos
    }
    for (i= 1; i <= maxServos; i++) {
      while (TCNT1 < newPulseWithds[i]); // hang here until pulse[i] expires (sorted and TCNT1 adjusted)
      digitalWrite(pulses[i].outputPin, LOW); // ok, pulse time expired, finish Servo pulse
    }
    wdt_reset();
    yield(); // free some CPU time now
    lastServoPulse= millis(); // all servo pulses are done, now wait 20mS for next pulse
  }
}
