// SBUS to PPM and PWM decoder
// (c) 2023 Pablo Montoreano

/*********************************************************
  @file       SBUS2PPM8PWM12.ino
  @brief      SBUS to PPM protocol converter and PWM servo driver
  @author     Pablo Montoreano
  @copyright  2023 Pablo Montoreano

  no 3rd party libraries used
  no additional hardware used (PCA9685 not necessary)

  Arduino Pro Micro with 1 PPM output and 9 PWM servo channels (debugging possible).
  Arduino Nano or UNO supported with 1 PPM and 12 PWM servo outputs but no debugging possible.
*********************************************************/

// 8 channel PPM output on Nano pin A0, Pro Micro pin 14
// 12 PWM Servo outputs on Nano pins D2 to D13
// or 9 PWM Servo outputs on Pro Micro pins 2 to 10
// signal LED (ON= no signal) on Nano or Pro Micro pin A3.
// failsafe LED (ON= failsafe) on Nano pin A4 or Pro Micro pin A0.
// low resolution LED on Nano pin A5 or Pro Micro pin A1.
// config pin A2 (Nano) or pin 16/MOSI (Pro Micro). Use 10K pullup, and jumper to ground

// Use 470 Ohm resistors for all LEDs

// **** PPM ****
// PPM signal is idle high, low 0.5 ms (start), 0.400 to 1.600 milliseconds channel pulse, 0.5 ms channel separation
// PPM output will be driven LOW when there is no signal or SBUS reports failsafe
// PPM pulse train separation is 10.5 ms.
// Max time for an 8 channel PPM train where all pulses are 1.6mS is 27.8mS
// PPM High/Low resolution (2048/1024) config jumper (1024 when pulled LOW)
// this config is used to reduce PPM flight simulator shaking

// **** Servos PWM ***
// use suitable external power supply for Servos ***do NOT use USB power***
// PWM servo pulses are 900 to 2100uS, 20mS minimum pulse separation
// output of 12 servos is done concurrently within the first 2.1mS (max pulse width)
// servo pulse separation is not critical so the 20mS depend on the end of the PPM stream

#include <avr/wdt.h>  // we need to disable watchdog interruptions during servo output

#define LEDON HIGH
#define LEDOFF LOW
static const unsigned int PPM_out= 14; // PPM output port (A0)
static const unsigned int portCfg= 16; // resolution configuration (A2), jumper to GND= 10 bits
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
static const unsigned int trainSep= 21000-5000;  // 10500 uS * 2 MHz (0.5 uS per tick using timer prescaler= 8) - 2.5mS overhead
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
static unsigned int ppm[maxChan+1];  // double buffering for interrupt data
static unsigned long lastReception;  // millis of last reception
static unsigned long lastServoPulse;
static unsigned int tempPulse;
static unsigned int numPulses;
static bool resol1024;  // low (1024) resolution flag
static bool gotData,updData;
static bool failsafeMode;
static bool noSignalMode;
static unsigned int i,j; // general counters

struct pulseArray {
  unsigned int pulseWidth;  // pulse width in uS
  unsigned int outputPin;   // pin number
  unsigned int outputLevel; // LOW or HIGH
};

// array dimension: pulses for servos + 1 for array[0] + 2 pulses for each PPM + 2 for trainSep
static pulseArray pulses[maxServos+1+((maxChan+1)<<1)];

void initTimer1() {
// setup timer prescaler to 8. This is equivalent to a 2MHz timer (16MHz/8), equal to 0.5uS per tick
  cli();  // clear interrupts
  TCCR1A= 0;  // disable compare mode
  TCCR1B= 0;  // no clock source (timer stopped)
  TCNT1= 0;
  TCCR1B= bit(CS11);  // only CS11 -> prescaler = 8
  //  CS12    CS11    CS10
  //  0       0       0     no clock source, timer stopped
  //  0       0       1     no prescale
  //  0       1       0     clk/8
  //  0       1       1     clk/64
  //  1       0       0     clk/256
  //  1       0       1     clk/1024
  //  1       1       0     extclk falling
  //  1       1       1     extclk rising
  TIMSK1= 0;  // disable timer1
  sei();  // enable interrupts
}

void addPulse(unsigned int thisPW, unsigned int thisOP, unsigned int thisOL) {  // add a pulse to the list
  numPulses++;
  pulses[numPulses].pulseWidth= thisPW;
  pulses[numPulses].outputPin= thisOP;
  pulses[numPulses].outputLevel= thisOL;
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

void sortPulses() { // a simple bubble sort
  for (i= 1; i < numPulses; i++) {
    for (j= i+1; j <= numPulses; j++) {
      if (pulses[j].pulseWidth < pulses[i].pulseWidth) {
        tempPulse= pulses[i].pulseWidth;
        pulses[i].pulseWidth= pulses[j].pulseWidth;
        pulses[j].pulseWidth= tempPulse;
        tempPulse= pulses[i].outputPin;
        pulses[i].outputPin= pulses[j].outputPin;
        pulses[j].outputPin= tempPulse;
        tempPulse= pulses[i].outputLevel;
        pulses[i].outputLevel= pulses[j].outputLevel;
        pulses[j].outputLevel= tempPulse;
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
  digitalWrite(LED_noSignal, LEDON);  // assum there is no signal on startup
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
  gotData= false;
  updData= false;
  lastServoPulse= millis();
  initTimer1(); // initialize timer registers
}

void loop() {
  if (getFrame()) {
    lastReception= millis();  // we have just received this
    decodeChannels(); // decode channel bitstream into 0 - 2047 channel values from frames into channels[i]
    resol1024= (digitalRead(portCfg) == LOW); // can change resolution while running
    digitalWrite(LED_lowRes, resol1024 ? LEDON : LEDOFF); // reflect in LED
    for (i= 1; i <= maxChan; i++) {
      // each PPM channel pulse is 400 to 1600 uS while SBUS data is 0 to 2047, ajust values to timer
      // ((int) (channel[i] * 1200.0 / 2047.0) + 400) << 1; simplified with a single float operation
      // We multiply by 2 with the << 1 because we have 2 ticks per uS in the timer and we are working in ticks
      // as an alternative we can use map(channel[i],0,2047,800,3200);
      // or map(channel[i] >> 1,0,1023,800,3200); for half resolution
      if (resol1024)
        ppm[i]= 800 + (int) ((channel[i] >> 1) * 2.344895);  // low (1024) resolution
      else
        ppm[i]= 800 + (int) (channel[i] * 1.172447);  // full 2048 resolution
    }
    noSignalMode= false;
    digitalWrite(LED_noSignal, LEDOFF);
    if ((channel[3] < signalLost) || (channel[0] & 8)) { // if signal lost disable PPM out and enable failsafe
      digitalWrite(LED_failsafe, LEDON);
      lastReception= 0; // we use lastReception as a no signal flag
      failsafeMode= true;
    }
    else if (failsafeMode) { // if we got signal and PPM is idle
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
    updData= true; // need to update servo data with failsafe
  }
  if (noSignalMode || failsafeMode) digitalWrite(PPM_out, LOW); // invalidate PPM output
  if (updData) {
// prepare pulses for the servos
    updData=false;
    numPulses= 0;
    // map channels (0-2047) into servo pulses in ticks (900-2100 uS x2)
    for (i= 1; i <= maxServos; i++) addPulse(((resol1024) ? map(channel[i] >> 1,0,1023,1800,4200) : map(channel[i],0,2047,1800,4200)),i+1,LOW);
    if (noSignalMode) pulses[3].pulseWidth= localFailsafe;  // if no signal we can't use SBUS failsafe
    if (!(noSignalMode || failsafeMode)) {  // if we got new data -> PPM active
      tempPulse= 0;
      for (i= 1; i <= maxChan; i++) { // create PPM stream in contiguous pulses (time added, not concurrent)
        tempPulse+= chanSep;
        addPulse(tempPulse, PPM_out, HIGH); // start for channel
        tempPulse+= ppm[i];
        addPulse(tempPulse, PPM_out, LOW);
      }
      tempPulse+= chanSep;
      addPulse(tempPulse, PPM_out, HIGH);
      tempPulse+= trainSep;
      addPulse(tempPulse, PPM_out, HIGH); // this will not change PPM output, but will make sure 10.5mS elapsed
    }
    sortPulses(); // sort the pulses so we can output in ascending (time) order
  }

// even if there is no signal we drive the servos (in case failsafe is active channel 3 should have changed)
  if (((millis() - lastServoPulse) >= servoSep) && gotData) { // if at least 20mS elapsed
    lastServoPulse= millis() + 1; // shortest servo pulse is about 1 mS, start counting servo pulse separation
    // servo pulses (concurrent)
    wdt_reset();  // lets make sure the watchdog does not interrupt this
    digitalWrite(PPM_out, LOW); // start (or disable) PPM
    for (i= 1; i <= maxServos; i++) digitalWrite(i+1, HIGH); // start pulse for all servos
    TCNT1= 0; // reset timer1 to start measuring pulses
    for (i= 1; i <= numPulses; i++) {
      while (TCNT1 < pulses[i].pulseWidth); // hang here until pulse[i] expires (sorted)
      digitalWrite(pulses[i].outputPin, pulses[i].outputLevel); // ok, pulse time expired
    }
    wdt_reset();
    yield(); // phew, free some CPU time now (not necessary because we are at the end of the loop, but doesn't hurt)
  }
}
