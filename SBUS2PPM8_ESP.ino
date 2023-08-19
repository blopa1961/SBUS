// SBUS to PPM converter
// (c) 2023 Pablo Montoreano

/*********************************************************
  @file       SBUS2PPM8_ESP.ino
  @brief      SBUS to PPM protocol converter
  @author     Pablo Montoreano
  @copyright  2023 Pablo Montoreano

  no 3rd party libraries used

  for ESP01, nodeMCU, Wemos D1 mini or any ESP8266 (no debugging possible)
  ESP8266 Boards Manager version 3.1.2 compiles OK with default MMU (32K cache + 32Kb IRAM)
*********************************************************/

// PPM signal is idle high, low 0.5 ms (start), 0.400 to 1.600 milliseconds channel pulse, 0.5 ms channel separation
// pulse train separation is 10.5 ms

#define SHOWSIGNAL  // if defined LED will light when there is no radio signal, else it will reflect the PPM resolution mode (1024 ON or 2048 OFF)
#define LEDOFF HIGH
#define LEDON LOW
static const unsigned int PPM_out= 2;  // PPM output port
static const unsigned int portCfg= 1;  // resolution configuration (Jumper= 10 bits) GPIOTX
static const unsigned int LED_noSignal= 0;  // LED (low speed or lost signal)
static const unsigned int maxChan= 8;  // number of PPM channels in pulse train
static const unsigned int trainSepStart= (maxChan+1)<<1;  // 2*(maxChan+1) start of train separation pulse pointer
static const unsigned int trainSep= 52500; // 10500 uS * 5 MHz DIV16
static const unsigned int chanSep= 2500;   // 500 uS * 5 MHz
// failsafe in channel 3 when value below signalLost
// for chinese receivers that do not report failsafe in SBUS (i.e. Microzone MC9002)
static const unsigned int signalLost= 100;
static const unsigned long timeOutMs= 1000; // timeout if no packet received after 1 sec
static const unsigned long SBUSbaudRate= 100000; // SBUS baudrate 100K

static unsigned int sbusByte, byteNmbr;
static byte frame[25];  // SBUS frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int ppm1[maxChan+1], ppm2[maxChan+1];  // double buffering for interrupt data
static volatile unsigned int pTrain;  // pulse train pointer (volatile because it is modified by ISR)
static bool lock1;  // use pulse train 2 when 1 locked
static bool resol1024;  // low resolution. lose a bit for a steadier output using 10 bits instead of 11
static unsigned long lastReception;  // millis of last reception
static unsigned int i; // general counter

// Timer1 interrupt
// default MMU (32K cache + 32Kb IRAM) will work ok
void IRAM_ATTR timerRoutine() {
  if (pTrain == 0)
    digitalWrite(PPM_out, LOW);  // disable PPM output
  else  {
    pTrain++;
    if (pTrain < trainSepStart) { // still processing channels
      if (pTrain & 1) { // now odd, process channel separation
        digitalWrite(PPM_out, LOW);
        timer1_write(chanSep); // channel separation pulse= 0.5mS
      }
      else {
        digitalWrite(PPM_out, HIGH); // start of channel output (0.4 to 1.6 mS)
        // pTrain counts both the pulse separations and the channel pulses, that's why we have to divide
        // by two the ppm pulses index with the >> 1
        if (lock1) timer1_write(ppm2[pTrain >> 1]);  // if ppm1 array is being written to use ppm2
        else timer1_write(ppm1[pTrain >> 1]);
      }
    }
    else if (pTrain > trainSepStart) {  // end of 10.5mS pulse train separation, start a channel sep pulse
      digitalWrite(PPM_out, LOW);
      pTrain= 1; // start new pulse train
      timer1_write(chanSep); // channel separation pulse= 0.5mS
    }
    else {  // ptrain == trainSepStart
      digitalWrite(PPM_out, HIGH);
      timer1_write(trainSep); // start pulse train separation
    }
  }
}

void decodeChannels() {
int bitPtr;   // bit pointer in SBUS byte being decoded
int bytePtr;  // byte pointer in SBUS frame
int chan;     // channel number being decoded
int chanBit;  // current channel bit being proccessed

  channel[0]= frame[23];
  bytePtr= 1;
  bitPtr=0;
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
  while (Serial.available()) {
    sbusByte= Serial.read();
    if (sbusByte == 0x0F) byteNmbr= 0;  // if this byte is SBUS start byte start counting bytes
    if (byteNmbr <= 24) { // 25 bytes total
      frame[byteNmbr]= sbusByte;  // save a byte
      byteNmbr++;
 // if a valid frame is complete (check pointer position, start byte 0F and end byte 0)
      if ((byteNmbr == 25) && (sbusByte == 0) && (frame[0] == 0x0F)) return true;
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
  Serial.begin(SBUSbaudRate, SERIAL_8E2);
  delay(100);
// since we do not transmit anything, after the Serial port is initialized we can change TX pin to input and use it for a config jumper
  pinMode(portCfg, FUNCTION_3);  // convert TX to GPIO 1 (serial RX still works ok)
  pinMode(portCfg, INPUT_PULLUP);
  byteNmbr= 255; // invalidate SBUS byte number
  lastReception= 0;
  pTrain= 0;  // idle
  lock1= true;
  timer1_isr_init();
  timer1_attachInterrupt(timerRoutine);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
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
      // TIM_DIV16 at 80MHz gives 0.2 uS per tick, so we have 5 ticks per uS
      // each PPM channel pulse is 400 to 1600 uS while SBUS data is 0 to 2047, ajust values to timer
      // ((int) (channel[i] * 1200.0 / 2047.0) + 400) * 5; simplified with a single float operation
      // as an alternative we can use ppmx[i]= map(channel[i],0,2047,2000,8000);
      // or map(channel[i] >> 1,0,1023,2000,8000); for half resolution
      if (resol1024) {
        if (lock1) ppm1[i]= 2000 + (int) ((channel[i] >> 1) * 5.862237);  // low (1024) resolution
        else ppm2[i]= 2000 + (int) ((channel[i] >> 1) * 5.862237);
      }
      else {
        if (lock1) ppm1[i]= 2000 + (int) (channel[i] * 2.93112);  // full 2048 resolution
        else ppm2[i]= 2000 + (int) (channel[i] * 2.93112);
      }
    }
    lock1= !lock1;  // new stream is now valid, switch it. Timer interrupt will output next channel from new reading
    if ((channel[3] < signalLost) || (channel[0] & 8)) { // if signal lost disable PPM out
      pTrain= 0;
      timer1_disable();
      digitalWrite(PPM_out, LOW);
#ifdef SHOWSIGNAL
      digitalWrite(LED_noSignal, LEDON);
#endif
    }
    else if (pTrain == 0) { // if we got signal and PPM is idle
      pTrain= trainSepStart; // start new channel separation (PPM output was LOW)
      digitalWrite(PPM_out, HIGH); // start pulse for channel sep
      timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
      timer1_write(trainSep);
#ifdef SHOWSIGNAL
      digitalWrite(LED_noSignal, LEDOFF);
#endif
    }
  }
  if ((millis() - lastReception) > timeOutMs) lastReception= 0;
  if (lastReception == 0) {
    pTrain= 0;
    timer1_disable();
    digitalWrite(PPM_out, LOW);
#ifdef SHOWSIGNAL
    digitalWrite(LED_noSignal, LEDON);
#endif
  }
}
