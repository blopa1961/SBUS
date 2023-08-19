// SBUS decoder - (c) 2023 Pablo Montoreano

/*********************************************************
  @file       SBUS_Decoder.ino
  @brief      SBUS protocol decoder
  @author     Pablo Montoreano
  @copyright  2023 Pablo Montoreano

  no 3rd party libraries used

  for Arduino Pro Micro (ATMega32U4) or Arduino Leonardo (2 serial ports are required)
  also works with STM32F103 (BluePill)
  does not work with Arduino Nano because it has a single serial port
*********************************************************/

// Arduino Pro Micro/Leonardo: connect inverted SBUS signal to RX1

// STM32F103: connect inverted SBUS to pin RXD1 (A10)
// enable CDC generic serial in Arduino IDE and connect USB to top connector

//  output to PC @115200 via USB

static unsigned int sbusByte, byteNmbr;
static byte frame[25];  // 25 bytes per SBUS frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int i; // a counter

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
  Serial.begin(115200);   // PC port speed
  Serial1.begin(100000, SERIAL_8E2);  // SBUS baud rate
  while (!Serial);  // wait for PC port ready
  Serial.println();
  Serial.println();
  byteNmbr= 255;  // invalidate current frame byte
}

void loop() {
  if (getFrame()) {
    decodeChannels();
    Serial.println();
    for (i= 1; i <= 16; i++) {
      Serial.print("Ch");
      Serial.print(i);
      Serial.print(":");
      Serial.print(channel[i]);
      Serial.print(" ");
    }
    Serial.print("Ch17:");
    Serial.print((channel[0] & 1) ? "H" : "L");
    Serial.print(" Ch18:");
    Serial.print((channel[0] & 2) ? "H" : "L");
    if (channel[0] & 4) Serial.print(" FL");  // frame lost
    if (channel[0] & 8) Serial.print(" FS");  // failsafe
  }
}