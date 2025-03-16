/*********************************************************
  @file       SBUS_Decoder.ino
  @brief      SBUS protocol decoder
  @author     Pablo Montoreano
  @copyright  2023~2015 Pablo Montoreano
  @version    1.2 - 10/Mar/25 added ESP32 support

  no 3rd party libraries used

  for Arduino Pro Micro, Leonardo, STM32F103 (bluepill) or ESP32-WROOM-32 (2 serial ports are required)
  output to PC @115200 via USB

  Inverter is not needed when using ESP32 (which supports hardware inversion)
*********************************************************/

// ESP32-WROOM-32: connect SBUS signal directly to RXD2 (GPIO16)
// select ESP32 Dev Module as target

// Arduino Pro Micro/Leonardo: connect inverted SBUS signal to RX1
// Select Arduino Leonardo as target

// STM32F103: connect inverted SBUS to pin RXD1 (A10)
// enable CDC generic serial in Arduino IDE and connect USB to top connector
// Select Bluepill F103C8 as target
// CH32F103C8T6: Compile with "Generic STM32F1 Series" and select "Bluepill F103CB (or C8 with 128K)"

static unsigned int sbusByte, byteNmbr;
static byte frame[25];  // 25 bytes per SBUS frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int i; // a counter
static bool newFrame;

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
// 0x0F is a valid value in the SBUS stream
// so we use a flag to detect the end of a packet (0) before enabling the capture of next frame
    if ((sbusByte == 0x0F) && newFrame) { // if this byte is SBUS start byte start counting bytes
      newFrame= false;
      byteNmbr= 0;
    }
    else if (sbusByte == 0) newFrame= true; // end of frame, enable start of next frame (to distinguish from 0x0F channel values)
    if (byteNmbr <= 24) {
      frame[byteNmbr]= sbusByte;
      byteNmbr++;
      if ((byteNmbr == 25) && (sbusByte == 0) && (frame[0] == 0x0F)) return true;  // byteNmbr is now invalid, ready for next frame
    }
  }
  return false;
}

void setup() {
#ifdef ESP32
  pinMode(16,INPUT);
  pinMode(17,OUTPUT);
  Serial1.begin(100000, SERIAL_8E2, 16, 17, true);  // baud rate, mode, RX pin, TX pin, inversion (TX is not used)
#else
  Serial1.begin(100000, SERIAL_8E2);  // these micros require hardware signal inversion
#endif
  Serial.begin(115200);   // PC port speed
  delay(500);
  while (!Serial);  // wait for PC port ready
  Serial.println();
#ifdef ESP32
  Serial.println("Running on ESP32");
#else
  Serial.println("Running on Pro Micro, Leonardo or STM32");
#endif  
  Serial.println();
  byteNmbr= 255;  // invalidate current frame byte
  newFrame= false;
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