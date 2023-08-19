# SBUS to PPM and PWM decoder using Arduino Interrupts
This project is not a library, it’s a set of programs used to decode Futaba’s Serial Bus (SBUS) protocol and output the received values via a Serial port, a PPM stream (for use with flight simulator USB dongles) and/or multiple PWM servo outputs. The input is always an inverted pulse train which must be connected to a hardware serial port in the target MCU.

![SBUS2PPM2PWM12_Nano](https://github.com/blopa1961/SBUS/blob/main/Images/SBUS2PPM2PWM12_Nano.jpg)

## Disclaimer:
No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.

In other words: use at YOUR OWN RISK, if you crash a 50000 dollar R/C Jet because an SBUS decoder fails or stops responding you are on your own. As a matter of fact, if you have a 50000 dollar R/C jet (or a 100 dollar balsa R/C plane for that matter) I suggest you buy a commercial SBUS decoder when needed.

## Target audience:
The project is targeted at radio control hobbyists who have knowledge of Arduino and its IDE. It’s not meant as a way to learn Arduino nor to teach how to setup a programming environment for the target MCU (like the ST-Link V2 necessary to program the bluepill MCU). I will assume you know all this and have some knowledge of electronics (resistors, transistors, Arduinos, etc.). You will need to build a signal inverter with a single NPN transistor and a couple of resistors. The inverter will also work as a level shifter from 5V to the 3.3V required by 3.3V devices like the ESP8266 (see both versions below).

The project is also targeted at Arduino developers who would like to learn and understand how to use bare bones hardware interrupts for various Arduino processors, namely Pro Micro (ATMega32U4), Nano (ATMega328P), STM32F103 (bluepill) and ESP8266 (ESP01, ESP12, nodeMCU, Wemos D1 mini, etc) by analyzing the source code which does not use third party libraries nor external calls.

## Description:
These interrupt routines are different for every MCU platform and sometimes require low level access to MCU registers (i.e. ATMega328P and ATMega32U4 which share the same register structure). Sometimes, interrupts are directly supported by the board manager (but not clearly documented), but overall, the code is highly incompatible between different MCU architectures.
To make the code clear I decided to create an individual sketch for each MCU architecture and refrained from using conditional ifs to compile a single sketch with multiple MCU architectures and different interrupt structures. You will find a list of target MCUs in the header of each INO file. Unless explicitly mentioned, no third party libs are used.

## SBUS to PPM:

•	SBUS_Decoder.ino: decodes SBUS and outputs the channel values and flags (frame lost, failsafe) to the serial port at 115200 baud. Requires an MCU with at least 2 serial ports. It will work with Arduino Pro Micro, Leonardo or STM32F103.

•	SBUS2PPM8.ino: decodes SBUS and outputs a flawless 8 channel PPM stream. Flawless because train separation is interrupt driven using double buffering and is exactly 10.5mS, not dependant on the MCU timing. It will work with Arduino Pro Micro, Leonardo, Nano or UNO (no debugging possible with Nano or UNO because they have a single Serial port).

•	SBUS2PPM8_ESP.ino: same as SBUS2PPM8 but using an ESP8266 MCU.

•	SBUS2PPM8_STM.ino: same as SBUS2PPM8 but using an STM32F103C8T6 MCU.

## SBUS to Joystick:

•	SBUS2Joy.ino: decodes SBUS and emulates an HID PC joystick. It does *not* emulate a copy protection dongle; you will need a free or properly licensed flight simulator supporting third party joysticks to use it. Uses the Joystick (2.1.1) library by Matthew Heironimus. Requires an MCU which can emulate HID devices like an Arduino Pro Micro or Leonardo.

## SBUS to PPM and PWM servos:
Warning: any sketches driving servos require a suitable external power supply. Do **NOT** connect servos to USB powered MCUs. You can test a small servo without loads using only USB power (if the MCU resets it’s because of the lack of power). Be warned that cheap servos like the SG90 or bootleg Tower Pros are not good at centering. The problem is the servo, not the timing; try using a good servo like the (original) MG92B (which I tested) or a Futaba servo.

•	SBUS2PPM8_PCA9685.ino: decodes SBUS, outputs a flawless 8 channel PPM stream (again double buffering and interrupts) and sends the data for 16 servos connected to a PCA9687 by use of Adafruit_PWM_Servo_Driver_Library (3.0.0) by Adafruit. The PCA9685 works, but if you want precision and resolution it is not the way to go. This sketch works with Pro Micro, Leonardo, Nano and UNO. It should also work with STM32F103 (untested).

•	SBUS2PPM8PWM12: decodes SBUS, outputs 8 channel PPM and up to 12 PWM servo signals. Works with Nano or UNO (12 servos + 1 PPM) and Pro Micro or Leonardo (9 servos + 1 PPM). All signal outputs are done in a tight loop, no interrupts used.

•	SBUS2PPM8PWM12INT: decodes SBUS, outputs a flawless 8 channel PPM and up to 12 PWM servo signals. Works with Nano or UNO (12 servos + 1 PPM) and Pro Micro or Leonardo (9 servos + 1 PPM). PPM signal is interrupt driven, PWM servo signals are output concurrently for all servos during the PPM train separation pause.

## Required signal inverter:

Signal inverter circuit

![Inverter_Sch](https://github.com/blopa1961/SBUS/blob/main/Images/Inverter_Sch.jpg)


PCB for 5V Inverter

![Inverter5_PCB](https://github.com/blopa1961/SBUS/blob/main/Images/Inverter5_PCB.jpg)


PCB for 5V to 3.3V level shifter inverter

![Inverter3_PCB](https://github.com/blopa1961/SBUS/blob/main/Images/Inverter3_PCB.jpg)

## Further reading:
I will soon be uploading a full description of this project, stay tuned.
