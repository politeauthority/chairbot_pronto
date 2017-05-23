/******************************************************************************************
  _____  ___     __  ____    ___ ______  __ __      ____   ___   ___   ____  _        ___ 
 / ___/ /   \   /  ]|    |  /  _]      ||  |  |    |    \ /  _] /   \ |    \| |      /  _]
(   \_ |     | /  /  |  |  /  [_|      ||  |  |    |  Oc-mod   )  [_ |     ||  o  ) |     /  [_ 
 \__  ||  O  |/  /   |  | |    _]_|  |_||  ~  |    |   _/    _]|  O  ||   _/| |___ |    _]
 /  \ ||     /   \_  |  | |   [_  |  |  |___, |    |  | |   [_ |     ||  |  |     ||   [_ 
 \    ||     \     | |  | |     | |  |  |     |    |  | |     ||     ||  |  |     ||     |
  \___| \___/ \____||____||_____| |__|  |____/     |__| |_____| \___/ |__|  |_____||_____|
                                                                                          
Name: test_2.ino
Desc: First collected test, we'll clean this up eventually and probably rename it.
      Ideally, this test will make sure that the microcontroller, serial RS485 line
      and other basics components are runnning.
Test: We got this script releasing the breaks, however was not able to init the chair
      without the joystick inline and online.
*******************************************************************************************/


// This sketch is to emulate the packet of data that the Shark Joystick sends
/******************************************************************************
      Author: Tony Matthews ammatthews at gmail dot com
      BitMath coding by - Irving
      Also coding help from Woody and Lenny from the UK
      Building on work by Jim with his RoboBoard + distance sensor's wheelchair.
      License: FreeBSD
    ******************************************************************************/
/******************************************************************************
  Copyright (c) 2015, Tony Matthews
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.

*******************************************************************************/

// This code is incomplete and still under revision.
// Its also got junk that may not be needed,


// SP -----
const int start_pin = 12;
boolean start_has_run = LOW;

const int start_led = 11;

// Pin outs for RS485 chip
const int roPin = 7;        // to RO of Max485 to RX of Arduino
const int dePin = 9;        // to DE anr RE of Max485
const int diPin = 10;       // to DI of Max485 to TX of Arduino

// Pins for DG419 data switch, between data and 12(min)- 24v(full battery volts) start pulse.
const int dataSwitch = 6;    //  Digital swtich for DG419

// Led Indicator monitor
const int led = 13;

//Boolean name holder for current state.
boolean onOffState = LOW;

// pulseIn Digital input from RC Reciever
const int speedPin = 4;    // Speed Pot input value
const int directionPin = 3;    // Direction Pot input value
const int onOffPin = 5;      // Digital Pin for pulseIN CH3 from Remote Control

// Names for PPM values of RC inputs
word speedPotVal;
word directionPotVal;
word onOffVal;



// Names for Mapped analogRead Values of Pots
word speedPotValMapped;
word directionPotValMapped;

int Deadband = 200;

// Integers for Irvings code
unsigned char maxSpeed;   // unsigned char, same as byte (8 bit) - unsigned number from 0-255
word speed;        // int (16 bit) - signed number from -32768 to 32767
word direction;    // int (16 bit) - signed number from -32768 to 32767


// data packet
unsigned char data[14];
unsigned char idledata[9];
unsigned long timer = 0;


#include <SoftwareSerial.h>
SoftwareSerial sharkSerial (roPin, diPin, false);    // 'true' sets this to invert software serial output.


void setup() {
  sharkSerial.begin(38400);

  delay(300);
  pinMode(led, OUTPUT);        // Arduino led to monitor on or off state.
  
  pinMode(start_led, OUTPUT);
  pinMode(start_pin, INPUT);


  // Setup pin modes for RC input
  pinMode(onOffPin, INPUT);
  pinMode(speedPin, INPUT);
  pinMode(directionPin, INPUT);

  pinMode(dataSwitch, OUTPUT); // Set Data switch pin to Output
  digitalWrite(dataSwitch, LOW);  //Start Data in LOW = data connected to bus, High=15-24v connected to bus
  digitalWrite(dePin, LOW);   // Low = RX mode , High = TX mode
  onOffState = LOW ;

  Serial.begin(38400);
  while (! Serial); // Wait untilSerial is ready - Leonardo
  Serial.println("Serial monitor enabled");
  delay(500);
  onOffState = HIGH;
}

void loop() {
  // Starts in SOFTWARE Off state from physical power on.

  // Code to monitor 'Software' on/off button 'onOffPin'

  // Check state, if "on" , "turn off"
  Serial.println("Script Idle");
  if (onOffState == HIGH && start_has_run == true){
    digitalWrite(led, onOffState);    //Keep arduino LED 'on' as indication of mode arduino is in
    Serial.println("Run Start");
    societyRun();
    Serial.println("Run End");
  }
  Serial.println(digitalRead(start_pin));
  if (start_has_run == LOW && digitalRead(start_pin) == HIGH){
    societyPeopleStartup();
    delay(12);
    societyPacket1();
    delay(12);
    societyPacket2();
    start_has_run = true;
  }
  delay (12); // this is the time between packets , delay put here to be debounce for onoff switch.
}

void societyPeopleStartup () {
  Serial.println("Scoiety People start");
  digitalWrite(dePin, HIGH); // Hold dePin high when transmitting.
  {
    // test 1
    digitalWrite(dataSwitch, HIGH);
    digitalWrite(diPin, LOW);
    digitalWrite(roPin, HIGH);
    delay(298);
    digitalWrite(diPin, HIGH);
    digitalWrite(roPin, LOW);
    digitalWrite(dataSwitch, LOW);

  }
  {
    data[0] = (0x74);   //
    data[1] = (129);    //
    data[2] = (141);    //
    data[3] = (131);    //
    data[4] = (128);    //
    data[5] = (154);    //
    data[6] = (223);    //
    data[7] = (167);    //
    data[8] = (128);    //
    data[9] = (218);
    data[10] = (15);
    data[11] = (5);
    data[12] = (160);
    data[13] = (218);
    data[14] = (15);
    byte sum = 0;
    for (unsigned char i = 0; i < 11; i++){
      sharkSerial.write(data[i]);
    }
    delay(1);
    for (unsigned char i = 11; i < 14; i++){
      sharkSerial.write(data[i]);
    }
    delay(1);
    Serial.println(data[14]);
    sharkSerial.write(data[14]);
    
  }
  //delay(12);
  Serial.println("Shark Starkup fin");
  digitalWrite(start_led, HIGH);
}

void societyRun() {
  digitalWrite(dePin, HIGH);     // set MAX485 to High = TX active.
  societyPacket3();
  digitalWrite(dePin, LOW);     // set MAX485 to low = RX active.
}

void societyPacket1(){
    idledata[0] = (0x60);
    idledata[1] = 191; 
    idledata[2] = 192;
    idledata[3] = 255;
    idledata[4] = 251;
    idledata[5] = 128;
    idledata[6] = 140;
    idledata[7] = 128;
    idledata[8] = 154;
    idledata[9] = 15;
    for (unsigned char i = 0; i < 10; i++){
      sharkSerial.write(idledata[i]);      
    }
    delayMicroseconds(1500);
    // SECOND HALF OF PACKET
    idledata[0] = (0x61);
    idledata[1] = 128;
    idledata[2] = 128;
    idledata[3] = 128;
    idledata[4] = 128;
    idledata[5] = 128;
    idledata[6] = 128;
    idledata[7] = 128;
    idledata[8] = 158;
    idledata[9] = 15;
    for (unsigned char i = 0; i < 9; i++){
      sharkSerial.write(idledata[i]);      
    }
    delay(1);
    sharkSerial.write(idledata[9]);
}

void societyPacket2(){
    idledata[0] = (0x60);
    idledata[1] = 191; 
    idledata[2] = 192;
    idledata[3] = 255;
    idledata[4] = 251;
    idledata[5] = 128;
    idledata[6] = 140;
    idledata[7] = 128;
    idledata[8] = 172;
    idledata[9] = 15;
    for (unsigned char i = 0; i < 10; i++){
      sharkSerial.write(idledata[i]);      
    }
    delayMicroseconds(1500);
    // SECOND HALF OF PACKET
    idledata[0] = (0x61);
    idledata[1] = 128;
    idledata[2] = 192;
    idledata[3] = 128;
    idledata[4] = 128;
    idledata[5] = 128;
    idledata[6] = 128;
    idledata[7] = 128;
    idledata[8] = 222;
    idledata[9] = 26;
    for (unsigned char i = 0; i < 9; i++){
      sharkSerial.write(idledata[i]);      
    }
    sharkSerial.write(idledata[9]);
    delayMicroseconds(500);
    // THIRD PART OF PACKET
    idledata[0] = 133;
    idledata[1] = 167;
    idledata[2] = 185;
    idledata[3] = 15;
    for (unsigned char i = 0; i < 3; i++){
      sharkSerial.write(idledata[i]);      
    }
    delay(1);
    sharkSerial.write(idledata[3]);
}

void societyPacket3(){
    idledata[0] = (0x60);
    idledata[1] = 191; 
    idledata[2] = 192;
    idledata[3] = 255;
    idledata[4] = 234;
    idledata[5] = 128;
    idledata[6] = 140;
    idledata[7] = 128;
    idledata[8] = 171;
    idledata[9] = 15;
    for (unsigned char i = 0; i < 10; i++){
      sharkSerial.write(idledata[i]);      
    }
    delayMicroseconds(1500);
    // SECOND HALF OF PACKET
    idledata[0] = (0x61);
    idledata[1] = 128;
    idledata[2] = 192;
    idledata[3] = 128;
    idledata[4] = 128;
    idledata[5] = 128;
    idledata[6] = 128;
    idledata[7] = 128;
    idledata[8] = 222;
    idledata[9] = 15;
    for (unsigned char i = 0; i < 9; i++){
      sharkSerial.write(idledata[i]);      
    }
    delay(1);
    sharkSerial.write(idledata[9]);
}
