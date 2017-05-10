/******************************************************************************************
  _____  ___     __  ____    ___ ______  __ __      ____   ___   ___   ____  _        ___ 
 / ___/ /   \   /  ]|    |  /  _]      ||  |  |    |    \ /  _] /   \ |    \| |      /  _]
(   \_ |     | /  /  |  |  /  [_|      ||  |  |    |  o  )  [_ |     ||  o  ) |     /  [_ 
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
NOTE: BEST WORKING TEST with A going LOW and B going HIGH

License: FreeBSD
Copyright (c) 2015, Tony Matthews
All rights reserved.

This sketch is to emulate the packet of data that the Shark Joystick sends

Author: Tony Matthews ammatthews at gmail dot com
BitMath coding by - Irving
Also coding help from Woody and Lenny from the UK
Building on work by Jim with his RoboBoard + distance sensor's wheelchair.

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
unsigned char data[12];
unsigned long timer = 0;


#include <SoftwareSerial.h>
SoftwareSerial sharkSerial (roPin, diPin, false);    // 'true' sets this to invert software serial output.


void setup() {
  sharkSerial.begin(38400);

  delay(300);
  pinMode(led, OUTPUT);        // Arduino led to monitor on or off state.

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
  sharkStartup();  
  onOffState = HIGH;
}

void loop() {
  // Starts in SOFTWARE Off state from physical power on.

  // Code to monitor 'Software' on/off button 'onOffPin'

  // Check state, if "on" , "turn off"
  
  // onOffVal = pulseIn(onOffPin, HIGH, 25000);
  // onOffVal = digitalRead(onOffPin);
  // if(onOffVal == HIGH){
  //     Serial.println("Im high");
  //     Serial.println(onOffVal);

  // }
  // if(onOffVal == LOW){
  //     Serial.println("Im low");
  //     Serial.println(onOffVal);
  // }


  if (onOffState == HIGH )
  {
    digitalWrite(led, onOffState);    //Keep arduino LED 'on' as indication of mode arduino is in
    sharkRun();
    Serial.println("Script Idle");
  }

  delay (16); // this is the time between packets , delay put here to be debounce for onoff switch.
}




// my Dynamic Shark start up sequence, other joysticks will have different start up packets.

void sharkStartup () {
  Serial.println("Shark Startup start");
  digitalWrite(dePin, HIGH); // Hold dePin high when transmitting.
  {
    digitalWrite(dataSwitch, HIGH);  // Flip data switch, HIGH = 24v is connected to emulator bus
    delay(298);         // Allow 24v pulse for 300ms +/- 20ms
    digitalWrite(dataSwitch, LOW);    // Flip dats switch, LOW = data connected to emulator bus
    delay(10);
  }

  {
    /*build start up packet
       factory notes the date of manufacture and such boring stuff...
    */

    data[0] = (0x74);   //
    data[1] = (130);    //
    data[2] = (133);    //
    data[3] = (130);    //
    data[4] = (128);    //
    data[5] = (136);    //
    data[6] = (205);    //
    data[7] = (160);    //
    data[8] = (128);    //
    byte sum = 0;
    for (int i = 0; i < 9; i++)
      sum += data[i] & 0x7f;
    data[9] = 0x7f - sum;       //Checksum  OK = (141)
    data[10] = (15);      // all packets end with this identifier

    Serial.println("Statup Data");
    for (unsigned char i = 0; i < 10; i++){
      Serial.println(data[i]);
      sharkSerial.write(data[i]);
    }
  }
  delay(12);
  Serial.println("Shark Starkup fin");
}


void sharkRun() {
  {
    digitalWrite(dePin, HIGH);     // set MAX485 to High = TX active.
    {
      directionPotVal = pulseIn(directionPin, HIGH);
      directionPotVal = constrain(directionPotVal, 1100, 1900);
      directionPotVal = map(directionPotVal, 1000, 2000, 1, 1023);  // Power base only recoginizes values in this range

     

      // Woody's Deadband code
      if (directionPotVal > (512 + Deadband)) //deadband defined as 200
        direction = map(directionPotVal, (512 + Deadband), 1024, 513, 1023); // Turn Right ??
      else if
      (directionPotVal < (512 - Deadband))
        direction = map(directionPotVal, 0, (512 - Deadband), 1, 511); //Turn Left ??
      else
        direction = 512;    // 512 = stop
    }
    {
      speedPotVal = pulseIn(speedPin, HIGH);
      speedPotVal = constrain(speedPotVal, 1000, 2000);
      speedPotVal = map(speedPotVal, 1000, 2000, 1023, 1);
      Serial.println(speedPotVal);


      // Woody's Deadband code
      if (speedPotVal > (512 + Deadband)){ //deadband defined as 200
        speed = map(speedPotVal, (512 + Deadband), 1024, 513, 1023); // Forwards ??
        Serial.println("speed");
        Serial.println(speed);
      }else if(speedPotVal < (512 - Deadband)){
        speed = map(speedPotVal, 0, (512 - Deadband), 1, 511);      // Reverse ??
        Serial.println("speed");
        Serial.println(speed);
      }else{
        speed = 512;    // 512 = stop
        Serial.println("speed");
        Serial.println(speed);
      }
    }


  }
  {
    /* Following are notes on Shark Remote data Byte information packets (what each byte holds data for)
        Type 00 SR General Information
      This packet contains the joystick speed and direction ( 10 bit readings), speed pot setting (8 bit reading), and input and status
      bits.
      The minimum allowable length for this packet is 6 data bytes.
      If the wheelchair motor controller (SPM) receives only bytes 0-5, and the checksum is correct, the SPM shall assume that it is operating with a previous
      version of the Shark Remote and take the following actions:
      􀁺 Disregard the Operating Mode in byte 5
      􀁺 Disable all lighting and actuator functionality.
      Note that these actions shall be taken regardless of whether the SR indicated in its power-up packet that it has lighting and/or
      actuator functionality.
    */


    /* Build "shark remote" data serial packet or in this case the emulator data packet
         Interpretation will be slotted in here for sensors added to the power chairs
    */
    maxSpeed = 255;     // this is a fixed value ...being full speed on the Shark Remote
    data[0] = (0x60);    // Packet type identifier

    // Assign joystick and speedpot MSBs values to data[1 ] - data[3], setting MSB = 1
    // Code thanks to Irvings bitgenius.

    // DYNAMIC DATA below
    // data[1] = 0x80 | ((speed >> 3) & 0x7f); // Joystick speed reading (7 MSbs)
    // data[2] = 0x80 | ((direction >> 3) & 0x7f); // Joystick direction reading (7 MSbs)
    // data[3] = 0x80 | ((maxSpeed >> 1) & 0x7f);   // Speed pot reading (7 MSbs)

    onOffVal = digitalRead(onOffPin);
    if(onOffVal == LOW){
      Serial.println("Im low");
      Serial.println(onOffVal);
      data[1] = 0xBF; // Joystick speed reading (7 MSbs) oXFF
      data[2] = 0xBF; // Joystick direction reading (7 MSbs)

    }
    if(onOffVal == HIGH){
        Serial.println("Im high");
        Serial.println(onOffVal);
        data[1] = 0xFF; // Joystick speed reading (7 MSbs) oXFF
        data[2] = 0xBF; // Joystick direction reading (7 MSbs)
    }

    data[3] = 0xFF;   // Speed pot reading (7 MSbs)

    /* encode LSBs into data[4] as follows
      bit 6: Speed pot reading LSB
      bits 5-3: Joystick speed reading (3 LSBs)
      bits 2-0: Joystick direction reading (3 LSBs)
      and set MSB = 1 */

    data[4] = 0x80 | ((maxSpeed & 0x1) << 6) | ((speed & 0x7) << 3) | (direction & 0x7);
    // end of code for data 4

    data[5] = 128;     // default horn off 128 , horn on value is 130
    /*
      bit 6: Joystick Error (indicates joystick mirror fault or some such problem when set).
      bit 5: Speed pot Error (indicates out-of range error or some such problem when set).
      bit 4: Local fault (such as CPU fault) (set when there is a fault)
      bit 3: Battery charger inhibit state (set when inhibit is active)
      bit 2: Power switch state (all bits in this byte are 1 for active, 0 for inactive).
      bit 1: Horn switch state ( set when switch is pressed)
      bit 0: Lock switch state ( set when switch is pressed)
    */

    data[6] = 132;     // Value read during data capture, taken to be the 'on' value.
    /*
      bit 6: Hazard light request
      bit 5: Left Indicator request
      bit 4: Right Indicator request
      bit 3: Remote Inhibit. When this bit is 1, the SPM shall not drive.
      bit 2: Programmer Comms flow control. Set when SPM MAY send HHP packets, clear when buffer space is low.
      bit 1: Joystick Calibration active ( set when JC mode is active )
      bit 0 : Power down / Sleep mode confirmation. Set when Sleep or Power down has been requested and the SR is
      ready to comply.
    */

    data[7] = 128;     // chair mode/ drive 128, tilt/aux output 129
    /*bit 6: Not used
      bit 5: Local non-critical fault (causes "limp" mode)
      bit 4: Headlight request
      bits 3-0: Operating mode, defined as
      00 Drive mode
      01 Actuator 1 mode
      02 Actuator 2 mode
      03 Actuator 1+2 mode
      04-0F currently undefined
    */


    // Checksum calculations - thanks to Irving and Jim...and Tony
    byte sum = 0;
    for (int i = 0; i < 8; i++)
      sum += data[i] & 0x7f;
    data[8] = (255 - (sum & 127));
    /*       Finally all packets except for the Transmit Finished packet
             shall include a one-byte checksum, which is defined as
             (0x7F - (least-significant 7 bits of ( sum of all data bytes and start byte ) ).
    */

    data[9] = 15;       // all packets end with this identifier
    Serial.println("Idle Data");
    for (unsigned char i = 0; i < 10; i++){
      sharkSerial.write(data[i]);
      Serial.print(data[i]);
      Serial.println("");
    }
  }
  digitalWrite(dePin, LOW);     // set MAX485 to low = RX active.


}






