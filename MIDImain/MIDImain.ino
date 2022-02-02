/*
Author: Ian Gunther
Created: 12/12/2021
for Project Sendai
MSOE Senior Project

Summary: Goal is to convert digital button press inputs on
an arduino uno (ATMEGA328P) into a UART serial output
that transmits MIDI data (acting as a MIDI controller)
to a raspberry PI for interpretation.

Iteration Log
12/12/2021
Started experimenting with input registry functionality
on the arduino. Outputs println over serial monitor in
arduino IDE to debug changes.
01/10/2022
Changed all inputs to work with arduino internal pull-up
resistors. Inverted the uint8_t values so that they can
be summed for quick reading of inputs and mathematical
applications.
01/15/2022
Changed I/O to reflect that encoders' channel A needs
to be connected to INT0 and INT1 pins (2 and 3). Started
writing interrupt code to test encoder functionality.
01/30/2022
Made beta encoder on polling based code to compare polling
vs External Interrupt based systems.
Implemented arduino MIDI library
https://github.com/FortySevenEffects/arduino_midi_library
And RotaryEncoder library
https://github.com/mathertel/RotaryEncoder
*/

//**INVOCATIONS**//
#include <Arduino.h>
#include <RotaryEncoder.h>
#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

#define alphaCHA 2
#define alphaCHB A4
#define betaCHA 3
#define betaCHB A5

// Example for Arduino UNO with input signals on pin 2 and 3
#define PIN_IN1 A5
#define PIN_IN2 3

#define midiLED 13 //LED at Pin 13 for MIDI signal indicator

uint8_t whiteKeys = 0;
uint8_t blackKeys = 0;
uint8_t whiteNum = 0;
uint8_t blackNum = 0;
uint8_t encAlpha = 0;
uint8_t encBeta = 0;

bool pinState[16];
/*
bool pinState2=1;
bool pinState3=1;
bool pinState4=1;
bool pinState5=1;
bool pinState6=1;
bool pinState7=1;
bool pinState8=1;
bool pinState9=1;
bool pinState10=1;
bool pinState11=1;
bool pinStateA0=1;
bool pinStateA1=1;
bool pinStateA2=1;
bool pinStateA3=1;
bool pinStateA4=1;
bool pinStateA5=1;
*/

volatile int master_count = 0; // universal count
volatile byte INTFLAG1 = 0; // interrupt status flag for external interrupts

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);


//**EXTERNAL INTERRUPT SERVICE ROUTINE**//
void flag() {
  INTFLAG1 = 1;
  // add 1 to count for CW
  if (digitalRead(alphaCHA) && !digitalRead(alphaCHB))
    master_count++ ;
  // subtract 1 from count for CCW
  if (digitalRead(alphaCHA) && digitalRead(alphaCHB))
    master_count-- ;
}


//**PCINT SETUP**//
void pciSetup(byte pin){
  *digitalPinToPCMSK(pin)|= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

//D8-D13 Interrupt Handler
ISR(PCINT0_vect){
  for(byte i=8;i++;i<12){
  if(digitalRead(i)==0 && pinState[i]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(i+65,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[i]=0;
  }
  elseif(digitalRead(i)==1 && pinState[i]==0){
    MIDI.sendNoteOff(i+65,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[i]=1;
    }
  }
}

//A0-A5 Interrupt Handler
ISR(PCINT1_vect){
  for(byte i=0;i++;i<4){
  if(digitalRead(i)==0 && pinState[i]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(i+65,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[i]=0;
  }
  elseif(digitalRead(i)==1 && pinState[i]==0){
    MIDI.sendNoteOff(i+65,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[i]=1;
    }
  }
}



//**GENERAL SETUP**//
void setup() {
  pinMode(midiLED, OUTPUT);
  MIDI.begin(4); // Launch MIDI with default options


  Serial.begin(115200); //sets baud rate for serial monitor and MIDI signalling
  // COMMENTED OUT FOR MIDI SIGNALLING
  //  Serial.println(master_count); //Print master_count for encAlpha monitoring

  //Input settings for keyboard keys in pullup resistor configuration
  //Black Keys
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  //White Keys
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);

  //Rotary Encoder input settings
  //encAlpha
  pinMode(alphaCHA, INPUT);
  pinMode(alphaCHB, INPUT);
  digitalWrite(alphaCHA, HIGH);
  digitalWrite(alphaCHB, HIGH);
  attachInterrupt(0, flag, RISING);
  //encBeta
  /*
  pinMode(betaCHA, INPUT);
  pinMode(betaCHB, INPUT);
  digitalWrite(betaCHA, HIGH);
  digitalWrite(betaCHB, HIGH);
  //TODO UART Settings
*/

}


//**MAIN LOOP**//
void loop() {
  //Set Keys bits by concatenation to bring together inputs from PB and PC Registers.
  //Bit 7 is set to 1 so when inverting, whiteNum gets correct summation
  whiteKeys = (digitalRead(5) << 0) + (digitalRead(6) << 1) + (digitalRead(7) << 2)
  + (digitalRead(8) << 3) + (digitalRead(9) << 4) + (digitalRead(10) << 5) +
  (digitalRead(11) << 6) + (1 << 7);

  blackKeys = (digitalRead(A0) << 0) + (digitalRead(A1) << 1) + (digitalRead(A2) << 2)
  + (digitalRead(A3) << 3) + (digitalRead(4) << 4) + (1 << 5) + (1 << 6) + (1 << 7);

  static int pos = 0;
  encoder.tick();

  int newPos = encoder.getPosition();
  if (pos != newPos) {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder.getDirection()));
    pos = newPos;
  }

  if (INTFLAG1)   {
     Serial.println(master_count);
     delay(100);
     INTFLAG1 = 0; // clear flag
  }

  //Inverting Keys and assigning to Num counterparts
  //so button presses are read as high logic level
  whiteNum = ~whiteKeys;
  blackNum = ~blackKeys;

// COMMENTED OUT FOR MIDI SIGNALLING
/*
  if((blackKeys == 255) && (whiteKeys == 255))
  return;
  else{
  Serial.println("White Keys");
  Serial.println(whiteNum);
  Serial.println("Black Keys");
  Serial.println(blackNum);
  delay(500); //Delay so serial monitor is not spammed with input responses
  }
*/
  if(blackNum == 1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(42,127,1);  // Send a Note (pitch 42, velo 127 on channel 1)
    delay(500);    // Wait for a second
    MIDI.sendNoteOff(42,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
  }
   if(blackNum == 2){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(43,127,1);  // Send a Note (pitch 42, velo 127 on channel 1)
    delay(500);    // Wait for a second
    MIDI.sendNoteOff(43,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
  }
     if(blackNum == 4){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(44,127,1);  // Send a Note (pitch 42, velo 127 on channel 1)
    delay(500);    // Wait for a second
    MIDI.sendNoteOff(44,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
  }
     if(blackNum == 8){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(45,127,1);  // Send a Note (pitch 42, velo 127 on channel 1)
    delay(500);    // Wait for a second
    MIDI.sendNoteOff(45,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
  }
     if(blackNum == 16){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(46,127,1);  // Send a Note (pitch 42, velo 127 on channel 1)
    delay(500);    // Wait for a second
    MIDI.sendNoteOff(46,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
  }
}
