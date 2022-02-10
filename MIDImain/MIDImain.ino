/*
Author: Ian Gunther
Created: 12/12/2021
for Project Sendai
MSOE Senior Project

Summary: Goal is to convert digital button press inputs
and rotary encoder inputs on
an arduino uno (ATMEGA328P) into a UART serial output
that transmits MIDI data (acting as a MIDI controller)
to a raspberry PI for interpretation.

Iteration Log
01/30/2022
Made beta encoder on polling based code to compare polling
vs External Interrupt based systems.
Implemented arduino MIDI library
https://github.com/FortySevenEffects/arduino_midi_library
And RotaryEncoder library
https://github.com/mathertel/RotaryEncoder
02/07/2022
Iteration log has been taken to github:
https://github.com/Jacher/SendaiMIDIController
Also shoutout to Nick Gammon for his extensive
documentation of arduino interrupt functionality
http://gammon.com.au/interrupts
*/

//**INVOCATIONS**//
#include <Arduino.h>
#include <RotaryEncoder.h>
#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

//**PORT <-> I/O DEFINITIONS**//
//Encoders
#define alphaCHA 2
#define alphaCHB A4
#define betaCHA 3
#define betaCHB A5

//Keys
#define key1 5 //MIDI 60
#define Midi1 60
#define key2 A0 //MIDI 61
#define Midi2 61
#define key3 6 //MIDI 62
#define Midi3 62
#define key4 A1 //MIDI 63
#define Midi4 63
#define key5 7 //MIDI 64
#define Midi5 64
#define key6 8 //MIDI 65
#define Midi6 65
#define key7 A2 //MIDI 66
#define Midi7 66
#define key8 9 //MIDI 67
#define Midi8 67
#define key9 A3 //MIDI 68
#define Midi9 68
#define key10 10 //MIDI 69
#define Midi10 69
#define key11 4 //MIDI 70
#define Midi11 70
#define key12 11 //MIDI 71
#define Midi12 71

//Lights
#define midiLED 13 //LED at Pin 13 for MIDI signal indicator

//**DECLARATIONS**//
volatile bool pinState[16]={1};//for state ISR state monitoring

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoderA(alphaCHB, alphaCHA, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoderB(betaCHB, betaCHA, RotaryEncoder::LatchMode::TWO03);

//**PCINT SETUP**//
void pciSetup(byte pin){
  *digitalPinToPCMSK(pin)|= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

//D8-D13 Interrupt Handler
//Keys 6,8,10,12
ISR(PCINT0_vect){
  if(digitalRead(key6)==0 && pinState[6]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi6,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[6]=0;
  }
  else if(digitalRead(key6)==1 && pinState[6]==0){
    MIDI.sendNoteOff(Midi6,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[6]=1;
  }
  if(digitalRead(key8)==0 && pinState[8]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi8,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[8]=0;
  }
  else if(digitalRead(key8)==1 && pinState[8]==0){
    MIDI.sendNoteOff(Midi8,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[8]=1;
  }
  if(digitalRead(key10)==0 && pinState[10]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi10,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[10]=0;
  }
  else if(digitalRead(key10)==1 && pinState[10]==0){
    MIDI.sendNoteOff(Midi10,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[10]=1;
  }
  if(digitalRead(key12)==0 && pinState[12]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi12,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[12]=0;
  }
  else if(digitalRead(key12)==1 && pinState[12]==0){
    MIDI.sendNoteOff(Midi12,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[12]=1;
  }
}

//A0-A5 Interrupt Handler
//Keys 2,4,7,9
ISR(PCINT1_vect){
  if(digitalRead(key2)==0 && pinState[2]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi2,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[2]=0;
  }
  else if(digitalRead(key2)==1 && pinState[2]==0){
    MIDI.sendNoteOff(Midi2,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[2]=1;
  }
  if(digitalRead(key4)==0 && pinState[4]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi4,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[4]=0;
  }
  else if(digitalRead(key4)==1 && pinState[4]==0){
    MIDI.sendNoteOff(Midi4,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[4]=1;
  }
  if(digitalRead(key7)==0 && pinState[7]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi7,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[7]=0;
  }
  else if(digitalRead(key7)==1 && pinState[7]==0){
    MIDI.sendNoteOff(Midi7,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[7]=1;
  }
  if(digitalRead(key9)==0 && pinState[9]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi9,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[9]=0;
  }
  else if(digitalRead(key9)==1 && pinState[9]==0){
    MIDI.sendNoteOff(Midi9,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[9]=1;
  }
}


//D0-D7 Interrupt Handler
//Keys 1,3,5,11
ISR(PCINT2_vect){
  if(digitalRead(key1)==0 && pinState[1]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi1,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[1]=0;
  }
  else if(digitalRead(key1)==1 && pinState[1]==0){
    MIDI.sendNoteOff(Midi1,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[1]=1;
  }
  if(digitalRead(key3)==0 && pinState[3]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi3,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[3]=0;
  }
  else if(digitalRead(key3)==1 && pinState[3]==0){
    MIDI.sendNoteOff(Midi3,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[3]=1;
  }
  if(digitalRead(key5)==0 && pinState[5]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi5,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[5]=0;
  }
  else if(digitalRead(key5)==1 && pinState[5]==0){
    MIDI.sendNoteOff(Midi5,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[5]=1;
  }
  if(digitalRead(key11)==0 && pinState[11]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi11,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[11]=0;
  }
  else if(digitalRead(key11)==1 && pinState[11]==0){
    MIDI.sendNoteOff(Midi11,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[11]=1;
  }
}


//**GENERAL SETUP**//
void setup() {
  pinMode(midiLED, OUTPUT);
  MIDI.begin(4); // Launch MIDI with default options

  Serial.begin(115200); //sets baud rate for serial monitor and MIDI signalling

  //Input settings for keyboard keys in pullup resistor configuration
  //Black Keys
  pinMode(A0, INPUT_PULLUP);
  pciSetup(A0);
  pinMode(A1, INPUT_PULLUP);
  pciSetup(A1);
  pinMode(A2, INPUT_PULLUP);
  pciSetup(A2);
  pinMode(A3, INPUT_PULLUP);
  pciSetup(A3);
  pinMode(4, INPUT_PULLUP);
  pciSetup(4);
  //White Keys
  pinMode(5, INPUT_PULLUP);
  pciSetup(5);
  pinMode(6, INPUT_PULLUP);
  pciSetup(6);
  pinMode(7, INPUT_PULLUP);
  pciSetup(7);
  pinMode(8, INPUT_PULLUP);
  pciSetup(8);
  pinMode(9, INPUT_PULLUP);
  pciSetup(9);
  pinMode(10, INPUT_PULLUP);
  pciSetup(10);
  pinMode(11, INPUT_PULLUP);
  pciSetup(11);

  //Rotary Encoder input settings
  //encAlpha
  pinMode(alphaCHA, INPUT);
  pinMode(alphaCHB, INPUT);
  digitalWrite(alphaCHA, HIGH);
  digitalWrite(alphaCHB, HIGH);
  //encBeta
  pinMode(betaCHA, INPUT);
  pinMode(betaCHB, INPUT);
  digitalWrite(betaCHA, HIGH);
  digitalWrite(betaCHB, HIGH);
}


//**MAIN LOOP**//
void loop() {
  encoderA.tick(); //Update encoder position
  switch(encoderA.getDirection())
  {
    case RotaryEncoder::Direction::NOROTATION:
      digitalWrite(midiLED,LOW);     // Blink the midiLED
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      digitalWrite(midiLED,HIGH);     // Blink the midiLED
      MIDI.sendControlChange(21,1,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      digitalWrite(midiLED,HIGH);     // Blink the midiLED 
      MIDI.sendControlChange(21,127,1);
      break;
  }
  encoderB.tick();//Update encoder position
  switch(encoderB.getDirection())
  {
    case RotaryEncoder::Direction::NOROTATION:
      digitalWrite(midiLED,LOW);     // Blink the midiLED
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      digitalWrite(midiLED,HIGH);     // Blink the midiLED
      MIDI.sendControlChange(20,1,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      digitalWrite(midiLED,HIGH);     // Blink the midiLED
      MIDI.sendControlChange(20,127,1);
      break;
  }
}
