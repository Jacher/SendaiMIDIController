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
//Alpha Encoder (Main Cursor)
#define alphaCHA 36
#define alphaCHB 34
#define alphaSW 38
#define MidiSel 28 //Select/OK
//Beta Encoder (Program Change)
#define betaCHA 30
#define betaCHB 28
#define betaSW 32
#define MidiSave 26 //Save Program
// 6 Encoder SETUP
//Chi Encoder (Grain Duration)
#define chiCHA 44
#define chiCHB 46
#define chiSW 26
//#define MidiCSW (unused)
//Delta Encoder (Number of Grains)
#define deltaCHA 40
#define deltaCHB 42
//Epsilon Encoder (Spray)
#define epsilonCHA 24
#define epsilonCHB 22
//Phi Encoder (Volume)
#define phiCHA 37
#define phiCHB 35
//Gamma Encoder (Grain Location)
#define gammaCHA 48
#define gammaCHB 50


//Keys
#define key1 2 //MIDI 60
#define Midi1 60
#define key2 3 //MIDI 61
#define Midi2 61
#define key3 4 //MIDI 62
#define Midi3 62
#define key4 5 //MIDI 63
#define Midi4 63
#define key5 6 //MIDI 64
#define Midi5 64
#define key6 7 //MIDI 65
#define Midi6 65
#define key7 8 //MIDI 66
#define Midi7 66
#define key8 9 //MIDI 67
#define Midi8 67
#define key9 10 //MIDI 68
#define Midi9 68
#define key10 11 //MIDI 69
#define Midi10 69
#define key11 12 //MIDI 70
#define Midi11 70
#define key12 13 //MIDI 71
#define Midi12 71
#define Rec 52 //Record button
#define MidiRec 25 //CC 25 EV 1


//**DECLARATIONS**//
volatile bool pinState[16]={1};//for pin state monitoring
bool fast=0;

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoderA(alphaCHB, alphaCHA, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder encoderB(betaCHB, betaCHA, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder encoderC(chiCHB, chiCHA, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder encoderD(deltaCHB, deltaCHA, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder encoderE(epsilonCHB, epsilonCHA, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder encoderF(phiCHB, phiCHA, RotaryEncoder::LatchMode::FOUR0);
RotaryEncoder encoderG(gammaCHB, gammaCHA, RotaryEncoder::LatchMode::FOUR0);

//**GENERAL SETUP**//
void setup() {
  MIDI.begin(4); // Launch MIDI with default options

  Serial.begin(115200); //sets baud rate for serial monitor and MIDI signalling

  //Input settings for keyboard keys in pullup resistor configuration
  pinMode(key1, INPUT_PULLUP);
  pinMode(key2, INPUT_PULLUP);
  pinMode(key3, INPUT_PULLUP);
  pinMode(key4, INPUT_PULLUP);
  pinMode(key5, INPUT_PULLUP);
  pinMode(key6, INPUT_PULLUP);
  pinMode(key7, INPUT_PULLUP);
  pinMode(key8, INPUT_PULLUP);
  pinMode(key9, INPUT_PULLUP);
  pinMode(key10, INPUT_PULLUP);
  pinMode(key11, INPUT_PULLUP);
  pinMode(key12, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(chiSW, INPUT_PULLUP);
  pinMode(52, INPUT_PULLUP);

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
  // 6 Encoder SETUP
  //encChi
  pinMode(chiCHA, INPUT);
  pinMode(chiCHB, INPUT);
  digitalWrite(chiCHA, HIGH);
  digitalWrite(chiCHB, HIGH);
  //encDelta
  pinMode(deltaCHA, INPUT);
  pinMode(deltaCHB, INPUT);
  digitalWrite(deltaCHA, HIGH);
  digitalWrite(deltaCHB, HIGH);
  //encEpsilon
  pinMode(epsilonCHA, INPUT);
  pinMode(epsilonCHB, INPUT);
  digitalWrite(epsilonCHA, HIGH);
  digitalWrite(epsilonCHB, HIGH);
  //encPhi
  pinMode(phiCHA, INPUT);
  pinMode(phiCHB, INPUT);
  digitalWrite(phiCHA, HIGH);
  digitalWrite(phiCHB, HIGH);
  //encGamma
  pinMode(gammaCHA, INPUT);
  pinMode(gammaCHB, INPUT);
  digitalWrite(gammaCHA, HIGH);
  digitalWrite(gammaCHB, HIGH);


  //debug
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  
}


//**MAIN LOOP**//
void loop() {
  encoderA.tick(); //Update encoder position
  switch(encoderA.getDirection()) //**MAIN CURSOR**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      MIDI.sendControlChange(27,1,1); //(CC,Value,Channel)
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      MIDI.sendControlChange(27,127,1);
      break;
  }
  encoderB.tick();//Update encoder position
  switch(encoderB.getDirection()) //**PROGRAM CHANGE**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      MIDI.sendControlChange(24,1,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      MIDI.sendControlChange(24,127,1);
      break;
  }
  encoderC.tick();//Update encoder position
  switch(encoderC.getDirection()) //**GRAIN DURATION**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      if(fast==0)
      MIDI.sendControlChange(21,10,1);
      else
      MIDI.sendControlChange(21,30,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      if(fast==0)
      MIDI.sendControlChange(21,117,1);
      else
      MIDI.sendControlChange(21,97,1);
      break;
  }
  encoderD.tick();//Update encoder position
  switch(encoderD.getDirection()) //**NUMBER OF GRAINS**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      MIDI.sendControlChange(23,1,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      MIDI.sendControlChange(23,127,1);
      break;
  }
  encoderE.tick();//Update encoder position
  switch(encoderE.getDirection()) //**SPRAY**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      if(fast==0)
      MIDI.sendControlChange(22,5,1);
      else
      MIDI.sendControlChange(22,20,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      if(fast==0)
      MIDI.sendControlChange(22,122,1);
      else
      MIDI.sendControlChange(22,107,1);
      break;
  }
  encoderF.tick();//Update encoder position
  switch(encoderF.getDirection()) //**VOLUME**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
        MIDI.sendControlChange(7,1,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
        MIDI.sendControlChange(7,127,1);
      break;
  }
  encoderG.tick();//Update encoder position
  switch(encoderG.getDirection()) //**GRAIN LOCATION**
  {
    case RotaryEncoder::Direction::NOROTATION:
      break;
    case RotaryEncoder::Direction::CLOCKWISE:
      if(fast==0)
        MIDI.sendControlChange(20,1,1);
      else
        MIDI.sendControlChange(20,10,1);
      break;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      if(fast==0)
        MIDI.sendControlChange(20,127,1);
      else
        MIDI.sendControlChange(20,117,1);
      break;
  }

//**Additional Button functionality**//
//Record, Select/OK, Program Save button functionality

//**RECORD**
  if(digitalRead(Rec)==0 && pinState[13]==1){
    MIDI.sendControlChange(MidiRec,1,1);  // Send a CC (CC,velocity,channel)
    pinState[13]=0;
  }
  else if(digitalRead(Rec)==1 && pinState[13]==0){
    pinState[13]=1;
  }
//**SELECT/OK
  if(digitalRead(alphaSW)==0 && pinState[14]==1){
    MIDI.sendControlChange(MidiSel,1,1);  // Send a CC (CC,velocity,channel)
    pinState[14]=0;
  }
  else if(digitalRead(alphaSW)==1 && pinState[14]==0){
    pinState[14]=1;
  }
//**SAVE**
  if(digitalRead(betaSW)==0 && pinState[15]==1){
    MIDI.sendControlChange(MidiSave,1,1);  // Send a Note (CC,velocity,channel)
    pinState[15]=0;
  }
  else if(digitalRead(betaSW)==1 && pinState[15]==0){
    pinState[15]=1;
  }
//**INPUT ACCELERATOR**
  if(digitalRead(chiSW)==0 && pinState[16]==1){
    fast=!fast;
    pinState[16]=0;
    auto val = fast ? HIGH:LOW;
    digitalWrite(14, val);
  }
  else if(digitalRead(chiSW)==1 && pinState[16]==0)
  {
    pinState[16]=1;
  }

//PIANO KEYS
  if(digitalRead(key1)==0 && pinState[1]==1){
    MIDI.sendNoteOn(Midi1,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[1]=0;
  }
  else if(digitalRead(key1)==1 && pinState[1]==0){
    MIDI.sendNoteOff(Midi1,0,1);   // Stop the note
    pinState[1]=1;
  }
  if(digitalRead(key2)==0 && pinState[2]==1){
    MIDI.sendNoteOn(Midi2,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[2]=0;
  }
  else if(digitalRead(key2)==1 && pinState[2]==0){
    MIDI.sendNoteOff(Midi2,0,1);   // Stop the note
    pinState[2]=1;
  }
  if(digitalRead(key3)==0 && pinState[3]==1){
    MIDI.sendNoteOn(Midi3,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[3]=0;
  }
  else if(digitalRead(key3)==1 && pinState[3]==0){
    MIDI.sendNoteOff(Midi3,0,1);   // Stop the note
    pinState[3]=1;
  }
  if(digitalRead(key4)==0 && pinState[4]==1){
    MIDI.sendNoteOn(Midi4,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[4]=0;
  }
  else if(digitalRead(key4)==1 && pinState[4]==0){
    MIDI.sendNoteOff(Midi4,0,1);   // Stop the note
    pinState[4]=1;
  }
  if(digitalRead(key5)==0 && pinState[5]==1){
    MIDI.sendNoteOn(Midi5,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[5]=0;
  }
  else if(digitalRead(key5)==1 && pinState[5]==0){
    MIDI.sendNoteOff(Midi5,0,1);   // Stop the note
    pinState[5]=1;
  }
  if(digitalRead(key6)==0 && pinState[6]==1){
    MIDI.sendNoteOn(Midi6,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[6]=0;
  }
  else if(digitalRead(key6)==1 && pinState[6]==0){
    MIDI.sendNoteOff(Midi6,0,1);   // Stop the note
    pinState[6]=1;
  }
  if(digitalRead(key7)==0 && pinState[7]==1){
    MIDI.sendNoteOn(Midi7,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[7]=0;
  }
  else if(digitalRead(key7)==1 && pinState[7]==0){
    MIDI.sendNoteOff(Midi7,0,1);   // Stop the note
    pinState[7]=1;
  }
  if(digitalRead(key8)==0 && pinState[8]==1){
    MIDI.sendNoteOn(Midi8,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[8]=0;
  }
  else if(digitalRead(key8)==1 && pinState[8]==0){
    MIDI.sendNoteOff(Midi8,0,1);   // Stop the note
    pinState[8]=1;
  }
  if(digitalRead(key9)==0 && pinState[9]==1){
    MIDI.sendNoteOn(Midi9,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[9]=0;
  }
  else if(digitalRead(key9)==1 && pinState[9]==0){
    MIDI.sendNoteOff(Midi9,0,1);   // Stop the note
    pinState[9]=1;
  }
  if(digitalRead(key10)==0 && pinState[10]==1){
    MIDI.sendNoteOn(Midi10,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[10]=0;
  }
  else if(digitalRead(key10)==1 && pinState[10]==0){
    MIDI.sendNoteOff(Midi10,0,1);   // Stop the note
    pinState[10]=1;
  }
  if(digitalRead(key11)==0 && pinState[11]==1){
    MIDI.sendNoteOn(Midi11,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[11]=0;
  }
  else if(digitalRead(key11)==1 && pinState[11]==0){
    MIDI.sendNoteOff(Midi11,0,1);   // Stop the note
    pinState[11]=1;
  }
  if(digitalRead(key12)==0 && pinState[12]==1){
    MIDI.sendNoteOn(Midi12,127,1);  // Send a Note (pitch,velocity,channel)
    pinState[12]=0;
  }
  else if(digitalRead(key12)==1 && pinState[12]==0){
    MIDI.sendNoteOff(Midi12,0,1);   // Stop the note
    pinState[12]=1;
  }
}
