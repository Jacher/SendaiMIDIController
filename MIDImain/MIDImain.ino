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

bool pinState[16]={1};
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
//Keys 6,8,10,12
ISR(PCINT0_vect){
  if(digitalRead(key6)==0 && pinState[6]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi6,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[6]=0;
  }
  else if(digitalRead(key6)==1 && pinState[6]==0){
    MIDI.sendNoteOff(Midi6,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[6]=1;
  }
  if(digitalRead(key8)==0 && pinState[8]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi8,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[8]=0;
  }
  else if(digitalRead(key8)==1 && pinState[8]==0){
    MIDI.sendNoteOff(Midi8,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[8]=1;
  }
  if(digitalRead(key10)==0 && pinState[10]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi10,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[10]=0;
  }
  else if(digitalRead(key10)==1 && pinState[10]==0){
    MIDI.sendNoteOff(Midi10,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[10]=1;
  }
  if(digitalRead(key12)==0 && pinState[12]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi12,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
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
    MIDI.sendNoteOn(Midi2,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[2]=0;
  }
  else if(digitalRead(key2)==1 && pinState[2]==0){
    MIDI.sendNoteOff(Midi2,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[2]=1;
  }
  if(digitalRead(key4)==0 && pinState[4]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi4,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[4]=0;
  }
  else if(digitalRead(key4)==1 && pinState[4]==0){
    MIDI.sendNoteOff(Midi4,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[4]=1;
  }
  if(digitalRead(key7)==0 && pinState[7]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi7,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[7]=0;
  }
  else if(digitalRead(key7)==1 && pinState[7]==0){
    MIDI.sendNoteOff(Midi7,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[7]=1;
  }
  if(digitalRead(key9)==0 && pinState[9]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi9,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
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
    MIDI.sendNoteOn(Midi1,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[1]=0;
  }
  else if(digitalRead(key1)==1 && pinState[1]==0){
    MIDI.sendNoteOff(Midi1,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[1]=1;
  }
  if(digitalRead(key3)==0 && pinState[3]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi3,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[3]=0;
  }
  else if(digitalRead(key3)==1 && pinState[3]==0){
    MIDI.sendNoteOff(Midi3,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[3]=1;
  }
  if(digitalRead(key5)==0 && pinState[5]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi5,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
    pinState[5]=0;
  }
  else if(digitalRead(key5)==1 && pinState[5]==0){
    MIDI.sendNoteOff(Midi5,0,1);   // Stop the note
    digitalWrite(midiLED,LOW);
    pinState[5]=1;
  }
  if(digitalRead(key11)==0 && pinState[11]==1){
    digitalWrite(midiLED,HIGH);     // Blink the midiLED
    MIDI.sendNoteOn(Midi11,127,1);  // Send a Note (pitch 65, velo 127 on channel 1)
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
  // COMMENTED OUT FOR MIDI SIGNALLING
  //  Serial.println(master_count); //Print master_count for encAlpha monitoring

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
    /*
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder.getDirection()));
    */
    send(ControlChange,20,newPos,1); //Grain Location CC Message
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
/* Commented out for new interrupt functionality
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
*/
}
