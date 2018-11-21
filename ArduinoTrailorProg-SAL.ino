/*=====================================================================================================================
Prog: ArduinoTrailorProg-SAL.ino

Author: Shane Lapain
Date: created 15/10/2018

Last Modified by: Shane Lapain
Last Modified Date:17/11/2018

Description: This is the main program for the Tractor Pull Tailer project. It is designed to take in external information
             From attached inputs and control multiple LED arrays and a stepper motor. This circuit also moniters the
             active states of the trailer to determin a emergency state and take the necesary action to disconnect the
             puts as required and indicate that it is in a emergency state outline in the eStop() function.
Input:None
Output:None

Sources: All references were source from https://www.arduino.cc/ and built in examples from compiler libraries.
=====================================================================================================================*/

#include <SPI.h>
#include <Stepper.h>

// ====================================================================================================================
//                                                      PRE-PROCESSOR
//=====================================================================================================================

#define LEDON LOW
#define LEDOFF HIGH
#define MAXSTEPS 800
#define LEDCOUNT 7 // Number of led pins on the side modules.

// ====================================================================================================================
//                                                        VARIABLES
//=====================================================================================================================

// PIN VARIABLES CONSTANTS: ===========================================================================================
const int ledSides[] = {30,32,34,36,38,40,42}; //LEDs on the side
const int ledFRed =   3; //Front LEDs (Red)
const int ledFGreen = 2; //Front LEDs (Green)
const int startButton = 24; // Clear and reset system to default state
const int eStopButton = 15; //Emergency stop state
const int restart = 16; //system restart pine
const int homeSwitch = 22; // Home possision Limit switch
const int encoderChA = 26; //3;
const int encoderChB = 28; //4;

// Stepper Motor constants: ==========================================================================================
const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution for your motor
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11); // initialize the stepper library on pins 8 through 11:

// GLOBAL VARIABLES: =================================================================================================

int ledState = LEDOFF;                   // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
int lastButtonState = LOW;               // the previous reading from the input pin
int buttonState;
int Mode = 0;                            //defines button state 0 is the default start position and 6 is the test position.
int encoderPos = 0;                      //stores the current count of encoder position
int encoderChALast = LOW;                //Last state stored of channel A pin
int encoderChACurrent = LOW;             //Current State of channel A pin
int stepCount = 0;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


// ====================================================================================================================
//                                                    FUNCTIONS
//=====================================================================================================================

/*setup():=============================================================================================================
Author: Shane Lapain

Description: setup() in the arduino is the replacement for the initialize function. This function set all the inputs and
             to their default values.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void setup() 
{
  Mode = 0;
  Serial.begin(9600);
  Serial.println("PROGRAM INITIALIZED");

  //LED declares
  for (int thisLed = 0; thisLed < LEDCOUNT; thisLed++) 
  {
    pinMode(ledSides[thisLed], OUTPUT);
    digitalWrite(ledSides[thisLed], LEDOFF);
  }
  pinMode(ledFRed, OUTPUT);
  digitalWrite(ledFRed, LEDOFF);
  pinMode(ledFGreen, OUTPUT);
  digitalWrite(ledFGreen, LEDOFF);

  //Buttons and inputs
  pinMode(startButton, INPUT_PULLUP);
  pinMode(restart, INPUT);
  pinMode(homeSwitch, INPUT);
  pinMode (encoderChA, INPUT);
  pinMode (encoderChB, INPUT);
}
// EO setup()::=========================================================================================================

/*Input():=============================================================================================================
Author: Shane Lapain

Description: Input() is the function for all inputs from the trailer and passed the appropriate output or gives control
             to another function.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void Input()//takes in button press and outputs answer
{
 if (digitalRead(homeSwitch) == LOW)
  {
    Serial.println("LIMIT SWTICH");   
    Serial.println(homeSwitch);
    digitalWrite(ledFRed, LEDON);// Front LED (red) test
    delay (200);
    digitalWrite(ledFRed, LEDOFF);
  }
 else if (digitalRead (startButton) == LOW)
  {
    Serial.println("START PULL");   
    digitalWrite(ledFGreen, LEDON);// Front LED (green) test
    delay (200);
    digitalWrite(ledFGreen, LEDOFF);
    //Mode = 2;
  }
 if (encoderChALast != digitalRead (encoderChA))
    {
    if (digitalRead(encoderChB) == LOW) 
    {
      encoderPos--;
      myStepper.step(-1);
      Serial.print("steps:");
      Serial.println(stepCount);
      stepCount--;
      delay(200);
    } 
    else
    {
      encoderPos++;
      myStepper.step(1);
      Serial.print("steps:");
      Serial.println(stepCount);
      stepCount++;
      delay(200);
    }
    Serial.print (encoderPos);
    Serial.print ("\n");
    }   
  encoderChALast = encoderChACurrent;
  //int restart = digitalRead (eStopButton);
}
// EO Input()::=========================================================================================================

/*LEDOutput():=========================================================================================================
Author: Shane Lapain

Description: n/a

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void LEDOutput()
{
  unsigned long interval;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
  }
  int ledLevel = map(previousMillis, 0, 345, 0, LEDCOUNT);
  
  for (int thisLed = 0; thisLed < LEDCOUNT; thisLed++) 
  {
    // if the array element's index is less than ledLevel,
    // turn the pin for this element on:
    if (thisLed < ledLevel)
    {
      digitalWrite(ledSides[thisLed], LEDON);
    }
    // turn off all pins higher than the ledLevel:
    else 
    {
      digitalWrite(ledSides[thisLed], LEDOFF);
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledFRed, ledState);
    digitalWrite(ledFGreen, ledState);
  }
}
// EO LEDOutput()::======================================================================================================

/*Estop():==============================================================================================================
Author: Shane Lapain

Description: Estop() monitors the state of the power input and if the state is 0 volts then it indicates it through 
             blinking all the red LEDs on the trailer until power is restored. Once power is restored it passes control
             over to the reset function to turn the system back on.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void Estop()
{
// Emergency stop function  
// if estop is low and estate == 0 then estate = 1
//  Mode = 1;
// else if estop is high and estate != 0 then reset is true and estate = 0
//  Mode = 2;
// else 
//  Mode = 1; 
//  Side LEDS blink and stop leds blink

if (digitalRead (eStopButton) == LOW)
  {
  for (int thisLed = 0; thisLed < 4; thisLed++)
    {
      pinMode(ledSides[thisLed], OUTPUT);
      digitalWrite(ledSides[thisLed], LEDON);
    }
  digitalWrite(ledFGreen, LEDON);
  digitalWrite(ledFRed, LEDON);
  delay (200);
  for (int thisLed = 0; thisLed < 4; thisLed++) 
    {
      pinMode(ledSides[thisLed], OUTPUT);
      digitalWrite(ledSides[thisLed], LEDOFF);
    }
  digitalWrite(ledFGreen, LEDOFF);
  digitalWrite(ledFRed, LEDOFF);
  Mode = 1;
 }
else
  {
    Mode = 2; // If in any other state asside from Estop then restart system.    
  }
}
// EO Estop()::=========================================================================================================

/*resetSys():==========================================================================================================
Author: Shane Lapain

Description: resetSys() returns the weight to its home position then send the system to the start function.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void resetSys()
{
 if (homeSwitch != LOW)
 {
    stepCount--;
    myStepper.step(-1);
    Mode = 2; 
 }
 else if (homeSwitch == LOW)
  {
    stepCount = 0;
    Mode = 3;
  }
 else
  {
    Mode = 2;
  }
 
}
// EO resetSys()::=======================================================================================================

/*startReady():=========================================================================================================
Author: Shane Lapain

Description: startReady() indicates that the system has been reset and is now waiting for the mull to begin. It updates
             the LED sidebars with incrementation from 0 to end distance and moved the weight up the rails between 0 and
             max position.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void startReady()
{
// Reset the trailor to default ready state 
// start LED = on
// blink side leds 5 times
// set side leds to off
// read encoder state
// if positive increment position and step by difference
// if negative decriment position and step by difference
// if step is equal to MAXSTEPS then stop and blink side red leds 5 times.
// stop leds = on and start leds = off. 
// Mode = 0;

int pullStop = 0;

if (digitalRead (startButton) == LOW)
  {
  for ( int count = 0; count < 5; count ++)
    {
    for (int thisLed = 0; thisLed < 4; thisLed++)
      {
        pinMode(ledSides[thisLed], OUTPUT);
        digitalWrite(ledSides[thisLed], LEDON);
      }
    digitalWrite(ledFGreen, LEDON);// Front LED (green) test
    delay (200);
    for (int thisLed = 0; thisLed < 4; thisLed++) 
      {
        pinMode(ledSides[thisLed], OUTPUT);
        digitalWrite(ledSides[thisLed], LEDOFF);
      }
    digitalWrite(ledFGreen, LEDOFF);
   }
  
  while (pullStop = 0)
    {
    if (encoderChALast != digitalRead (encoderChA))
      {
      if (digitalRead(encoderChB) == LOW) 
        {
          encoderPos--;
          myStepper.step(1);
          Serial.print("steps:");
          Serial.println(stepCount);
          stepCount--;
          delay(500);
        } 
      else
        {
          encoderPos++;
          myStepper.step(1);
          Serial.print("steps:");
          Serial.println(stepCount);
          stepCount++;
          delay(500);
        }
      Serial.print("Encoder Position:");
      Serial.print (encoderPos);
      Serial.print ("\n");
      }   
    encoderChALast = encoderChACurrent;
    }
   if (stepCount == MAXSTEPS)
    {
      pullStop = 1;
    } 
  }
}
// EO startReady()::=====================================================================================================

/*LEDTest():=============================================================================================================
Author: Shane Lapain

Description: LEDTest() steps through all of the output LEDs and makes sure they are all working.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void LEDTest()
{
  Serial.println("LED TEST"); 
  int ledLevel = LEDCOUNT;
  
  for (int thisLed = 0; thisLed < LEDCOUNT; thisLed++) 
    {
    digitalWrite(ledSides[thisLed], LEDON);
    delay (200);
    digitalWrite(ledSides[thisLed], LEDOFF);
    }
    digitalWrite(ledFRed, LEDON);// Front LED (red) test
    delay (200);
    digitalWrite(ledFRed, LEDOFF);
    digitalWrite(ledFGreen, LEDON); //Front LED (green) test
    delay (200);
    digitalWrite(ledFGreen, LEDOFF);
  Mode = 6;
}
// EO LEDTest()::=======================================================================================================

/*motorTest():==========================================================================================================
Author: Shane Lapain

Description: The motorTest() function reads the current encoder position and current step position and then takes the
             difference of the two values and keeps stepping either positively or negatively depending on the 

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void motorTest()
{
  {
  if (stepCount <= 200 ) 
    {
      myStepper.step(1);
      Serial.print("steps:");
      Serial.println(stepCount);
      stepCount++;
      delay(125);
    } 
  else if (stepCount > 200 && stepCount < 400)
    {
      myStepper.step(-1);
      Serial.print("steps:");
      Serial.println(stepCount);
      stepCount--;
      delay(125);
    }
 else if (stepCount == 400)
    {
      stepCount = 0;
    }
  Serial.print (encoderPos);
  Serial.print ("\n");
  }   
encoderChALast = encoderChACurrent;
  //int restart = digitalRead (eStopButton);  
}
// EO motorTest()::===================================================================================================

/*loop():=============================================================================================================
Author: Shane Lapain

Description: This function replaces the main function in other programming languages. This is were the program will
             will begin.

             In this version of loop(), a switch statement is used to section off each action or task that the control
             module will have to complete. This is done to keep the code as small as possible, because just like the 
             define statement.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 17/11/2018

=====================================================================================================================*/
void loop() // "Main()"equivalent
{
  int state = 0;  
  switch (Mode) 
    {
      case 0:
        Input(); // Input read
        break;
      case 1:
        Estop();
        break;
      case 2:
        resetSys();
        break;
      case 3:
        startReady();
        break;  
      case 6:
        LEDTest();
        motorTest();
        break;
    }
}
// EO loop()::=========================================================================================================
