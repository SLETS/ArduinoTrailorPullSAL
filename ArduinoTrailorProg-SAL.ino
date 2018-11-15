#include <SPI.h>
#include <Stepper.h>
#include <TMC26XStepper.h> // DFRobot Library

#define LEDON LOW
#define LEDOFF HIGH
#define MAXSTEPS 800

// constants won't change. Used here to set a pin number :
const int ledCount = 10; 
const int ledSides[] = {13,12,11,10,9,8,7,6,5,4}; //LEDs on the side
const int ledFRed =   3; //Front LEDs (Red)
const int ledFGreen = 2; //Front LEDs (Green)
const int startButton = 24; // Clear and reset system to default state
const int eStopButton = 15; //Emergency stop state
const int restart = 16; //system restart pin
const int homeSwitch = 22; // Home possision Limit switch
const int encoderChA = 26; //3;
const int encoderChB = 28; //4;
// Stepper Motor constants 
const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution for your motor
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11); // initialize the stepper library on pins 8 through 11:

// Variables will change :
int ledState = LEDOFF;                   // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
int lastButtonState = LOW;               // the previous reading from the input pin
int buttonState;
int Mode;                                //defines button state
int encoderPos = 0;                      //stores the current count of encoder position
int encoderChALast = LOW;                //Last state stored of channel A pin
int encoderChACurrent = LOW;             //Current State of channel A pin
int stepCount = 0;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() 
{
  Mode = 0;
  Serial.begin(9600);
  Serial.println("PROGRAM INITIALIZED");

  //LED declares
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
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
    Serial.print (encoderPos);
    Serial.print ("\n");
    }   
  encoderChALast = encoderChACurrent;
  //int restart = digitalRead (eStopButton);
}

void LEDOutput()
{
  unsigned long interval;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
  }
  int ledLevel = map(previousMillis, 0, 345, 0, ledCount);
  
  for (int thisLed = 0; thisLed < ledCount; thisLed++) 
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
//
}

void resetSys()
{
 if (homeSwitch != LOW && stepCount != 0)
 {
    stepCount--;
    Mode = 2; 
 }
 else if (homeSwitch == LOW && stepCount == 0)
  {
    Mode = 3;
  }
 else
  {
    Mode = 2;
  }
 
}


int startReady()
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
}

void LEDTest()
{
  Serial.println("LED TEST"); 
  int ledLevel = ledCount;
  
  for (int thisLed = 0; thisLed < ledCount; thisLed++) 
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

void motorTest()
{
//Stepper control  
}

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
