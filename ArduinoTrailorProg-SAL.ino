#include <SPI.h>
#include <TMC26XStepper.h> // DFRobot Library

#define LEDON LOW
#define LEDOFF HIGH

// constants won't change. Used here to set a pin number :
const int ledCount = 10; 
const int ledSides[] = {13,12,11,10,9,8,7,6,5,4}; //LEDs on the side
const int ledFRed =   3; //Front LEDs (Red)
const int ledFGreen = 2; //Front LEDs (Green)
const int startButton = 24; // Clear and reset system to default state
const int eStopButton = 15; //Emergency stop state
const int restart = 16; //system restart pin
const int homeSwitch = 22; // Home possision Limit switch



// Variables will change :
int ledState = LEDOFF;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
int lastButtonState = LOW;   // the previous reading from the input pin
int buttonState;
int Mode; //defines button state


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() 
{
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
}
void Input()//takes in button press and outputs answer
{
 if (digitalRead(homeSwitch) == false)
  {
    Serial.println("LIMIT SWTICH");   
    digitalWrite(ledFRed, LEDON);// Front LED (red) test
    delay (200);
    digitalWrite(ledFRed, LEDOFF);
  }
  else if (digitalRead (startButton) == LOW)
  {
    Serial.println("START PULL");   
    digitalWrite(ledFGreen, LEDON);// Front LED (red) test
    delay (200);
    digitalWrite(ledFGreen, LEDOFF);
  }
  //int restart = digitalRead (eStopButton);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
 /* if (startPull != lastButtonState) 
  {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (eStopButton != buttonState)
    {
      buttonState = restart;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH)
      {
       Mode = 1; //Kill condition
      }
    }
    else if (restart != buttonState && eStopButton == buttonState) 
    {
      buttonState = restart;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH)
      {
       Mode = 2; //Resets program
      }
    }
    else if (startPull != buttonState) 
    {
      buttonState = startPull;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH)
      {
        Mode = 3;
      }
    }  
  }*/
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
}

void restartSys()
{
// restarting after a emergency stop state.  
}


int startReset()
{
// Reset the trailor to default ready state  
return (6);
}

void LEDTest()
{
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
  Mode = 0;
}

void motorTest()
{
//Stepper control  
}

void loop() // "Main()"equivalent
{
  int state = 0; 
  Mode = 0;
  
  switch (Mode) 
    {
      case 0:
        Input(); // Input read
        break;
      case 1:
        Estop();
        break;
      case 2:
        restartSys();
        break;
      case 3:
        startReset();
        break;  
      case 6:

        LEDTest();
        motorTest();
        break;
    }
}
