/*=====================================================================================================================
Prog: ArduinoTrailorProg-SAL.ino

Author: Shane Lapain
Date: created 15/10/2018

Last Modified by: Shane Lapain
Last Modified Date:28/11/2018

Description: This is the main program for the Tractor Pull Tailer project. It is designed to take in external information
             From attached inputs and control multiple LED arrays and a stepper motor. This circuit also moniters the
             active states of the trailer to determin a emergency state and take the necesary action to disconnect the
             puts as required and indicate that it is in a emergency state outline in the eStop() function.
Input:None
Output:None

Sources: All references were source from https://www.arduino.cc/ and built in examples from compiler libraries.
         Examples: Stepper/stepper_oneStepAtATime.ino, encoder/basic.ino,
=====================================================================================================================*/

// ====================================================================================================================
//                                                      PRE-PROCESSOR
//=====================================================================================================================
#include <SPI.h>
#include <Stepper.h>
#include <Encoder.h>

#define LEDON LOW
#define LEDOFF HIGH
#define LEDCOUNT 7              //Number of led pins on the side modules.
#define MAXDISTANCE 10          //Maximum distance in feet.
#define MAXSTEPS 4490           //adjust depending on size of rail used.
#define STEPSPERREVOLUTION 40  //change this to fit the number of steps per revolution for your motor.
#define ENC_POS_PER_REV 400     //total numeber or positions pre tire revolution.
#define CIRWHEEL  479           //Circomferance of the tire based on [2*PI*R] based on a 6 in diameter wheel.
#define TIMEOUT 7000UL            //The default timeout is 3 seconds.
#define STEPOFFSET 20           //The delay for each step interval.

// ====================================================================================================================
//                                                        VARIABLES
//=====================================================================================================================

// PIN VARIABLES CONSTANTS: ===========================================================================================
const int ledSides[] = {30,32,34,36,38,40,42}; //LEDs on the side
const int ledFRed =   44; //Front LEDs (Red)
const int ledFGreen = 46; //Front LEDs (Green)
const int startButton = 24; // Clear and reset system to default state
const int eStopButton = 48; //Emergency stop state
const int homeSwitch = 22; // Home possision Limit switch
const int encoderChA = 26; //3;
const int encoderChB = 28; //4;

// Stepper Motor constants: ==========================================================================================
Stepper trailerStepper(STEPSPERREVOLUTION, 8, 10, 9, 11); // initialize the stepper library on pins 8 through 11: 
                                                          // pattern: 1-3-2-4
Encoder axilEnc(26,28);                                   //declaration of Encoder type def. pin assignment.

// GLOBAL VARIABLES: =================================================================================================
int encoderChALast = LOW;                //Last state stored of channel A pin
int encoderChACurrent = LOW;             //Current State of channel A pin
int Mode = 0;                            //Defines button state 0 is the default start position and 6 is the test position.
int encoderPos = 0;                      //Stores the current count of encoder position
int stepCount = 0;                       //Stores the current number of steps
unsigned long timer = 0;                 //The in program timer
unsigned long stopTimeout = 0;
int  stepsPerRev = 0;
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
  pinMode(eStopButton, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  pinMode(homeSwitch, INPUT);
  pinMode (encoderChA, INPUT);
  pinMode (encoderChB, INPUT);

  stepsPerRev = (((MAXDISTANCE * 304.8) / LEDCOUNT)*(ENC_POS_PER_REV/CIRWHEEL));
}
// EO setup()::=========================================================================================================

/*Input():=============================================================================================================
Author: Shane Lapain

Description: Input() is the function for all inputs from the trailer and passed the appropriate output or gives control
             to another function.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 28/11/2018

=====================================================================================================================*/
void Input()//takes in button press and outputs answer
{
 if (digitalRead (eStopButton) == LOW)
  {
    //Serial.println("EMERGENCY STOP");   
    Mode = 1;
  }
 else if (digitalRead (startButton) == LOW)
  {
    Serial.println("START BUTTON");
    /*digitalWrite(ledFGreen, LEDON);// Front LED (green) test
    delay (200);
    digitalWrite(ledFGreen, LEDOFF);*/
    Mode = 2;
  }
  //int restart = digitalRead (eStopButton);
}
// EO Input()::=========================================================================================================

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
while (digitalRead (eStopButton) != LOW);
  {
  Serial.println("EMERGENCY STOP");     
  for (int thisLed = 4; thisLed < 7; thisLed++)
    {
      digitalWrite(ledSides[thisLed], LEDON);
    }
  digitalWrite(ledFRed, LEDON);
  delay (200);
  for (int thisLed = 4; thisLed < 7; thisLed++) 
    {
      digitalWrite(ledSides[thisLed], LEDOFF);
    }
  digitalWrite(ledFRed, LEDOFF);
 }
 Mode = 2; // If in any other state asside from Estop then restart system.    
}
// EO Estop()::=========================================================================================================

/*resetSys():==========================================================================================================
Author: Shane Lapain

Description: resetSys() returns the weight to its home position then send the system to the start function.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 9/120/2018

=====================================================================================================================*/
void resetSys()
{
  Serial.println("RESET SYSTEM");
  while (digitalRead (homeSwitch) != LOW) 
  {
    trailerStepper.step(-1);
    Serial.println("Step -1"); 
    if (digitalRead(homeSwitch) == LOW)
    {
      Serial.println("LIMIT SWTICH");
      while (digitalRead (homeSwitch) == LOW)
        {
        trailerStepper.step(1);
        Serial.println("Step 1");    
        }
      break;
    } 
  }
  
  encoderChACurrent = 0; 
  Serial.print("ENCODER RESET:");
  Serial.println(encoderChACurrent); 
  stepCount = 0;
  Serial.print("STEP COUNT RESET:");
  Serial.println(stepCount); 
  Mode = 6;//regular run Mode = 0, Test Mode = 6.
  
  
}
// EO resetSys()::=======================================================================================================

/*stepUp():==========================================================================================================
Author: Shane Lapain

Description: The motorTest() function evaluates the stepCount. If the count is less than MAXSTEPS it takes a step in
             the clickwise direction. If the count is greater than MAXSTEPS but less than two times MAXSTEPS the stepper
             takes a step in the counter clockwise direction. Finally if the count is any other number the step count is 
             reset to zero. after each evaluation the stepCount and direction are printed to the debug window for evaluation
             
Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 9/12/2018

=====================================================================================================================*/
void stepUp()
{ 
  while (1)
  {
   if (stepCount <= (MAXSTEPS /(LEDCOUNT))) 
      {
      trailerStepper.step(1);
      delay(8);
      stepCount++;
      
      } 
   else
      {
        stepCount = 0;
        break;
      }
     
  }   
}
// EO stepUp()::===================================================================================================

/*startReady():=========================================================================================================
Author: Shane Lapain

Description: startReady() indicates that the system has been reset and is now waiting for the mull to begin. It updates
             the LED sidebars with incrementation from 0 to end distance and moved the weight up the rails between 0 and
             max position.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 16/12/2018

=====================================================================================================================*/
void startReady()
{
  int pullStop = 0;
  int difference = 0;
  unsigned long timeDiff = 0;
  encoderChACurrent = 0,encoderChALast = 0;
  axilEnc.write(0);
  int ledLevel = 0;
  int stepPos = 0;
  Serial.println("WAIT FOR START"); 
  while (Mode == 3)
  { 
  if (digitalRead (startButton) == LOW)
    {
    for ( int count = 0; count < 5; count ++)
      {
      for (int thisLed = 0; thisLed < 4; thisLed++)
        {
        digitalWrite(ledSides[thisLed], LEDON);
        }
      digitalWrite(ledFGreen, LEDON);// Front LED (green) test
      delay (500);
      for (int thisLed = 0; thisLed < 4; thisLed++) 
        {
        digitalWrite(ledSides[thisLed], LEDOFF);
        }
      digitalWrite(ledFGreen, LEDOFF);
      }
    stopTimeout = timer; 
    delay(2000);
    while (encoderChALast <= STEPSPERREVOLUTION * MAXDISTANCE)
      {
      timeDiff = timer - stopTimeout;
      timer = millis();
      digitalWrite(ledFGreen, LEDON);
      long encoderChACurrent = axilEnc.read();
      if (encoderChACurrent > encoderChALast) 
        { 
        stopTimeout = timer;  
        encoderChALast = encoderChACurrent;
        ledLevel = (encoderChALast / ((STEPSPERREVOLUTION * MAXDISTANCE) / (LEDCOUNT))); 
        for (int thisLed = 0; thisLed < ledLevel; thisLed++) 
          {
          if (stepPos < ledLevel)
            {
            stepUp();
            stepPos = ledLevel;
            Serial.print("STEP POS: ");
            Serial.println(stepPos);
            }
          digitalWrite(ledSides[thisLed], LEDON);  
          Serial.print("ENCODER POS: ");
          Serial.println(encoderChACurrent);
          Serial.print("LED LEVEL: ");
          Serial.println(ledLevel);
          }    
        }
      else if (encoderChACurrent >= STEPSPERREVOLUTION * MAXDISTANCE/*|| TIMEOUT <=  timeDiff*/)
        {
        for (int thisLed = 0; thisLed <= LEDCOUNT; thisLed++) 
          {
          digitalWrite(ledSides[thisLed], LEDOFF);
          }  
        encoderChALast = 0;
        encoderChACurrent = 0; 
      while (1)
        {
        for (int count = 0; count < 6; count++)
          {
          digitalWrite(ledFRed, LEDOFF);
          for (int thisLed = 4; thisLed <= 6; thisLed++)
            {
            digitalWrite(ledSides[thisLed], LEDOFF);
            }
          delay(200);
          digitalWrite(ledFRed, LEDON);
          for (int thisLed = 4; thisLed <= 6; thisLed++)
            {
            digitalWrite(ledSides[thisLed], LEDON);
            }
          delay(200);
          }
          break;
        }
        break;
        }
    
      }
      Serial.println("END PULL");
    }//eo if statement.
  }//eo while   
}

// EO startReady()::=====================================================================================================

/*LEDTest():=============================================================================================================
Author: Shane Lapain

Description: LEDTest() steps through all of the output LEDs and makes sure they are all working.

Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 9/12/2018

=====================================================================================================================*/
void LEDTest()
{
  Serial.println("LED TEST"); 
  int ledLevel = LEDCOUNT;
  
  for (int thisLed = 0; thisLed < LEDCOUNT; thisLed++) 
    {
    digitalWrite(ledSides[thisLed], LEDON);
    delay (400);
    digitalWrite(ledSides[thisLed], LEDOFF);
    }
    digitalWrite(ledFRed, LEDON);// Front LED (red) test
    delay (400);
    digitalWrite(ledFRed, LEDOFF);
    digitalWrite(ledFGreen, LEDON); //Front LED (green) test
    delay (400);
    digitalWrite(ledFGreen, LEDOFF);
}
// EO LEDTest()::=======================================================================================================

/*motorTest():==========================================================================================================
Author: Shane Lapain

Description: The motorTest() function evaluates the stepCount. If the count is less than MAXSTEPS it takes a step in
             the clickwise direction. If the count is greater than MAXSTEPS but less than two times MAXSTEPS the stepper
             takes a step in the counter clockwise direction. Finally if the count is any other number the step count is 
             reset to zero. after each evaluation the stepCount and direction are printed to the debug window for evaluation
             
Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 9/12/2018

=====================================================================================================================*/
void motorTest()
{
 Serial.println("MOTOR TEST"); 
  while (1)
  {
    if (stepCount <= MAXSTEPS / 2) 
      {
     trailerStepper.step(+1);
        stepCount++;
      } 
    else if (stepCount > MAXSTEPS /2 && stepCount < (MAXSTEPS))
       {
        trailerStepper.step(-1);
        stepCount++;
      }
   else
      {
        stepCount = 0;
        break;
      }
    Serial.print("steps:");
    Serial.println(stepCount);
  }   
}
// EO motorTest()::===================================================================================================

/*encoderTest():==========================================================================================================
Author: Shane Lapain

Description: The encoderTest() function takes in the channel state and compares them to determin active or inactive and direction.
             After the last update the basic logic was changed to the encoder/basic.ino structure because the samples read were
             more stable and produced more accurate counts.
Input: None
Output: None

Last Modified by: Shane Lapain
Last Modified Date: 14/12/2018

=====================================================================================================================*/
void encoderTest()
{
  unsigned long timeDiff = 0;
  encoderChACurrent = 0,encoderChALast = 0;
  Serial.println("ENCODER TEST");
  axilEnc.write(0);
  int ledLevel = 0;
  int stepPos = 0;
  digitalWrite(ledFRed, LEDOFF);
  for (int thisLed = 4; thisLed <= 6; thisLed++)
    {
    digitalWrite(ledSides[thisLed], LEDOFF);
    }
  while (1)
  {
  for (int count = 0; count < 5; count++)
    {
    digitalWrite(ledFGreen, LEDON);
    for (int thisLed = 0; thisLed <= 3; thisLed++)
      {
      digitalWrite(ledSides[thisLed], LEDON);
      }
    delay(200);
    digitalWrite(ledFGreen, LEDOFF);
    for (int thisLed = 0; thisLed <= 3; thisLed++)
      {
      digitalWrite(ledSides[thisLed], LEDOFF);
      }
    delay(200);
    }  
  break;
  }
 stopTimeout = timer; 
 delay(2000);
 while (encoderChALast <= STEPSPERREVOLUTION * MAXDISTANCE)
  {
  timeDiff = timer - stopTimeout;
  timer = millis();
  digitalWrite(ledFGreen, LEDON);
  long encoderChACurrent = axilEnc.read();
  if (encoderChACurrent > encoderChALast) 
    { 
    stopTimeout = timer;  
    Serial.print("STOP TIME: ");
    Serial.println(stopTimeout);
    Serial.print("TIMER: ");
    Serial.println(timer);
    encoderChALast = encoderChACurrent;
    ledLevel = (encoderChALast / ((STEPSPERREVOLUTION * MAXDISTANCE) / (LEDCOUNT))); 
    for (int thisLed = 0; thisLed < ledLevel; thisLed++) 
      {
      if (stepPos < ledLevel)
        {
        stepUp();
        stepPos = ledLevel;
        Serial.print("STEP POS: ");
        Serial.println(stepPos);
        }
      digitalWrite(ledSides[thisLed], LEDON);  
      Serial.print("ENCODER POS: ");
      Serial.println(encoderChACurrent);
      Serial.print("LED LEVEL: ");
      Serial.println(ledLevel);
      }    
    }
  else if (encoderChACurrent >= STEPSPERREVOLUTION * MAXDISTANCE/*|| TIMEOUT <=  timeDiff*/)
    {
    for (int thisLed = 0; thisLed <= LEDCOUNT; thisLed++) 
      {
      digitalWrite(ledSides[thisLed], LEDOFF);
      }  
    encoderChALast = 0;
    encoderChACurrent = 0; 
  while (1)
    {
    for (int count = 0; count < 6; count++)
      {
      digitalWrite(ledFRed, LEDOFF);
      for (int thisLed = 4; thisLed <= 6; thisLed++)
        {
        digitalWrite(ledSides[thisLed], LEDOFF);
        }
      delay(200);
      digitalWrite(ledFRed, LEDON);
      for (int thisLed = 4; thisLed <= 6; thisLed++)
        {
        digitalWrite(ledSides[thisLed], LEDON);
        }
      delay(200);
      }
      break;
    }
    break;
    }

  }
  Serial.print("TIME DIFFERENCE: ");
  Serial.println( timeDiff);  
  Serial.println("END PULL");
  Mode = 0;// gives control back to input()   
}
// EO encoderTest()::===================================================================================================

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
  //if sysReset() is used in test then change"Mode = 3" to "Mode = 6" for testing.
  switch (Mode) 
    {
      case 0:
        Input(); // Input read function
        break;
      case 1:
        Estop(); //Emergency function
        break;
      case 2:
        resetSys(); // Home and reset function
        break;
      case 3:
        startReady(); //Pull function
        break;  
      case 6: // If overall program function is slow Mode 6 can be removed
        LEDTest(); //LED test function
        motorTest(); // Motor test function
        encoderTest(); // Encoder test function
        break;
    }
}
// EO loop()::=========================================================================================================
