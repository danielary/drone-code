#include <Wire.h>
#include "mpu.h"

const int LFpin = 3;
const int RFpin = 5;
const int LBpin = 6;
const int RBpin = 11;

const int joystickLow = 0; 
const int joystickHigh = 100; 

const int delayTime = 2000;      //Time to wait between input from Iphone
const int NUMBER_OF_FIELDS = 4; //Number of flight values from Iphone
const int TimeOutNum = 30;      //Number of times with no signal from Iphone
const int LowerNum = 50;        //Decrease balance number to lower drone
const int lowVal = 0;           //Constrain() lower bound for flight values
const int deltaAngle = 5;       // Angle test difference
const int balValChangeNum = 10; //4% of possible motor value

int TimesWithNoInput = 0;                           //Times with no input from Iphone
int fieldIndex = -1;
int values[NUMBER_OF_FIELDS] = {0,0,0,0};
int SavedLFbal, SavedRFbal, SavedLBbal, SavedRBbal; //Write check EEPROM values
int UpDwnInput, YawInput, PitchInput, RollInput;    //Input from Iphone
int LFval, RFval, LBval, RBval;                     //Variables for flight values
int LFbal = 0, RFbal = 0, LBbal = 0, RBbal = 0;     //Balance values
int newPitchA = 0, newRollA = 0, newYawA = 0;                   //Flight angles in used
int testPitchA, testRollA, testYawA;                //Test angles


void setup()
{
    Serial.begin(9600);
    Serial.println(" abcdefghijklmnopqrstuvwxyz start ");
    while( !Serial ){;}          //Wait for Serial connection

    pinMode(LFpin,OUTPUT);
    pinMode(RFpin,OUTPUT);
    pinMode(LBpin,OUTPUT);
    pinMode(RBpin,OUTPUT);


    for(int i=0; i < NUMBER_OF_FIELDS; i++)  //Set values array to 0
    {
      
      //Serial.println(values[i]);
      values[i] = 0;
    }
}

void loop()
{
  if(TimesWithNoInput < TimeOutNum)
  {
    //Serial.print(TimesWithNoInput);
    if(Serial.available())
    {
      char ch = Serial.read();
      if(ch == 'n')      // Received a New flight values
       {
        values[0] = 0; values[1] = 0; values[2] = 0; values[3] = 0;
        Serial.println(ch);
        while(fieldIndex <= NUMBER_OF_FIELDS - 1)
        {
           ch = Serial.read();
           if(ch >= '0' && ch <= '9')
           {
             values[fieldIndex] = (values[fieldIndex] * 10) + (ch - '0');
           }
           else if (ch == ' ' || ch == '\n')
           {
             if(fieldIndex < NUMBER_OF_FIELDS ) fieldIndex ++;
           }
         }
       if(fieldIndex == NUMBER_OF_FIELDS)  //Copy array values received (0-100).
         {
           UpDwnInput = map(values[0],joystickLow,joystickHigh,-50,50);
           YawInput   = map(values[0],joystickLow,joystickHigh,-50,50);
           PitchInput = map(values[0],joystickLow,joystickHigh,-50,50);
           RollInput  = map(values[0],joystickLow,joystickHigh,-50,50);

           values[0] = 0; values[1] = 0; values[2] = 0; values[3] = 0;

           moveMotorsFromInputs();
           getAccelerometerAngles(1);      //Get flight angles to test against

           while(!Serial.available()) {
            AngleAdjustBalanceValues();    // Adjust balance values to reflect flight angle change
           }

           TimesWithNoInput = 0;
           delay(delayTime);
           fieldIndex = -1;
           }

        else if (ch == 's')// stop all rotors
          {
            Serial.println(ch);
            analogWrite(LFpin, 0);
            analogWrite(RFpin, 0);
            analogWrite(LBpin, 0);
            analogWrite(RBpin, 0);

            TimesWithNoInput = 0;
            delay(delayTime);
            fieldIndex = -1;
          }
       }
     else  //No characters receives
       {
         TimesWithNoInput = TimesWithNoInput + 1;
         delay(delayTime);
       }
    }
    else  // Too much time since input so we balance and lower drone
    {
      analogWrite(LFpin,  LowerNum);
      analogWrite(RFpin,  LowerNum);
      analogWrite(LBpin,  LowerNum);
      analogWrite(RBpin,  LowerNum);

      TimesWithNoInput = 0;
   }
  }
}
void moveMotorsFromInputs() {

  LFval = LFbal + UpDwnInput - YawInput - PitchInput + RollInput;
  RFval = RFbal + UpDwnInput + YawInput - PitchInput - RollInput;
  LBval = LBbal + UpDwnInput + YawInput + PitchInput + RollInput;
  RBval = RBbal + UpDwnInput - YawInput + PitchInput - RollInput;

  Serial.println("Writing Motors: ");
  Serial.println(LFval);
  Serial.println(RFval);
  Serial.println(LBval);
  Serial.println(RBval);

  analogWrite(LFpin, LFval);
  analogWrite(RFpin, RFval);
  analogWrite(LBpin, LBval);
  analogWrite(RBpin, RBval);
           
  Serial.println("=============================");
}

void AngleAdjustBalanceValues() {
  delay(1000); //Just for testing purposes so it doesn't run too fast.
  getAccelerometerAngles(2);   //Place values in testPitchA, testRollA, testYawA

  //Change from getting new Pitch Angle
  if((newPitchA - testPitchA) > deltaAngle)
  {
    LBbal = LBbal + balValChangeNum;
    RBbal = RBbal + balValChangeNum;
    Serial.println("aaa");
  }
  else if ((newPitchA - testPitchA) < - deltaAngle)
  {
    LFbal = LFbal - balValChangeNum;
    RFbal = RFbal - balValChangeNum;
    Serial.println("bbb");
  }

  //Change from getting new Roll Angle
  if((newRollA - testRollA) > deltaAngle)
  {
    RFbal = RFbal + balValChangeNum;
    RBbal = RBbal + balValChangeNum;
    Serial.println("ccc");
  }

  else if ((newRollA - testRollA) < - deltaAngle)
  {
    LFbal = LFbal - balValChangeNum;
    LBbal = LBbal - balValChangeNum;
    Serial.println("ddd");
  }

  //Change from getting new Yaw Angle

  if((newYawA - testYawA) > deltaAngle)
  {
    LBbal = LBbal + balValChangeNum;
    RFbal = RFbal + balValChangeNum;
  } else if ((newYawA - testYawA) < - deltaAngle)
  {
    LBbal = LBbal - balValChangeNum;
    RFbal = RFbal - balValChangeNum;
  }

/*Recalculate flight values to reflect any change in balance adjustment of rotor values (0-255)*/

  moveMotorsFromInputs();
}
void getAccelerometerAngles(int x)
{
  
  Serial.println("get accelerometer values");

  
  
}  

