#include <Stepper.h>

char ch=0;
int length;
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
//1 represents north,2 represents east,3 represents south,4 represents west
const int red=1;
const int blue=2;
const int green=4;
const int yellow=3;
int current_position=1;
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(20);
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  while(!(Serial.available()>0))
  {
    
  }
if(Serial.available()>0)
{
  ch=Serial.read();
  if(ch=='r')
  {
    /*myStepper.step(-stepsPerRevolution/4);
    delay(500);
    */
    myStepper.step((red-current_position)*(-stepsPerRevolution/4));
    current_position=red;
  }
  else if(ch=='b')
  {
    myStepper.step((blue-current_position)*(-stepsPerRevolution/4));
    current_position=blue;
  }
  else if(ch=='g')
  {
    myStepper.step((green-current_position)*(-stepsPerRevolution/4));
    current_position=green;
  }
  else if(ch=='y')
  {
    myStepper.step((yellow-current_position)*(-stepsPerRevolution/4));
    current_position=yellow;
  }
  else if(ch=='1')
  {
    myStepper.step(-stepsPerRevolution/4);
    current_position=yellow;
  }else if(ch=='3')
  {
    myStepper.step(stepsPerRevolution/4);
    current_position=yellow;
  }
  
}
}

