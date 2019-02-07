/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
#include <Servo.h>

Servo servo1; //D4 thumb
Servo servo2; //D5 index
Servo servo3; //D6 middle
Servo servo4; //D7 ring and pinky
Servo servo5; //D8 wrist



int threshold1 = 500; // lower forearm
int threshold2 = 500; //upper forearm
int threshold3 = 600; //biceps



int sensorValue1 = 0; //lower forearm
int sensorValue2 = 0; //upper forearm
int sensorValue3 = 0; //biceps

int gesture = 0; //variable to hold gesture#

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:

  servo1.attach(4); //D4 thumb
  servo2.attach(5); //D5 index
  servo3.attach(6); //D6 middle
  servo4.attach(7); //D7 ring and pinky
  servo5.attach(8);   //D8 wrist


  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  
 sensorValue1 = analogRead(A2); //lower forearm
 sensorValue2 = analogRead(A3); //upper forearm
 sensorValue3 = analogRead(A4); //biceps

 
 //float voltage = sensorValue * (5.0 / 1023.0);
 // print out the value you read:
 Serial.print(sensorValue1); 
 Serial.print(" ");
 Serial.print(sensorValue2);
 Serial.print(" ");
 Serial.println(sensorValue3);

  if(sensorValue1 >= threshold1)
  {
    if(sensorValue2 >= threshold2)
    { 
      if(sensorValue3 >= threshold3)
      {
        gesture = 7;
      }
      else
      {
        gesture = 6;
      }
     }
     else
     {
      if(sensorValue3 >= threshold3)
      {
        gesture = 5;
      }
      else
      {
        gesture = 4;
      }
     }
    }
    else
    {
      if(sensorValue2 >= threshold2)
      { 
        if(sensorValue3 >= threshold3)
        {
          gesture = 3;
        }
        else
        {
         gesture = 2;
        }
      }
      else
      {
        if(sensorValue3 >= threshold3)
        {
          gesture = 1;
        }
        else
        {
          gesture = 0;
        }
     }
    }
    

    switch(gesture)
    {
      case 0:   //rest
        servo1.write(0);
        servo2.write(0);
        servo3.write(0);
        servo4.write(0);
        servo5.write(0);
        delay(100);
        break;
     /*case 1:
        servo1.write(180);
        servo2.write(0);
        servo3.write(180);
        break;*/
     case 2:  //pincer
          servo1.write(45);
          servo2.write(45);
          servo3.write(45);
          servo4.write(0);
          servo5.write(0);        
          delay(100);
          break;
    /* case 3:
        servo1.write(45);
        servo2.write(45);
        servo3.write(180);
        break;*/
        case 4: //pointing
          servo1.write(45);
          servo2.write(0);
          servo3.write(90);
          servo4.write(90);
          servo5.write(0);        
          delay(100);
        break;
        case 7: //fist
          servo1.write(45);
          servo2.write(90);
          servo3.write(90);
          servo4.write(90);
          servo5.write(0);
          delay(100);
          break;
        }
        delay(100);
}
