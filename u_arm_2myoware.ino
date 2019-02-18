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
//Servo servo5; //D8 wrist



int threshold1 = 600; // lower forearm
int threshold2 = 400; //upper forearm
//int threshold3 = 600; //biceps

int minSensor1 =57;
int minSensor2=69;

int sensorValue1 = 0; //lower forearm
int sensorValue2 = 0; //upper forearm
//int sensorValue3 = 0; //biceps

int gesture = 0; //variable to hold gesture#

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:

  servo1.attach(4); //D4 thumb
  servo2.attach(5); //D5 index
  servo3.attach(6); //D6 middle
  servo4.attach(7); //D7 ring and pinky
//  servo5.attach(8);   //D8 wrist


  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  
 sensorValue1 = analogRead(A2); //lower forearm
 sensorValue2 = analogRead(A3); //biceps
 //sensorValue3 = analogRead(A4); //biceps

 
 //float voltage = sensorValue * (5.0 / 1023.0);
 // print out the value you read:
 Serial.println("sensor 1 value: ");
 Serial.println(sensorValue1); 
 Serial.println("sensor 2 value: ");
 Serial.println(sensorValue2);
 //Serial.print(" ");
// Serial.println(sensorValue3);

  if((sensorValue1 >= threshold1)&&(sensorValue2 >= threshold2))
    {        
          servo1.write(45);
          servo2.write(0);
          servo3.write(90);
          servo4.write(90);
          delay(1700);

    }

   if((sensorValue1 < threshold1)&&(sensorValue2 >= threshold2)){

          servo1.write(45);
          servo2.write(45);
          servo3.write(45);
          servo4.write(0);
          delay(1500);
    
    }

      else if((sensorValue1 >= threshold1)&&(sensorValue2 < threshold2)){
          servo1.write(45);
          servo2.write(90);
          servo3.write(90);
          servo4.write(90);
          delay(1500);
        
        }

        else if ((sensorValue1 < (threshold1/2))&&(sensorValue2 < (threshold2/2))){
          
        servo1.write(0);
        servo2.write(0);
        servo3.write(0);
        servo4.write(0);
        delay(1500);
          
          }
//delay(100);
}
