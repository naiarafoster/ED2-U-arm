#include <Servo.h>

Servo thumb, index, middle, pinky, thumbSide;


int threshold1 = 100;
int threshold2 = 100;
int threshold3 = 100 ; //biceps
                  //sensor variables    
int lowerarm = 0;
int upperarm = 0;
int biceps = 0;

void setup() {
   thumb.attach(4);    //Attach the servo for the 1st finger to pin6
   index.attach(5);    //Attach the servo for the 2nd finger to pin7
   middle.attach(6);   //Attach the servo for the 3rd finger to pin8
   pinky.attach(7);    //Attach the servo for the 4th finger to pin9
   //Thumb side to side motion servo
   thumbSide.attach(8);
   
   
   Serial.begin(9600);  // this is for serial monitor to see the muscle sensor value you're getting   
}

void loop() 
{
     
  // lowerarm = analogRead(A2); //lower forearm
  upperarm = analogRead(A3); //upper forearm
  biceps = analogRead(A4);   //bicept

  // Serial.print("sensor 1 value: ");
  //   Serial.print(lowerarm); 
  // Serial.print("   "); 
  // Serial.println(threshold1); 
  Serial.print("sensor 2 value: ");
  Serial.print(upperarm); 
  Serial.print("   "); 
  Serial.println(threshold2); 
  Serial.print("sensor 3 value: ");
  Serial.print(biceps);
  Serial.print("   "); 
  Serial.println(threshold3); 




if( biceps > threshold3 && upperarm > threshold2)         //11
{  close_hand();   }

else if (  biceps > threshold3 && upperarm < threshold2)                            //10 
{ pincing();}

  
 else                                                        //00
  {open_hand();  }


}



void open_hand()
{thumb.write(100);
 index.write(70);
 middle.write(0);
 pinky.write(100);
 delay(500); }
 
 void pointing()
 {thumb.write(10);
 index.write(70);
 middle.write(70);
 pinky.write(180);
 delay(500); }
 
 void pincing()
 {thumb.write(40);
 index.write(170);
 middle.write(120);
 pinky.write(100);
 delay(500); }
 
 void close_hand() 
 {thumb.write(10);
 index.write(130);
 middle.write(70);
 pinky.write(180);
 delay(500);
 }

 void peace_out()
 {thumb.write(10);
 index.write(70);
 middle.write(0);
 pinky.write(180);
 delay(500); }

 void alright()
 {thumb.write(10);
 index.write(170);
 middle.write(0);
 pinky.write(100);
 delay(500);}

 void  thumb_up()
 {thumb.write(100);
 index.write(130);
 middle.write(70);
 pinky.write(180);
 delay(500); }

 void come_here()
 {
 while( biceps >= threshold3 &&  upperarm >= threshold1)
 {
  close_hand();
  delay(400);
  pointing();
  delay(400);
  upperarm = analogRead(A3);
 // lowerarm = analogRead(A2); 
  biceps = analogRead(A4);  
 }  }
