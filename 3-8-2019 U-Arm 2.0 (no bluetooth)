#include <Servo.h>
Servo thumb, index, middle, pinky;
int n=0;
int a = 1;
int button;
int threshold1 = 100;
int threshold2 = 100;
int threshold3 = 230;
                  //sensor variables    
//int lowerarm = 0;
int upperarm = 0;
int biceps = 0;

void setup() {
   thumb.attach(4);    //Attach the servo for the 1st finger to pin6
   index.attach(5);    //Attach the servo for the 2nd finger to pin7
   middle.attach(6);   //Attach the servo for the 3rd finger to pin8
   pinky.attach(7);    //Attach the servo for the 4th finger to pin9
   
   Serial.begin(9600);  // this is for serial monitor to see the muscle sensor value you're getting   
}

void loop() 
{
  if(n==0)
  {                       //ask user to relax the arm, and press the button when ready. this is the calibration stage
  // threshold1 = calibrate(1);
  threshold2 = calibrate(2);          
   threshold3 = calibrate(3);
    n++;
    }
    else
    
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

//stages
  button = analogRead(A7);
   if (button > 900 && a == 1)               // when button is not pressed do nothing
        {
          a = 0;
          delay(2000);
        }
             else if (button > 900 && a == 0)   
             {
             a= 1; 
                       delay(2000);
}

if(a== 1)                   //stage 1
{
 if( biceps >= threshold3 && upperarm >= threshold1)         //11
{  close_hand();   }

else if (  biceps >= threshold3 )                            //10 
{ peace_out();}

else if(  upperarm >= threshold1)                            //01 
{pointing(); }
 else                                                        //00
  {open_hand();  }
}
else if(a ==0)              //stage 2
{
if( biceps >= threshold3 && upperarm >= threshold1)         //11
{  thumb_up();    }

else if (  biceps >= threshold3 )                           //10 
{ pincing(); }

else if(  upperarm >= threshold1)                           //01
{ alright();  }

 else                                                       //00
  {open_hand();  }

}}

int calibrate(int set)
{
  int thresh =0;
 
  if(set==1)
  {
  for (int i = 0; i < 100; i++)
  { thresh += analogRead(A2);  }
  thresh = thresh/100 +110; }
  
  else if(set==2)
  {
  for (int i = 0; i < 100; i++)
  {    thresh += analogRead(A3); }
  thresh = thresh/100 +110;}
  
   else if(set==3)
  {  for (int i = 0; i < 100; i++)
  {  thresh += analogRead(A4);  }
  thresh = thresh/100 +110;}
  
  return thresh;
  }

void open_hand()
{thumb.write(160);
 index.write(100);
 middle.write(80);
 pinky.write(60);
 delay(500); }
 
 void pointing()
 {thumb.write(50);
 index.write(100);
 middle.write(180);
 pinky.write(180);
 delay(500); }
 
 void pincing()
 {thumb.write(50);
 index.write(0);
 middle.write(180);
 pinky.write(60);
 delay(500); }
 
 void close_hand() 
 {thumb.write(50);
 index.write(0);
 middle.write(180);
 pinky.write(180);
 delay(500); }

 void peace_out()
 {thumb.write(50);
 index.write(100);
 middle.write(80);
 pinky.write(180);
 delay(500); }

 void alright()
 {thumb.write(50);
 index.write(0);
 middle.write(80);
 pinky.write(60);
 delay(500); }

 void  thumb_up()
 {thumb.write(160);
 index.write(0);
 middle.write(180);
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
