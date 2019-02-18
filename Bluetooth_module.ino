#include <SoftwareSerial.h>
SoftwareSerial Bluetooth(2, 3); // RX|TX

#define LED 13
//int LED = 13; // the on-board LED
int Data; // the data received
 
void setup() {
  Bluetooth.begin(9600);
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
  Serial.println("Waiting for command...");
  Bluetooth.println("Send 1 to turn on the LED. Send 0 to turn Off");
  pinMode(LED,OUTPUT);
 
}
 
void loop() {
  if (Bluetooth.available()){ //wait for data received
    Data = Bluetooth.read();
    if(Data=='1'){  
      digitalWrite(LED,HIGH);
      Serial.println("LED On!");
      Bluetooth.println("LED On!");
      Data= ' ';
    }
    else if(Data=='0'){
       digitalWrite(LED, LOW);
       Serial.println("LED Off!");
       Bluetooth.println("LED  On D13 Off ! ");

       Data= ' ';
    }
  //  else{;}
  }
delay(100);
}
