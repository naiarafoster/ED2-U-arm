#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // SRX | STX
// D2 pin of NANO is SRX-pin of NANO; it will have connection with TX-pin of HC-05 
// D3 pin of NANO is STX-pin of NANO; it will have connection with RX-pin of HC-05 via voltage divider.

#define ledPin 7
#define analogPin A0

int val = 0;
char c = ' '; //initializes to a space
 
void setup() 
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    Serial.begin(9600);
    BTserial.println("Arduino is ready");
 
    // HC-05 default serial speed for communication mode is 9600
    BTserial.begin(9600);  
    BTserial.println("BTserial started at 9600");
}
 
void loop()
{
   // Keep reading from HC-05 and send to Arduino Serial Monitor
    
    
    if (BTserial.available())
    {  
        c = BTserial.read();
        //Serial.write(c);
    }
     if (c == '0') {
        digitalWrite(ledPin, LOW); // Turn LED OFF
        BTserial.println("LED: OFF"); // Send back, to the phone, the String "LED: ON"
        c = ' ';
    }
    else if (c == '3') {
      digitalWrite(ledPin, HIGH);
      BTserial.println("LED: ON");;
      c = ' ';
    } 
    else if (c == '2')
    {
    
      val = analogRead(analogPin);     // read the input pin
     //float voltage = val * 500.0;
    // voltage /= 1024.0; 
   val =  constrain(val, 100, 1000);
     BTserial.println(val); 
      delay(500);
      
      }
}
