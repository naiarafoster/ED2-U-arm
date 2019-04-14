//library for servos
#include <Servo.h>
//library for bluetooth
#include <SoftwareSerial.h>
//library for non-volatile memory
#include <EEPROM.h>
//library for Voltage reading and LCD display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
//#include <Fonts/FreeMonoBoldOblique12pt7b.h>

//Instances of Servo class
Servo thumbServo, indexServo, middleServo, ringPinkyServo, thumbSideServo;

//Initialize serial communication on pins 2 and 3
SoftwareSerial BTserial(2, 3);

//define Pins for each servo
#define thumbPin 4
#define indexPin 5
#define middlePin 6
#define ringPinkyPin 7
#define thumbSidePin 8

//define Pins for MyoWare sensors
#define lowerArmPin A2
#define upperArmPin A3
#define bicepsPin A1

//define pin for button
#define buttonPin 12

//define display screen
#define OLED_RESET 9
Adafruit_SSD1306 Display(OLED_RESET);

//declare and initialize variables
//thresholds for each sensor
int lowerArmThreshold = 300;
int upperArmThreshold = 300;
int bicepsThreshold = 300;
//a minimum and a maximum threshold used to check if calculated thresholds are correct
int minThreshold = 200;
int maxThreshold = 600;

//variables to read Myoware Sensor input
int lowerArmValue = 0;
int upperArmValue = 0;
int bicepsValue = 0;

//flag
int flag = 1;

//number of readings for each sensor
const int numReadings = 100;


//variable to ceck if button pressed
int buttonState = 0;
//variable for mode (can be 1 and 2)
int mode = 1;
//variable for gesture name for LCD display
String gesture = " open hand";

//boolean variables for calibration
bool calibrated = false;
bool calibratedLowerArmSensor = false;
bool calibratedUpperArmSensor = false;
bool calibratedBicepsSensor = false;

//array and size of array to hold data read by bluetooth
const byte numCharacters = 32;
char receivedChars[numCharacters];
//boolean variable to keep track when new data received from BT
bool newData = false;




//addresses where thresholds are stored in EEPROM
int addressLowerArmThreshold = 0;
int addressUpperArmThreshold = 2;
int addressBicepsThreshold = 4;

//functions declaration

//Reads "numReadings" times from the analog input pin and it returns the average
int sensorCalibration(int pin);
//Checks validity of a threshold
bool isThresholdValid(int thresh);

//Reads the analog input pin 10x and returns an average
int getSensorValue(int pin);

//Functions for gestures of the arm
void open_hand();
void close_hand();
void pointing();
void pincing();
void peace_out();
void alright();
void  thumb_up();
//void come_here();
//void doAllGestures();

//Bluetooth communications functions
void showNewData();
void receiveData();
void processIncomingData();

//writing thresholds to EEPROM
void writeThresholdsToEEPROM();
void readThresholdsFromEEPROM();

boolean now = true;


/**********************************************************************
  This is the setup function. It runs once when the Arduino is turned on
 **********************************************************************/
void setup() {
  // put your setup code here, to run once:

  //start serial communication with Serial Monitor, bluetooth module, and LCD display
  Display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
  Serial.begin(9600);
  BTserial.begin(9600);


  //attach servos
  thumbServo.attach(thumbPin);
  indexServo.attach(indexPin);
  middleServo.attach(middlePin);
  ringPinkyServo.attach(ringPinkyPin);
  thumbSideServo.attach(thumbSidePin);

  //set UArm to open position
  open_hand();

    // LCD setup
  Display.clearDisplay();
  //Display.setFont(&FreeMonoBoldOblique12pt7b);
  Display.setTextSize(0);
  Display.setTextColor(WHITE, BLACK);
  Display.setCursor(0, 18);
  Display.println("U-Arm");
  Display.drawLine(1,22,72,22,WHITE);  
  Display.drawLine(0,24,70,24,WHITE);
  Display.fillRect(123,9.5,5,8,WHITE);
  Display.drawRect(75,0,48,27,WHITE);
  Display.display();
  delay (1000);
  Display.setFont(NULL);

  //We need to read all the thresholds stored in EEPROM and turn the calibrated boolean variables true
  //writeThresholdsToEEPROM();
  //we need to check that there exist a valid threshold in eeprom.
  //readThresholdsFromEEPROM();
  //calibrated = true;

  //Run next code if you need to see on the Serial Monitor what values were read from EEPROM
  //Serial.println(lowerArmThreshold);
  //Serial.println(upperArmThreshold);
  //Serial.println(bicepsThreshold);



  //run this function to check that all servos are working properly
  //doAllGestures();

}


/******************************************
  This is the main loop that repeats forever
 *****************************************/
void loop() {
  // put your main code here, to run repeatedly:

  

  //read data coming from Android app via bluetooth
 // receiveData();
  //display data on Serial Monitor
 // showNewData();
  // process data (turn calibration flags false in order to start the calibration process)
  //processIncomingData();
  


  //check if calibrated
  /*if (!calibrated) {
    //If all sensor were calibrated, make calibrated variable true
    if (calibratedLowerArmSensor && calibratedUpperArmSensor && calibratedBicepsSensor) {
      calibrated = true;
      //We should store this values in the EEPROM memory
      writeThresholdsToEEPROM();
      BTserial.print(F("Your sensors were calibrated successfully!"));
    }
  }
  else 
  {
    if(!isThresholdValid(lowerArmThreshold) || !isThresholdValid(upperArmThreshold) || !isThresholdValid(bicepsThreshold)) {
      //print a message to the LCD screen stating that UArm needs calibration
      
    }
    else 
    {*/
      //get sensor values at lowerArmPin, upperArmPin, bicepsPin
      //Display battery level on LCD
     // DrawTitles(double(readVcc())/1024);
      lowerArmValue = getSensorValue(lowerArmPin);
      upperArmValue = getSensorValue(upperArmPin);
      bicepsValue = getSensorValue(bicepsPin);

      //check if sensor value is higher then threshold and converting to digital
       if (upperArmValue > upperArmThreshold) { upperArmValue = HIGH; }
       else if (lowerArmValue > lowerArmThreshold) { lowerArmValue = HIGH; }
       else  if (bicepsValue > bicepsThreshold) { bicepsValue = HIGH; }

      //Read button state, if pressed
      buttonState = digitalRead(buttonPin);

      // check if the pushbutton was pressed and change modes accordingly
      if (buttonState == HIGH && mode == 1) { mode = 2; }
      else if (buttonState == HIGH && mode == 2) { mode = 1; }

      //Checking modes
      if (mode == 1) {
        //when flag=1 do gesture, when flag=2 open hand
        if(flag == 1) {
          if(bicepsValue == HIGH) {
            close_hand();
            Serial.println(F("close hand"));
            gesture = " close hand";
            flag = 2;
          }
          else if (lowerArmValue == HIGH) {
            pincing();
            Serial.println(F("pincing"));
            gesture = " pincing";
            flag = 2;
          }
          else if(upperArmValue == HIGH) {
            pointing();
            gesture = " pointing";
            Serial.println(F("pointing"));
            flag = 2;
          }
        }
        else if(flag == 2) {
          if(bicepsValue == HIGH || lowerArmValue == HIGH || upperArmValue == HIGH) {
            open_hand();
            Serial.println(F("open hand"));
            gesture = " open hand";
            flag = 1;
            Serial.println(flag);
          } 
        }
      }
      else if (mode == 2) {
         if(flag == 1) {
          if(bicepsValue == HIGH) {
            peace_out();
            gesture = " peace out";
            flag = 2;
          }
          else if (lowerArmValue == HIGH) {
            alright();            
            gesture = " alright";
            flag = 2;
          }
          else if(upperArmValue == HIGH) {
            thumb_up();            
            gesture = " thumb up";
            flag = 2;
          }
        }
        else if(flag == 2) {
          if(bicepsValue == HIGH || lowerArmValue == HIGH || upperArmValue == HIGH) {
            open_hand();
            gesture = " open hand";
            flag == 1;
          } 
        }
      }
      delay(2000);
    //}
  //}
}
//end of main loop


/**********************************************
  Function definition
 ***********************************************/

int sensorCalibration(int pin)
{
  int reading[numReadings] = {0};
  long average = 0;
  for (int i = 0; i < numReadings; i++)
  {
    reading[i] = analogRead(pin);
    average+=reading[i];
    delay(50);
  }
 
  return average / numReadings;
}

int getSensorValue(int pin)
{
  int value = 0;
  for (int i = 0; i < 10; i++)
  {
    value += analogRead(pin);
  }
  return value / 10;
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void DrawTitles(double volt) {
   Display.setTextSize(2);
    Display.setTextColor(WHITE, BLACK);
    Display.setCursor(25, 30);
    Display.print(" Mode ");
    Display.println(mode);
    Display.print(gesture);
   Display.display();
   if(volt >= 4.65)
   { 
    Display.fillRect(77,2,8,23,WHITE);
    Display.fillRect(86,2,8,23,WHITE);
    Display.fillRect(95,2,8,23,WHITE);
    Display.fillRect(104,2,8,23,WHITE);
    Display.fillRect(113,2,8,23,WHITE);
    Display.fillTriangle(85,22,  105,16,  99,11, BLACK);
    Display.fillTriangle(112,4,  93,8,  99,13, BLACK);}
   else if(volt < 4.72 &&volt >= 4.62)
   {
    Display.fillRect(77,2,8,23,WHITE);
    Display.fillRect(86,2,8,23,WHITE);
    Display.fillRect(95,2,8,23,WHITE);
    Display.fillRect(104,2,8,23,WHITE);
    Display.fillRect(113,2,8,23,BLACK);
    Display.fillTriangle(85,22,  105,16,  99,11, BLACK);
    Display.fillTriangle(112,4,  93,8,  99,13, BLACK);}
   else if(volt < 4.62 &&volt >= 4.52)
   {
    Display.fillRect(77,2,8,23,WHITE);
    Display.fillRect(86,2,8,23,WHITE);
    Display.fillRect(95,2,8,23,WHITE);
    Display.fillRect(104,2,8,23,BLACK);
    Display.fillRect(113,2,8,23,BLACK);
    Display.fillTriangle(85,22,  105,16,  99,11, BLACK);
    Display.fillTriangle(112,4,  93,8,  99,13, BLACK);
    Display.fillTriangle(103,17,  105,16,  103,15, WHITE);
    Display.fillTriangle(112,5,  103,7,  103,10, WHITE);
   }
   else if(volt < 4.52 &&volt >= 4.42)
   {
    Display.fillRect(77,2,8,23,WHITE);
    Display.fillRect(86,2,8,23,WHITE);
    Display.fillRect(95,2,8,23,BLACK);
    Display.fillRect(104,2,8,23,BLACK);
    Display.fillRect(113,2,8,23,BLACK);
    Display.fillTriangle(85,22,  105,16,  99,11, BLACK);
    Display.fillTriangle(112,4,  93,8,  99,13, WHITE);
    Display.fillTriangle(94,20,  105,16,  99,11, WHITE);
    Display.fillTriangle(94,15,  94,20,  99,11, WHITE);
   }
   else if(volt < 3.90 &&volt >= 3.75)
   {
    Display.fillRect(77,2,8,23,WHITE);
    Display.fillRect(86,2,8,23,BLACK);
    Display.fillRect(95,2,8,23,BLACK);
    Display.fillRect(104,2,8,23,BLACK);
    Display.fillRect(113,2,8,23,BLACK);
    Display.fillTriangle(85,22,  105,16,  99,11, WHITE);
    Display.fillTriangle(112,4,  93,8,  99,13, WHITE);}
   else 
   {
    Display.fillRect(77,2,8,23,BLACK);
    Display.fillRect(86,2,8,23,BLACK);
    Display.fillRect(95,2,8,23,BLACK);
    Display.fillRect(104,2,8,23,BLACK);
    Display.fillRect(113,2,8,23,BLACK);
    Display.fillTriangle(85,22,  105,16,  99,11, WHITE);
    Display.fillTriangle(112,4,  93,8,  99,13, WHITE);}
   
   Display.display();
}

/*void doAllGestures()
{
  close_hand();
  delay(500);
  open_hand();
  delay(500);
  pincing();
  delay(500);
  open_hand();
  delay(500);
  pointing();
  delay(500);
  open_hand();
  delay(500);
  peace_out();
  delay(500);
  open_hand();
  delay(500);
  thumb_up();
  delay(500);
  open_hand();
  delay(500);
}*/


void open_hand()
{
  thumbServo.write(100);
  indexServo.write(180);
  middleServo.write(0);
  ringPinkyServo.write(0);
  thumbSideServo.write(70);
}

void close_hand()
{
  thumbServo.write(10);
  indexServo.write(0);
  middleServo.write(180);
  ringPinkyServo.write(180);
  thumbSideServo.write(30);
}

void pointing()
{
  thumbServo.write(10);
  indexServo.write(180);
  middleServo.write(180);
  ringPinkyServo.write(180);
  thumbSideServo.write(30);
}

void pincing()
{
  thumbServo.write(30);
  indexServo.write(20);
  middleServo.write(100);
  ringPinkyServo.write(0);
  thumbSideServo.write(0);
}

void peace_out()
{
  thumbServo.write(10);
  indexServo.write(180);
  middleServo.write(0);
  ringPinkyServo.write(180);
  thumbSideServo.write(20);
}

void alright()
{
  thumbServo.write(40);
  indexServo.write(20);
  middleServo.write(0);
  ringPinkyServo.write(0);
  thumbSideServo.write(0);
}

void  thumb_up()
{
  thumbServo.write(100);
  indexServo.write(0);
  middleServo.write(180);
  ringPinkyServo.write(180);
  thumbSideServo.write(70);
}

/*void come_here()
{
  while ( bicepsValue >= bicepsThreshold &&  upperArmValue >= upperArmThreshold)
  {
    close_hand();
    delay(400);
    pointing();
    delay(400);
    upperArmValue = analogRead(upperArmPin);
    // lowerarm = analogRead(A2);
    bicepsValue = analogRead(bicepsPin);
  }
}*/
void receiveData() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char character;

  while (BTserial.available() > 0 && newData == false) {
    character = BTserial.read();

    if (recvInProgress == true) {
      if (character != endMarker) {
        receivedChars[ndx] = character;
        ndx++;
        if (ndx >= numCharacters) {
          ndx = numCharacters - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (character == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print(F("This just in ... "));
    Serial.println(receivedChars);

  }
}

void processIncomingData() {
  //used for millis
  unsigned long startMillis = 0;
  unsigned long currentMillis = 0;
  int period = 4000;
  //constant characters to be sent or received via bluetooth
  //these caracters will be received from Android app
  const char START_CALIBRATION = 'C';
  const char CALIBRATE_LAS = '1';
  const char CALIBRATE_UAS = '2';
  const char CALIBRATE_BS = '3';
  if (newData == true) {
    char data = receivedChars[0];
    if (data == CALIBRATE_LAS) {
      calibrated = false;
      startMillis = millis();
      while(millis() < startMillis + period) {//do nothing, a delay that runs exactly period/1000 seconds
        } 
      lowerArmThreshold = sensorCalibration(lowerArmPin);
      Serial.println(F("The new value of lower arm threshold is: "));
      Serial.println(lowerArmThreshold);
      if (isThresholdValid(lowerArmThreshold)) {
        calibratedLowerArmSensor = true;
      }  
    }
    if (data == CALIBRATE_UAS) {
      startMillis = millis();
      while(millis() < startMillis + period) {//do nothing, a delay that runs exactly period/1000 seconds
        } 
      upperArmThreshold = sensorCalibration(upperArmPin);
      Serial.println(F("The new value of upper arm threshold is: "));
      Serial.println(upperArmThreshold);
      if (isThresholdValid(upperArmThreshold)) {
        calibratedUpperArmSensor = true;
      } 
    }
    if (data == CALIBRATE_BS) {
      startMillis = millis();
      while(millis() < startMillis + period) {//do nothing, a delay that runs exactly period/1000 seconds
        } 
      bicepsThreshold = sensorCalibration(bicepsPin);
      Serial.println(F("The new value of biceps threshold is: "));
      Serial.println(bicepsThreshold);
      if (isThresholdValid(bicepsThreshold)) {
        calibratedBicepsSensor = true;
      } 
    }
  }
  newData = false;
}


bool isThresholdValid(int thresh) {
  if (thresh >= minThreshold && thresh <= maxThreshold) {
    return true;
  }
  else
  {
    return false;
    //some code to send data to UArm app to redo calibration
  }
}

void writeThresholdsToEEPROM() {
  EEPROM.put(addressLowerArmThreshold, lowerArmThreshold);
  EEPROM.put(addressUpperArmThreshold, upperArmThreshold);
  EEPROM.put(addressBicepsThreshold, bicepsThreshold);
}
void readThresholdsFromEEPROM() {
  EEPROM.get(addressLowerArmThreshold, lowerArmThreshold);
  EEPROM.get(addressUpperArmThreshold, upperArmThreshold);
  EEPROM.get(addressBicepsThreshold, bicepsThreshold);
}



