int analogPin = 3;    
int val = 0;

  void setup()
  {
    Serial.begin(9600);  // this is for serial monitor to see the muscle sensor value you're getting
   
     Serial.begin(14400);                 //for
    Serial.println("CLEARDATA");          //excel
Serial.println("LABEL,Acolumn");          //use
Serial.println("RESETTIMER");             //only
  }
  void loop()
{
            val = analogRead(analogPin); //muscle sensor connected to pin A3 being stated as val
            Serial.println(val);
    
                                  Serial.print("DATA, read:, ");    //excel read, 
                                  Serial.print(val);       //not needed 
                                  Serial.println();                 //after knowing 
                                  delay(5);                         //it works
}
