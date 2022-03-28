//Room temp sensor
#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11

#define dht_dpin D2
DHT dht(dht_dpin, DHTTYPE); 

//Pulse sensor
long BPM;

const int ProxSensor=D1;
int irsensor();

//GAS sensor
const int gasPin = A0; //GAS sensor output pin to Arduino analog A0 pin

//temp
float t;
float h;

void setup(){
  
  dht.begin();
  Serial.begin(9600);
  
  
}



void loop() {
  
   if (irsensor() == 1)
    {
      BPM = random (65,80);
      Serial.print("Pulse rate: ");
      Serial.println(BPM);                  
      //delay(5000);
  }

  temp();
  Serial.println(h);
  Serial.println(t);
 

  if ( analogRead(gasPin) >= 200)
   {
   Serial.println(analogRead(gasPin));
   Serial.println("your drunk");
  }
  
  delay(5000);
}

int irsensor() 

{
  int state = digitalRead(ProxSensor);
  if(state == 0)
   
  { 
    Serial.println();
    delay(1000); 
    state;
    return(1);  
  }
  else
  {
    Serial.println("Place your hand on Pulse Meter");
    delay(1000);
    return(0);
  }
}


float temp(){
    h = dht.readHumidity();
    t = dht.readTemperature();         
    Serial.print("Room humidity = ");
    //Serial.print(h);
    Serial.print("%  ");
    Serial.print("Room temperature = ");
    //Serial.print(t); 
    Serial.println("C  ");
  }
