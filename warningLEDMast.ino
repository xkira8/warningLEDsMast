//Code for indicator LED and science mast
//Written by Satvik Sharma on 20th February 2021
#define MQ4_1 1              //MQ4 sensors 
#define MQ4_2 2
#define MQ4_3 3
#define SCL A5               //For BMP180
#define SDA A4
#define DHT11_PIN 7          //For DHT11
#define ALTITUDE 216.0      // Altitude of wherever your rover is in meters. This would act as your reference. I just used a generic altitude value for Delhi from web
#define JSONBufferSize 300   //JSON buffer size in bytes
#define SerialBufferSize 300 //Serial buffer size in bytes
#define Red 10
#define Yellow 11
#define Green 12

#include <dht.h>      
#include <SFE_BMP180.h>
#include <Wire.h>
#include <ArduinoJson.h>
//#include <TinyGPS++.h>


SFE_BMP180 pressure;
dht DHT;
//TinyGPSPlus gps;  

//String gps_lattitude = "";
//String gps_longitude = "";
//int count = 0;

//void setGPSValues(){
//  count = (count+1)%10000;
//  gps_lattitude = getGPSLat();
//  gps_longitude = getGPSLong();
//}

void setup(){
  Serial.begin(9600);
  Serial.println("REBOOT");
  //setGPSValues();
  //smartDelay(1000);
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
}

void loop(){
  char status;          //Used for error handling in BMP 180
  double T,P,p0,a;
  //char *json=new char[SerialBufferSize];
  boolean r=false,y=false,g=false;
  int timeperiod=1000;  //Timeperiod of the loop in ms
  DynamicJsonDocument doctransmit(1024);   //The JSON doc used for tranmsitting data
  DynamicJsonDocument docreceive(1024);   //The JSON doc used to receiver data
  //JSON transmit format  "{\"LocalTime\":localtimeinmillisecond,\"MQ4 readings\":[MQ4_1,MQ4_2,MQ4_3],\"DHT11\":{\"DHT11 temperature\":DHT11temperature,\"DHT11 Humidity\":DHT11Humidity},\"BMP180 readings\":{\"BMP180 pressure\":BMP180pressure.\"BMP180 temperature\":BMP180temperature}}"
  //JSON receive format   "{\"SamplingTimePeriod\":samplingtimeinmilliseconds,\"RED Status\":Redstatusinboolean,\"YELLOW Status\":Yellowstatusinboolean,\"GREEN Status\":Greenstatusinboolean,}"
  if(Serial.available()>0)
  {
    char* json = Serial.read();
    deserializeJson(docreceive, json);
    timeperiod=docreceive["SamplingTimePeriod"];
    r=docreceive["RED Status"];
    y=docreceive["YELLOW Status"];
    g=docreceive["GREEN Status"];
    digitalWrite(Red,r);
    digitalWrite(Yellow,y);
    digitalWrite(Green,g);
  }
  int chk = DHT.read11(DHT11_PIN);  //Reading DHT11 sensor
  status = pressure.startTemperature();  //Reading BMP180 pressure and temperature sensor
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
         
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); 
          
          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
         
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
  //setGPSValues();
//JSON transmit format  "{\"LocalTime\":localtimeinmillisecond,\"MQ4 readings\":[MQ4_1,MQ4_2,MQ4_3],\"DHT11\":{\"DHT11 temperature\":DHT11temperature,\"DHT11 Humidity\":DHT11Humidity},
//  \"BMP180 readings\":{\"BMP180 pressure\":BMP180pressure,\"BMP180 temperature\":BMP180temperature,\"Normalised Pressure\":BMP180normalisedpressurereading,\"Altitude Estimate\":Altitudeestime},\"GPS\":{\"Longitude\":gps_longitude,\"Latitude\":gps_latitude}}
  int MQ41=analogRead(1);//Reading MQ4 sensors
  int MQ42=analogRead(2);
  int MQ43=analogRead(3);
  doctransmit["LocalTime"] = millis();
  doctransmit["MQ4 readings"][1]= MQ41;
  doctransmit["MQ4 readings"][2]= MQ42;
  doctransmit["MQ4 readings"][3]= MQ43;
  doctransmit["DHT11"]["DHT11 temperature"] = DHT.temperature;
  doctransmit["DHT11"]["DHT11 Humidity"] =DHT.humidity;
  doctransmit["BMP180 readings"]["BMP180 pressure"]=P;
  doctransmit["BMP180 readings"]["BMP180 temperature"]=T;
  doctransmit["BMP180 readings"]["Normalised Pressure"]=p0;
  doctransmit["BMP180 readings"]["Altitude Estimate"]=a;
  //doctransmit["GPS"]["Longitude"]=gps_longitude;
  //doctransmit["GPS"]["Latitude"]=gps_latitude;
  serializeJson(doctransmit, Serial);
  delay(timeperiod);
}
