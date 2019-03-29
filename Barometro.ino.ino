#include <Aerospace.h>

Aerospace aerospace;

void setup()
{
  Serial.begin(9600);
 
}

void loop()
{
  float pressure;
  
  //get and print temperatures
  Serial.print("Temp: ");
  Serial.print(aerospace.BME_getTemperature());
  Serial.println("C");//The unit for  Celsius because original arduino don't support special symbols
  
  //get and print atmospheric pressure data
  Serial.print("Pressure: ");
  Serial.print(pressure = aerospace.BME_getPressure());
  Serial.println("Pa");

  //get and print altitude data
  Serial.print("Altitude: ");
  Serial.print(aerospace.BME_getAltitude(pressure));
  Serial.println("m");

  //get and print humidity data
  Serial.print("Humidity: ");
  Serial.print(aerospace.BME_getHumidity());
  Serial.println("%");

  delay(1000);
}
