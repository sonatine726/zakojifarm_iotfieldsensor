#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include "Ambient.h"

#define SENSOR_PIN    (WIOLTE_D38)

#define APN               "soracom.io"
#define USERNAME          "sora"
#define PASSWORD          "sora"

#define LOOP_PERIOD 10000 // milliceconds

WioLTE Wio;
WioLTEClient WioClient(&Wio);

unsigned int AMBIENT_CHANNEL_ID = 5701;
const char AMBIENT_WRITE_KEY[] = "13a769c5371e81ce";
Ambient ambient;

void setup()
{
  SerialUSB.println("setup()");

  delay(200);

  //Setup LTE
  SerialUSB.println("Setup LTE");
  if(!setup_lte()){
    return;
  }

  //Setup Ambient
  SerialUSB.println("Setup Ambient");
  ambient.begin(AMBIENT_CHANNEL_ID, AMBIENT_WRITE_KEY, &WioClient);

  //Setup DHT11
  SerialUSB.println("Setup DHT11");
  TemperatureAndHumidityBegin(SENSOR_PIN);
}

bool setup_lte()
{
  Wio.Init();
  Wio.PowerSupplyLTE(true);
  delay(5000);

  if (!Wio.TurnOnOrReset())
  {
    SerialUSB.println("ERROR: Wio.TurnOnOrReset");
    return false;
  }

  if (!Wio.Activate(APN, USERNAME, PASSWORD))
  {
    SerialUSB.println("ERROR: Wio.Activate");
    return false;
  }

  return true;
}

void loop()
{
  unsigned long stime = millis();

  float temp;
  float humi;

  if (TemperatureAndHumidityRead(&temp, &humi))
  {
    //Send to serial
    SerialUSB.print("Current humidity = ");
    SerialUSB.print(humi);
    SerialUSB.print("%  ");
    SerialUSB.print("temperature = ");
    SerialUSB.print(temp);
    SerialUSB.println("C");

    //Send to Ambient
    if(!ambient.set(1, temp) || !ambient.set(2, humi))
    {
      SerialUSB.println("ERROR: ambient.set");
    }

    if(!ambient.send())
    {
      SerialUSB.println("ERROR: ambient.send");
    }
  }
  else
  {  
    SerialUSB.println("ERROR!");
  }
  

  unsigned long elapse = millis() - stime;
  if (elapse < LOOP_PERIOD) {
      delay(LOOP_PERIOD - elapse);
  }
}

////////////////////////////////////////////////////////////////////////////////////////
//

int TemperatureAndHumidityPin;

void TemperatureAndHumidityBegin(int pin)
{
  TemperatureAndHumidityPin = pin;
  DHT11Init(TemperatureAndHumidityPin);
}

bool TemperatureAndHumidityRead(float* temperature, float* humidity)
{
  byte data[5] = {0};
  
  DHT11Start(TemperatureAndHumidityPin);
  for (int i = 0; i < 5; i++)
  {
    data[i] = DHT11ReadByte(TemperatureAndHumidityPin);
  }
  DHT11Finish(TemperatureAndHumidityPin);
  
  if(!DHT11Check(data))
  {
    return false;
  }

  *humidity = (float)data[0] + (float)data[1] / 10.0f;
  *temperature = (float)data[2] + (float)data[3] / 10.0f;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////
//

void DHT11Init(int pin)
{
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

void DHT11Start(int pin)
{
  // Host the start of signal
  digitalWrite(pin, LOW);
  delay(18);
  
  // Pulled up to wait for
  pinMode(pin, INPUT);
  while (!digitalRead(pin)) ;
  
  // Response signal
  while (digitalRead(pin)) ;
  
  // Pulled ready to output
  while (!digitalRead(pin)) ;
}

byte DHT11ReadByte(int pin)
{
  byte data = 0;
  
  for (int i = 0; i < 8; i++) {
    while (digitalRead(pin)) ;

    while (!digitalRead(pin)) ;
    unsigned long start = micros();

    while (digitalRead(pin)) ;
    unsigned long finish = micros();

    if ((unsigned long)(finish - start) > 50) data |= 1 << (7 - i);
  }
  
  return data;
}

void DHT11Finish(int pin)
{
  // Releases the bus
  while (!digitalRead(pin)) ;
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

bool DHT11Check(const byte data[5])
{
  byte sum = 0;
  for (int i = 0; i < 4; ++i) {
    sum += data[i];
  }

  return (data[4] == sum) && (data[1] < 10) && (data[3] < 10);
}


