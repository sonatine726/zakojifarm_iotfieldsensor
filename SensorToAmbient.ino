#include <stdlib.h>
#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include <TinyGPS++.h>
#include "Ambient.h"

//Common global
#define SENSOR_PIN    (WIOLTE_D38)
#define LOOP_PERIOD 10000 // milliceconds

WioLTE Wio;
WioLTEClient WioClient(&Wio);

constexpr int LOG_TEMP_BUF_SIZE = 32;


//LTE global
#define APN       "soracom.io"
#define USERNAME  "sora"
#define PASSWORD  "sora"

//Ambient global
unsigned int AMBIENT_CHANNEL_ID = 5701;
const char AMBIENT_WRITE_KEY[] = "13a769c5371e81ce";
Ambient ambient;

//GPS global
#define GPS_OVERFLOW_STRING "OVERFLOW"

HardwareSerial* GpsSerial;
TinyGPSPlus gps;
const double INVALID_GPS_VALUE = -1;

void setup()
{
    SerialUSB.println("setup()");

    delay(200);

    //Setup LTE
    SerialUSB.println("Setup LTE");
    if(!SetupLTE()){
        return;
    }

    //Setup Ambient
    SerialUSB.println("Setup Ambient");
    ambient.begin(AMBIENT_CHANNEL_ID, AMBIENT_WRITE_KEY, &WioClient);

    //Setup DHT11
    SerialUSB.println("Setup DHT11");
    TemperatureAndHumidityBegin(SENSOR_PIN);

    //Setup GPS
    GpsBegin(&Serial);
    Wio.PowerSupplyGrove(true);
    delay(500);
}

void loop()
{
    unsigned long stime = millis();
    char cbuf[32] = {0};
    snprintf(cbuf, sizeof(cbuf), "loop() : %lu", stime);
    SerialUSB.println(cbuf);

    /* Get temperature and humidity */
    float temp;
    float humi;

    SerialUSB.println("TemperatureAndHumidityRead()");
    if(TemperatureAndHumidityRead(&temp, &humi))
    {
        //Send to serial
        SerialUSB.print("Current humidity = ");
        SerialUSB.print(humi);
        SerialUSB.print("%  ");
        SerialUSB.print("temperature = ");
        SerialUSB.print(temp);
        SerialUSB.println("C");
    }
    else
    {
        SerialUSB.println("ERROR: TemperatureAndHumidityRead");
    }


    /* Get GPS */
    SerialUSB.println("GpsRead()");
    double lat, lng, meter;
    bool validGps = GpsRead(lat, lng, meter);
    if(validGps)
    {
        SerialUSB.println("GPS Value:");
        snprintf(cbuf, sizeof(cbuf), "lat: %12.8f", lat);
        SerialUSB.println(cbuf);

        snprintf(cbuf, sizeof(cbuf), "lng: %12.8f", lng);
        SerialUSB.println(cbuf);

        snprintf(cbuf, sizeof(cbuf), "meter: %4.2f", meter);
        SerialUSB.println(cbuf);
    }

    /* Send to Ambient */
    SerialUSB.println("SendToAmbient()");
    bool isSendSuccess;
    if(validGps)
    {
        isSendSuccess = SendToAmbient(temp, humi, lat, lng, meter);
    }
    else
    {
        isSendSuccess = SendToAmbient(temp, humi);
    }
    
    if(!isSendSuccess)
    {
        SerialUSB.println("ERROR: SendToAmbient");
    }

    /* Wait next loop */
    unsigned long elapse = millis() - stime;
    if (elapse < LOOP_PERIOD)
    {
      delay(LOOP_PERIOD - elapse);
    }
}


//LTE functions
bool SetupLTE()
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


//Temperature and humidity functions
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

    for (int i = 0; i < 8; i++)
    {
        while (digitalRead(pin)) ;

        while (!digitalRead(pin)) ;
        unsigned long start = micros();

        while (digitalRead(pin)) ;
        unsigned long finish = micros();

        if ((unsigned long)(finish - start) > 50)
        {
            data |= 1 << (7 - i);
        }
    }

    return data;
}

void DHT11Finish(int pin)
{
    while (!digitalRead(pin)) ;
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
}

bool DHT11Check(const byte data[5])
{
    byte sum = 0;
    for (int i = 0; i < 4; ++i)
    {
        sum += data[i];
    }

    return (data[4] == sum) && (data[1] < 10) && (data[3] < 10);
}


//GPS functions
void GpsBegin(HardwareSerial* serial)
{
    GpsSerial = serial;
    GpsSerial->begin(9600);
}

bool GpsRead(double& lat, double& lng, double& meter)
{
    while(GpsSerial->available())
    {
        if(gps.encode(GpsSerial->read()))
        {
          break;
        }
    }

    if(!gps.location.isValid())
    {
        SerialUSB.println("INFO: GPS location not valid");
        return false;
    }

    lat = gps.location.lat();
    lng = gps.location.lng();
    meter = gps.altitude.meters();

    return true;
}

// LTE functions
bool SendToAmbient(float temp, float humi)
{
    return SendToAmbient(temp, humi, INVALID_GPS_VALUE, INVALID_GPS_VALUE, INVALID_GPS_VALUE);
}

bool SendToAmbient(float temp, float humi, double lat, double lng, double meter)
{
    if(!ambient.set(1, temp))
    {
        char logBuf[LOG_TEMP_BUF_SIZE] = {0};
        snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(1, %f)", temp);
        SerialUSB.println(logBuf);
        return false;
    }

    if(!ambient.set(2, humi))
    {
        char logBuf[LOG_TEMP_BUF_SIZE] = {0};
        snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(2, %f)", humi);
        SerialUSB.println(logBuf);
        return false;
    }

    char cbuf[16];
    if(lat != INVALID_GPS_VALUE)
    {
        snprintf(cbuf, sizeof(cbuf), "%12.8f", lat);
        if(!ambient.set(9, cbuf))
        {
            char logBuf[LOG_TEMP_BUF_SIZE] = {0};
            snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(9, %s)", cbuf);
            SerialUSB.println(logBuf);
            return false;
        }
    }

    if(lng != INVALID_GPS_VALUE)
    {
        snprintf(cbuf, sizeof(cbuf), "%12.8f", lng);
        if(!ambient.set(10, cbuf))
        {
            char logBuf[LOG_TEMP_BUF_SIZE] = {0};
            snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(10, %s)", cbuf);
            SerialUSB.println(logBuf);
            return false;
        }
    }

    if(meter != INVALID_GPS_VALUE)
    {
        snprintf(cbuf, sizeof(cbuf), "%4.2f", meter);
        if(!ambient.set(3, cbuf))
        {
            char logBuf[LOG_TEMP_BUF_SIZE] = {0};
            snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(3, %s)", cbuf);
            SerialUSB.println(logBuf);
            return false;
        }
    }

    if(!ambient.send())
    {
        SerialUSB.println("ERROR: ambient.send");
        return false;
    }

    return true;
}
