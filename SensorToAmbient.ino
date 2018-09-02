#include <stdlib.h>
#include <assert.h>

#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include <TinyGPS++.h>
#include <RTClock.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <HardwareSPI.h>

typedef int IRQn_Type;
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    1
#include <libmaple/libmaple_types.h>
#include <libmaple/usbF4/VCP/core_cm4.h>
#include <libmaple/usbF4/VCP/core_cmInstr.h>
#include <libmaple/pwr.h>
#include "Ambient.h"

// Compile Switch
#define C_SW_LTE 1
#define C_SW_AMBIENT 0
#define C_SW_DHT11 0
#define C_SW_DS18B20 0
#define C_SW_GPS 0
#define C_SW_BATTERY_V 0
#define C_SW_MS5540C 1
#define C_SW_SLEEP_WAIT 0

//Common global
#define SENSOR_PIN    (WIOLTE_D38)
//#define LOOP_PERIOD_MSEC 60000 // milliseconds
#define LOOP_PERIOD_MSEC 20000 // milliseconds
#define SLEEP_MSEC_WHEN_OVER_LOOP_PERIOD 10000 // milliseconds

WioLTE Wio;
WioLTEClient WioClient(&Wio);

constexpr int LOG_TEMP_BUF_SIZE = 32;

RTClock rtc(RTCSEL_LSI);
constexpr time_t JAPAN_TIME_DIFF = 9 * 60 * 60; // UTC + 9h

// constexpr uint32* P_RTC_BKP0R = reinterpret_cast<uint32*>(RTC_BASE) + 0x50;

constexpr float INVALID_TEMP_AND_HUMID = -127;


#if C_SW_LTE
//LTE global
#define APN       "soracom.io"
#define USERNAME  "sora"
#define PASSWORD  "sora"
#endif //C_SW_LTE

#if C_SW_AMBIENT
//Ambient global
unsigned int AMBIENT_CHANNEL_ID = 5701;
const char AMBIENT_WRITE_KEY[] = "13a769c5371e81ce";
Ambient ambient;
#endif //C_SW_AMBIENT

constexpr double INVALID_GPS_VALUE = -1;
#if C_SW_GPS
//GPS global
HardwareSerial* GpsSerial;
TinyGPSPlus gps;
#endif //C_SW_GPS

//NTP
const char NTP_SERVER[] = "ntp.nict.jp";

#if C_SW_DS18B20
//DS18B20
OneWire oneWire(WIOLTE_A4);
DallasTemperature dS18b20(&oneWire);
constexpr uint8_t DS18B20_TEMPERATURE_RESOLUTION_BIT = 12;
#endif //C_SW_DS18B20

#if C_SW_MS5540C
TwoWire exRtcI2c;

constexpr uint8 RTC_I2C_ADDR = 0x51;

constexpr uint8 RTC_REG_CTR1 = 0x00;
constexpr uint8 RTC_REG_CTR2 = 0x01;
constexpr uint8 RTC_REG_SEC = 0x02;
constexpr uint8 RTC_REG_MIN = 0x03;
constexpr uint8 RTC_REG_HOUR = 0x04;
constexpr uint8 RTC_REG_DAYS = 0x05;
constexpr uint8 RTC_REG_WEEK = 0x06;
constexpr uint8 RTC_REG_MONTH = 0x07;
constexpr uint8 RTC_REG_YEAR = 0x08;
constexpr uint8 RTC_REG_MIN_ALR = 0x09;
constexpr uint8 RTC_REG_HOUR_ALR = 0x0A;
constexpr uint8 RTC_REG_DAY_ALR = 0x0B;
constexpr uint8 RTC_REG_WEEK_ALR = 0x0C;
constexpr uint8 RTC_REG_CLKO_FREQ = 0x0D;
constexpr uint8 RTC_REG_TIMER_CTR = 0x0E;
constexpr uint8 RTC_REG_TIMER = 0x0F;

constexpr uint8 CLOCK_PIN_FOR_MS5540C = 9;

HardwareSPI ms5540cSpi(1);
#endif //C_SW_MS5540C


#if C_SW_BATTERY_V
constexpr unsigned int EXTERNAL_BATTERY_ADC_PIN = 5;
constexpr unsigned int INTERNAL_BATTERY_ADC_CHANNEL = 18;
constexpr unsigned int INTERNAL_TEMPERATURE_ADC_CHANNEL = 16;
#endif //C_SW_BATTERY_V

void setup()
{
	Wio.Init();

    SerialUSB.println("INFO: setup()");

	Wio.PowerSupplyGrove(true);

#if C_SW_LTE
    //Setup LTE
    SerialUSB.println("INFO: Setup LTE");
    if(!SetupLTE()){
        return;
    }

	UpdateRtcByNtp();

#if C_SW_AMBIENT
    //Setup Ambient
    SerialUSB.println("INFO: Setup Ambient");
    ambient.begin(AMBIENT_CHANNEL_ID, AMBIENT_WRITE_KEY, &WioClient);
#endif //C_SW_AMBIENT

#else //C_SW_LTE
	delay(500);
#endif //C_SW_LTE

#if C_SW_DHT11
    //Setup DHT11
    SerialUSB.println("INFO: Setup DHT11");
    TemperatureAndHumidityBegin(SENSOR_PIN);
#endif //C_SW_DHT11

#if C_SW_DS18B20
    //Setup DS18B20
    SerialUSB.println("INFO: SetupDS18B20");
    SetupDS18B20();
#endif //C_SW_DS18B20

#if C_SW_MS5540C
	struct tm current_time = { 0 };
	rtc.getTime(&current_time);

	SetupExtRtc(current_time);

	SetupMs5540c();
#endif //C_SW_MS5540C

#if C_SW_BATTERY_V
	SetupADC();
#endif //C_SW_BATTERY_V

#if C_SW_GPS
    //Setup GPS
    GpsBegin(&Serial);
    Wio.PowerSupplyGrove(true);
    delay(500);
#endif
}

void loop()
{
	char cbuf[64] = {0};
    struct tm current_time = {0};
    rtc.getTime(&current_time);
    snprintf(cbuf, sizeof(cbuf), "loop() : %s", asctime(&current_time));
    SerialUSB.println(cbuf);

    /* Get temperature and humidity */
    float temp = INVALID_TEMP_AND_HUMID;
    float humi = INVALID_TEMP_AND_HUMID;

#if C_SW_DHT11
    SerialUSB.println("INFO: TemperatureAndHumidityRead()");
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
#endif //C_SW_DHT11

	float tempDs18b20 = INVALID_TEMP_AND_HUMID;
#if C_SW_DS18B20
    /* Get water temperature from DS18B20 */
    SerialUSB.println("INFO: Get DS18B20 Temperature");
    tempDs18b20 = GetTemperatureDS18B20();
    snprintf(cbuf, sizeof(cbuf), "INFO: DS18B20 temp: %f", tempDs18b20);
    SerialUSB.println(cbuf);

	// [DEBUG]
	//{
	//	SerialUSB.println("DEBUG: GPIOA_PUPDR = ");
	//	char logBuf[LOG_TEMP_BUF_SIZE] = {0};
	//	snprintf(logBuf, sizeof(logBuf), "%#08X", GPIOA_BASE->PUPDR);
	//	SerialUSB.println(logBuf);
	//}
#endif //C_SW_DS18B20

	float press_ms5540c = 0;
	float temp_ms5540c = INVALID_TEMP_AND_HUMID;
#if C_SW_MS5540C
	//[DEBUG]
	{
		exRtcI2c.beginTransmission(RTC_I2C_ADDR);
		exRtcI2c.write(RTC_REG_SEC);
		exRtcI2c.endTransmission();

		uint8_t regv;
		exRtcI2c.requestFrom(RTC_I2C_ADDR, 1);
		while (exRtcI2c.available())
		{
			regv = exRtcI2c.read(); // receive a byte as character
		}

		SerialUSB.println("DEBUG: RTC8564 RTC_REG_SEC = ");
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "%#02X", regv);
		SerialUSB.println(logBuf);
	}

	{
		exRtcI2c.beginTransmission(RTC_I2C_ADDR);
		exRtcI2c.write(RTC_REG_CLKO_FREQ);
		exRtcI2c.endTransmission();

		uint8_t regv;
		exRtcI2c.requestFrom(RTC_I2C_ADDR, 1);
		while (exRtcI2c.available())
		{
			regv = exRtcI2c.read(); // receive a byte as character
		}

		SerialUSB.println("DEBUG: RTC8564 RTC_REG_CLKO_FREQ = ");
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "%#02X", regv);
		SerialUSB.println(logBuf);
	}

	GetPressureAndTemperatureFromMs5540c(press_ms5540c, temp_ms5540c);
	SerialUSB.println("INFO: MS5540C press and temp= ");
	SerialUSB.println(press_ms5540c);
	SerialUSB.println(temp_ms5540c);
#endif //C_SW_MS5540C

	bool validGps = false;
	double lat, lng, meter;
#if C_SW_GPS
    /* Get GPS */
    SerialUSB.println("INFO: GpsRead()");
    validGps = GpsRead(lat, lng, meter);
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
#endif //C_SW_GPS

	float ext_battery_v = 0;
	float in_battery_v = 0;
	float in_temperature = 0;
#if C_SW_BATTERY_V
	ext_battery_v = GetExternalBatteryV();
	snprintf(cbuf, sizeof(cbuf), "INFO: External Battery V %f mV", ext_battery_v);
	SerialUSB.println(cbuf);

	in_battery_v = GetInternalBatteryV();
	snprintf(cbuf, sizeof(cbuf), "INFO: Internal Battery V %f mV", in_battery_v);
	SerialUSB.println(cbuf);

	in_temperature = GetInternalTemperature();
	snprintf(cbuf, sizeof(cbuf), "INFO: Internal Temp %f", in_temperature);
	SerialUSB.println(cbuf);


	//{
	//	snprintf(cbuf, sizeof(cbuf), "DEBUG: ADC1_BASE->CR1 %08X", ADC1_BASE->CR1);
	//	SerialUSB.println(cbuf);
	//}

	//{
	//	snprintf(cbuf, sizeof(cbuf), "GPIOA_BASE->MODER %08X", GPIOA_BASE->MODER);
	//	SerialUSB.println(cbuf);
	//}

	//{
	//	snprintf(cbuf, sizeof(cbuf), "GPIOA_BASE->AFRL %08X", GPIOA_BASE->AFR[0]);
	//	SerialUSB.println(cbuf);
	//}

	//{
	//	snprintf(cbuf, sizeof(cbuf), "GPIOA_BASE->AFRH %08X", GPIOA_BASE->AFR[1]);
	//	SerialUSB.println(cbuf);
	//}
	//[DEBUG END]
#endif //C_SW_BATTERY_V

#if C_SW_LTE && C_SW_AMBIENT
    /* Send to Ambient */
    SerialUSB.println("INFO: SendToAmbient()");
    bool isSendSuccess;
    if(validGps)
    {
        isSendSuccess = SendToAmbient(temp, humi, tempDs18b20, ext_battery_v, in_battery_v, in_temperature, lat, lng, meter);
    }
    else
    {
        isSendSuccess = SendToAmbient(temp, humi, tempDs18b20, ext_battery_v, in_battery_v, in_temperature);
    }

    if(!isSendSuccess)
    {
        SerialUSB.println("ERROR: SendToAmbient");
    }
#endif //C_SW_LTE && C_SW_AMBIENT

    /* Wait next loop */
    unsigned long elapse = millis();
    snprintf(cbuf, sizeof(cbuf), "Run elapse: %ld msec", elapse);
    SerialUSB.println(cbuf);

#if C_SW_SLEEP_WAIT
	time_t waittime_sec = 0;
	if (LOOP_PERIOD_MSEC > elapse)
	{
		waittime_sec = (LOOP_PERIOD_MSEC - elapse) / 1000;
	}
	else
	{
		waittime_sec = SLEEP_MSEC_WHEN_OVER_LOOP_PERIOD / 1000;
	}
    
	snprintf(cbuf, sizeof(cbuf), "Wait next loop: %ld sec", waittime_sec);
    SerialUSB.println(cbuf);

	Wio.PowerSupplyGrove(false);


#if C_SW_LTE
	ShutdownLTE();
#endif //C_SW_LTE

    SleepUntilNextLoop(waittime_sec);
#else //C_SW_SLEEP_WAIT
	delay(LOOP_PERIOD_MSEC);
#endif //C_SW_SLEEP_WAIT
}

#if C_SW_LTE
//LTE functions
bool SetupLTE()
{
    Wio.PowerSupplyLTE(true);
    delay(500);

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


void ShutdownLTE()
{
    Wio.Deactivate();  // Deactivate a PDP context. Added at v1.1.9
    Wio.TurnOff(); // Shutdown the LTE module. Added at v1.1.6
    Wio.PowerSupplyLTE(false); // Turn the power supply to LTE module off
}
#endif //C_SW_LTE


#if C_SW_DHT11
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
#endif //C_SW_DHT11


#if C_SW_DS18B20
/* Get water temperature from DS18B20 functions */
void SetupDS18B20()
{
    dS18b20.begin();
    dS18b20.setResolution(DS18B20_TEMPERATURE_RESOLUTION_BIT);
}

float GetTemperatureDS18B20()
{
	dS18b20.requestTemperatures();
    return dS18b20.getTempCByIndex(0);
}
#endif //C_SW_DS18B20



#if C_SW_MS5540C
//External RTC functions
void SetupExtRtc(tm& current_time)
{
	// [DEBUG]
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "DEBUG: SetupExtRtc : %s", asctime(&current_time));
		SerialUSB.println(cbuf);
	}

	exRtcI2c.begin();

	exRtcI2c.beginTransmission(RTC_I2C_ADDR);
	exRtcI2c.write(RTC_REG_CTR1);
	exRtcI2c.write(0x20); //Write to RTC_REG_CTR1.STOP=1
	exRtcI2c.write(0x00); //Write to RTC_REG_CTR2

	const uint8 sec_to_reg = GetRtcTimeRegValue(current_time.tm_sec);
	exRtcI2c.write(sec_to_reg); //Write to RTC_REG_SEC

	const uint8 min_to_reg = GetRtcTimeRegValue(current_time.tm_min);
	exRtcI2c.write(min_to_reg); //Write to RTC_REG_MIN

	const uint8 hour_to_reg = GetRtcTimeRegValue(current_time.tm_hour);
	exRtcI2c.write(hour_to_reg); //Write to RTC_REG_HOUR

	const uint8 day_to_reg = GetRtcTimeRegValue(current_time.tm_mday);
	exRtcI2c.write(day_to_reg); //Write to RTC_REG_DAYS

	const uint8 weekday_to_reg = GetRtcTimeRegValue(current_time.tm_wday);
	exRtcI2c.write(weekday_to_reg); //Write to RTC_REG_WEEK

	const uint8 month_to_reg = GetRtcTimeRegValue(current_time.tm_mon);
	exRtcI2c.write(month_to_reg); //Write to RTC_REG_MONTH

	const uint8 year_to_reg = GetRtcTimeRegValue(current_time.tm_year);
	exRtcI2c.write(year_to_reg); //Write to RTC_REG_YEAR

	exRtcI2c.write(0x00); //Write to RTC_REG_MIN_ALR
	exRtcI2c.write(0x00); //Write to RTC_REG_HOUR_ALR
	exRtcI2c.write(0x00); //Write to RTC_REG_DAY_ALR
	exRtcI2c.write(0x00); //Write to RTC_REG_WEEK_ALR

	exRtcI2c.write(0x80); //Write to RTC_REG_CLKO_FREQ

	exRtcI2c.write(0x00); //Write to RTC_REG_TIMER_CTR
	exRtcI2c.write(0x00); //Write to RTC_REG_TIMER

	exRtcI2c.endTransmission();
}

uint8 GetRtcTimeRegValue(uint32 time)
{
	uint8 regv = 0;
	if (time >= 80)
	{
		regv |= 1 << 7;
		time -= 80;
	}
	if (time >= 40)
	{
		regv |= 1 << 6;
		time -= 40;
	}
	if (time >= 20)
	{
		regv |= 1 << 5;
		time -= 20;
	}
	if (time >= 10)
	{
		regv |= 1 << 4;
		time -= 10;
	}
	if (time >= 8)
	{
		regv |= 1 << 3;
		time -= 8;
	}
	if (time >= 4)
	{
		regv |= 1 << 2;
		time -= 4;
	}
	if (time >= 2)
	{
		regv |= 1 << 1;
		time -= 2;
	}
	if (time == 1)
	{
		regv |= 1;
		time -= 1;
	}
	assert(time == 0);

	return regv;
}

void SetupMs5540c()
{
	ms5540cSpi.begin(SPI_281_250KHZ, MSBFIRST, SPI_MODE0);

	//[DEBUG]
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "DEBUG: GPIOA_BASE->AFR[0] %08X", GPIOA_BASE->AFR[0]);
		SerialUSB.println(cbuf);
		snprintf(cbuf, sizeof(cbuf), "DEBUG: GPIOA_BASE->AFR[1] %08X", GPIOA_BASE->AFR[1]);
		SerialUSB.println(cbuf);
		snprintf(cbuf, sizeof(cbuf), "DEBUG: GPIOB_BASE->AFR[0] %08X", GPIOB_BASE->AFR[0]);
		SerialUSB.println(cbuf);
		snprintf(cbuf, sizeof(cbuf), "DEBUG: GPIOB_BASE->AFR[1] %08X", GPIOB_BASE->AFR[1]);
		SerialUSB.println(cbuf);
	}
}

void GetPressureAndTemperatureFromMs5540c(float& pressure, float& temperature)
{
	//Read calibration register
	const uint32 calib_reg1 = SendCommandAndGetWord(0x1D, 0x50, 0);
	const uint32 calib_reg2 = SendCommandAndGetWord(0x1D, 0x60, 0);
	const uint32 calib_reg3 = SendCommandAndGetWord(0x1D, 0x90, 0);
	const uint32 calib_reg4 = SendCommandAndGetWord(0x1D, 0xA0, 0);

	//Get calibration value
	const uint32 c1 = calib_reg1 >> 1;
	const uint32 c2 = ((calib_reg3 & 0x3F) << 6) | (calib_reg4 & 0x3F);
	const uint32 c3 = (calib_reg4 >> 6);
	const uint32 c4 = (calib_reg3 >> 6);
	const uint32 c5 = (calib_reg2 >> 6) | ((calib_reg1 & 0x1) << 10);
	const uint32 c6 = calib_reg2 & 0x3F;

	// [DEBUG]
	{
		SerialUSB.println("DEBUG: Ms5540c calib values =");
		SerialUSB.println(c1);
		SerialUSB.println(c2);
		SerialUSB.println(c3);
		SerialUSB.println(c4);
		SerialUSB.println(c5);
		SerialUSB.println(c6);
	}

	//Get temperature register value
	const uint16 temp_reg = SendCommandAndGetWord(0x1D, 0x20, 35);

	//Get pressure register value
	const uint16 press_reg = SendCommandAndGetWord(0x1D, 0x40, 35);

	//Calculate temperature
	long dT, raw_temperature_decuple, temperature_compensation;
	temperature = CalculateTemperatureByMs5540c(temp_reg, c5, c6, dT, raw_temperature_decuple, temperature_compensation);

	//Calculate pressure
	pressure = CalculatePressureByMs5540c(press_reg, c1, c2, c3, c4, dT, raw_temperature_decuple, temperature_compensation);
}

void ResetMs5540c()
{
	BeginSPItoMs5540c(SPI_MODE0);
	ms5540cSpi.transfer(0x15);
	ms5540cSpi.transfer(0x55);
	ms5540cSpi.transfer(0x40);
}

uint16 SendCommandAndGetWord(uint8 command_msb, uint8 command_lsb, unsigned int wait_after_command_msec)
{
	ResetMs5540c();

	ms5540cSpi.transfer(command_msb);
	ms5540cSpi.transfer(command_lsb);

	delay(wait_after_command_msec);

	BeginSPItoMs5540c(SPI_MODE1);
	uint16 word_msb = ms5540cSpi.transfer(0x00);
	word_msb <<= 8;
	uint16 word_lsb = ms5540cSpi.transfer(0x00);
	return word_msb | word_lsb;
}

float CalculateTemperatureByMs5540c(uint16 temp_reg, uint32 c5, uint32 c6, long& r_dT, long& r_raw_temperature_decuple, long& r_temperature_compensation)
{
	const long UT1 = (c5 * 8) + 20224;
	const long dT = (temp_reg - UT1);
	long TEMP = 200 + ((dT * (c6 + 50)) / 1024);
	const long TEMP_COMPENSATION = GetCompensateValueForTemperature(TEMP, c6);

	r_dT = dT;
	r_raw_temperature_decuple = TEMP;
	r_temperature_compensation = TEMP_COMPENSATION;

	return static_cast<float>(TEMP - TEMP_COMPENSATION) / 10;
}

float CalculatePressureByMs5540c(uint16 press_reg, uint32 c1, uint32 c2, uint32 c3, uint32 c4, long dT, long TEMP, long temperature_compensation)
{
	const long OFF = (c2 * 4) + (((c4 - 512) * dT) / 4096);
	const long SENS = c1 + ((c3 * dT) / 1024) + 24576;
	const long X = ((SENS * (press_reg - 7168)) / 16384) - OFF;
	long P = X * 10 / 32 + 2500;
	P -= GetCompensateValueForPressure(P, TEMP, temperature_compensation);
	return P;
}

long GetCompensateValueForTemperature(long TEMP, uint32 c6)
{
	long t2 = 0;
	if (TEMP < 200)
	{
		t2 = 11 * (c6 + 24) * (200 - TEMP) * (200 - TEMP) / 1048576;
	}
	else if (TEMP > 450)
	{
		t2 = 3 * (c6 + 24) * (450 - TEMP) * (450 - TEMP) / 1048576;
	}

	return t2;
}

long GetCompensateValueForPressure(uint16 press_reg, long TEMP, long temperature_compensation)
{
	long p2 = 0;
	if (TEMP < 200)
	{
		p2 = 3 * temperature_compensation * (press_reg - 3500) / 16384;
	}
	else if (TEMP > 450)
	{
		p2 = temperature_compensation * (press_reg - 10000) / 8192;
	}

	return p2;
}


void BeginSPItoMs5540c(uint32 mode)
{
	ms5540cSpi.begin(SPI_281_250KHZ, MSBFIRST, mode);
}
#endif //C_SW_MS5540C


#if C_SW_GPS
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
#endif //C_SW_GPS

#if C_SW_AMBIENT
// Ambient functions
bool SendToAmbient(float temp, float humi, float water_temp, float ext_battery_v, float in_battery_v, float in_temperature)
{
    return SendToAmbient(temp, humi, water_temp, ext_battery_v, in_battery_v, in_temperature, INVALID_GPS_VALUE, INVALID_GPS_VALUE, INVALID_GPS_VALUE);
}

bool SendToAmbient(float temp, float humi, float water_temp, float ext_battery_v, float in_battery_v, float in_temperature, double lat, double lng, double meter)
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

	if (!ambient.set(3, water_temp))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(3, %f)", water_temp);
		SerialUSB.println(logBuf);
		return false;
	}

	//float ext_battery_v, float in_battery_v, float in_temperature
	if (!ambient.set(4, ext_battery_v))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(4, %f)", ext_battery_v);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(5, in_battery_v))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(5, %f)", in_battery_v);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(6, in_temperature))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(6, %f)", in_temperature);
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
#endif //C_SW_AMBIENT

#if C_SW_BATTERY_V
void SetupADC()
{
	ADC_COMMON->CCR |= (3UL << 22);
	GPIOA_BASE->MODER &= ~(3UL << 10);
	GPIOA_BASE->MODER |= (3UL << 10);
}

float GetExternalBatteryV()
{
	const uint16 v = analogRead(EXTERNAL_BATTERY_ADC_PIN);
	//SerialUSB.println(v);
	return v * 3300 / 2048;  //v is Vbat/2.  ADC resolution is 12bit(4096)and max indicates 3300mv. So Vbat = v / 4096 * 3300 * 2 
}

float GetInternalBatteryV()
{
	const uint16 v = adc_read(ADC1, INTERNAL_BATTERY_ADC_CHANNEL);
	//SerialUSB.println(v);
	return v * 3300 / 2048; //v is Vbat/2.  ADC resolution is 12bit(4096)and max indicates 3300mv. So Vbat = v / 4096 * 3300 * 2
}

float GetInternalTemperature()
{
	const uint16 t = adc_read(ADC1, INTERNAL_TEMPERATURE_ADC_CHANNEL);
	//SerialUSB.println(t);
	return (t * 3300 / 4096 - 760) / 2.5 + 25; //Refer to STM32F405RG datasheet.
}
#endif //C_SW_BATTERY_V

//RTC functions
void UpdateRtcByNtp()
{
	struct tm current_time = { 0 };
	if (GetNtpTime(Wio, current_time) == true)
	{
		time_t epoch = mktime(&current_time);
		epoch += JAPAN_TIME_DIFF;

		gmtime_r(&epoch, &current_time);
		SerialUSB.println("INFO: Get Time From NTP Server. UTC = ");
		SerialUSB.println(asctime(&current_time));

		rtc.setTime(&current_time);
		delay(10);  // RTCへの反映待ち
	}
}

bool GetNtpTime(WioLTE& wio, tm& current_time)
{
	if (!wio.SyncTime(NTP_SERVER))
	{
		SerialUSB.println("ERROR: SyncTime() error");
		return false;
	}

	return wio.GetTime(&current_time);
}

//StanbyMode functions
void EnterStandbyMode(time_t wakeup_time)
{
    //Set AlarmA
    rtc.turnOffAlarmA();
    rtc_enter_config_mode();
    RTC_BASE->ISR &= ~(1 << RTC_ISR_ALRAF_BIT);
    rtc_exit_config_mode();

    *bb_perip(&EXTI_BASE->PR, EXTI_RTC_ALARM_BIT) = 1;
    {
        struct tm tm_waketime = {0};
        gmtime_r(&wakeup_time, &tm_waketime);
        SerialUSB.println("INFO: Next wakeup time : ");
        SerialUSB.println(asctime(&tm_waketime));
        delay(10);
    }

    PWR_BASE->CR |= (1 << PWR_CR_PDDS);
    PWR_BASE->CR |= (1 << PWR_CR_CWUF);
    while(PWR_BASE->CSR & (1 << PWR_CSR_WUF))
    {
        SerialUSB.println("DEBUG: Wait PWR_CSR_WUF clear");
    } // WUPがクリアされるまで待機

    rtc.setAlarmATime(wakeup_time, false, false);

    SerialUSB.println("DEBUG: Enter Standby Mode");
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
    delay(10);

    __WFI();
}

void SleepUntilNextLoop(time_t sleeptime_sec)
{
	struct tm current_time = { 0 };
    rtc.getTime(&current_time);
    SerialUSB.println("INFO: Current RTC time = ");
    SerialUSB.println(asctime(&current_time));

	time_t epoch = mktime(&current_time);
    epoch += sleeptime_sec;

    EnterStandbyMode(epoch);
}
