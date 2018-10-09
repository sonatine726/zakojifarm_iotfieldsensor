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
#include <HardwareTimer.h>
#include "Ms5540cSPI.h"

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
#define C_SW_AMBIENT 1
#define C_SW_DHT11 0
#define C_SW_DS18B20 0
#define C_SW_GPS 1
#define C_SW_BATTERY_V 1
#define C_SW_MS5540C 1
#define C_SW_SLEEP_WAIT 1
#define C_SW_EXT_RTC 1
#define C_SW_MS5540C_ADC_CLOCK_BY_EXT_RTC 0
#define C_SW_SLEEP_BY_POWERDOWN 1


//Common global
#define SENSOR_PIN    (WIOLTE_D38)
#define LOOP_PERIOD_MSEC 60000 // milliseconds
#define SLEEP_MSEC_WHEN_OVER_LOOP_PERIOD 10000 // milliseconds

WioLTE Wio;
WioLTEClient WioClient(&Wio);

constexpr int LOG_TEMP_BUF_SIZE = 64;

RTClock rtc(RTCSEL_LSI);
constexpr time_t JAPAN_TIME_DIFF = 9 * 60 * 60; // UTC + 9h

// constexpr uint32* P_RTC_BKP0R = reinterpret_cast<uint32*>(RTC_BASE) + 0x50;

constexpr float INVALID_TEMP_AND_WATERLEVEL = -127;

constexpr uint8 I2C1_SCL_PIN = Port2Pin('B', 8);
constexpr uint8 I2C1_SDA_PIN = Port2Pin('B', 9);

bool isSuccessSetup;

#if C_SW_LTE
//LTE global
#define APN       "soracom.io"
#define USERNAME  "sora"
#define PASSWORD  "sora"
#endif //C_SW_LTE

#if C_SW_AMBIENT
//Ambient global
//unsigned int AMBIENT_CHANNEL_ID = 5701;
//const char AMBIENT_WRITE_KEY[] = "13a769ms5540c_c5371e81ce";
unsigned int AMBIENT_CHANNEL_ID = 6359;
const char AMBIENT_WRITE_KEY[] = "12ad25a29e628a86";
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
TwoWire i2c;

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

constexpr uint8 ADC_CLOCK_PIN_FOR_MS5540C = Port2Pin('B', 4); // D20;
constexpr uint8 OE_PIN_FOR_MS5540C_AIR = Port2Pin('A', 4); // D4;
constexpr uint8 OE_PIN_FOR_MS5540C_WATER = Port2Pin('C', 7); // D39;

Ms5540cSPI ms5540cSpi(1);

enum MS5540C_SENSOR_ID
{
	MS5540C_AIR,
	MS5540C_WATER,
	MS5540C_SENSOR_NUMBER
};

uint32 ms5540c_c1[MS5540C_SENSOR_NUMBER];
uint32 ms5540c_c2[MS5540C_SENSOR_NUMBER];
uint32 ms5540c_c3[MS5540C_SENSOR_NUMBER];
uint32 ms5540c_c4[MS5540C_SENSOR_NUMBER];
uint32 ms5540c_c5[MS5540C_SENSOR_NUMBER];
uint32 ms5540c_c6[MS5540C_SENSOR_NUMBER];
#endif //C_SW_MS5540C


#if C_SW_BATTERY_V
constexpr unsigned int EXTERNAL_BATTERY_ADC_PIN = Port2Pin('A', 5);
constexpr unsigned int INTERNAL_BATTERY_ADC_CHANNEL = 18;
constexpr unsigned int INTERNAL_TEMPERATURE_ADC_CHANNEL = 16;
#endif //C_SW_BATTERY_V

void setup()
{
	isSuccessSetup = false;

	Wio.Init();

	SerialUSB.println("INFO: setup()");

	Wio.PowerSupplyGrove(true);

	i2c.begin();

#if C_SW_EXT_RTC
	if (!InitializeExtRtc())
	{
		SerialUSB.println("ERROR: InitializeExtRtc");
		//return;
	}
#endif //C_SW_EXT_RTC

#if C_SW_LTE
	//Setup LTE
	SerialUSB.println("INFO: Setup LTE");
	if (!SetupLTE())
	{
		SerialUSB.println("ERROR: SetupLTE");
		return;
	}

	UpdateRtcByNtp();

#if C_SW_EXT_RTC
	struct tm current_time = { 0 };
	rtc.getTime(&current_time);

	if (!SetExtRtcTime(current_time))
	{
		SerialUSB.println("ERROR: SetExtRtcTime");
		return;
	}
#endif //C_SW_EXT_RTC

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
	if (!SetupMs5540c())
	{
		SerialUSB.println("ERROR: SetupMs5540c");
		return;
	}
#endif //C_SW_MS5540C

#if C_SW_BATTERY_V
	SetupADC();
#endif //C_SW_BATTERY_V

#if C_SW_GPS
	//Setup GPS
	GpsBegin(&Serial);
	delay(500);
#endif

	isSuccessSetup = true;
}

void loop()
{
	char cbuf[64] = { 0 };
	struct tm current_time = { 0 };
	rtc.getTime(&current_time);
	snprintf(cbuf, sizeof(cbuf), "loop() : %s", asctime(&current_time));
	SerialUSB.println(cbuf);

	if (isSuccessSetup)
	{
		/* Get temperature and humidity */
		float temp = INVALID_TEMP_AND_WATERLEVEL;
		float humi = INVALID_TEMP_AND_WATERLEVEL;

#if C_SW_DHT11
		SerialUSB.println("INFO: TemperatureAndHumidityRead()");
		if (TemperatureAndHumidityRead(&temp, &humi))
		{
			snprintf(cbuf, sizeof(cbuf), "INFO: humid = %f%%, temp = %f C", humi, temp);
			SerialUSB.println(cbuf);
		}
		else
		{
			SerialUSB.println("ERROR: TemperatureAndHumidityRead");
		}
#endif //C_SW_DHT11

		float tempDs18b20 = INVALID_TEMP_AND_WATERLEVEL;
#if C_SW_DS18B20
		/* Get water temperature from DS18B20 */
		SerialUSB.println("INFO: Get DS18B20 Temperature");
		tempDs18b20 = GetTemperatureDS18B20();
		snprintf(cbuf, sizeof(cbuf), "INFO: DS18B20 temp: %f", tempDs18b20);
		SerialUSB.println(cbuf);
#endif //C_SW_DS18B20

		float press_ms5540c[MS5540C_SENSOR_NUMBER] = { 0 };
		float temp_ms5540c[MS5540C_SENSOR_NUMBER] = { INVALID_TEMP_AND_WATERLEVEL };
		float water_level_cm = INVALID_TEMP_AND_WATERLEVEL;
#if C_SW_MS5540C
		for (int i = 0; i < MS5540C_SENSOR_NUMBER; ++i)
		{
			GetPressureAndTemperatureFromMs5540c(static_cast<MS5540C_SENSOR_ID>(i), press_ms5540c[i], temp_ms5540c[i]);
			snprintf(cbuf, sizeof(cbuf), "INFO: MS5540C[%d] press = %f mbar, temp = %f C", i, press_ms5540c[i], temp_ms5540c[i]);
			SerialUSB.println(cbuf);
		}

		water_level_cm = CalculateWaterLevelCm(press_ms5540c[MS5540C_WATER], press_ms5540c[MS5540C_AIR]);
		snprintf(cbuf, sizeof(cbuf), "INFO: Water level = %f cm", water_level_cm);
		SerialUSB.println(cbuf);
#endif //C_SW_MS5540C

		bool validGps = false;
		double lat, lng, meter;
#if C_SW_GPS
		/* Get GPS */
		SerialUSB.println("INFO: GpsRead()");
		validGps = GpsRead(lat, lng, meter);
		if (validGps)
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
#endif //C_SW_BATTERY_V

#if C_SW_LTE && C_SW_AMBIENT
		/* Send to Ambient */
		SerialUSB.println("INFO: SendToAmbient()");
		bool isSendSuccess;
		if (validGps)
		{
			isSendSuccess = SendToAmbient(press_ms5540c[MS5540C_WATER], temp_ms5540c[MS5540C_WATER], press_ms5540c[MS5540C_AIR], temp_ms5540c[MS5540C_AIR], water_level_cm, ext_battery_v, in_battery_v, in_temperature, lat, lng, meter);
		}
		else
		{
			isSendSuccess = SendToAmbient(press_ms5540c[MS5540C_WATER], temp_ms5540c[MS5540C_WATER], press_ms5540c[MS5540C_AIR], temp_ms5540c[MS5540C_AIR], water_level_cm, ext_battery_v, in_battery_v, in_temperature);
		}

		if (!isSendSuccess)
		{
			SerialUSB.println("ERROR: SendToAmbient");
		}
#endif //C_SW_LTE && C_SW_AMBIENT
	}
	else
	{
		SerialUSB.println("ERROR: Skip loop because of setup failure");
	}

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
	byte data[5] = { 0 };

	DHT11Start(TemperatureAndHumidityPin);
	for (int i = 0; i < 5; i++)
	{
		data[i] = DHT11ReadByte(TemperatureAndHumidityPin);
	}
	DHT11Finish(TemperatureAndHumidityPin);

	if (!DHT11Check(data))
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
	while (!digitalRead(pin));

	// Response signal
	while (digitalRead(pin));

	// Pulled ready to output
	while (!digitalRead(pin));
}

byte DHT11ReadByte(int pin)
{
	byte data = 0;

	for (int i = 0; i < 8; i++)
	{
		while (digitalRead(pin));

		while (!digitalRead(pin));
		unsigned long start = micros();

		while (digitalRead(pin));
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
	while (!digitalRead(pin));
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


#if C_SW_EXT_RTC
//External RTC functions

bool InitializeExtRtc()
{
	i2c.beginTransmission(RTC_I2C_ADDR);
	i2c.write(RTC_REG_CTR2);
	i2c.write(0x00); //Clear all interupts
	uint8 result = i2c.endTransmission();
	if (result != SUCCESS)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.endTransmission() ret %d", result);
		SerialUSB.println(cbuf);
		return false;
	}

	return true;
}

bool SetExtRtcTime(const tm& current_time)
{
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "INFO: SetExtRtcTime : %s", asctime(&current_time));
		SerialUSB.println(cbuf);
	}

	i2c.beginTransmission(RTC_I2C_ADDR);
	i2c.write(RTC_REG_SEC);

	const uint8 sec_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_sec);
	i2c.write(sec_to_reg); //Write to RTC_REG_SEC

	const uint8 min_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_min);
	i2c.write(min_to_reg); //Write to RTC_REG_MIN

	const uint8 hour_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_hour);
	i2c.write(hour_to_reg); //Write to RTC_REG_HOUR

	const uint8 day_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_mday);
	i2c.write(day_to_reg); //Write to RTC_REG_DAYS

	const uint8 weekday_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_wday);
	i2c.write(weekday_to_reg); //Write to RTC_REG_WEEK

	const uint8 month_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_mon);
	i2c.write(month_to_reg); //Write to RTC_REG_MONTH

	const uint8 year_to_reg = TranslateTimeToRegValueForExtRtc(current_time.tm_year);
	i2c.write(year_to_reg); //Write to RTC_REG_YEAR

	i2c.write(0x00); //Write to RTC_REG_MIN_ALR
	i2c.write(0x00); //Write to RTC_REG_HOUR_ALR
	i2c.write(0x00); //Write to RTC_REG_DAY_ALR
	i2c.write(0x00); //Write to RTC_REG_WEEK_ALR

	i2c.write(0x00); //Write to RTC_REG_CLKO_FREQ

	i2c.write(0x00); //Write to RTC_REG_TIMER_CTR
	i2c.write(0x00); //Write to RTC_REG_TIMER

	uint8 result = i2c.endTransmission();
	if (result != SUCCESS)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.endTransmission() ret %d", result);
		SerialUSB.println(cbuf);
		return false;
	}

	return true;
}

bool GetExtRtcTime(tm& current_time)
{
	i2c.beginTransmission(RTC_I2C_ADDR);
	i2c.write(RTC_REG_SEC);
	uint8 result = i2c.endTransmission();
	if (result != SUCCESS)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.endTransmission() to write addr ret %d", result);
		SerialUSB.println(cbuf);
		return false;
	}

	result = i2c.requestFrom(RTC_I2C_ADDR, 7);
	if (result != 7)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.requestFrom(7) ret %d", result);
		SerialUSB.println(cbuf);
		return false;
	}
	   
	const uint8 reg_sec = i2c.read();
	const uint8 reg_min = i2c.read();
	const uint8 reg_hour = i2c.read();
	const uint8 reg_day = i2c.read();
	const uint8 reg_weekday = i2c.read();
	const uint8 reg_month = i2c.read();
	const uint8 reg_year = i2c.read();

	current_time.tm_sec = TranslateRegValueToTimeForExtRtc(reg_sec);
	current_time.tm_min = TranslateRegValueToTimeForExtRtc(reg_min);
	current_time.tm_hour = TranslateRegValueToTimeForExtRtc(reg_hour);
	current_time.tm_mday = TranslateRegValueToTimeForExtRtc(reg_day);
	current_time.tm_wday = TranslateRegValueToTimeForExtRtc(reg_weekday);
	current_time.tm_mon = TranslateRegValueToTimeForExtRtc(reg_month);
	current_time.tm_year = TranslateRegValueToTimeForExtRtc(reg_year);
}

uint8 TranslateTimeToRegValueForExtRtc(uint32 time)
{
	return static_cast<uint8>(time / 10) << 4 | static_cast<uint8>(time % 10);
}

uint32 TranslateRegValueToTimeForExtRtc(uint8 regv)
{
	return 10 * (regv >> 4) + (regv & 0x0F);
}

bool SetWakeupAlarmToExtRTC(time_t wakeup_time)
{
	struct tm tm_waketime = { 0 };
	gmtime_r(&wakeup_time, &tm_waketime);
	
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "INFO: SetWakeupAlarmToExtRTC : %s", asctime(&tm_waketime));
		SerialUSB.println(cbuf);
	}

	//Set alarm time
	i2c.beginTransmission(RTC_I2C_ADDR);
	i2c.write(RTC_REG_MIN_ALR);
	const uint8 min_to_reg = TranslateTimeToRegValueForExtRtc(tm_waketime.tm_min);
	i2c.write(min_to_reg); //Write to RTC_REG_MIN
	i2c.write(0x80);
	i2c.write(0x80);
	i2c.write(0x80);
	uint8 result = i2c.endTransmission();
	if (result != SUCCESS)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.endTransmission() for set time ret %d", result);
		SerialUSB.println(cbuf);
		return false;
	}

	//Enable Alarm
	i2c.beginTransmission(RTC_I2C_ADDR);
	i2c.write(RTC_REG_CTR2);
	i2c.write(0x02);
	result = i2c.endTransmission();
	if (result != SUCCESS)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.endTransmission() for enable alarm ret %d", result);
		SerialUSB.println(cbuf);
		return false;
	}

	return true;
}
#endif //C_SW_EXT_RTC

#if C_SW_MS5540C

bool SetupMs5540c()
{
	//Set AF of pins for MS5540C
	////SCLK from B3
	gpio_set_af_mode(GPIOA, 5, 0);
	gpio_set_af_mode(GPIOB, 3, 5);
	////MCLK by pwm from B4
	gpio_set_af_mode(GPIOB, 4, 2);
	////OE signal from A4, C7
	gpio_set_af_mode(GPIOA, 4, 0);
	pinMode(OE_PIN_FOR_MS5540C_AIR, OUTPUT);
	gpio_set_af_mode(GPIOC, 7, 0);
	pinMode(OE_PIN_FOR_MS5540C_WATER, OUTPUT);

	ms5540cSpi.begin(SPI_281_250KHZ, MSBFIRST, SPI_MODE0);

	if (!MakeAdcClockToMs5540c())
	{
		SerialUSB.println("ERROR: MakeAdcClockToMs5540c");
		return false;
	}

	GetMs5540cCalibrationValues(MS5540C_AIR);
	GetMs5540cCalibrationValues(MS5540C_WATER);

	return true;
}

void EnableMs5540c(MS5540C_SENSOR_ID sensor_id)
{
	if (sensor_id == MS5540C_AIR)
	{
		digitalWrite(OE_PIN_FOR_MS5540C_AIR, 0);
		digitalWrite(OE_PIN_FOR_MS5540C_WATER, 1);
	}
	else if (sensor_id == MS5540C_WATER)
	{
		digitalWrite(OE_PIN_FOR_MS5540C_AIR, 1);
		digitalWrite(OE_PIN_FOR_MS5540C_WATER, 0);
	}
	else
	{
		digitalWrite(OE_PIN_FOR_MS5540C_AIR, 1);
		digitalWrite(OE_PIN_FOR_MS5540C_AIR, 1);
	}
}

void GetMs5540cCalibrationValues(MS5540C_SENSOR_ID sensor_id)
{
	EnableMs5540c(sensor_id);

	//Read calibration register
	const uint32 calib_reg1 = SendCommandAndGetWord(0x1D, 0x50, 0);
	const uint32 calib_reg2 = SendCommandAndGetWord(0x1D, 0x60, 0);
	const uint32 calib_reg3 = SendCommandAndGetWord(0x1D, 0x90, 0);
	const uint32 calib_reg4 = SendCommandAndGetWord(0x1D, 0xA0, 0);

	//Get calibration value
	ms5540c_c1[sensor_id] = calib_reg1 >> 1;
	ms5540c_c2[sensor_id] = ((calib_reg3 & 0x3F) << 6) | (calib_reg4 & 0x3F);
	ms5540c_c3[sensor_id] = (calib_reg4 >> 6);
	ms5540c_c4[sensor_id] = (calib_reg3 >> 6);
	ms5540c_c5[sensor_id] = (calib_reg2 >> 6) | ((calib_reg1 & 0x1) << 10);
	ms5540c_c6[sensor_id] = calib_reg2 & 0x3F;

	char cbuf[64] = { 0 };
	snprintf(cbuf, sizeof(cbuf), "INFO: Ms5540c[%d] calibration values =", sensor_id);
	SerialUSB.println(cbuf);
	SerialUSB.println(ms5540c_c1[sensor_id]);
	SerialUSB.println(ms5540c_c2[sensor_id]);
	SerialUSB.println(ms5540c_c3[sensor_id]);
	SerialUSB.println(ms5540c_c4[sensor_id]);
	SerialUSB.println(ms5540c_c5[sensor_id]);
	SerialUSB.println(ms5540c_c6[sensor_id]);
}

float CalculateWaterLevelCm(float water_pressure_mbar, float air_pressure_mbar)
{
	const float diff_press = water_pressure_mbar - air_pressure_mbar;
	return diff_press * 0.9999999999999971325; // Right operand = (0.00098692326671601 * 1013.25). 1mbar = 0.00098692326671601 atm. 1atm = 1013.25 hPa. 1cm Water pressure is 1hPa. 
}

void GetPressureAndTemperatureFromMs5540c(MS5540C_SENSOR_ID sensor_id, float& pressure, float& temperature)
{
	EnableMs5540c(sensor_id);

	long dT, raw_temperature_decuple, temperature_compensation;//data holders for interim results of temperature calculation

	//Get temperature register value and calculate temperature
	const uint16 temp_reg = SendCommandAndGetWord(0x0F, 0x20, 35);
	temperature = CalculateTemperatureByMs5540c(sensor_id, temp_reg, dT, raw_temperature_decuple, temperature_compensation);


	//Get pressure register value and calculate pressure
	const uint16 press_reg = SendCommandAndGetWord(0x0F, 0x40, 35);
	pressure = CalculatePressureByMs5540c(sensor_id, press_reg, dT, raw_temperature_decuple, temperature_compensation);
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

float CalculateTemperatureByMs5540c(MS5540C_SENSOR_ID sensor_id, uint16 temp_reg, long& r_dT, long& r_raw_temperature_decuple, long& r_temperature_compensation)
{
	const long UT1 = (ms5540c_c5[sensor_id] * 8) + 20224;
	const long dT = (temp_reg - UT1);
	long TEMP = 200 + ((dT * (ms5540c_c6[sensor_id] + 50)) / 1024);
	const long TEMP_COMPENSATION = GetCompensateValueForTemperature(TEMP, ms5540c_c6[sensor_id]);

	r_dT = dT;
	r_raw_temperature_decuple = TEMP;
	r_temperature_compensation = TEMP_COMPENSATION;

	return static_cast<float>(TEMP - TEMP_COMPENSATION) / 10;
}

float CalculatePressureByMs5540c(MS5540C_SENSOR_ID sensor_id, uint16 press_reg, long dT, long TEMP, long temperature_compensation)
{
	const long OFF = (ms5540c_c2[sensor_id] * 4) + (((ms5540c_c4[sensor_id] - 512) * dT) / 4096);
	const long SENS = ms5540c_c1[sensor_id] + ((ms5540c_c3[sensor_id] * dT) / 1024) + 24576;
	const long X = ((SENS * (press_reg - 7168)) / 16384) - OFF;
	long P = X * 10 / 32 + 2500;

	P -= GetCompensateValueForPressure(P, TEMP, temperature_compensation);
	return static_cast<float>(P) / 10;
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

bool MakeAdcClockToMs5540c()
{
	bool isSuccess = true;

#if C_SW_EXT_RTC && C_SW_MS5540C_ADC_CLOCK_BY_EXT_RTC
	i2c.beginTransmission(RTC_I2C_ADDR);
	i2c.write(RTC_REG_CLKO_FREQ);
	i2c.write(0x80);
	uint8 result = i2c.endTransmission();
	if (result != SUCCESS)
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: i2c.endTransmission() ret %d", result);
		SerialUSB.println(cbuf);
		isSuccess = false;
	}
#else //C_SW_EXT_RTC && C_SW_MS5540C_ADC_CLOCK_BY_EXT_RTC
	isSuccess = MakeClockByPwm(ADC_CLOCK_PIN_FOR_MS5540C);
#endif //C_SW_EXT_RTC && C_SW_MS5540C_ADC_CLOCK_BY_EXT_RTC

	return isSuccess;
}

struct TimerDevToHardwareTimer
{
	timer_dev* tdev;
	HardwareTimer* timer;
};

//Currently, not extern Timer6 and Timer7
extern HardwareTimer Timer6;
extern HardwareTimer Timer7;

const TimerDevToHardwareTimer TIMERDEV_TO_HARDWARETIMER_MAP[] =
{
	{TIMER1, &Timer1},
	{TIMER2, &Timer2},
	{TIMER3, &Timer3},
	{TIMER4, &Timer4},
#ifdef STM32_HIGH_DENSITY
	{TIMER5, &Timer5},
	{TIMER6, &Timer6},
	{TIMER7, &Timer7},
	{TIMER8, &Timer8}
#endif //STM32_HIGH_DENSITY
};

bool MakeClockByPwm(uint8_t pin)
{
	timer_dev *tdev = PIN_MAP[pin].timer_device;     // ピン対応のタイマーデバイスの取得 
	const uint8 cc_channel = PIN_MAP[pin].timer_channel;  // ピン対応のタイマーチャンネルの取得

	HardwareTimer* timer = NULL;
	for (int i = 0; i < (sizeof(TIMERDEV_TO_HARDWARETIMER_MAP) / sizeof(TimerDevToHardwareTimer)); ++i)
	{
		if (TIMERDEV_TO_HARDWARETIMER_MAP[i].tdev == tdev)
		{
			timer = TIMERDEV_TO_HARDWARETIMER_MAP[i].timer;
			break;
		}
	}

	if (!(tdev && cc_channel && timer))
	{
		char cbuf[64];
		snprintf(cbuf, sizeof(cbuf), "ERROR: Target pin invalid for pwm. %d %08X %d %08X ", pin, tdev, cc_channel, timer);
		SerialUSB.println(cbuf);

		return false;
	}

	pinMode(pin, PWM);

	timer->pause();
	timer->setPrescaleFactor(640); //Clock prescale from 42MHz to 32.768 * 2 KHz
	timer->setOverflow(2);        // Cycle is 32.768 KHz
	timer->setMode(cc_channel, TIMER_PWM);
	timer->setCompare(cc_channel, 1); // Duty ratio is 50%
	timer->setCount(0);
	timer->refresh();
	timer->resume();

	return true;
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
	while (GpsSerial->available())
	{
		if (gps.encode(GpsSerial->read()))
		{
			break;
		}
	}

	if (!gps.location.isValid())
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
bool SendToAmbient(float water_press, float water_temp, float air_press, float air_temp, float water_level, float ext_battery_v, float in_battery_v, float in_temperature)
{
	return SendToAmbient(water_press, water_temp, air_press, air_temp, water_level, ext_battery_v, in_battery_v, in_temperature, INVALID_GPS_VALUE, INVALID_GPS_VALUE, INVALID_GPS_VALUE);
}

bool SendToAmbient(float water_press, float water_temp, float air_press, float air_temp, float water_level, float ext_battery_v, float in_battery_v, float in_temperature, double lat, double lng, double meter)
{
	if (!ambient.set(1, water_press))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(1, %f)", water_press);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(2, water_temp))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(2, %f)", water_temp);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(3, air_press))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(3, %f)", air_press);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(4, air_temp))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(4, %f)", air_temp);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(5, water_level))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(5, %f)", water_level);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(6, ext_battery_v))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(6, %f)", ext_battery_v);
		SerialUSB.println(logBuf);
		return false;
	}

	if (!ambient.set(7, in_temperature))
	{
		char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
		snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(7, %f)", in_temperature);
		SerialUSB.println(logBuf);
		return false;
	}

	//// Exclude in_battery_v because Ambient data is full. 
	//if (!ambient.set(7, in_battery_v))
	//{
	//	char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
	//	snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(7, %f)", in_battery_v);
	//	SerialUSB.println(logBuf);
	//	return false;
	//}

	char cbuf[16];
	if (meter != INVALID_GPS_VALUE)
	{
		snprintf(cbuf, sizeof(cbuf), "%4.2f", meter);
		if (!ambient.set(8, cbuf))
		{
			char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
			snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(8, %s)", cbuf);
			SerialUSB.println(logBuf);
			return false;
		}
	}

	if (lat != INVALID_GPS_VALUE)
	{
		snprintf(cbuf, sizeof(cbuf), "%12.8f", lat);
		if (!ambient.set(9, cbuf))
		{
			char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
			snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(9, %s)", cbuf);
			SerialUSB.println(logBuf);
			return false;
		}
	}

	if (lng != INVALID_GPS_VALUE)
	{
		snprintf(cbuf, sizeof(cbuf), "%12.8f", lng);
		if (!ambient.set(10, cbuf))
		{
			char logBuf[LOG_TEMP_BUF_SIZE] = { 0 };
			snprintf(logBuf, sizeof(logBuf), "ERROR: ambient.set(10, %s)", cbuf);
			SerialUSB.println(logBuf);
			return false;
		}
	}

	if (!ambient.send())
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
		delay(10);  // Wait to reflect to RTC
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
		struct tm tm_waketime = { 0 };
		gmtime_r(&wakeup_time, &tm_waketime);
		SerialUSB.println("INFO: Next wakeup time : ");
		SerialUSB.println(asctime(&tm_waketime));
		delay(10);
	}

	PWR_BASE->CR |= (1 << PWR_CR_PDDS);
	PWR_BASE->CR |= (1 << PWR_CR_CWUF);
	while (PWR_BASE->CSR & (1 << PWR_CSR_WUF))
	{
		SerialUSB.println("DEBUG: Wait PWR_CSR_WUF clear");
	}

	rtc.setAlarmATime(wakeup_time, false, false);

	//Enter standby mode=
	SerialUSB.println("DEBUG: Enter Standby Mode");
	SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
	delay(10);

	__WFI();
}

void SleepUntilNextLoop(time_t sleeptime_sec)
{
	struct tm current_time = { 0 };
	rtc.getTime(&current_time);
	SerialUSB.println("INFO: Current Internal RTC time = ");
	SerialUSB.println(asctime(&current_time));

	struct tm current_time_extrtc = { 0 };
	GetExtRtcTime(current_time_extrtc);
	SerialUSB.println("INFO: Current External RTC time = ");
	SerialUSB.println(asctime(&current_time_extrtc));

	time_t epoch = mktime(&current_time);
	epoch += sleeptime_sec;

#if C_SW_EXT_RTC && C_SW_SLEEP_BY_POWERDOWN
	//RTC Test
	epoch += 60;
	//END of RTC Test

	if (!SetWakeupAlarmToExtRTC(epoch))
	{
		SerialUSB.println("ERROR: SetWakeupAlarmToExtRTC");
	}
	Wio.PowerSupplyGrove(false);//

	//RTC Test
	epoch += 60;
	//END of RTC Test
#endif //C_SW_SLEEP_BY_POWERDOWN
	EnterStandbyMode(epoch);
}
