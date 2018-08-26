#include <stdlib.h>
#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include <TinyGPS++.h>
#include <RTClock.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>

typedef int IRQn_Type;
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    1
#include <libmaple/libmaple_types.h>
#include <libmaple/usbF4/VCP/core_cm4.h>
#include <libmaple/usbF4/VCP/core_cmInstr.h>
#include <libmaple/pwr.h>
#include "Ambient.h"

// Compile Switch
#define C_SW_LTE 0
#define C_SW_AMBIENT 0
#define C_SW_DHT11 0
#define C_SW_DS18B20 0
#define C_SW_GPS 0
#define C_SW_BATTERY_V 1



//Common global
#define SENSOR_PIN    (WIOLTE_D38)
#define LOOP_PERIOD_MSEC 60000 // milliseconds

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

#if C_SW_BATTERY_V
constexpr unsigned int EXTERNAL_BATTERY_ADC_PIN = 5;
constexpr unsigned int INTERNAL_BATTERY_ADC_PIN = 16;
constexpr unsigned int INTERNAL_TEMPERATURE_ADC_PIN = 17;
#endif //C_SW_BATTERY_V

void setup()
{
	Wio.Init();

    SerialUSB.println("INFO: setup()");

	if (!Wio.TurnOnOrReset())
	{
		SerialUSB.println("ERROR: Wio.TurnOnOrReset");
	}

#if C_SW_LTE
    //Setup LTE
    SerialUSB.println("INFO: Setup LTE");
    if(!SetupLTE()){
        return;
    }

#if C_SW_AMBIENT
    //Setup Ambient
    SerialUSB.println("INFO: Setup Ambient");
    ambient.begin(AMBIENT_CHANNEL_ID, AMBIENT_WRITE_KEY, &WioClient);
#endif //C_SW_AMBIENT
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

#if C_SW_BATTERY_V
	ADC_COMMON->CCR |= (3UL << 22);
	GPIOA_BASE->MODER &= ~(3UL << 10);
	GPIOA_BASE->MODER |= (3UL << 10);
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
    SerialUSB.flush();

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

	{
		SerialUSB.println("DEBUG: GPIOA_PUPDR = ");
		char logBuf[LOG_TEMP_BUF_SIZE] = {0};
		snprintf(logBuf, sizeof(logBuf), "%#08X", GPIOA_BASE->PUPDR);
		SerialUSB.println(logBuf);
	}
#endif //C_SW_DS18B20

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

	{
		snprintf(cbuf, sizeof(cbuf), "DEBUG: ADC_COMMON->CCR %08X", ADC_COMMON->CCR);
		SerialUSB.println(cbuf);
	}

	{
		snprintf(cbuf, sizeof(cbuf), "DEBUG: ADC1_BASE->CR1 %08X", ADC1_BASE->CR1);
		SerialUSB.println(cbuf);
	}

	{
		snprintf(cbuf, sizeof(cbuf), "GPIOA_BASE->MODER %08X", GPIOA_BASE->MODER);
		SerialUSB.println(cbuf);
	}

	{
		snprintf(cbuf, sizeof(cbuf), "GPIOA_BASE->AFRL %08X", GPIOA_BASE->AFR[0]);
		SerialUSB.println(cbuf);
	}

	{
		snprintf(cbuf, sizeof(cbuf), "GPIOA_BASE->AFRH %08X", GPIOA_BASE->AFR[1]);
		SerialUSB.println(cbuf);
	}
#endif //C_SW_BATTERY_V

#if C_SW_LTE && C_SW_AMBIENT
    /* Send to Ambient */
    SerialUSB.println("INFO: SendToAmbient()");
    bool isSendSuccess;
    if(validGps)
    {
        isSendSuccess = SendToAmbient(temp, humi, tempDs18b20, lat, lng, meter);
    }
    else
    {
        isSendSuccess = SendToAmbient(temp, humi, tempDs18b20);
    }

    if(!isSendSuccess)
    {
        SerialUSB.println("ERROR: SendToAmbient");
    }
#endif //C_SW_LTE && C_SW_AMBIENT

    /* Wait next loop */
#if C_SW_LTE
	UpdateRtcByNtp();
	ShutdownLTE();
#endif //C_SW_LTE

    unsigned long elapse = millis();
    snprintf(cbuf, sizeof(cbuf), "Run elapse: %ld msec", elapse);
    SerialUSB.println(cbuf);
    if(LOOP_PERIOD_MSEC > elapse)
    {
        time_t waittime_sec = (LOOP_PERIOD_MSEC - elapse) / 1000;
        snprintf(cbuf, sizeof(cbuf), "Wait next loop: %ld sec", waittime_sec);
        SerialUSB.println(cbuf);
        SleepUntilNextLoop(waittime_sec);
    }
}

#if C_SW_LTE
//LTE functions
bool SetupLTE()
{
    Wio.PowerSupplyLTE(true);
    delay(500);

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
bool SendToAmbient(float temp, float humi, float water_temp)
{
    return SendToAmbient(temp, humi, water_temp, INVALID_GPS_VALUE, INVALID_GPS_VALUE, INVALID_GPS_VALUE);
}

bool SendToAmbient(float temp, float humi, float water_temp, double lat, double lng, double meter)
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
float GetExternalBatteryV()
{
	uint16 v = analogRead(EXTERNAL_BATTERY_ADC_PIN);
	SerialUSB.println(v);
	return v * 3300 / 2048;
	//return static_cast<float>(analogRead(EXTERNAL_BATTERY_ADC_PIN)) * 3300 / 2048;
}

float GetInternalBatteryV()
{
	uint16 v = adc_read(ADC1, 18);
	//uint16 v = analogRead(INTERNAL_BATTERY_ADC_PIN);
	SerialUSB.println(v);
	return v * 3300 / 2048;
	//return static_cast<float>(analogRead(INTERNAL_BATTERY_ADC_PIN)) * 3300 / 2048;
}

float GetInternalTemperature()
{
	uint16 t = adc_read(ADC1, 16);
	SerialUSB.println(t);
	return (t * 3300 / 4096 - 760) / 2.5 + 25;
	//return ((static_cast<float>(analogRead(INTERNAL_TEMPERATURE_ADC_PIN)) * 3300 / 4096 - 0.76) / 2.5) + 25;
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
