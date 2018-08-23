#include <stdlib.h>
#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include <TinyGPS++.h>
#include <RTClock.h>
#include <time.h>

typedef int IRQn_Type;
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    1
#include <libmaple/libmaple_types.h>
#include <libmaple/usbF4/VCP/core_cm4.h>
#include <libmaple/usbF4/VCP/core_cmInstr.h>
#include <libmaple/pwr.h>
#include "Ambient.h"

//Common global
#define SENSOR_PIN    (WIOLTE_D38)
#define LOOP_PERIOD_MSEC 40000 // milliseconds

WioLTE Wio;
WioLTEClient WioClient(&Wio);

constexpr int LOG_TEMP_BUF_SIZE = 32;

RTClock rtc;
const time_t JAPAN_TIME_DIFF = 9 * 60 * 60; // UTC + 9h


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

//NTP
const char NTP_SERVER[] = "ntp.nict.jp";

void setup()
{
    SerialUSB.println("setup()");

    delay(200);

    //Setup LTE
    SerialUSB.println("Setup LTE");
    if(!SetupLTE()){
        return;
    }

    // //Setup Ambient
    // SerialUSB.println("Setup Ambient");
    // ambient.begin(AMBIENT_CHANNEL_ID, AMBIENT_WRITE_KEY, &WioClient);

    // //Setup DHT11
    // SerialUSB.println("Setup DHT11");
    // TemperatureAndHumidityBegin(SENSOR_PIN);

    // //Setup GPS
    // GpsBegin(&Serial);
    // Wio.PowerSupplyGrove(true);
    // delay(500);
}

void loop()
{
    unsigned long stime = millis();
    char cbuf[32] = {0};
    snprintf(cbuf, sizeof(cbuf), "loop() : %lu", stime);
    SerialUSB.println(cbuf);

    // /* Get temperature and humidity */
    // float temp;
    // float humi;

    // SerialUSB.println("TemperatureAndHumidityRead()");
    // if(TemperatureAndHumidityRead(&temp, &humi))
    // {
    //     //Send to serial
    //     SerialUSB.print("Current humidity = ");
    //     SerialUSB.print(humi);
    //     SerialUSB.print("%  ");
    //     SerialUSB.print("temperature = ");
    //     SerialUSB.print(temp);
    //     SerialUSB.println("C");
    // }
    // else
    // {
    //     SerialUSB.println("ERROR: TemperatureAndHumidityRead");
    // }


    // /* Get GPS */
    // SerialUSB.println("GpsRead()");
    // double lat, lng, meter;
    // bool validGps = GpsRead(lat, lng, meter);
    // if(validGps)
    // {
    //     SerialUSB.println("GPS Value:");
    //     snprintf(cbuf, sizeof(cbuf), "lat: %12.8f", lat);
    //     SerialUSB.println(cbuf);

    //     snprintf(cbuf, sizeof(cbuf), "lng: %12.8f", lng);
    //     SerialUSB.println(cbuf);

    //     snprintf(cbuf, sizeof(cbuf), "meter: %4.2f", meter);
    //     SerialUSB.println(cbuf);
    // }

    // /* Send to Ambient */
    // SerialUSB.println("SendToAmbient()");
    // bool isSendSuccess;
    // if(validGps)
    // {
    //     isSendSuccess = SendToAmbient(temp, humi, lat, lng, meter);
    // }
    // else
    // {
    //     isSendSuccess = SendToAmbient(temp, humi);
    // }

    // if(!isSendSuccess)
    // {
    //     SerialUSB.println("ERROR: SendToAmbient");
    // }

    /* Wait next loop */
    unsigned long elapse = millis() - stime;
    snprintf(cbuf, sizeof(cbuf), "Loop elapse: %ld msec", elapse);
    SerialUSB.println(cbuf);
    if(LOOP_PERIOD_MSEC > elapse)
    {
        time_t waittime_sec = (LOOP_PERIOD_MSEC - elapse) / 1000;
        snprintf(cbuf, sizeof(cbuf), "Wait next loop: %ld sec", waittime_sec);
        SerialUSB.println(cbuf);
        SleepUntilNextLoop(waittime_sec);
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

void ShutdownLTE()
{
    Wio.Deactivate();  // Deactivate a PDP context. Added at v1.1.9
    Wio.TurnOff(); // Shutdown the LTE module. Added at v1.1.6
    Wio.PowerSupplyLTE(false); // Turn the power supply to LTE module off
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
        delay(100);
    }

    PWR_BASE->CR |= (1 << PWR_CR_PDDS);
    PWR_BASE->CR |= (1 << PWR_CR_CWUF);
    while(PWR_BASE->CSR & (1 << PWR_CSR_WUF))
    {
        SerialUSB.println("DEBUG: Wait PWR_CSR_WUF clear");
    } // WUPがクリアされるまで待機

    rtc.setAlarmATime(wakeup_time, false, false);
    {
        struct tm current_time = {0};
        rtc.getTime(&current_time);
        SerialUSB.println("DEBUG: Current RTC time = ");
        SerialUSB.println(asctime(&current_time));
    }

    SerialUSB.println("DEBUG: Enter Standby Mode");
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
    delay(10);

    __WFI();
}

bool GetNtpTime(WioLTE& wio, tm& current_time)
{
  if(!wio.SyncTime(NTP_SERVER))
  {
    SerialUSB.println("ERROR: SyncTime() error");
    return false;
  }

  return wio.GetTime(&current_time);
}

void SleepUntilNextLoop(time_t sleeptime_sec)
{
    //Setup RTC and RCC
    ////Change RTC clock to LSI
    ///
    RCC_BASE->APB1ENR |= RCC_APB1RSTR_PWRRST;
    
    bkp_init();
 
    RCC_BASE->BDCR |= RCC_BDCR_BDRST;
    delay(1);
    bkp_enable_writes();
    RCC_BASE->CFGR |= (0x08 << 16); // Set the RTCPRE to HSE / 8.
    RCC_BASE->BDCR = RCC_BDCR_RTCSEL_LSI;
    // RCC_BASE->BDCR = RCC_BDCR_RTCSEL_LSE;
    RCC_BASE->BDCR |= BIT(RCC_BDCR_RTCEN_BIT);
    bkp_disable_writes();
    rcc_start_lsi();
    rtc_enter_config_mode();
    RTC_BASE->PRER = 249 | (127 << 16);
    rtc_exit_config_mode();

    delay(10);
    {
        SerialUSB.println("DEBUG: RCC_CR HSI bit = ");
        SerialUSB.println(bb_peri_get_bit(&RCC_BASE->CR, RCC_CR_HSIRDY_BIT));
        SerialUSB.println(bb_peri_get_bit(&RCC_BASE->CR, RCC_CR_HSION_BIT));
 
        SerialUSB.println("DEBUG: RCC_CSR LSI bit = ");
        SerialUSB.println(bb_peri_get_bit(&RCC_BASE->CSR, RCC_CSR_LSIRDY_BIT));
        SerialUSB.println(bb_peri_get_bit(&RCC_BASE->CSR, RCC_CSR_LSION_BIT));
 
        SerialUSB.println("DEBUG: RCC_BDCR LSE bit = ");
        SerialUSB.println(bb_peri_get_bit(&RCC_BASE->BDCR, RCC_BDCR_LSERDY_BIT));
        SerialUSB.println(bb_peri_get_bit(&RCC_BASE->BDCR, RCC_BDCR_LSEON_BIT));

        SerialUSB.println("DEBUG: RCC_BDCR bit = ");
        char logBuf[LOG_TEMP_BUF_SIZE] = {0};
        snprintf(logBuf, sizeof(logBuf), "%#08X", RCC_BASE->BDCR);
        SerialUSB.println(logBuf);

        SerialUSB.println("DEBUG: RCC_APB1ENR bit = ");
        snprintf(logBuf, sizeof(logBuf), "%#08X", RCC_BASE->APB1ENR);
        SerialUSB.println(logBuf);
    }
    delay(1);

    struct tm ntc_time = {0};
    struct tm current_time = {0};
    time_t epoch = 0;
    if (GetNtpTime(Wio, ntc_time) == true)
    {
        epoch = mktime(&ntc_time);
        epoch += JAPAN_TIME_DIFF;

        gmtime_r(&epoch, &ntc_time);
        SerialUSB.println("INFO: Get Time From NTP Server. UTC = ");
        SerialUSB.println(asctime(&ntc_time));

        rtc.setTime(&ntc_time);
        delay(10);  // RTCへの反映待ち
    }

    rtc.getTime(&current_time);
    SerialUSB.println("INFO: Current RTC time = ");
    SerialUSB.println(asctime(&current_time));

    epoch = mktime(&current_time);
    epoch += sleeptime_sec;

    ShutdownLTE();

    EnterStandbyMode(epoch);
}
