/**
 ********************************************************************
 * @file    test_fc_subscription.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

#include <stdio.h>
#include <stdint.h>
// C library headers
#include <string.h>
#include <stdlib.h>
#include "simple_uart.h"
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <unistd.h>

// BME280
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "wiringPiI2C.h"
#include "data_structure.h"
#include "bme280.h"
#include "led.h"
#include "MAX31856.h"

// struct bme280_dev dev;
#define ThermocoupleBreakoutChipSelectGPIO 8


//Zmienne dla czujnika pylow
const double PM01_a = 1;                        // Wspolczynniki kalibracyne dla kazdej frakcji pylu
const double PM01_b = 0;
const double PM2_5_a = 1;
const double PM2_5_b = 0;
const double PM10_a = 1;
const double PM10_b = 0;
double PM01Value=0;          //define PM1.0 value of the air detector module
double PM2_5Value=0;         //define PM2.5 value of the air detector module
double PM10Value=0;         //define PM10 value of the air detector module
double fr03=0;
double fr05=0;
double fr10=0;
double fr25=0;
double fr50=0;
double fr100=0;
int receiveDatIndex=32;  
uint8_t buff[32];
int buflen=0;
int event_char = 0;
int i;

char buffer;
int fd;
struct simple_uart *uart;
int baudrate;
const char *port = NULL;
const char *flags = "8N1";
int opt;
char c;
int received = 0;
char data_collected[2000];
char fileName[200];
FILE *file_ptr;
FILE *log_ptr;
int t0;

// thermocouple:
struct MAX31856_S ThermocoupleBreakout;
double ValR;
int k;

// data structure:
struct payloadData dataStructure = {.date={0, 0, 0}, .coordinates = {0, 0, 0}, .GPSdataAge=0, .satelliteNum={0., 0., 0.}, .thermocouple={0., 0.}, .bme={0., 0., 0.}, .pm={0., 0., 0.}, .fr={0., 0., 0., 0., 0., 0.}, .batteryLevel=0};

// LEDs:
struct Status flag;

//=================================================================================================

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <math.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "widget_interaction_test/test_widget_interaction.h"

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (2048)

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void smartDelay();
char* printData(char*, struct payloadData);
static void PMS_Encode(char c);
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode Dji_FcSubscriptionReceiveRtkPositionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = true;
static uint8_t s_totalSatelliteNumberUsed = 0;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    // setting the status controlling LEDs and handliing system's status informations:
    flag = (struct Status) {.led_light={.led=green, .toggle=false}};

    // startup sequence of led's:
    startupSequence(&flag);
    
    // LOGs file opening:
    log_ptr = fopen("log.txt", "a");
    if(log_ptr == NULL)
    {
         flag.led_light.led = red;
         trigger(&flag);
         USER_LOG_DEBUG("data file pointer logging failure.");
    }

    osalHandler = DjiPlatform_GetOsalHandler();
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = red;
        trigger(&flag);
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               Dji_FcSubscriptionReceiveRtkPositionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = red;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic RTK error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic RTK success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = yellow;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic velocity success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = yellow;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = yellow;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps details success.");
    }

    //wywołanie funkcji UserFcSubscription_Task
    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                 FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_userFcSubscriptionThread) !=
         DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
         flag.led_light.led = red;
         trigger(&flag);
         USER_LOG_ERROR("user data subscription task create error.");
         return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};

    USER_LOG_INFO("Fc subscription sample start");

    USER_LOG_INFO("--> Step 1: Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    fd = wiringPiI2CSetup(BME280_ADDRESS);
    if(fd < 0) {
      flag.led_light.led = red;
      trigger(&flag);
      USER_LOG_ERROR("I2C failure");
      printf("Device not found");
      return -1;
    }
     
    uart = simple_uart_open("/dev/ttySOFT0", 9600, "8N1");
    if(uart == NULL){
        flag.led_light.led = red;
        trigger(&flag);
        USER_LOG_ERROR("simple (soft) uart cannot open the device.");
    }
    t0 = (int)time(NULL);

    USER_LOG_INFO("--> Step 2: Subscribe the topics of quaternion, velocity and gps position");

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               Dji_FcSubscriptionReceiveRtkPositionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = red;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic RTK error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic RTK success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = yellow;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = yellow;
        trigger(&flag);
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 3: Deinit fc subscription module");

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        flag.led_light.led = yellow;
        trigger(&flag);
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    fclose(file_ptr);

    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

 static void *UserFcSubscription_Task(void *arg)
 {
     T_DjiReturnCode djiStat;
     T_DjiFcSubscriptionVelocity velocity = {0};
     T_DjiDataTimestamp timestamp = {0};
     T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
     T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
     T_DjiOsalHandler *osalHandler = NULL;

    // THERMOCOUPLE CONFIGURATION:
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    sprintf(fileName, "data_%d-%02d-%02d_%02d:%02d:%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    gpioInitialise();
    ThermocoupleBreakout = SetupMAX31856(ThermocoupleBreakoutChipSelectGPIO, MAX31856_ThermocoupleType_J, MAX31856_Unit_Celcius);

    ThermocoupleBreakout.AveragingMode = MAX31856_AveragingMode_2;
    MAX31856_CombineSettingsAndSend(&ThermocoupleBreakout);
    // END OF CONFIGURATION

     USER_UTIL_UNUSED(arg);
     osalHandler = DjiPlatform_GetOsalHandler();

     fd = wiringPiI2CSetup(BME280_ADDRESS);
     if(fd < 0) {
       printf("Device not found");
     }
     
     uart = simple_uart_open("/dev/ttySOFT0", 9600, "8N1");
     t0 = (int)time(NULL);

     while (1) {
         osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

         djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                           (uint8_t *) &velocity,
                                                           sizeof(T_DjiFcSubscriptionVelocity),
                                                           &timestamp);
         if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
             USER_LOG_ERROR("get value of topic velocity error.");
         }

         if (s_userFcSubscriptionDataShow == true) {
             USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.", velocity.data.x, velocity.data.y,
                           velocity.data.z, velocity.health);
         }

         djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                           (uint8_t *) &gpsPosition,
                                                           sizeof(T_DjiFcSubscriptionGpsPosition),
                                                           &timestamp);
         if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
             USER_LOG_ERROR("get value of topic gps position error.");
         }

         if (s_userFcSubscriptionDataShow == true) {
             USER_LOG_INFO("gps position: x %d y %d z %d.", gpsPosition.x, gpsPosition.y, gpsPosition.z);
         }

         djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                           (uint8_t *) &gpsDetails,
                                                           sizeof(T_DjiFcSubscriptionGpsDetails),
                                                           &timestamp);
         if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
             USER_LOG_ERROR("get value of topic gps details error.");
         }

	 if(gpsDetails.totalSatelliteNumberUsed == 0){
            flag.led_light.led = yellow;
         }else{
            flag.led_light.led = green;
         }

         if (s_userFcSubscriptionDataShow == true) {
             USER_LOG_INFO("gps total satellite number used: %d %d %d.",
                           gpsDetails.gpsSatelliteNumberUsed,
                           gpsDetails.glonassSatelliteNumberUsed,
                           gpsDetails.totalSatelliteNumberUsed);
             s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;
         }

         dataStructure.satelliteNum.gpsSatelliteNumberUsed = gpsDetails.gpsSatelliteNumberUsed;
         dataStructure.satelliteNum.glonassSatelliteNumberUsed = gpsDetails.glonassSatelliteNumberUsed;
         dataStructure.satelliteNum.totalSatelliteNumberUsed =  gpsDetails.totalSatelliteNumberUsed;

        smartDelay();
     }
 }

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode Dji_FcSubscriptionReceiveRtkPositionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionRtkPosition *rtk_pos = (T_DjiFcSubscriptionRtkPosition*) data;

    USER_UTIL_UNUSED(dataSize);

    if (s_userFcSubscriptionDataShow == true) {
        USER_LOG_INFO("receive rtk position.");

        USER_LOG_INFO("timestamp: seconds %f", timestamp->millisecond/1000.);
	USER_LOG_INFO("rtk position: longitude: %f, latitude: %f %f %f.\r\n", rtk_pos->longitude, rtk_pos->latitude, rtk_pos->hfsl);
    }
    dataStructure.coordinates.latitude = rtk_pos->latitude;
    dataStructure.coordinates.longitude = rtk_pos->longitude;
    dataStructure.coordinates.altitude = rtk_pos->hfsl;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/// @brief smartDelay()
static void smartDelay()
{
    while(received != 1){
        int d = simple_uart_read(uart, &c, 1);
        PMS_Encode(c);
    }

    if(received == 1){
    //   file_ptr = fopen("data.txt", "a");
    file_ptr = fopen(fileName, "a");

      if(file_ptr == NULL)
      {
          /* File not created hence exit */
          printf("Unable to create file.\n");
          flag.led_light.led = red;
          trigger(&flag); // Turn on diode (blink)
          exit(EXIT_FAILURE);
      }

      // ------------------------------------- BME DATA COLLECTION: ----------------------------------------------
      bme280_calib_data cal;
      readCalibrationData(fd, &cal);
      
      wiringPiI2CWriteReg8(fd, 0xf2, 0x01);   // humidity oversampling x 1
      wiringPiI2CWriteReg8(fd, 0xf4, 0x25);   // pressure and temperature oversampling x 1, mode normal 

      bme280_raw_data raw;
      getRawData(fd, &raw);

      int32_t t_fine = getTemperatureCalibration(&cal, raw.temperature);
      float t = compensateTemperature(t_fine);                        // C
      float p = compensatePressure(raw.pressure, &cal, t_fine) / 100; // hPa
      float h = compensateHumidity(raw.humidity, &cal, t_fine);       // %
      float a = getAltitude(p);                                       // meters

      dataStructure.bme.temperature = t;
      dataStructure.bme.humidity = h;
      dataStructure.bme.pressure = p;
      // ----------------------------------- END OF BME DATA COLLECTION ------------------------------------------

      // ---------------------------------- THERMOCOUPLE DATA COLLECTION: ----------------------------------------
      ValR = MAX31856GetTemperature(&ThermocoupleBreakout);
      dataStructure.thermocouple.temperature = ValR;
      // ------------------------------ END OF THERMOCOUPLE DATA COLLECTION --------------------------------------
      
      trigger(&flag); // Turn off diode (blink)

      fputs(printData(data_collected, dataStructure), file_ptr);
      received = 0;
      fclose(file_ptr);
      trigger(&flag);
    }   
}

/// @brief PMS_Encode
/// @param c 
void PMS_Encode(char c)
{
  uint16_t receiveSum=0;
  
  buff[buflen]=c;
  if (buff[buflen]==66)   
     event_char = 1;
  if (event_char)
     buflen++;
  
  if(buflen==receiveDatIndex)
  {
    received = 1;
    for(i=0;i<receiveDatIndex;i++)
       receiveSum=receiveSum+buff[i];
  
     if(receiveSum==((buff[receiveDatIndex-2]<<8)+buff[receiveDatIndex-1]+buff[receiveDatIndex-2]+buff[receiveDatIndex-1]))
     {
       PM01Value  = PM01_a  * ((buff[4]<<8) + buff[5]) + PM01_b;  //count PM1.0 value of the air detector module
       dataStructure.pm.PM1 = PM01Value;
       PM2_5Value = PM2_5_a * ((buff[6]<<8) + buff[7]) + PM2_5_b; //count PM2.5 value of the air detector module
       dataStructure.pm.PM2_5 = PM2_5Value;
       PM10Value  = PM10_a  * ((buff[8]<<8) + buff[9]) + PM10_b;  //count PM10 value of the air detector module
       dataStructure.pm.PM10 = PM10Value;

       fr03 = (buff[16]<<8) + buff[17];
       dataStructure.fr.fr03 = fr03;
       fr05 = (buff[18]<<8) + buff[19];
       dataStructure.fr.fr05 = fr05;
       fr10 = (buff[20]<<8) + buff[21];
       dataStructure.fr.fr10 = fr10;
       fr25 = (buff[22]<<8) + buff[23];
       dataStructure.fr.fr25 = fr25;
       fr50 = (buff[24]<<8) + buff[25];
       dataStructure.fr.fr50 = fr50;
       fr100 = (buff[26]<<8) + buff[27];
       dataStructure.fr.fr100 = fr100;
     }
     buflen = 0;
     event_char = 0;
  }  
}

char* printData(char* dataString, struct payloadData data){
    sprintf(dataString, "%d-%d-%d %d:%d:%d;", data.date.day, data.date.month, data.date.year, 
                                              data.date.hour, data.date.minute, data.date.second);
    sprintf(dataString, "%f;%f;%f;%u;%u;%f;%f;%f;%f;%f;-999;%f;%f;%f;%f;%f;%f;%f;%f;%f;-999;-999;\n", data.coordinates.latitude, data.coordinates.longitude, data.coordinates.altitude, data.GPSdataAge, data.satelliteNum.totalSatelliteNumberUsed, data.thermocouple.temperature, data.thermocouple.referenceTemperature, data.bme.temperature, data.bme.humidity, data.bme.pressure, data.pm.PM1, data.pm.PM2_5, data.pm.PM10, data.fr.fr03, data.fr.fr05, data.fr.fr10, data.fr.fr25, data.fr.fr50, data.fr.fr100);

    return dataString;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
