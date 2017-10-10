//NOTE: I don't know why I need this here but it won't compile as
//      c++ if I don't include it, it works just fine as C
#include <stdio.h>

#include "esp_deep_sleep.h"

#include "common/platform.h"
#include "common/cs_dbg.h"
#include "fw/src/mgos_app.h"
#include "fw/src/mgos_timers.h"
#include "fw/src/mgos_gpio.h"
#include "fw/src/mgos_uart.h"
#include "fw/src/mgos_hal.h"
#include "fw/src/mgos_wifi.h"
#include "mgos_mqtt.h"
#include "mgos_arduino_sht1x.h"
#include "mgos_arduino_dht.h"
#include "TinyGPS.h"

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define internal static
#define global   static

#define GPS_BAUD_RATE 9600

#define SHT1x_DATA_PIN  25 
#define SHT1x_CLOCK_PIN 26

#define DHT_DATA_PIN 21
#define DHT_TYPE     DHT22

#define REPEAT    1
#define NO_REPEAT 0

#define DEBUG_PIN 27

#define GPS_ENABLE_PIN 14

#define UART_NO     2
#define UART_RX_PIN 16
#define UART_TX_PIN 17

#define WAKEUP_PIN 34

#define GPS_FIX_SIGNAL_PIN 35
#define PASTER(x,y) x ## _ ## y
#define EVALUATOR(x,y)  PASTER(x,y)
#define ESP32_SDK_PIN_NAME(pin_num) EVALUATOR(GPIO_NUM, pin_num)

#define SECONDS(s) s*1000 
#define MINUTES(m) SECONDS(m * 60)

#define RESOLVE_PHASE_FUNCTION resolve_phase
#define RESOLVE_PHASE void RESOLVE_PHASE_FUNCTION(void* param)

RTC_DATA_ATTR internal time_t last_cycle_time;

DHT*    dht;
SHT1x*  sht1x;
TinyGPS gps;

mgos_gpio_int_handler_f interrupt_handler_pos = 0;
mgos_gpio_int_handler_f interrupt_handler_neg = 0;
global void* interrupt_user_data = 0;

enum phases
{
  PHASE_STARTUP = 0,
  PHASE_ACQUIRING_FIX,
  PHASE_LOCATION_LOCKON,
  PHASE_SENSOR_SAMPLING,
  PHASE_SEND_DATA
};

enum used_sensors
{
  SENSOR_SHT = 0,
  SENSOR_DHT,
  SENSOR_COUNT
};

typedef struct
{
  //NOTE, TODO: It may be incorrect to presume that gettimeofday() returns actual time, so this value may not be
  //		that useful. But timestamps of samples is something we are interested in so for now this will be included
  //		as is but we may need a different source for time, maybe it would be possible to somehow use the GPS or something
  //		equally ridiculous
  struct timeval sample_time;
  
  float temperatureC;
  float temperatureF;
  float humidity;
}THSample;

typedef struct
{
  mgos_timer_id timer_id;
  
  THSample SHT_sample;
  THSample DHT_sample;
}SensorSample;

typedef struct
{
  mgos_timer_id timer_id;

  unsigned long age;
  int32_t sample_count;
  float   latitude_decimal_degrees;
  float   longitude_decimal_degrees;
}LocationInfo;

typedef struct
{
  unsigned long age;
  float   latitude_decimal_degrees;
  float   longitude_decimal_degrees;
}LocationSample;

typedef struct 
{
  int32_t 	 half_transition_count;
  struct timeval time_since_last_blink;
  struct timeval last_pulse_end_time;
#ifdef MANURE_DEBUG
  struct timeval pulse_start_time;
#endif
}GPSFixInfo;

typedef struct
{
  s32 phase;

  LocationInfo location_info;
  SensorSample sensor_sample;
}StateInfo;

inline internal struct timeval 
get_time_passed(struct timeval TimeLaterOnTimeline, struct timeval TimeEarlierOnTimeline)
{
  struct timeval Result;
  Result.tv_sec  = TimeLaterOnTimeline.tv_sec  - TimeEarlierOnTimeline.tv_sec;
  if(TimeLaterOnTimeline.tv_usec >= TimeEarlierOnTimeline.tv_usec)
  {
    Result.tv_usec = TimeLaterOnTimeline.tv_usec - TimeEarlierOnTimeline.tv_usec;
  }
  else
  {
    Result.tv_usec = TimeEarlierOnTimeline.tv_usec - TimeLaterOnTimeline.tv_usec;
  }
  return(Result);
}
