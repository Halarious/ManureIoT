#include "main.h"

//NOTE/TODO: It confirmed from cesanta that wifi being active messes up the timings. For now the problem is that we get
//	     weird values for some of our data, for example the DHT won't work at all (with the current adafruit lib) and
//	     GPS gives ERROR 'age', the pulse times are werid etc.
//	     For now we can work through it and either the issue will be addressed or the wifi can be turned off most of the 
//	     time unless we are using it in a controlled code section, which may be better of an idea anyway.

internal void 
EventHandler(struct mg_connection *c, 
	     int ev, void *p, 
	     void *user_data)
{
  (void)c;
  (void)p;
  (void)user_data;

  //struct mg_mqtt_message *msg = (struct mg_mqtt_message *) p;
  //LOG(LL_INFO, ("CONNACK: %d", msg->connack_ret_code));
  switch(ev)
  {
    case(MG_EV_MQTT_PUBACK):
    {
  	LOG(LL_INFO, ("MG_EV_MQTT_PUBACK:"));
    } break;
    case(MG_EV_MQTT_CONNECT):
    {
  	LOG(LL_INFO, ("MG_EV_MQTT_CONNECT"));
        //LOG(LL_INFO, ("CONNACK: %d", msg->connack_ret_code));
    } break;
    case(MG_EV_MQTT_CONNACK):
    {
	LOG(LL_INFO, ("MG_EV_MQTT_CONNACK"));
    } break;
    case(MG_EV_CLOSE):
    {
	LOG(LL_INFO, ("MG_EV_CLOSE"));
    } break;
    case(MG_EV_POLL):
    {
	//LOG(LL_INFO, ("MG_EV_POLL"));
    } break;
    case(MG_EV_MQTT_PUBREL):
    {	
	LOG(LL_INFO, ("MG_EV_MQTT_PUBREL"));
    } break;
    case(MG_EV_MQTT_PUBCOMP):
    {  
	LOG(LL_INFO, ("MG_EV_MQTT_PUBCOMP:"));
    } break;
    case (MG_EV_MQTT_SUBSCRIBE):
    { 
	LOG(LL_INFO, ("MG_EV_MQTT_SUBSCRIBE:"));
    } break;
    case(MG_EV_MQTT_UNSUBSCRIBE):
    {  
	LOG(LL_INFO, ("MG_EV_MQTT_UNSUBSCRIBE:"));
    } break;
    case(MG_EV_MQTT_UNSUBACK):
    {  
	LOG(LL_INFO, ("MG_EV_MQTT_UNSUBACK:"));
    } break;
    case(MG_EV_MQTT_PINGREQ):
    {  
	LOG(LL_INFO, ("MG_EV_MQTT_PINGREQ:"));
    } break;
    case(MG_EV_MQTT_DISCONNECT):
    {  
	LOG(LL_INFO, ("MG_EV_MQTT_DISCONNECT:"));
    } break;
    case(MG_EV_MQTT_SUBACK):
    {
	LOG(LL_INFO, ("MG_EV_MQTT_SUBACK:"));
    } break;
    case(MG_EV_MQTT_PUBLISH):
    {
	LOG(LL_INFO, ("MG_EV_MQTT_PUBLISH:"));
    } break;
    case(MG_EV_MQTT_PUBREC): 
    {
	LOG(LL_INFO, ("MG_EV_MQTT_PUBREC"));
    } break;
    default:
    {
	LOG(LL_INFO, ("MG_EV_WHATISIT:"));
    } break;
  }
}

void 
my_interrupt_handler_pos(int signal_pin, void* arg)
{
  struct timeval current_time;
  gettimeofday(&current_time, 0);
  mgos_gpio_remove_int_handler(signal_pin, &interrupt_handler_pos, &arg);
  mgos_gpio_set_int_handler(signal_pin, MGOS_GPIO_INT_EDGE_NEG, interrupt_handler_neg, arg);
  //NOTE: Removing a handler disables the interrupt on the pin as far as I could gather. Also we should maybe make sure
  //      that we cannot 'skip' a pulse if something her takes too long, but so far it seemed to be good enough for the GPS 
  mgos_gpio_enable_int(signal_pin);
  LOG(LL_INFO, ("Interrupt POS on %d, user data: %p", signal_pin, arg));
  
  GPSFixInfo* gps_fix_info = (GPSFixInfo*) arg;
#ifdef MANURE_DEBUG
  gps_fix_info->pulse_start_time.tv_sec  = current_time.tv_sec;
  gps_fix_info->pulse_start_time.tv_usec = current_time.tv_usec;
#endif
  if((gps_fix_info->last_pulse_end_time.tv_sec  != 0) && 
     (gps_fix_info->last_pulse_end_time.tv_usec != 0))
  {
    //NOTE: There are some sources which say that some timing functions on the ESP wrap after 71 mins, but I am not sure
    //      weather it is true for gettimeofday (seems not but depends if it comforms to the POSIX function on linux?). Could be
    //      a problem since this definitely has a chance to run 71+ mins at a time.
    gps_fix_info->time_since_last_blink = get_time_passed(current_time, gps_fix_info->last_pulse_end_time);
    LOG(LL_INFO, ("Since last blink:  %lds:%ldus", gps_fix_info->time_since_last_blink.tv_sec, 
			    			   gps_fix_info->time_since_last_blink.tv_usec));
  }  
  gps_fix_info->half_transition_count++;
}

void 
my_interrupt_handler_neg(int signal_pin, void *arg)
{
  struct timeval current_time;
  gettimeofday(&current_time, 0);
  mgos_gpio_remove_int_handler(signal_pin, &interrupt_handler_neg, &arg);
  mgos_gpio_set_int_handler(signal_pin, MGOS_GPIO_INT_EDGE_POS, interrupt_handler_pos, arg);
  mgos_gpio_enable_int(signal_pin);
  LOG(LL_INFO, ("Interrupt NEG on %d, user data: %p", signal_pin, arg));
  
  GPSFixInfo* gps_fix_info = (GPSFixInfo*) arg;
  gps_fix_info->last_pulse_end_time = current_time;
  gps_fix_info->half_transition_count++;
#ifdef MANURE_DEBUG
#endif
}

struct timeval
read_SHT_data(float* SHT_temperatureC, float* SHT_temperatureF, float* SHT_humidity)
{
  struct timeval sample_timestamp;
  gettimeofday(&sample_timestamp, 0);
  
  if(SHT_humidity)
    *SHT_humidity     = sht1x->readHumidity();
  if(SHT_temperatureC)
    *SHT_temperatureC = sht1x->readTemperatureC();
  if(SHT_temperatureF)
    *SHT_temperatureF = sht1x->readTemperatureF();
  printf("%f\n", *SHT_temperatureC);
  printf("%f\n", *SHT_temperatureF);
  printf("%f\n", *SHT_humidity);
#ifdef MANURE_DEBUG
  LOG(LL_INFO, ("SHT readings:"));
  if(SHT_humidity)
    LOG(LL_INFO, ("%f", *SHT_humidity));
  if(SHT_temepratureC)
    LOG(LL_INFO, ("%f", *SHT_temperatureC));
  if(SHT_temperatureF)
    LOG(LL_INFO, ("%f", *SHT_temperatureF));
#endif
  return sample_timestamp;
}

struct timeval
read_DHT_data(float* DHT_temperatureC, float* DHT_temperatureF, float* DHT_humidity)
{
  struct timeval sample_timestamp;
  gettimeofday(&sample_timestamp, 0);
  
  if(DHT_humidity)
    *DHT_humidity     = dht->readHumidity();
  if(DHT_temperatureC)
    *DHT_temperatureC = dht->readTemperature();
  if(DHT_temperatureF)
    *DHT_temperatureF = dht->readTemperature(true);
  printf("%f\n", *DHT_temperatureC);
  printf("%f\n", *DHT_temperatureF);
  printf("%f\n", *DHT_humidity);
#ifdef MANURE_DEBUG
  LOG(LL_INFO, ("DHT readings:"));
  if(DHT_humidity)
    LOG(LL_INFO, ("%f", *DHT_humidity));
  if(DHT_temperatureC)
    LOG(LL_INFO, ("%f", *DHT_temperatureC));
  if(DHT_temperatureF)
    LOG(LL_INFO, ("%f", *DHT_temperatureF));  
#endif
  return sample_timestamp;
}

#define MARK_SAMPLED(id, mask) (mask = ((0x1 << id) ^ mask))
u8
read_sensor_data(SensorSample* sensor_sample)
{
  u8 sample_mask = 0;
  THSample* SHT_sample = &sensor_sample->SHT_sample;
  THSample* DHT_sample = &sensor_sample->DHT_sample;
  SHT_sample->sample_time  = read_SHT_data(&SHT_sample->temperatureC, &SHT_sample->temperatureF, &SHT_sample->humidity);
  DHT_sample->sample_time  = read_DHT_data(&DHT_sample->temperatureC, &DHT_sample->temperatureF, &DHT_sample->humidity);
  if(!isnan(SHT_sample->humidity)     && 
     !isnan(SHT_sample->temperatureC) && 
     !isnan(SHT_sample->temperatureF))
  {
    MARK_SAMPLED(SENSOR_SHT, sample_mask);
  }
  if(!isnan(DHT_sample->humidity)     &&
     !isnan(DHT_sample->temperatureC) && 
     !isnan(DHT_sample->temperatureF))
  {
    MARK_SAMPLED(SENSOR_DHT, sample_mask);
  }
  return(sample_mask);
}

#define SENSOR_SAMPLED(index, mask) ((0x1 << index) & mask) ? true : false
void
sample_sensors(void* param)
{
  SensorSample sensor_sample;
  u8 sample_mask = read_sensor_data(&sensor_sample);
  
  SensorSample* sensor_data = (SensorSample*) param;
  int index = 0;
  while(index < SENSOR_COUNT)
  {
    if(SENSOR_SAMPLED(index, sample_mask))
    {
      LOG(LL_INFO, ("Sensor %d sampled", index));
      //sensor_;
    }
    else
    {
      LOG(LL_INFO, ("Sensor %d failed sample", index));
    }
    ++index;
  } 
  sensor_data->timer_id = mgos_set_timer(SECONDS(3), NO_REPEAT, sample_sensors, param); 
}

void
init_sensor_sampling(SensorSample* sensor_sample)
{
  sample_sensors((void*) sensor_sample); 
}

bool
read_gps_data(float* lat, float* lon, unsigned long* age)
{
    bool success;

    gps.f_get_position(lat, lon, age);
    //TODO: This is the GPS_INVALID_ANGLE from TinyGPS, we should define this somehow or see if we can use it from there
    if(*lat != 1000.000000f ||
       *lon != 1000.000000f)
    {
      printf("GPS position:\n");
      printf("Age:  	 %lu\n", *age); 
      printf("Latitude:  %f\n",  *lat); 
      printf("Longitude: %f\n",  *lon);
      success = true; 
    }
    else
    {
      printf("GPS fix not valid\n");
      success = false; 
    }
    return success;
}

void 
location_lockon(void* param)
{
  LocationInfo*   location_info = (LocationInfo*) param;
  LocationSample  location_sample = {};
  if(read_gps_data(&location_sample.latitude_decimal_degrees, 
	           &location_sample.longitude_decimal_degrees, 
		   &location_sample.age))
  {
    ++location_info->sample_count;
    float last_sample_lat_sum = location_info->latitude_decimal_degrees  * (location_info->sample_count - 1);
    float last_sample_lon_sum = location_info->longitude_decimal_degrees * (location_info->sample_count - 1);
    location_info->latitude_decimal_degrees  = (location_sample.latitude_decimal_degrees  + last_sample_lat_sum) / location_info->sample_count;
    location_info->longitude_decimal_degrees = (location_sample.longitude_decimal_degrees + last_sample_lon_sum) / location_info->sample_count;
    LOG(LL_INFO, ("Latitude: %f, Longitude: %f,  from %d samples", location_info->latitude_decimal_degrees,
			    					   location_info->longitude_decimal_degrees,
								   location_info->sample_count));
  }
  location_info->timer_id = mgos_set_timer(SECONDS(10), NO_REPEAT, location_lockon, param);
}

void
init_location_lockon(LocationInfo* location_info)
{
  location_lockon((void*) location_info);
}

void
resolve_gps_fix()
{
  float time_since_last_pulse;
  
  struct timeval current_time;
  gettimeofday(&current_time, 0);
  
  mgos_gpio_remove_int_handler(GPS_FIX_SIGNAL_PIN, 0, 0);
  GPSFixInfo* gps_fix_info = (GPSFixInfo*) interrupt_user_data;
  LOG(LL_INFO, ("Half transition count: %d",  gps_fix_info->half_transition_count )); 
  bool ended_up = gps_fix_info->half_transition_count & 0x1; 
  if(ended_up)
  {
    time_since_last_pulse = gps_fix_info->time_since_last_blink.tv_sec 
	  		    + gps_fix_info->time_since_last_blink.tv_usec / 1000000.0f;
    LOG(LL_INFO, ("Last pulse end time:   %lds %ldus", current_time.tv_sec, 
			  			       current_time.tv_usec)); 
  }
  else
  {
    //TODO: This could be 0 if there was only one blink, but do we care though?
    struct timeval time_passed_since_last_pulse = get_time_passed(current_time, gps_fix_info->last_pulse_end_time);
    time_since_last_pulse = time_passed_since_last_pulse.tv_sec + time_passed_since_last_pulse.tv_usec / 1000000.0f;
    LOG(LL_INFO, ("Last pulse end time:   %lds %ldus", gps_fix_info->last_pulse_end_time.tv_sec, 
  			  			       gps_fix_info->last_pulse_end_time.tv_usec)); 
  }
  LOG(LL_INFO, ("Time since last pulse: %fs", time_since_last_pulse)); 
  
  if(time_since_last_pulse >= 2.0f)
  {
    //NOTE: This is probably a fix
    LOG(LL_INFO, ("GPS fix get"));
    mgos_uart_set_rx_enabled(UART_NO, true);
  }
  else
  {
    //TODO: Try a second pass on the interrupts, there shouldn't be anything on the line for ~13-15s if
    //      we have a fix  
    if(gps_fix_info->half_transition_count <= 2)
    {
      LOG(LL_INFO, ("GPS fix probaby get")); 
    }
    LOG(LL_INFO, ("GPS fix uncertain"));
  }
}

void
init_gps_fix_check()
{
  //NOTE: This runs on interrupts which DO NOT actually interrupt per se, but are checked at some poll, similar to timers. Keep it in mind
#ifdef MANURE_DEBUG
  mgos_gpio_write(DEBUG_PIN, 1);
#endif
  mgos_gpio_remove_int_handler(GPS_FIX_SIGNAL_PIN, 0, 0);
  if(interrupt_user_data)
  {
    free(interrupt_user_data);
  }
  interrupt_user_data = calloc(1, sizeof(GPSFixInfo));
  
  mgos_gpio_set_int_handler(GPS_FIX_SIGNAL_PIN, MGOS_GPIO_INT_EDGE_POS, interrupt_handler_pos, interrupt_user_data);
  mgos_gpio_enable_int(GPS_FIX_SIGNAL_PIN);
#ifdef MANURE_DEBUG
  mgos_msleep(300);
  mgos_gpio_write(DEBUG_PIN, 0);
#endif  
}

void
my_uart_dispatcher(int uart_no, void *arg)
{
  //TODO: We should check out how the dispatcher works, I am kind of worried it could be holding old
  //      data if it is not read out, but maybe it just ovewrittes the data as new stuff comes in (but 
  //      in what way, does it overwrittet just part of the buffer?)
  size_t rx_bytes_available_to_read = mgos_uart_read_avail(uart_no);
  if(rx_bytes_available_to_read > 0) 
  {
    struct mbuf rx_buffer;
    mbuf_init(&rx_buffer, 0);
    mgos_uart_read_mbuf(uart_no, &rx_buffer, rx_bytes_available_to_read);
    if(rx_buffer.len > 0) 
    {
#ifdef MANURE_DEBUG
      //printf("%.*s", (int) rxb.len, rxb.buf);
#endif
      char* BufferPointer = rx_buffer.buf;
      while(BufferPointer != '\0' && ((BufferPointer - rx_buffer.buf) < rx_buffer.len))
      {
        gps.encode(*BufferPointer++);
      }
    }
    mbuf_free(&rx_buffer);
  }
   
  (void)arg;
}

#define STARTUP 	   void* param = calloc(1, sizeof(StateInfo)); \
			   mgos_set_timer(0,  NO_REPEAT, RESOLVE_PHASE_FUNCTION, param)
#define RECALL_RESOLVE(ms) mgos_set_timer(ms, NO_REPEAT, RESOLVE_PHASE_FUNCTION, param)
#define GET_STATE ((StateInfo*) param)
RESOLVE_PHASE
{
  StateInfo* state_info = GET_STATE;
  switch(state_info->phase)
  {
    case(PHASE_STARTUP):
    {
      LOG(LL_INFO, ("Start-up"));
      RECALL_RESOLVE(SECONDS(14));
      
      state_info->phase = PHASE_ACQUIRING_FIX;
      init_gps_fix_check();
    } break;
    case(PHASE_ACQUIRING_FIX):
    {
      LOG(LL_INFO, ("Acquiring Fix"));
      resolve_gps_fix();
      if(0)
      {
         RECALL_RESOLVE(SECONDS(5));
         init_gps_fix_check();
      } 
      
      RECALL_RESOLVE(MINUTES(1));
      state_info->phase = PHASE_LOCATION_LOCKON;
      init_location_lockon(&state_info->location_info);
    } break;
    case(PHASE_LOCATION_LOCKON):
    {
      LOG(LL_INFO, ("Location lock-on"));
      RECALL_RESOLVE(SECONDS(20));

      mgos_clear_timer(state_info->location_info.timer_id);

      state_info->phase = PHASE_SENSOR_SAMPLING;
      init_sensor_sampling(&state_info->sensor_sample);
    } break;
    case(PHASE_SENSOR_SAMPLING):
    {
      LOG(LL_INFO, ("Sensor sampling"));
      RECALL_RESOLVE(SECONDS(5));

      mgos_clear_timer(state_info->sensor_sample.timer_id);
      
      state_info->phase = PHASE_SEND_DATA;
      mgos_wifi_connect();
    } break;
    case(PHASE_SEND_DATA):
    {
      LOG(LL_INFO, ("Sending Data"));

      mgos_wifi_disconnect();
    } break;
    default:
    {
      ;
    }
  }  
}

enum mgos_app_init_result 
mgos_app_init(void) 
{
  enum mgos_app_init_result returnStatus = MGOS_APP_INIT_ERROR;

  struct timeval current_time;
  gettimeofday(&current_time, 0);
 
  esp_deep_sleep_wakeup_cause_t wakeup_cause = esp_deep_sleep_get_wakeup_cause();
  switch(wakeup_cause)
  {
    case(ESP_DEEP_SLEEP_WAKEUP_EXT0):
    {
      LOG(LL_INFO, ("WAKEUP EXT0"));
    } break;
    case(ESP_DEEP_SLEEP_WAKEUP_EXT1):
    {
      LOG(LL_INFO, ("WAKEUP EXT1"));
    } break;
    case(ESP_DEEP_SLEEP_WAKEUP_UNDEFINED):
    {
      LOG(LL_INFO, ("WAKEUP UNDEFINED"));
    } break;
    default:
    {
      LOG(LL_INFO, ("WAKEUP ERROR"));
    }
  }

  mgos_mqtt_add_global_handler(EventHandler, 0); 

  sht1x = mgos_sht1x_create(SHT1x_DATA_PIN, SHT1x_CLOCK_PIN);
  mgos_gpio_set_mode(SHT1x_DATA_PIN, MGOS_GPIO_MODE_INPUT);
  mgos_gpio_set_mode(SHT1x_CLOCK_PIN, MGOS_GPIO_MODE_OUTPUT);
  gpio_pullup_dis(ESP32_SDK_PIN_NAME(SHT1x_DATA_PIN));
  gpio_pulldown_dis(ESP32_SDK_PIN_NAME(SHT1x_DATA_PIN));
  
  dht = mgos_dht_create(DHT_DATA_PIN, DHT_TYPE);
  //gpio_pullup_dis(ESP32_SDK_PIN_NAME(DHT_DATA_PIN));
  //gpio_pulldown_dis(ESP32_SDK_PIN_NAME(DHT_DATA_PIN));
  dht->begin();
  
  //TODO: Move this somewhere sensible, maybe. Define the interrupt pin, could be 35 and would best be an input only pin (all 32+ I think?)
  //NOTE: I don't actually know why I have to malloc this, but when passed by stack allocation, I get garbage on the interrupts
  interrupt_handler_pos = my_interrupt_handler_pos;
  interrupt_handler_neg = my_interrupt_handler_neg;
  LOG(LL_INFO, ("Interrupt user data: %p", interrupt_user_data));
  gpio_pullup_dis(ESP32_SDK_PIN_NAME(GPS_FIX_SIGNAL_PIN));
  gpio_pulldown_en(ESP32_SDK_PIN_NAME(GPS_FIX_SIGNAL_PIN));
  mgos_gpio_set_mode(GPS_FIX_SIGNAL_PIN, MGOS_GPIO_MODE_INPUT);

#ifdef MANURE_DEBUG
  mgos_gpio_set_mode(DEBUG_PIN, MGOS_GPIO_MODE_OUTPUT);
#endif
  mgos_gpio_set_mode(GPS_ENABLE_PIN, MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_write(GPS_ENABLE_PIN, HIGH);
  
  struct mgos_uart_config uart_config;
  mgos_uart_config_set_defaults(UART_NO, &uart_config);
  uart_config.baud_rate = GPS_BAUD_RATE;
  if(mgos_uart_configure(UART_NO, &uart_config))
  { 
    LOG(LL_INFO, ("UART success"));
    mgos_uart_set_rx_enabled(UART_NO, false);
    mgos_uart_set_dispatcher(UART_NO, my_uart_dispatcher, 0);

    //NOTE: Apparently the pull-ups and pull-downs in this way are in the 'digital domain' so we might consider doing these
    //	    externally or we could be woken up at random because of random interferance.
    gpio_pullup_dis(ESP32_SDK_PIN_NAME(WAKEUP_PIN));
    gpio_pulldown_en(ESP32_SDK_PIN_NAME(WAKEUP_PIN));
    esp_deep_sleep_enable_ext0_wakeup(ESP32_SDK_PIN_NAME(WAKEUP_PIN), 1);
    
    STARTUP;
    struct sys_config_wifi_sta cfg = {};
    cfg.enable = true;
    cfg.ssid   = "Ana 5";
    cfg.pass   = "12lajkica";
    mgos_wifi_setup_sta(&cfg);
    
    printf("Current: %lds, Last: %lds\n", current_time.tv_sec, current_time.tv_sec - last_cycle_time);
    last_cycle_time = current_time.tv_sec;

    returnStatus = MGOS_APP_INIT_SUCCESS;
  }

  return(returnStatus);
}
