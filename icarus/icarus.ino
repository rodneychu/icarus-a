#include <SFE_BMP180.h>
#include <LiquidCrystal.h>
#include "ant.h"

#define FAILURE_LIMIT        3
#define NETWORK_KEY          0xB9A521FBBD72C345
#define NETWORK_NUM          0
#define RADIO_FREQUENCY      57
#define SEARCH_CHANNEL_NUM   0
#define POWER_CHANNEL_NUM    2
#define POWER_CHANNEL_PERIOD 8182
#define POWER_DEVICE_TYPE    0x0B
#define SPEED_CHANNEL_NUM    1
#define SPEED_CHANNEL_PERIOD 8118
#define SPEED_DEVICE_TYPE    0x7B

#define CELSIUS_TO_KELVIN    273.15
#define GAS_CONSTANT_FOR_AIR 287.05
#define GRAVITY              9.80665
#define HOUR_IN_SECONDS      3600
#define KM_IN_METERS         1000
#define MILLIBAR_IN_PASCALS  100
#define SEA_LEVEL_DENSITY    1.225
#define SECOND_IN_MILLIS     1000
#define SAMPLE_BUFFER_SIZE   40
#define SAMPLE_INTERVAL      250   // milliseconds
#define SAMPLE_RATE          4     // hertz
#define SAMPLE_WINDOW        3     // seconds
#define SAMPLES_PER_HOUR     14400 // 60 * 60 * 4

// TODO: make these adjustable
#define CRR                  0.002324
#define RIDER_MASS           79.37    // kilograms
#define WHEEL_CIRCUMFERENCE  2096     // millimeters

/* variables */
boolean initialized = false;
boolean search_channel_open = false;
uint8_t failure_count = 0;
boolean power_channel_open = false;
uint16_t power_device_id = 0;
boolean speed_channel_open = false;
uint16_t speed_device_id = 0;

uint16_t speed[SAMPLE_BUFFER_SIZE];
uint16_t prev_speed_revs = 0;
uint8_t  prev_speed_time = 0;
uint32_t prev_speed_millis = 0;

uint16_t power[SAMPLE_BUFFER_SIZE];
uint32_t prev_power_millis = 0;

double airspeed[SAMPLE_BUFFER_SIZE];
uint32_t prev_airspeed_millis = 0;
int airspeed_adj = 0;

double pressure[SAMPLE_BUFFER_SIZE];
uint32_t prev_pressure_millis = 0;
double current_temp = 0;
double current_pressure = 0;

uint32_t prev_display_refresh = 0;

/* objects */
USB usb;
Ant ant(&usb);
LiquidCrystal lcd(2, 1, 4, 5, 6, 3);
SFE_BMP180 barometer;

void setup() {
  memset(speed, 0, SAMPLE_BUFFER_SIZE * sizeof(uint16_t));
  memset(power, 0, SAMPLE_BUFFER_SIZE * sizeof(uint16_t));
  memset(airspeed, 0, SAMPLE_BUFFER_SIZE * sizeof(double));
  memset(pressure, 0, SAMPLE_BUFFER_SIZE * sizeof(double));

#ifdef EXTRADEBUG
  Serial.begin(115200);
#else
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("C:");
  lcd.setCursor(7, 0);
  lcd.print("P:");
  lcd.setCursor(0, 1);
  lcd.print("W:");
  lcd.setCursor(6, 1);
  lcd.print("G:");
  lcd.setCursor(12, 1);
  lcd.print("S:");
#endif

  if (!barometer.begin()) {
    while(1); // Pause forever.
#ifdef EXTRADEBUG
    Serial.println(F("BMP180 init fail\n\n"));
  } else {
    Serial.println(F("BMP180 init success"));
#endif
  }

  // calibrate airspeed meter
  uint16_t total = 0;
  for (uint8_t i = 0; i < 20; i++) {
    total += analogRead(1);
  }
  airspeed_adj = 512 - (total / 20);

  usb.Init();
}

void loop() {
  uint8_t rcode;

  record_airspeed_sample();
  record_pressure_sample();
  refresh_display();
  
  usb.Task();

  if (ant.is_ready()) {
    if (!initialized) {
      rcode = ant.ResetSystem();
      if (rcode) goto Fail;
      rcode = ant.SetNetworkKey(NETWORK_NUM, NETWORK_KEY);
      if (rcode) goto Fail;
      initialized = true;
    }
    if (!search_channel_open && (!speed_device_id || !power_device_id)) {
      rcode = open_search_channel();
      if (rcode) goto Fail;
      search_channel_open = true;
    }
    if (search_channel_open && speed_device_id && power_device_id) {
      rcode = close_search_channel();
      if (rcode) goto Fail;
      search_channel_open = false;
    }
    if (speed_device_id && !speed_channel_open) {
      rcode = open_data_channel(SPEED_CHANNEL_NUM, speed_device_id,
          SPEED_DEVICE_TYPE, SPEED_CHANNEL_PERIOD, record_speed_sample);
      if (rcode) goto Fail;
      speed_channel_open = true;
    }
    if (power_device_id && !power_channel_open) {
      rcode = open_data_channel(POWER_CHANNEL_NUM, power_device_id,
          POWER_DEVICE_TYPE, POWER_CHANNEL_PERIOD, record_power_sample);
      if (rcode) goto Fail;
      power_channel_open = true;
    }
    return;

Fail:
#ifdef EXTRADEBUG
    Serial.print(F("\r\nFailure count:\t"));
    Serial.print(++failure_count, DEC);
#endif
    if (failure_count == FAILURE_LIMIT)
      while(1);
  }
}

uint8_t open_search_channel() {
  uint8_t rcode;
  rcode = ant.AssignChannel(SEARCH_CHANNEL_NUM, NETWORK_NUM, true);
  if (rcode) return rcode;
  rcode = ant.SetChannelId(SEARCH_CHANNEL_NUM, 0, 0);
  if (rcode) return rcode;
  rcode = ant.EnableExtended(0x01);
  if (rcode) return rcode;
  rcode = ant.SetLowPrioritySearchTimeout(SEARCH_CHANNEL_NUM, 0xFF); // no timeout
  if (rcode) return rcode;
  rcode = ant.SetChannelSearchTimeout(SEARCH_CHANNEL_NUM, 0); // disable HP search
  if (rcode) return rcode;
  rcode = ant.SetChannelFrequency(SEARCH_CHANNEL_NUM, RADIO_FREQUENCY);
  if (rcode) return rcode;
  rcode = ant.OpenChannel(SEARCH_CHANNEL_NUM, device_scan);
  return rcode;
}

uint8_t close_search_channel() {
  uint8_t rcode = ant.CloseChannel(SEARCH_CHANNEL_NUM);
  if (rcode) return rcode;
  rcode = ant.EnableExtended(0);
  return rcode;
}

uint8_t open_data_channel(uint8_t channel_num, uint16_t device_id, uint8_t device_type, uint16_t channel_period, DataInCallback callback) {
  uint8_t rcode;
  rcode = ant.AssignChannel(channel_num, NETWORK_NUM, false);
  if (rcode) return rcode;
  rcode = ant.SetChannelId(channel_num, device_id, device_type);
  if (rcode) return rcode;
  rcode = ant.SetChannelPeriod(channel_num, channel_period);
  if (rcode) return rcode;
  rcode = ant.SetChannelFrequency(channel_num, RADIO_FREQUENCY);
  if (rcode) return rcode;
  rcode = ant.OpenChannel(channel_num, callback);
  return rcode;
}

void device_scan(uint8_t* buf, uint16_t bytes) {
  if (bytes < 13 || buf[8] != 0x80) {
#ifdef EXTRADEBUG
    Serial.print(F("\r\nNot extended message"));
#endif
    return;
  }

  switch(buf[11]) {
    case SPEED_DEVICE_TYPE:
      if (speed_device_id)
        return;
      speed_device_id = buf[9] << 8;
      speed_device_id |= buf[10];
#ifdef EXTRADEBUG
      Serial.print(F("\r\nspeed device:\t"));
      Serial.print(speed_device_id, HEX);
#endif
      break;
    case POWER_DEVICE_TYPE:
      if (power_device_id)
        return;
      power_device_id = buf[9] << 8;
      power_device_id |= buf[10];
#ifdef EXTRADEBUG
      Serial.print(F("\r\npower device:\t"));
      Serial.print(power_device_id, HEX);
#endif
      break;
  }
}

void record_speed_sample(uint8_t* buf, uint16_t bytes) {
  if (bytes < 8) {
#ifdef EXTRADEBUG
    Serial.print(F("\r\nExpected 8 bytes"));
#endif
    return;
  }

  uint32_t current_millis = millis();
  if (prev_speed_millis && (current_millis - prev_speed_millis)
      > ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) {
    memset(speed, 0, SAMPLE_BUFFER_SIZE * sizeof(uint16_t));
    prev_speed_revs = 0;
    prev_speed_time = 0;
    prev_speed_millis = 0;
  }
    
  uint16_t current_time = buf[5] << 8;
  current_time |= buf[4];
  if (current_time == prev_speed_time) { // the ant time counter -- not system time
    return;
  }

  uint8_t current_index = (current_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;
  uint8_t prev_index = (prev_speed_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;
  for (uint8_t i = prev_index + 1; i < current_index; i++) {
    speed[i] = 0;
  }

  uint16_t current_revs = buf[7] << 8;
  current_revs |= buf[6];
  if (prev_speed_revs == 0) {
    speed[current_index] = 0;
  } else {
    speed[current_index] = current_revs - prev_speed_revs;
  }

  prev_speed_revs = current_revs;
  prev_speed_time = current_time;
  prev_speed_millis = current_millis;

#ifdef EXTRADEBUG
  Serial.print(F("\r\nspeed (revs):\t"));
  Serial.print(speed[current_index], DEC);
#endif  
}

void record_power_sample(uint8_t* buf, uint16_t bytes) {
  if (bytes < 8) {
#ifdef EXTRADEBUG
    Serial.print(F("\r\nExpected 8 bytes"));
#endif
    return;
  }

  if (buf[0] != 0x10) return;
      
  uint32_t current_millis = millis();
  if (prev_power_millis && (current_millis - prev_power_millis)
      > ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) {
    memset(power, 0, SAMPLE_BUFFER_SIZE);
    prev_power_millis = 0;
  }
    
  uint16_t current_watts = buf[7] << 8;
  current_watts = current_watts |= buf[6];

  uint8_t current_index = (current_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;
  uint8_t prev_index = (prev_power_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;

  int skipped = (int) current_index - (int) prev_index;
  if (skipped < 0) {
    skipped = SAMPLE_BUFFER_SIZE - skipped;
  }
  uint16_t average = (power[prev_index] + current_watts) / (current_index - prev_index);
  for (uint8_t i = 1; i < skipped; i++) {
    power[(prev_index + i) % SAMPLE_BUFFER_SIZE] = average;
  }
  power[current_index] = current_watts;

  prev_power_millis = current_millis;

#ifdef EXTRADEBUG
  Serial.print(F("\r\npower (watts):\t"));
  Serial.print(current_watts, DEC);
#endif  
}

void refresh_display() {
  uint32_t current_millis = millis();
  if (current_millis < (prev_display_refresh + SECOND_IN_MILLIS)) return;
  
  uint8_t current_index = (millis() %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;

  // speed (3 sec avg)
  uint16_t window_total = 0;
  bool count_started = false;
  uint8_t counter = 0;
  uint8_t delta = 0;
  uint16_t start_index = (current_index + SAMPLE_BUFFER_SIZE
      - (SAMPLE_WINDOW * SAMPLE_RATE)) % SAMPLE_BUFFER_SIZE;
  for (uint8_t i = 0; i < (SAMPLE_WINDOW * SAMPLE_RATE); i++) {
    uint8_t index = (start_index + i) % SAMPLE_BUFFER_SIZE;
    if (!count_started && speed[index] > 0) {
      count_started = true;
      continue;
    }
    counter++;
    if (speed[index] > 0) {
      window_total += speed[index];
      delta += counter;
      counter = 0;
    }
  }

  uint32_t dist_mm = (uint32_t) window_total * WHEEL_CIRCUMFERENCE;
  double meters_per_sec = 0;
  double km_per_hour = 0;
  if (delta > 0) {
    meters_per_sec = dist_mm / (delta * (SECOND_IN_MILLIS / SAMPLE_RATE)); // mm/ms == m/s  
    km_per_hour = meters_per_sec * ((double)HOUR_IN_SECONDS / KM_IN_METERS);
  }
  
#ifdef EXTRADEBUG
  Serial.print(F("\r\nspeed (km/h):\t"));
  Serial.print(km_per_hour, 2);
#else
  char buffer[4];
  dtostrf(km_per_hour, 2, 0, buffer);
  lcd.setCursor(14, 1);
  lcd.print(buffer);
#endif

  // power (3 sec avg)
  uint16_t sum_power = 0;
  for (int i = 0; i < (SAMPLE_WINDOW * SAMPLE_RATE); i++) {
    sum_power += power[(start_index + i) % SAMPLE_BUFFER_SIZE];
  }
  double avg_power = sum_power / (SAMPLE_WINDOW * SAMPLE_RATE);
#ifdef EXTRADEBUG
  Serial.print(F("\r\npower (watts):\t"));
  Serial.print(avg_power, 2);
#else
  dtostrf(avg_power, 3, 0, buffer);
  lcd.setCursor(9, 0);
  lcd.print(buffer);
#endif
    
  // airspeed (3 sec avg)
  double dbl_total = 0;
  for (int i = 0; i < (SAMPLE_WINDOW * SAMPLE_RATE); i++) {
    dbl_total += airspeed[(start_index + i) % SAMPLE_BUFFER_SIZE];
  }
  double avg_airspeed = (dbl_total / (SAMPLE_WINDOW * SAMPLE_RATE))
      * 3.6;  // m/s converted to km/h
      
#ifdef EXTRADEBUG
  Serial.print(F("\r\nairspeed (km/h):\t"));
  Serial.print(avg_airspeed, 2);
#else
  dtostrf(avg_airspeed, 3, 0, buffer);
  lcd.setCursor(2, 1);
  lcd.print(buffer);
#endif

  // elevation change (3 sec delta)
  double elev_change = (pressure[start_index] <= 0 || pressure[current_index] <= 0) ?
      0 : barometer.altitude(pressure[current_index], pressure[start_index]);
#ifdef EXTRADEBUG
  Serial.print(F("\r\nelevation change (m):\t"));
  Serial.print(elev_change, 2);
#else
  dtostrf(elev_change, 3, 0, buffer);
  lcd.setCursor(8, 1);
  lcd.print(buffer);
#endif

  prev_display_refresh = current_millis;

  if (meters_per_sec == 0 || current_pressure == 0) {
    return;
  }
  double prr = GRAVITY * RIDER_MASS * CRR * meters_per_sec;
  double pgr = GRAVITY * RIDER_MASS * (elev_change / SAMPLE_WINDOW); // mps cancelled out
  double pw = avg_power - prr - pgr;
  double air_density = (current_pressure * MILLIBAR_IN_PASCALS) /
      (GAS_CONSTANT_FOR_AIR * (current_temp + CELSIUS_TO_KELVIN));
  double cda = meters_per_sec <= 0 ? 0 :
      pw / (meters_per_sec * (pow(meters_per_sec, 2)/2) * air_density);
  
#ifdef EXTRADEBUG
  Serial.print(F("\r\nCdA:\t"));
  Serial.print(cda, 2);
#else
  if (abs(cda) < 10) {
    dtostrf(cda, 3, 2, buffer);
    lcd.setCursor(2, 0);
    lcd.print(buffer);
  }
#endif  

}

void record_airspeed_sample() {
  uint32_t current_millis = millis();
  if (current_millis < (prev_airspeed_millis + SAMPLE_INTERVAL)) return;
  
  uint8_t current_index = (current_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;
  uint8_t prev_index = (prev_airspeed_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;

  uint16_t total = 0;
  for (uint8_t i = 0; i < 10; i++) {
    total += analogRead(1);    // read the input pin
  }
  uint16_t val = (total/10) + airspeed_adj;
  double differential = (((double)val/1024) * 5) - 2.5; // in kPA
  airspeed[current_index] = sqrt((2.0f * abs(differential * 1000)) / SEA_LEVEL_DENSITY)
      * (differential < 0 ? -1 : 1);

  int skipped = (int) current_index - (int) prev_index;
  if (skipped < 0) {
    skipped = SAMPLE_BUFFER_SIZE - skipped;
  }
  for (uint8_t i = 1; i < skipped; i++) {
    airspeed[(prev_index + i) % SAMPLE_BUFFER_SIZE] = airspeed[current_index];
  }

  prev_airspeed_millis = current_millis;

#ifdef EXTRADEBUG
  Serial.print(F("\r\nairspeed (raw):\t"));
  Serial.print(val);
  Serial.print(F("\r\nair density:\t"));
  Serial.print(air_density, 2);
  Serial.print(F("\r\ntemperature:\t"));
  Serial.print(current_temp, 2);
#endif
}

void record_pressure_sample() {
  uint32_t current_millis = millis();
  if (current_millis < (prev_pressure_millis + SAMPLE_INTERVAL)) return;
  
  uint8_t current_index = (current_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;
  uint8_t prev_index = (prev_pressure_millis %
      ((uint16_t)SAMPLE_BUFFER_SIZE * SAMPLE_INTERVAL)) / SAMPLE_INTERVAL;

  char status;
  double T,P;
  status = barometer.startTemperature();
  if (!status) {
#ifdef EXTRADEBUG
    Serial.println(F("error starting temperature measurement\n"));
#endif
    return;
  }
  delay(status);
  status = barometer.getTemperature(T);
  if (!status) {
#ifdef EXTRADEBUG
    Serial.println(F("error retrieving temperature measurement\n"));
#endif
    return;
  }
  current_temp = T;
  status = barometer.startPressure(3);
  if (!status) {
#ifdef EXTRADEBUG
    Serial.println(F("error starting pressure measurement\n"));
#endif
    return;
  }
  delay(status);
  status = barometer.getPressure(P,T);
  if (!status) {
#ifdef EXTRADEBUG
    Serial.println(F("error retrieving pressure measurement\n"));
#endif
    return;
  }
  current_pressure = P;

  int skipped = (int) current_index - (int) prev_index;
  if (skipped < 0) {
    skipped = SAMPLE_BUFFER_SIZE - skipped;
  }
  double incr = (P - pressure[prev_index]) / skipped;
  for (uint8_t i = 1; i < skipped; i++) {
    pressure[(prev_index + i) % SAMPLE_BUFFER_SIZE] = pressure[prev_index] + (incr * i);
  }
  pressure[current_index] = P;
  prev_pressure_millis = current_millis;

#ifdef EXTRADEBUG
  Serial.print(F("\r\npressure (millibars):\t"));
  Serial.print(pressure[current_index]);
#endif
}
