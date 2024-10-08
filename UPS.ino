#define USE_SERIAL
#define LCD

#include <avr/wdt.h>
#include <EEPROM.h>

#ifdef LCD
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <pcf8574.h>   // https://github.com/MSZ98/pcf8574.git
#include "src/OneButton/OneButton.h"  // https://github.com/mathertel/OneButton , modified for PCF8574 expander
// LCD geometry
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
#define EXP_ADDR 0x22
#define CURRENT_SENSOR 7
#define BTN_RED 5
#define BTN_GREEN 6
#endif

#if defined ( USE_SERIAL ) || defined ( LCD )
// some compatibility with esp8266/esp32
#define FPSTR(pstr) (const __FlashStringHelper*)(pstr) 
#endif

#define EEPROM_STATE_BYTE 0
#define EEPROM_MARK_BYTE 0xa

#define STATE_UNKNOWN 0
#define STATE_STANDBY 1
#define STATE_INVERTING 2
#define STATE_CHARGING 3

const int main_loop_delay = 500;                   // ms
const int iverter_start_delay = 2;                 // s
const int iverter_stop_delay = 30;                 // s
const int charger_start_delay = 180;               // s
const int current_sensor_start_delay = 60;         // s
const int current_sensor_charger_off_delay = 300;  // s
const int charge_measure_delay = (3600 * 2);       // s
const int max_charger_work_time = (3600 * 3);      // s
const int battery_measuring_period = 30;           // s
const int delta_series_long = 20;                  // count of battery measurements

const float R1 = 33350.0;                 // ohms
const float R2 = 7480.0;                  // ohms
const float VREF = 5.12;                  // volts
const int NBITS = 3;                      // oversampling bits
const float MIN_BATTERY_VOLTAGE = 12.7;   // volts
const float MAX_BATTERY_VOLTAGE = 14.35;  // volts
const int RAW_DATA_LENGTH = 32;
const int LOW_DELTA = -10000;
const int HIGH_DELTA = 10000;

const int tics_before_inverter_start = (iverter_start_delay * (1000 / main_loop_delay));
const int tics_before_inverter_stop = (iverter_stop_delay * (1000 / main_loop_delay));
const int max_charger_work_tics = (max_charger_work_time * (1000l / main_loop_delay));
const int tics_before_charger_start = (charger_start_delay * (1000l / main_loop_delay));
const int tics_before_measurement_start = (charge_measure_delay * (1000l / main_loop_delay));
const int BITRESOLUTION = pow(2, 10 + NBITS);
const float RR = (R2 / (R1 + R2));
const int SAMPLES = (int)(pow(4, (float)NBITS) + 0.5);
const int tics_between_battery_measure = (battery_measuring_period * (1000 / main_loop_delay));
#ifdef LCD
const int tics_before_current_sensor_start = ( current_sensor_start_delay * (1000l / main_loop_delay));
const int ticks_before_chrg_off = ( current_sensor_charger_off_delay * (1000l / main_loop_delay));
const int display_tics = ( 5 * (1000l / main_loop_delay));
#endif

byte external_power_state = HIGH;
byte external_power_state_prev = HIGH;
byte led_state = 0;
unsigned int inverter_state = 0;
unsigned int charger_state = 0;
unsigned int last_change_state = 0;
unsigned int charger_working_tics = 0;
unsigned int last_battery_measure = 0;
unsigned int current_ticks_before_chrg_off = 0;
bool battery_needs_charge = false;
bool battery_needs_reload = true;
float actual_battery_voltage = 0.0;
float average_battery_voltage = 0.0;
unsigned long prev_timer = 0;
unsigned int raw_battery_level[RAW_DATA_LENGTH];
int deltas[RAW_DATA_LENGTH];
byte cursor = 0;
float avg_prev = 0;
unsigned int delta_is_ok = 0;
float delta = 0;
long delta_sum = 0;

#ifdef LCD
LiquidCrystal_I2C lcd(LCD_ADDR,  LCD_COLS, LCD_ROWS);
const uint8_t max_lrow = 4;
uint8_t c_lrow = 0;
char lrow[max_lrow][LCD_COLS+1] = {0};
uint8_t c_screen = 0;
unsigned int current_display_tics = 0;
bool warn_charger_off = false;
bool warn_charger_on = false;
uint8_t charger_mode = 0;
int8_t current_sensor_state = HIGH;
int8_t current_sensor_state_prev = HIGH;

PCF8574 ex0(EXP_ADDR);
bool is_pcf8574_ready = true;

OneButton button_red(BTN_RED);
OneButton button_grn(BTN_GREEN);

void fill_msg_buf(PGM_P s);
#endif

/*
 * без скидання MCUSR буде bootloop після спрацювання watchdog після включення живлення на чіпах PA & PB
 * що цікаво, після скидання сигналом RESET або у випадку чіпів P, такого ефекту нема.
 * Взагалі-то досить код із цієї функції помістити в функцію setup, але мануал на чіп радить так.
 */

void clr_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));

void clr_mcusr(void) {
  MCUSR = 0;
  wdt_disable();
}

void setup() {
  byte state_eeprom = STATE_UNKNOWN;
#if defined ( USE_SERIAL ) || defined ( LCD )
  PGM_P msg_booting = PSTR("Booting...");
  PGM_P msg_booted = PSTR("UPS booted");
#endif

  pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT);      //  external power sensor
  pinMode(3, OUTPUT);     //  charger relay
  pinMode(4, OUTPUT);     //  inverter relay
  digitalWrite(4, HIGH);  //  inverter relay is inverted
#ifdef USE_SERIAL
  Serial.begin(9600);
#endif
#ifdef LCD
    lcd.init();
    lcd.backlight();
    lcd.print(FPSTR(msg_booting));
    for( uint8_t i=0; i < 8; i++ ) {
      pinMode(ex0, i, INPUT_PULLUP);
    }
    button_red.attachClick(click_red);
    button_grn.attachClick(click_grn_single);
    button_grn.attachDoubleClick(click_grn_double);
    button_red.attachLongPressStart(lstart_red);
    button_grn.attachLongPressStart(lstart_grn);
    button_red.attachLongPressStop(lstop);
    button_grn.attachLongPressStop(lstop);
    #endif
#ifdef USE_SERIAL
  delay(500);
  Serial.println(FPSTR(msg_booting));
#endif
  read_battery_voltage();
  if (!is_eeprom_correct()) {
    eeprom_init();
  } else {
    state_eeprom = EEPROM.read(EEPROM_STATE_BYTE);
    if (state_eeprom == STATE_INVERTING or state_eeprom == STATE_CHARGING) {
      battery_needs_charge = true;
    }
  }
  wdt_enable(WDTO_8S);
#ifdef USE_SERIAL
  print_previous_state(state_eeprom);
  Serial.println(FPSTR(msg_booted));
#endif
#ifdef LCD
  lcd.clear();
  fill_msg_buf(msg_booted);
#endif
}

void charger_off(PGM_P s){
  digitalWrite(3, LOW);
  battery_needs_charge = false;
  EEPROM.update(EEPROM_STATE_BYTE, STATE_STANDBY);
#ifdef USE_SERIAL
  Serial.println(FPSTR(s));
#endif
#ifdef LCD
  fill_msg_buf(s);
  current_ticks_before_chrg_off = 0;
#endif
}  

void loop() {
  unsigned long current_timer = millis();
#if defined ( USE_SERIAL ) || defined ( LCD ) 
  PGM_P msg_pwr_fail = PSTR("Ext.power failed");
  PGM_P msg_pwr_restore = PSTR("Ext.power restored");
  PGM_P msg_chgr_on = PSTR("Charger ON");
  PGM_P msg_chgr_off = PSTR("Charger OFF");
  PGM_P msg_chgr_on_b = PSTR("Charger ON batt.low (");
  PGM_P msg_inv_on = PSTR("Inverter ON");
  PGM_P msg_inv_off = PSTR("Inverter OFF");
  PGM_P msg_chgr_off_over = PSTR("Charger OFF overtime");
  PGM_P msg_chgr_off_chgr = PSTR("Charger OFF charged");
  PGM_P msg_chgr_off_max = PSTR("Charger OFF max.volt.");
  PGM_P msg_timer_overflown = PSTR("Timer overflown");
  PGM_P msg_io_expander_error = PSTR("IO expander not ready");
  PGM_P msg_chgr_off_sensor = PSTR("Charger OFF by sens");
  PGM_P msg_current_detect = PSTR("Chr. current detect.");
  PGM_P msg_current_off = PSTR("Chr. current off");
 // PGM_P msg_ = PSTR();
#endif

  if (current_timer < prev_timer) {  // timer overflown
    prev_timer = 0;
#ifdef LCD
    fill_msg_buf(msg_timer_overflown);
#endif
    return;
  }

#ifdef LCD
  if ( is_pcf8574_ready ) {   // buttons
    if ( ex0.read() < 0 ) {
      is_pcf8574_ready = false;
      fill_msg_buf( msg_io_expander_error );
      return;
    } else {
      button_red.tick();
      button_grn.tick();
    }
  }
#endif
  
  if ((current_timer - prev_timer) < main_loop_delay) {
    return;
  }
  prev_timer = current_timer;

  wdt_reset();    // reset watchdog

  led_state = digitalRead(LED_BUILTIN);   // blink
  digitalWrite(LED_BUILTIN, led_state ^ HIGH);

  external_power_state = digitalRead(2);   // read and logging external power state
  if (external_power_state_prev == external_power_state) {
    last_change_state++;
  } else {
    last_change_state = 0;
    external_power_state_prev = external_power_state;
#ifdef USE_SERIAL
    if (external_power_state == LOW) Serial.println(FPSTR(msg_pwr_fail));
    else Serial.println(FPSTR(msg_pwr_restore));
#endif
#ifdef LCD
    if (external_power_state == LOW) fill_msg_buf(msg_pwr_fail);
    else fill_msg_buf(msg_pwr_restore);
#endif
  }

  inverter_state = digitalRead(4);
  charger_state = digitalRead(3);

  if (external_power_state == LOW) {  // power failed
    if (last_change_state > tics_before_inverter_start) {   // inverter ON 
      if (charger_state == HIGH) {    // charger OFF if needed
        digitalWrite(3, LOW);
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_chgr_off));
#endif
#ifdef LCD
    fill_msg_buf(msg_chgr_off);
#endif
      }
      if (inverter_state != LOW) {
        digitalWrite(4, LOW);
        EEPROM.update(EEPROM_STATE_BYTE, STATE_INVERTING);
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_inv_on));
#endif
#ifdef LCD
    fill_msg_buf(msg_inv_on);
#endif
      }
    }
  } else {  // power exists
    if (last_change_state > tics_before_inverter_stop) {    // inverter off
      if (inverter_state != HIGH) {
        digitalWrite(4, HIGH);
        battery_needs_charge = true;
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_inv_off));
#endif
#ifdef LCD
    fill_msg_buf(msg_inv_off);
#endif
      }
      if ((inverter_state == HIGH) and (charger_state == LOW) and (last_change_state > tics_before_charger_start)) {
         // charger ON
        if (battery_needs_charge) {
          digitalWrite(3, HIGH);
          charger_working_tics = 0;
          current_ticks_before_chrg_off = 0;
          delta_is_ok = 0;
          EEPROM.update(EEPROM_STATE_BYTE, STATE_CHARGING);
#ifdef USE_SERIAL
          Serial.println(FPSTR(msg_chgr_on));
#endif
#ifdef LCD
    fill_msg_buf(msg_chgr_on);
#endif
        } else {
          if (average_battery_voltage <= MIN_BATTERY_VOLTAGE) {
            digitalWrite(3, HIGH);
            battery_needs_charge = true;
            charger_working_tics = 0;
            current_ticks_before_chrg_off = 0;
            delta_is_ok = 0;
            EEPROM.update(EEPROM_STATE_BYTE, STATE_CHARGING);
#ifdef USE_SERIAL
            Serial.print(FPSTR(msg_chgr_on_b));
            Serial.print(average_battery_voltage, 3);
            Serial.println(")");
#endif
#ifdef LCD
    fill_msg_buf(msg_chgr_on_b);
#endif
          }
        }
      }
    }
  }

#ifdef LCD
  // read and logging a current sensor state
  current_sensor_state = digitalRead( ex0, CURRENT_SENSOR );
  if ( current_sensor_state > -1 and current_sensor_state != current_sensor_state_prev ){
    if ( current_sensor_state == LOW ) {
#ifdef USE_SERIAL
      Serial.println(FPSTR(msg_current_detect));
#endif
      fill_msg_buf(msg_current_detect);
    } else { 
#ifdef USE_SERIAL
      Serial.println(FPSTR(msg_current_off));
#endif
      fill_msg_buf(msg_current_off);
    }
    current_sensor_state_prev = current_sensor_state;
  }
#endif

  if (charger_state == HIGH) {   // charger OFF
    charger_working_tics++;
#ifdef LCD
    if ( ( current_sensor_state > -1 ) and ( charger_working_tics > tics_before_current_sensor_start ) and ( charger_mode != 2 ) ){
      if ( current_sensor_state == HIGH ){
        current_ticks_before_chrg_off++;
        if ( current_ticks_before_chrg_off > ticks_before_chrg_off ) {
          charger_off(msg_chgr_off_sensor);
        }
      }
    }
#endif
        
    if (charger_working_tics > max_charger_work_tics) {
      charger_off(msg_chgr_off_over);
    }
  }

  if (last_battery_measure < tics_between_battery_measure) {
    last_battery_measure++;
  } else {
    last_battery_measure = 0;
    read_battery_voltage();
    if (charger_state == HIGH) {
      if ( (charger_working_tics > tics_before_measurement_start) && is_battery_charged() ) {
        charger_off(msg_chgr_off_chgr);
      } else {
        if (average_battery_voltage >= MAX_BATTERY_VOLTAGE) {
          charger_off(msg_chgr_off_max);
        }
      }
    }
  }

#ifdef LCD
  // LCD drawing
  current_display_tics++;
  if ( current_display_tics > display_tics ) {
    refresh_lcd();
    current_display_tics = 0;
  }
#endif
} // loop()

#ifdef LCD
void refresh_lcd(){
  lcd.clear();
  if ( c_screen == 0 ) {
    lcd_print_1st_screen();
  } else {
    lcd_print_2nd_screen();
  }
  c_screen = c_screen ^ 1;
  if ( warn_charger_off ) 
    warn_charger_off = false;
  if ( warn_charger_on )
    warn_charger_on = false;
}

void lcd_print_1st_screen(){
  char stri[21] = {0};
  char sensor_state = sensor_represent(current_sensor_state);
  sprintf( stri, "%u %u %u %c %04u %04u %02u", external_power_state, (inverter_state ^ HIGH), charger_state, sensor_state,
    (int)(actual_battery_voltage * 100), (int)(average_battery_voltage * 100), cursor );
  lcd.print("L I C S  Bt   Av   ^");
  lcd.setCursor(0,1);
  lcd.print(stri);
  lcd.setCursor(0,2);
  if (charger_state == HIGH) {
    if ( charger_working_tics < tics_before_measurement_start ){
      lcd.print( "remain " );
      lcd.print( (tics_before_measurement_start - charger_working_tics) / (1000l / main_loop_delay) );
    }else{
      lcd.print("measurement");
    }
  } else {
    if ( inverter_state == LOW ) {
      lcd.print("discharge");
    } else {
      lcd.print("rest");
    }
  }
/*
  lcd.setCursor(0,3);
  memset( stri, 0, sizeof(stri) - 1 );
  sprintf( stri,  "%d %u %d", (int)(delta * 1000 + 0.5), delta_is_ok, delta_sum );
  lcd.print(stri);
*/
}

void lcd_print_2nd_screen(){
  uint8_t i;
  for ( i = 0; i < LCD_ROWS; i++ ){
    if ( strlen( lrow[i] ) == 0 ) {
      return;
    } else {
      lcd.setCursor(0,i);
      lcd.print(lrow[i]);
    }
  }
}

void lcd_print_warning(PGM_P s){
  char buf[21] = {0};
  strncpy_P( buf, s, sizeof(buf) - 1 );
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(buf);
  current_display_tics = 0;
}

char sensor_represent(int state){
  if ( state != LOW and state != HIGH )
    return('.'); 
  if ( state == HIGH ) // inverted
    return('0');
  return('1');
}

void click_red() {
  PGM_P msg_charger_now_off = PSTR("Charger off already");
  PGM_P msg_charger_will_off = PSTR("Long press 4 confirm");
  if ( charger_state == LOW) {
    lcd_print_warning(msg_charger_now_off);
  } else {
    lcd_print_warning(msg_charger_will_off);
    warn_charger_off = true;
  }
}

void click_grn_single() {
  click_grn(1);
}

void click_grn_double() {
  click_grn(2);
}

void click_grn(uint8_t cl) {
  PGM_P msg_charger_now_on = PSTR("Charger on already");
  PGM_P msg_charger_denied = PSTR("Charger denied");
  PGM_P msg_charger_will_on = PSTR("Long press 4 confirm");
  if ( external_power_state == LOW or inverter_state == LOW ) {
    lcd_print_warning(msg_charger_denied);
  } else if ( charger_state == HIGH ) {
    lcd_print_warning(msg_charger_now_on);
  } else {
    lcd_print_warning(msg_charger_will_on);
    warn_charger_on = true;
    charger_mode = cl;
  }
}

void lstart_red() {
  PGM_P msg_charger_now_off = PSTR("Charger off already");
  PGM_P msg_charger_warn = PSTR("         NO");
  PGM_P msg_charger_goes_off = PSTR("Charger goes OFF");
  PGM_P msg_chgr_off = PSTR("Charger OFF manually");
  if ( ! warn_charger_off ) {
    lcd_print_warning(msg_charger_warn);
  } else {
    if ( charger_state == LOW) {
      lcd_print_warning(msg_charger_now_off);
    } else {
      lcd_print_warning(msg_charger_goes_off);
      charger_off(msg_chgr_off);
    }
  }
}

void lstart_grn() {
  PGM_P msg_charger_now_on = PSTR("Charger on already");
  PGM_P msg_charger_warn = PSTR("         NO");
  PGM_P msg_charger_denied = PSTR("Charger denied");
  PGM_P msg_charger_goes_on = PSTR("Charger goes ON");
  PGM_P msg_chgr_on1 = PSTR("Charger ON autoOff");
  PGM_P msg_chgr_on2 = PSTR("Charger ON contin.");
  if ( ! warn_charger_on ) {
    lcd_print_warning(msg_charger_warn);
  } else {  
    if ( external_power_state == LOW or inverter_state == LOW ) {
      lcd_print_warning(msg_charger_denied);
    } else if ( charger_state == HIGH ) {
      lcd_print_warning(msg_charger_now_on);
    } else {
      lcd_print_warning(msg_charger_goes_on);
      digitalWrite(3, HIGH);
      battery_needs_charge = true;
      charger_working_tics = 0;
      current_ticks_before_chrg_off = 0;
      delta_is_ok = 0;
      if ( charger_mode == 2 ) {
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_chgr_on2));
#endif
        fill_msg_buf(msg_chgr_on2);
      } else {
        EEPROM.update(EEPROM_STATE_BYTE, STATE_CHARGING);
#ifdef USE_SERIAL
      Serial.println(FPSTR(msg_chgr_on1));
#endif
        fill_msg_buf(msg_chgr_on1);
      }
    }
  }
}

void lstop(){
  current_display_tics = 0;
  c_screen = 0;
  refresh_lcd();
}

void fill_msg_buf(PGM_P s){
  uint8_t i;
  if ( c_lrow > 0 ) {
    for ( i=c_lrow; i > 0; i-- ){
      memset( lrow[i], 0, sizeof(lrow[i]-1 ) );
      strncpy( lrow[i], lrow[i-1], sizeof(lrow[i]) - 1 );
    }
  }
  memset( lrow[0], 0, sizeof(lrow[0]) - 1 );
  strncpy_P( lrow[0], s, sizeof(lrow[0]) - 1 );
  
  if ( c_lrow  < ( max_lrow - 1 ) ) {
    c_lrow++;
  }
}

int digital_read_wrapper(int pin){ // IDK how to make this properly
  return(digitalRead( ex0, pin ));
}
#endif

void read_battery_voltage() {
  long dv = 0;
  long bl = 0;
  byte i = 0;
  float avg = 0;
#if defined ( USE_SERIAL ) || defined ( LCD ) 
  PGM_P msg_bat_dis = PSTR("Battery disconnected");
#endif

  // there is oversampling
  for (i = 0; i < 10; i++)
    for (int j = 0; j < SAMPLES; j++)
      dv += analogRead(A0);
  dv = (dv / 10);
  dv = dv >> NBITS;
  actual_battery_voltage = (((float)dv * VREF) / BITRESOLUTION) / RR;

  if (actual_battery_voltage < 3.0) {
#ifdef USE_SERIAL
    Serial.println(FPSTR(msg_bat_dis));
#endif
#ifdef LCD
    fill_msg_buf(msg_bat_dis);
#endif
    battery_needs_charge = true;
    battery_needs_reload = true;
    return;
  }

  if (battery_needs_reload) {
    for (i = 0; i < RAW_DATA_LENGTH; i++) {
      raw_battery_level[i] = dv;
      deltas[i] = 0;
    }
    average_battery_voltage = actual_battery_voltage;
    battery_needs_reload = false;
    avg_prev = dv;
   } else {
    raw_battery_level[cursor] = dv;

    for (i = 0; i < RAW_DATA_LENGTH; i++) {
      bl += raw_battery_level[i];
    }
    avg = (float)bl / RAW_DATA_LENGTH;
    average_battery_voltage = ((avg * VREF) / BITRESOLUTION) / RR;

    delta = avg - avg_prev;
    avg_prev = avg;
    deltas[cursor] = (int)(delta * 1000 + 0.5);

    cursor++;
    if (cursor >= RAW_DATA_LENGTH) {
      cursor = 0;
    }
  }
  /*
#ifdef USE_SERIAL
  Serial.print(actual_battery_voltage,3);
  Serial.print(" ");
  Serial.print(average_battery_voltage,3);
  Serial.print(" ");
  Serial.println(delta_sum);
#endif
*/
}

bool is_battery_charged() {
  delta_sum = 0;

  for (byte i = 0; i < RAW_DATA_LENGTH; i++) {
    delta_sum += deltas[i];
  }
  /*
#ifdef USE_SERIAL
  Serial.print(delta_is_ok);
  Serial.print(" ");
  Serial.print(actual_battery_voltage,3);
  Serial.print(" ");
  Serial.print(average_battery_voltage,3);
  Serial.print(" ");
  Serial.println(delta_sum);
#endif
*/
  if ((delta_sum < LOW_DELTA) or (delta_sum > HIGH_DELTA)) {
    delta_is_ok = 0;
    return (false);
  }
  delta_is_ok++;
  if (delta_is_ok >= delta_series_long) {
    return (true);
  }
  return (false);
}

bool is_eeprom_correct() {
#if defined ( USE_SERIAL ) || defined ( LCD ) 
  PGM_P msg_eeprom_ok = PSTR("EEPROM correct");
  PGM_P msg_eeprom_bad = PSTR("EEPROM invalid");
#endif
  if (EEPROM.read(EEPROM_MARK_BYTE) == 0x55 and EEPROM.read(EEPROM_MARK_BYTE + 1) == 0xaa) {
#ifdef USE_SERIAL
    Serial.println(FPSTR(msg_eeprom_ok));
#endif
    return (true);
  }
#ifdef USE_SERIAL
  Serial.println(FPSTR(msg_eeprom_bad));
#endif
#ifdef LCD
    fill_msg_buf(msg_eeprom_bad);
#endif
  return (false);
}

bool eeprom_init() {
#if defined ( USE_SERIAL ) || defined ( LCD ) 
  PGM_P msg_eeprom_init = PSTR("EEPROM initialized");
#endif
  for (byte i = 0; i < EEPROM_MARK_BYTE; i++) {
    EEPROM.update(i, 0);
  }
  EEPROM.update(EEPROM_MARK_BYTE, 0x55);
  EEPROM.update(EEPROM_MARK_BYTE + 1, 0xaa);
#ifdef USE_SERIAL
  Serial.println(FPSTR(msg_eeprom_init));
#endif
#ifdef LCD
    fill_msg_buf(msg_eeprom_init);
#endif
  return (is_eeprom_correct());
}

#ifdef USE_SERIAL
void print_previous_state(byte state_eeprom) {
  PGM_P msg_prev_state = PSTR("Previous state is ");
  PGM_P msg_st_unkn = PSTR("unknown");
  PGM_P msg_st_stndb = PSTR("standby");
  PGM_P msg_st_inv = PSTR("inverting");
  PGM_P msg_st_chrg = PSTR("charging");
  PGM_P msg_st_undef = PSTR("not defined (");
  
  Serial.print(FPSTR(msg_prev_state));
  switch (state_eeprom) {
    case STATE_UNKNOWN:
      Serial.println(FPSTR(msg_st_unkn));
      break;
    case STATE_STANDBY:
      Serial.println(FPSTR(msg_st_stndb));
      break;
    case STATE_INVERTING:
      Serial.println(FPSTR(msg_st_inv));
      break;
    case STATE_CHARGING:
      Serial.println(FPSTR(msg_st_chrg));
      break;
    default:
      Serial.print(FPSTR(msg_st_undef));
      Serial.print(state_eeprom);
      Serial.println(")");
      break;
  }
}
#endif
