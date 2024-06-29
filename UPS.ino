#define USE_SERIAL
#define LCD

#include <avr/wdt.h>
#include <EEPROM.h>

#ifdef LCD
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
// LCD geometry
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
#endif

#define FPSTR(pstr) (const __FlashStringHelper*)(pstr)  // some compatibility with esp8266/esp32

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
const int max_charger_work_time = (4 * 3600 * 3);  // s
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
const int BITRESOLUTION = pow(2, 10 + NBITS);
const float RR = (R2 / (R1 + R2));
const int SAMPLES = (int)(pow(4, (float)NBITS) + 0.5);
const int tics_between_battery_measure = (battery_measuring_period * (1000 / main_loop_delay));

byte external_power_state = HIGH;
byte external_power_state_prev = HIGH;
byte led_state = 0;
unsigned int inverter_state = 0;
unsigned int charger_state = 0;
unsigned int last_change_state = 0;
unsigned int charger_working_tics = 0;
unsigned int last_battery_measure = 0;
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

#ifdef LCD
LiquidCrystal_I2C lcd(LCD_ADDR,  LCD_COLS, LCD_ROWS);
char lrow[3][20] = {0};
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
  //int lcd_status = 0;
  PGM_P msg_booting = PSTR("Booting...");
  PGM_P msg_booted = PSTR("UPS booted");

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
    lcd.print(FPSTR(msg_booted));
#endif
}

void loop() {
  unsigned long current_timer = millis();
  PGM_P msg_pwr_fail = PSTR("External power failed");
  PGM_P msg_pwr_restore = PSTR("External power restored");
  PGM_P msg_chgr_on = PSTR("Charger ON");
  PGM_P msg_chgr_off = PSTR("Charger OFF");
  PGM_P msg_chgr_on_b = PSTR("Charger ON - battery discharged (");
  PGM_P msg_inv_on = PSTR("Inverter ON");
  PGM_P msg_inv_off = PSTR("Inverter OFF");
  PGM_P msg_chgr_off_over = PSTR("Charger OFF by overtime");
  PGM_P msg_chgr_off_chgr = PSTR("Charger OFF - battery charged");
  PGM_P msg_chgr_off_max = PSTR("Charger OFF - battery reached max. voltage");
  // PGM_P msg_ = PSTR();

  if (current_timer < prev_timer) {  // timer overflown
    prev_timer = 0;
    return;
  }

  if ((current_timer - prev_timer) < main_loop_delay) {
    return;
  }
  prev_timer = current_timer;

  wdt_reset();

  led_state = digitalRead(LED_BUILTIN);
  digitalWrite(LED_BUILTIN, led_state ^ HIGH);

  external_power_state = digitalRead(2);
  if (external_power_state_prev == external_power_state) {
    last_change_state++;
  } else {
    last_change_state = 0;
    external_power_state_prev = external_power_state;
#ifdef USE_SERIAL
    if (external_power_state == LOW) Serial.println(FPSTR(msg_pwr_fail));
    else Serial.println(FPSTR(msg_pwr_restore));
#endif
  }

  inverter_state = digitalRead(4);
  charger_state = digitalRead(3);
  if (external_power_state == LOW) {  // power failed
    if (last_change_state > tics_before_inverter_start) {
      if (charger_state == HIGH) {
        digitalWrite(3, LOW);
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_chgr_off));
#endif
      }
      if (inverter_state != LOW) {
        digitalWrite(4, LOW);
        EEPROM.update(EEPROM_STATE_BYTE, STATE_INVERTING);
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_inv_on));
#endif
      }
    }
  } else {  // power exists
    if (last_change_state > tics_before_inverter_stop) {
      if (inverter_state != HIGH) {
        digitalWrite(4, HIGH);
        battery_needs_charge = true;
#ifdef USE_SERIAL
        Serial.println(FPSTR(msg_inv_off));
#endif
      }
      if ((inverter_state == HIGH) and (charger_state == LOW) and (last_change_state > tics_before_charger_start)) {
        if (battery_needs_charge) {
          digitalWrite(3, HIGH);
          charger_working_tics = 0;
          delta_is_ok = 0;
          EEPROM.update(EEPROM_STATE_BYTE, STATE_CHARGING);
#ifdef USE_SERIAL
          Serial.println(FPSTR(msg_chgr_on));
#endif
        } else {
          if (average_battery_voltage <= MIN_BATTERY_VOLTAGE) {
            digitalWrite(3, HIGH);
            battery_needs_charge = true;
            charger_working_tics = 0;
            delta_is_ok = 0;
            EEPROM.update(EEPROM_STATE_BYTE, STATE_CHARGING);
#ifdef USE_SERIAL
            Serial.print(msg_chgr_on_b);
            Serial.print(average_battery_voltage, 3);
            Serial.println(")");
#endif
          }
        }
      }
    }
  }

  if (charger_state == HIGH) {
    charger_working_tics++;
    if (charger_working_tics > max_charger_work_tics) {
      digitalWrite(3, LOW);
      battery_needs_charge = false;
      EEPROM.update(EEPROM_STATE_BYTE, STATE_STANDBY);
#ifdef USE_SERIAL
      Serial.println(FPSTR(msg_chgr_off_over));
#endif
    }
  }

  if (last_battery_measure < tics_between_battery_measure) {
    last_battery_measure++;
  } else {
    last_battery_measure = 0;
    read_battery_voltage();
    if (charger_state == HIGH) {
      if (is_battery_charged()) {
        digitalWrite(3, LOW);
        battery_needs_charge = false;
        EEPROM.update(EEPROM_STATE_BYTE, STATE_STANDBY);
#ifdef USE_SERIAL
        Serial.println(msg_chgr_off_chgr);
#endif
      } else {
        if (average_battery_voltage >= MAX_BATTERY_VOLTAGE) {
          digitalWrite(3, LOW);
          battery_needs_charge = false;
          EEPROM.update(EEPROM_STATE_BYTE, STATE_STANDBY);
#ifdef USE_SERIAL
          Serial.println(msg_chgr_off_max);
#endif
        }
      }
    }
  }
#ifdef LCD
  refresh_lcd();
#endif
} // loop()


#ifdef LCD
void refresh_lcd(){
    lcd.setCursor(0,0);
    lcd.print(" L I C  Bt   Av   ^");
    lcd.setCursor(0,1);
    lcd.print(" 1 0 0 xxxx xxxx ");
    lcd.print(cursor);
    lcd.setCursor(0,3);
    //lcd.print();
}

void fill_status_lcd(char *){

}
#endif

void read_battery_voltage() {
  long dv = 0;
  long bl = 0;
  byte i = 0;
  float avg = 0;
  float delta = 0;

  // there is oversampling
  for (i = 0; i < 10; i++)
    for (int j = 0; j < SAMPLES; j++)
      dv += analogRead(A0);
  dv = (dv / 10);
  dv = dv >> NBITS;
  actual_battery_voltage = (((float)dv * VREF) / BITRESOLUTION) / RR;

  if (actual_battery_voltage < 3.0) {
#ifdef USE_SERIAL
    Serial.println("Battery disconnected");
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
  long delta_sum = 0;

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
  if (EEPROM.read(EEPROM_MARK_BYTE) == 0x55 and EEPROM.read(EEPROM_MARK_BYTE + 1) == 0xaa) {
#ifdef USE_SERIAL
    Serial.println("EEPROM correct");
#endif
    return (true);
  }
#ifdef USE_SERIAL
  Serial.println("EEPROM invalid");
#endif
  return (false);
}

bool eeprom_init() {
  for (byte i = 0; i < EEPROM_MARK_BYTE; i++) {
    EEPROM.update(i, 0);
  }
  EEPROM.update(EEPROM_MARK_BYTE, 0x55);
  EEPROM.update(EEPROM_MARK_BYTE + 1, 0xaa);
#ifdef USE_SERIAL
  Serial.println("EEPROM initialized");
#endif
  return (is_eeprom_correct());
}

#ifdef USE_SERIAL
void print_previous_state(byte state_eeprom) {
  Serial.print("Previous state is ");
  switch (state_eeprom) {
    case STATE_UNKNOWN:
      Serial.println("unknown");
      break;
    case STATE_STANDBY:
      Serial.println("standby");
      break;
    case STATE_INVERTING:
      Serial.println("inverting");
      break;
    case STATE_CHARGING:
      Serial.println("charging");
      break;
    default:
      Serial.print("not defined (");
      Serial.print(state_eeprom);
      Serial.println(")");
      break;
  }
}
#endif
