
#include <FlashStorage_SAMD.h>
#include <FlashStorage_SAMD21.h>
#include <Arduino.h>
//#include <u8x8lib.h>
#include <EasyButton.h>
#include <TimerTC3.h>
#include "DFRobot_RGBLCD.h"
//#include <LiquidCrystal_PCF8574.h>



//u8x8_SSD1306_128X64_NONAME_HW_I2C //u8x8(/* reset=*/ //u8x8_PIN_NONE);
//LiquidCrystal_PCF8574 lcd(0x27);
DFRobot_RGBLCD lcd(16,2);  //16 characters and 2 lines of show
// //u8x8_SSD1306_128X64_NONAME_SW_I2C //u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ //u8x8_PIN_NONE);   // OLEDs without Reset of the Display

// Define pins used
int left_gate_pin = 10;    // the number of the pin for left gate
int right_gate_pin = 11;  // number of pin for right gate
int i = 0;
int ledPin[2] =  {5, 6}; //LEFT, RIGHT
int ledState[2] = {LOW, LOW};
#define speakerPin A3
int interupt_test_pin = 12;
const int initcolorR = 255;
const int initcolorG = 255;
const int initcolorB = 255;
const int runcolorR = 120;
const int runcolorG = 255;
const int runcolorB = 120;
const int resultcolorR = 255;
const int resultcolorG = 255;
const int resultcolorB = 255;


int debounce = 50;
bool pullup = true;
bool invert = true;
int gate_high_delay = 200;
int presses_init = 1;
int timeout = 2000;
int duration = 2000;
#define VBATPIN A7
#define sound_length 28
#define screen_period 800
#define display_wait 200
unsigned long  battery_wait = 500;
#define run_end_delay 3000
#define led_interval 250 //time to leave the interval 
unsigned long inital_reset_delay = 2500; //in 100 seconds so 6000 = 60 seconds 2500 = 25 seconds
unsigned long reset_delay = 2500;


// Variables user in timng will change:
uint8_t presses = presses_init;
unsigned long time_now = 0;
unsigned long time_now_screen = 0;
unsigned long time_now_battery = 0;
unsigned long start_time[11];
unsigned long stop_time[11];
unsigned long last_stop_time = 0;
unsigned long last_start_time = 0;
unsigned long reset_start;
unsigned long reset_stop;
unsigned long last_display_time;
int n_start_time = 0;
int n_stop_time = 0;
unsigned long run_time;
int run_time_seconds;
int run_time_fraction;
bool run_once = false;
bool run_once_time_out = false;
//Control the backlight
unsigned long batt_last_read = 12000;
unsigned long back_light_on = 0;
bool debug = false;


//Menu status
int MenuId = 0;


unsigned long previousMillis [3] = {0, 0, 0};
unsigned long currentMillis [3] = {0, 0, 0};


EasyButton left_gate(left_gate_pin, debounce, pullup, invert);
EasyButton right_gate(right_gate_pin, debounce, pullup, invert);

FlashStorage(presses_storage, int);
FlashStorage(reset_delay_storage, unsigned long);
void setup() {
  if (debug) {
    while (! Serial);
  }
  if (Serial) Serial.begin(9600);
  if (Serial) Serial.println(F("Starting New Version...."));
  Serial.setTimeout(100000);
  tone(A3, 600, 200);
  // initialize the LED pin as an output:
  left_gate.begin();
  right_gate.begin();
  //u8x8.begin();
  //u8x8.setFlipMode(1);   // set number from 1 to 3, the screen word will rotary 180
  //u8x8.setFont(//u8x8_font_chroma48medium8_r);
  //u8x8.setCursor(0, 0);
  //u8x8.clearDisplay();
  //u8x8.println("Ready");
  //lcd.begin(16, 2); // initialize the lcd
  lcd.init();
  lcd.setRGB(initcolorR, initcolorG, initcolorB);
  //lcd.setBacklight(20); //turn on backlight

  lcd.home(); lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Ready");
  //lcd.print(10);
  pinMode(speakerPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // set the digital pin as output:
  pinMode(ledPin[0], OUTPUT);
  pinMode(ledPin[1], OUTPUT);
  //pinMode(ledPin[2], OUTPUT);
  tone(A3, 600, 200);
  pinMode(interupt_test_pin, OUTPUT);
  pinMode(left_gate_pin, INPUT_PULLUP);
  pinMode(right_gate_pin, INPUT_PULLUP);
  //pinMode(back_light_enable, INPUT_PULLUP);
  //digitalWrite(bat_read_pin, HIGH);
  delay(1000);
  //get setting stored in flash ram
  reset_delay = reset_delay_storage.read();
  if (reset_delay == 0)reset_delay = inital_reset_delay;
  lcd.setCursor(0, 0);
  lcd.print("Run delay:"); lcd.print(reset_delay); lcd.print("msec");

  presses = presses_storage.read();
  if (presses == 0) presses = presses_init;
  lcd.setCursor(0, 1);
  lcd.print("Trig pulses:"); lcd.print(presses);
  // Add the callback function to be called when the button is pressed for at least the given time.
  left_gate.onSequence(presses, timeout, left_gate_triggered);
  right_gate.onSequence(presses, timeout, right_gate_triggered);
  delay(2000);
  lcd.home(); lcd.clear();
  lcd.setCursor(0, 0);


  // user a timer interup to update the gates every 100 mseconds
  //TimerTc3.initialize(100000);
  //TimerTc3.attachInterrupt(read_gates);
}

void loop() {
  time_now = millis();
  read_gates();
  //if (Serial) Serial.println(start_time[0]);
  //if (Serial) Serial.println(n_start_time);
  //if (Serial) Serial.println(time_now);

  // IDisplay the time every second following the start of the run from the right
  if (n_stop_time > 0 && n_start_time == 0 && (time_now > last_display_time + display_wait))
  {
    run_from_right();
  }
  //function checks for a run from the left gate and stores the data
  if (n_start_time > 0 && n_stop_time == 0 && (time_now > last_display_time + display_wait))
  {
    run_from_left();
  }
  // Once we have a start time and a stop time diplay the run time
  if (n_stop_time > 0 && n_start_time > 0  && run_once == false)
  {
    display_run_time();
  }
  //
  //resent the counters ready for the next run
  if (n_stop_time > 0 && n_start_time > 0 && (time_now > last_display_time + run_end_delay))
  {
    reset_gates();
  }
  //If start but no 40 seconds passed then reset the counters.
  if (run_time > reset_delay && run_once_time_out == false) {
    time_out_reset();
  }
  turn_off_leds();
  if (Serial) MenuId = menu(); // call menu and display status = 0 sucess
  // read the state of the pushbutton value:
  battery_read(); //read and diaplay the battery voltage
}

void left_gate_triggered() //call back from easy button
{
  //Serial.println("left Gate Triggered");
  digitalWrite(LED_BUILTIN, HIGH);
  ////u8x8.setCursor(0, 3);
  ////u8x8.print("Left");;
  //tone(speakerPin, frequency,led_interval);
  tone(A3, 600, 200);
  start_time[n_start_time] = millis();
  ledState[0] = (HIGH);
  digitalWrite(ledPin[0], ledState[0]);
  previousMillis[0] = millis();
  //lcd.setBacklight(60); //turn on back light
  lcd.setRGB(runcolorR, runcolorG, runcolorB);
  back_light_on = millis();
  //if (Serial) Serial.println(n_start_time); if (Serial) Serial.print(start_time[n_start_time]);

  if (n_start_time < 9)
    n_start_time = n_start_time + 1;
  else n_start_time = 9;
}

void right_gate_triggered() //call back from easy button
{
  //Serial.println("Right Gate Triggered");
  digitalWrite(LED_BUILTIN, HIGH);
  ////u8x8.setCursor(6, 3);
  ////u8x8.print("Right");;
  //tone(speakerPin, frequency,led_interval);
  //tone(ledPin[2], frequency,led_interval);
  tone(A3, 440, 200);
  stop_time[n_stop_time] = millis();
  ledState[1] = (HIGH);
  digitalWrite(ledPin[1], ledState[1]);
  previousMillis[1] = millis();
  //lcd.setBacklight(125); //turn on back light
  lcd.setRGB(runcolorR, runcolorG, runcolorB);
  back_light_on = millis();
  //if (Serial) Serial.println(n_stop_time);
  //if (Serial) Serial.println(stop_time[n_stop_time]);
  if (n_stop_time < 9)
    n_stop_time = n_stop_time + 1;
  else n_stop_time = 9;
}


void read_gates()
{
  digitalWrite(interupt_test_pin, HIGH);
  left_gate.read();
  right_gate.read();

}

void run_from_left()
//function checks for a run from the left gate and stores the data
{
  time_now = millis();
  //if (Serial) Serial.println(start_time[0]);
  run_time = (time_now - start_time[0]) / 10;
  run_time_seconds = run_time / 100;
  run_time_fraction = run_time - (run_time_seconds * 100);
  //if (Serial) Serial.println(F("Run Time from Left...."));
  //if (Serial) Serial.print(run_time_seconds);
  //if (Serial) Serial.print(".");
  //if (Serial) Serial.println(run_time_fraction);
  lcd.setCursor(0, 0);
  lcd.print("Run from left...");
  lcd.setCursor(0, 1);
  lcd.print(run_time_seconds);
  lcd.print(".");
  lcd.print(run_time_fraction);
  lcd.print("  ");
  //u8x8.clearLine(0);
  //u8x8.setCursor(0, 0);
  //u8x8.print("Run from left");
  //u8x8.clearLine(1);
  //u8x8.clearLine(2);
  //u8x8.setCursor(0, 1);
  //u8x8.print(run_time_seconds);
  //u8x8.print(".");
  //u8x8.print(run_time_fraction);
  //u8x8.print("  ");
  last_display_time = time_now;
  run_once_time_out = false;
}

void run_from_right()
{
  time_now = millis();
  //if (Serial) Serial.println(start_time[0]);
  run_time = (time_now - stop_time[0]) / 10;
  run_time_seconds = run_time / 100;
  run_time_fraction = run_time - (run_time_seconds * 100);
  //if (Serial) Serial.println(F("Run Time from Right"));
  //if (Serial) Serial.print(run_time_seconds);
  //if (Serial) Serial.print(".");
  //if (Serial) Serial.println(run_time_fraction);
  lcd.setCursor(0, 0);
  lcd.print("Run from right..");
  lcd.setCursor(0, 1);
  lcd.print(run_time_seconds);
  lcd.print(".");
  lcd.print(run_time_fraction);
  lcd.print("  ");
  //u8x8.clearLine(0);
  //u8x8.setCursor(0, 0);
  //u8x8.print("Run from right");
  //u8x8.clearLine(1);
  //u8x8.clearLine(2);
  //u8x8.setCursor(0, 1);
  //u8x8.print(run_time_seconds);
  //u8x8.print(".");
  //u8x8.print(run_time_fraction);
  //u8x8.print("  ");
  last_display_time = time_now;
  run_once_time_out = false;
}


void display_run_time()
{
  // calculate and display the result
  //if (Serial) Serial.println(F("Start time...."));
  //if (Serial) Serial.println(start_time[0]);
  //if (Serial) Serial.println(F("Stop time...."));
  //if (Serial) Serial.println(stop_time[0]);
  if (stop_time[0] > start_time[0])
  {
    run_time = (stop_time[0] - start_time[0]) / 10;
  }
  else
  {
    run_time = (start_time[0] - stop_time[0]) / 10;
  }
  run_time_seconds = run_time / 100;
  run_time_fraction = run_time - (run_time_seconds * 100);
  if (Serial) Serial.print(F("Run Time: "));
  //u8x8.setCursor(0, 0);
  //u8x8.clearLine(0);
  //u8x8.print("Run Time");
  if (Serial) Serial.print(run_time_seconds);
  if (Serial) Serial.print(".");
  if (Serial) Serial.println(run_time_fraction);
  //u8x8.clearLine(1);
  //u8x8.setCursor(0, 1);
  //u8x8.print(run_time_seconds);
  //u8x8.print(".");
  //u8x8.print(run_time_fraction);
  //u8x8.print(" ");
  lcd.setRGB(resultcolorR, resultcolorG, resultcolorB);

  lcd.setCursor(0, 0);
  lcd.print("Run Time            "); lcd.setCursor(0, 1);
  lcd.print(run_time_seconds);
  lcd.print(".");
  lcd.print(run_time_fraction);
  lcd.print(" ");
  run_once = true;
}

void reset_gates() {
  n_stop_time = 0;
  n_start_time = 0;
  run_once = false;
  tone(A3, 350, 400);
  previousMillis[2] = millis();
  ledState[0] = (HIGH);
  digitalWrite(ledPin[0], ledState[0]);
  previousMillis[0] = millis();
  ledState[1] = (HIGH);
  digitalWrite(ledPin[1], ledState[1]);
  digitalWrite(LED_BUILTIN, LOW);
  previousMillis[1] = millis();
  lcd.setCursor(0, 0);
  lcd.print("Next Run Ready");
  //lcd.setBacklight(125); //turn on back light
  lcd.setRGB(resultcolorR, resultcolorG, resultcolorB);
  back_light_on = millis();
  //u8x8.setCursor(0, 0);
  //u8x8.clearLine(0);
  //u8x8.print("Ready For Next");
}

void time_out_reset()
{
  //if (Serial) Serial.println(start_time[0]);
  //if (Serial) Serial.println(F("Time Out Reset........."));
  lcd.setCursor(0, 0);
  lcd.print("Time Out Reset..");
  //u8x8.clearLine(0);
  //u8x8.setCursor(0, 0);
  //u8x8.print("Time Out Reset");
  n_stop_time = 0;
  n_start_time = 0;
  run_once_time_out = true;
  tone(A3, 350, 800);
  previousMillis[2] = millis();
  ledState[0] = (HIGH);
  digitalWrite(ledPin[0], ledState[0]);
  previousMillis[0] = millis();
  ledState[1] = (HIGH);
  digitalWrite(ledPin[1], ledState[1]);
  previousMillis[1] = millis();
  //lcd.setBacklight(60); //turn on back light
  lcd.setRGB(resultcolorR, resultcolorG, resultcolorB);
  back_light_on = millis();
}
void turn_off_leds()
{
  // Turns off the status LEDs and the Buzzer after a wait time.
  for (i = 0; i <= 1; i++) {
    currentMillis [i] = millis();
    if (currentMillis[i] - previousMillis[i] >= led_interval) {
      // save the last time you blinked the LED
      // if the LED is off turn it on and vice-versa:
      if (ledState [i] == LOW) {
        ledState [i] = LOW;
      } else {
        ledState [i] = LOW;
      }
      // set the LED with the ledState of the variable:
      digitalWrite(ledPin[i], ledState[i]);
    }
  }
  if (millis() > time_now_screen + screen_period) {
    time_now_screen = millis();
    //u8x8.clearLine(5);
  }
}

void battery_read()
{
  if (millis() > (time_now_battery +  battery_wait ))
  {
    time_now_battery = millis();
    int measuredvbat = analogRead(VBATPIN);
    //measuredvbat *= 2;    // we divided by 2, so multiply back
    //measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage minimun 2 volts
    //measuredvbat /= 1024; // convert to voltage
    // 155 per volt vmin (3.2v) = 496 vmax (4.2v) = 651.
    // Battry Charge = 100*(measuredvbat - vmin) / (vmax-vmin) = (measuredvbat - 496)/ 155
    //  max 100*(651 - 496) / 155 = 100
    // 100 (496 - 496) / 155 =0
    measuredvbat = 100 * (measuredvbat - 496) / 155;
    lcd.setCursor(12, 1);
    //lcd.print("VBat:" );
    lcd.print(measuredvbat); lcd.print("%" );
  }
}

int menu() {
  int menuId = 0;
  if (Serial.available() > 0 ) {
    switch (Serial.read())
    {
      case '1':
        flushRx();
        settings();
        menuId = 1;
        break;

      case '2':
        flushRx();
        readAndStorePresses();
        menuId = 2;
        break;

      case '3':
        flushRx();
        readAndStoreRunTimeout();
        menuId = 3;
        break;

      default:
        flushRx();
        Serial.println();
        Serial.println("Timing gate Control Menu");
        Serial.println("-------------------------------------");
        Serial.println("1) Display Setting");
        Serial.println("2) Change number of triggers to set of gate");
        Serial.println("3) Change run timeout");
        menuId = 0;
        break;
    }
  }
  return menuId;
}
void settings() {
  Serial.println();
  Serial.println("Settings");
  Serial.println("-------------------------------------");
  Serial.print("Number of triggers (legs) to set off gate: "); Serial.println(presses);
  int timeout_sec = timeout / 1000;
  Serial.print("Time out delay for start or end gate     : "); Serial.print(timeout_sec); Serial.println(" sec");
  int reset_delay_sec = reset_delay / 100;
  Serial.print("Time out to reset run                    : "); Serial.print(reset_delay_sec); Serial.println(" sec");
}
void readAndStorePresses()
{ uint8_t new_presses = 0;
  Serial.println("Please number of presses required between 1-5:");
  do {
    if (Serial.available() > 0) {
      String new_presses_string = Serial.readStringUntil('\n');
      new_presses = new_presses_string.toInt();
    }
  } while (new_presses == 0);
  presses_storage.write(new_presses);  // <-- save the new number of presses
  presses = new_presses;
  Serial.print("New presses: "); Serial.print(presses); Serial.println(" reset to activate");
}

void readAndStoreRunTimeout()
{ uint8_t new_run_timeout = 0;
  Serial.println("Please maximum expected run time before reset (90 m track = 25 Polish = 120");
  do {
    if (Serial.available() > 0) {
      String new_run_timeout_string = Serial.readStringUntil('\n');
      new_run_timeout = new_run_timeout_string.toInt();
    }
  } while (new_run_timeout == 0);
  reset_delay = new_run_timeout * 100;
  reset_delay_storage.write(reset_delay);  // <-- save the new number of presses
  Serial.print("New reset: "); Serial.print(new_run_timeout); Serial.println(" reset to activate");
}

void flushRx() {
  while (Serial.available()) {
    Serial.read();
    delay(1);
  }
}
