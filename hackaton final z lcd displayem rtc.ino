#include <elapsedMillis.h> //load the library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <Servo.h>
#define LED_PIN     7
#define NUM_LEDS    1
#include "RTClib.h"
#include <TM1637Display.h>

#define CLK 6
#define DIO 5

RTC_DS3231 rtc;
TM1637Display display = TM1637Display(CLK, DIO);

CRGB leds[NUM_LEDS];
Servo servo;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
elapsedMillis sensors;//Create an Instance
elapsedMillis staty;//Create an Instance
elapsedMillis buttonms;//Create an Instance
elapsedMillis lcdms;//Create an Instance


int red_light_pin= 11;
int green_light_pin = 10;
int blue_light_pin = 9;


const int buttonPin = 3;
int counter = 0;
float temp;
float humidity;
int pressure;
int particles;
int srparticles;
int opary;

//-------------------DHT-------------------
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 2
#define DHTTYPE    DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);
//---------------DUST-------------------
#include <GP2YDustSensor.h>
const uint8_t SHARP_LED_PIN = 12;   // Sharp Dust/particle sensor Led Pin
const uint8_t SHARP_VO_PIN = A3;    // Sharp Dust/particle analog out pin used for reading
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, SHARP_LED_PIN, SHARP_VO_PIN);

//-----------BAR-----------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

int sensorValue;






void setup() {
  Serial.begin(9600);
  servo.attach(8);
  servo.write(15);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(buttonPin, INPUT);
  //-----------------DHT------------------
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  //-----------------LCD-------------------
  lcd.init();
  lcd.backlight();

  //------------------DUST----------------
  //dustSensor.setBaseline(0.4); // set no dust voltage according to your own experiments
  //dustSensor.setCalibrationFactor(1.1); // calibrate against precision instrument
  dustSensor.begin();

  //-------------------BAR----------------
  bmp.begin();

  //--------------------------------------
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);

if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // Check if the RTC lost power and if so, set the time:
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // The following line sets the RTC to the date & time this sketch was compiled:
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // Set the display brightness (0-7):
  display.setBrightness(5);
  // Clear the display:
  display.clear();



}


void loop() {
  if (counter >= 5)
  {
    counter = 0;
  }
  //-----------------GUZIK--------------------------------
  if (buttonms > 750 && digitalRead(buttonPin) == HIGH)
  {
    delay(250);
    counter++;
    lcd.clear();
  }
  //-----------------KONRTOKLI--------------------------------
  if (staty> 500)
  {
  if (srparticles >= 180) {
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    servo.write(0);
    RGB_color(255, 0, 0);
  }
  else if (srparticles >= 130) {
    leds[0] = CRGB(0, 255, 255);
    FastLED.show();
    servo.write(50);
    RGB_color(0, 255, 255);
  }
  else if (srparticles >= 70) {
    leds[0] = CRGB(0, 0, 255);
    FastLED.show();
    servo.write(100);
    RGB_color(0, 0, 255);
  }
  else if (srparticles >= 25) {
    leds[0] = CRGB(150, 255, 0);
    FastLED.show();
    servo.write(140);
    RGB_color(0, 255, 0);
  }
  else {
    leds[0] = CRGB(0, 255, 0);
    FastLED.show();
    servo.write(180);
    RGB_color(0, 0, 0);
  }
  staty=0;
  }
  particles = dustSensor.getDustDensity();
  srparticles = dustSensor.getRunningAverage();
  if (sensors > 1000)
  {
    //------------------bar-------------
    /* Get a new sensor event */
    sensors_event_t event;
    bmp.getEvent(&event);
    /* Display the results (barometric pressure is measure in hPa) */
    if (event.pressure)
    {
      pressure = event.pressure;
      Serial.print("Pressure:"); Serial.print(event.pressure); Serial.println(" hPa");
      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      Serial.print("Altitude:    ");
      Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure));

    }
    //-----------------DHT--------------------
    // Get temperature event and print its value.
    dht.temperature().getEvent(&event);
    if (event.temperature) {
      temp = event.temperature;
      Serial.print(F("Temperature: ")); Serial.print(event.temperature); Serial.println(F("°C"));

    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (event.relative_humidity) {
      humidity = event.relative_humidity;
      Serial.print(F("Humidity: ")); Serial.print(event.relative_humidity); Serial.println(F("%"));

    }
    //----------------CO2-------------------
    sensorValue = analogRead(0);       // read analog input pin 0
    opary = sensorValue;
    Serial.print("Co2="); Serial.print(sensorValue, DEC); Serial.println(" PPM");

    //-----------------DUST--------------------------
    Serial.print("Dust density: ");
    Serial.print(dustSensor.getDustDensity());

    Serial.print(" ug/m3; Running average: ");
    Serial.print(dustSensor.getRunningAverage());
    Serial.println(" ug/m3");
    Serial.println("-------------------------------------------------------------");
    sensors = 0;              // reset the counter to 0 so the counting starts over...
  }

  if (counter == 1)
  {
    if (lcdms > 500) {
      lcd.setCursor(4, 1); lcd.print("Temperatura:");
      lcd.setCursor(7, 2); lcd.print(temp + String("C  "));
      lcdms = 0;
    }
  }
  if (counter == 2)
  {
    if (lcdms > 500) {
      lcd.setCursor(4, 1); lcd.print("Wilgotnosc:");
      lcd.setCursor(7, 2); lcd.print(humidity + String("%  "));
      lcdms = 0;
    }
  }

  if (counter == 3)
  {
    if (lcdms > 500) {
      lcd.setCursor(5, 1); lcd.print("Cisnienie:");
      lcd.setCursor(6, 2); lcd.print(pressure + String("hPa  "));
      lcdms = 0;
    }
  }
  if (counter == 4)
  {
    if (lcdms > 500) {
      lcd.setCursor(2, 1); lcd.print("Stezenie oparow:");
      lcd.setCursor(7, 2); lcd.print(opary + String("PPM  "));
      lcdms = 0;
    }
  }
  if (counter == 0)
  {
    if (lcdms > 500) {
      lcd.setCursor(3, 0); lcd.print("Czastki stale:");
      lcd.setCursor(3, 1); lcd.print(String("Teraz ") + particles + String("ug/m3  "));
      lcd.setCursor(3, 2); lcd.print(String("Srednia ") + srparticles + String("ug/m3  "));
      if (srparticles >= 180) {
        lcd.setCursor(0, 3); lcd.print("B.Zla j. powietrza  ");
      }
      else if (srparticles >= 130) {
        lcd.setCursor(0, 3); lcd.print("Zla j. powietrza    ");
      }
      else if (srparticles >= 70) {
        lcd.setCursor(0, 3); lcd.print("Srednia j. powietrza ");
      }
      else if (srparticles >= 25) {
        lcd.setCursor(0, 3); lcd.print("Dobra j. powietrza  ");
      }
      else {
        lcd.setCursor(0, 3); lcd.print("B.Dobra j. powietrza ");
      }
      lcdms = 0;

    }
  }


// Get current date and time
  DateTime now = rtc.now();

  // Create time format to display:
  int displaytime = (now.hour() * 100) + now.minute();

  // Print displaytime to the Serial Monitor
  Serial.println(displaytime);

  // Display the current time in 24 hour format with leading zeros enabled and a center colon:
  display.showNumberDecEx(displaytime, 0b11100000, true);

  // Remove the following lines of code if   you want a static instead of a blinking center colon:
 

   // Prints displaytime without center colon.

  



}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

