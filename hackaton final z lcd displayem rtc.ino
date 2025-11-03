#include <elapsedMillis.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <GP2YDustSensor.h>
#include <Adafruit_BMP085_U.h>
#include "RTClib.h"
#include <TM1637Display.h>

// ==================== PIN DEFINITIONS ====================
#define LED_PIN           7
#define NUM_LEDS          1
#define CLK_PIN           6
#define DIO_PIN           5
#define BUTTON_PIN        3
#define DHT_PIN           2
#define RED_PIN           11
#define GREEN_PIN         10
#define BLUE_PIN          9
#define SERVO_PIN         8
#define DUST_LED_PIN      12
#define DUST_ANALOG_PIN   A3
#define CO2_ANALOG_PIN    A0

// ==================== AIR QUALITY THRESHOLDS ====================
#define AQ_EXCELLENT      25
#define AQ_GOOD           70
#define AQ_MODERATE       130
#define AQ_POOR           180

// ==================== TIMING CONSTANTS ====================
#define SENSOR_UPDATE_MS      1000
#define STATUS_UPDATE_MS      500
#define LCD_UPDATE_MS         500
#define BUTTON_DEBOUNCE_MS    750

// ==================== DISPLAY MODES ====================
#define MODE_PARTICLES    0
#define MODE_TEMPERATURE  1
#define MODE_HUMIDITY     2
#define MODE_PRESSURE     3
#define MODE_VAPOR        4
#define MODE_COUNT        5

// ==================== LCD ADDRESS ====================
#define LCD_ADDRESS       0x27  // Change if your LCD has different address

// ==================== SENSOR VALIDATION ====================
#define TEMP_MIN          -40.0
#define TEMP_MAX          80.0
#define HUMIDITY_MIN      0.0
#define HUMIDITY_MAX      100.0
#define PRESSURE_MIN      800.0
#define PRESSURE_MAX      1200.0

// ==================== GLOBAL OBJECTS ====================
RTC_DS3231 rtc;
TM1637Display display = TM1637Display(CLK_PIN, DIO_PIN);
CRGB leds[NUM_LEDS];
Servo servo;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(LCD_ADDRESS, 20, 4);
DHT_Unified dht(DHT_PIN, DHT22);
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, DUST_LED_PIN, DUST_ANALOG_PIN);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// ==================== TIMING VARIABLES ====================
elapsedMillis sensorTimer;
elapsedMillis statusTimer;
elapsedMillis lcdTimer;
elapsedMillis buttonTimer;

// ==================== STATE VARIABLES ====================
int displayMode = MODE_PARTICLES;
bool buttonPressed = false;
bool lastButtonState = LOW;

// ==================== SENSOR DATA ====================
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  int particles;
  int avgParticles;
  int vaporConcentration;
  bool valid;
} sensorData = {0, 0, 0, 0, 0, 0, false};

// ==================== FUNCTION PROTOTYPES ====================
void initializeSensors();
void handleButton();
void updateSensors();
void updateAirQualityIndicators();
void updateLCD();
void updateTimeDisplay();
void setRGBColor(int red, int green, int blue);
float validateTemperature(float temp);
float validateHumidity(float humidity);
float validatePressure(float pressure);
const char* getAirQualityText(int particles);

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  
  // Pin modes
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  // Servo initialization
  servo.attach(SERVO_PIN);
  servo.write(15);
  
  // FastLED initialization
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
  // LCD initialization
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  
  // Initialize all sensors
  initializeSensors();
  
  // RTC initialization
  if (!rtc.begin()) {
    Serial.println("ERROR: RTC not found!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RTC ERROR!");
    while (1);
  }
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // 7-segment display initialization
  display.setBrightness(5);
  display.clear();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready!");
  delay(1000);
  lcd.clear();
  
  Serial.println("System initialized successfully");
}

// ==================== MAIN LOOP ====================
void loop() {
  handleButton();
  
  if (sensorTimer >= SENSOR_UPDATE_MS) {
    updateSensors();
    sensorTimer = 0;
  }
  
  if (statusTimer >= STATUS_UPDATE_MS) {
    updateAirQualityIndicators();
    statusTimer = 0;
  }
  
  if (lcdTimer >= LCD_UPDATE_MS) {
    updateLCD();
    lcdTimer = 0;
  }
  
  updateTimeDisplay();
}

// ==================== INITIALIZATION ====================
void initializeSensors() {
  // DHT sensor
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  Serial.println("DHT22 initialized");
  
  // Dust sensor
  dustSensor.begin();
  Serial.println("Dust sensor initialized");
  
  // Barometric pressure sensor
  if (!bmp.begin()) {
    Serial.println("WARNING: BMP085 not found!");
  } else {
    Serial.println("BMP085 initialized");
  }
}

// ==================== BUTTON HANDLING ====================
void handleButton() {
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Debouncing logic
  if (currentButtonState == HIGH && lastButtonState == LOW && buttonTimer >= BUTTON_DEBOUNCE_MS) {
    buttonPressed = true;
    buttonTimer = 0;
  }
  
  lastButtonState = currentButtonState;
  
  // Process button press
  if (buttonPressed) {
    displayMode++;
    if (displayMode >= MODE_COUNT) {
      displayMode = MODE_PARTICLES;
    }
    lcd.clear();
    buttonPressed = false;
    Serial.print("Display mode changed to: ");
    Serial.println(displayMode);
  }
}

// ==================== SENSOR UPDATES ====================
void updateSensors() {
  sensors_event_t event;
  
  // Temperature
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    sensorData.temperature = validateTemperature(event.temperature);
    Serial.print("Temperature: ");
    Serial.print(sensorData.temperature);
    Serial.println(" C");
  }
  
  // Humidity
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    sensorData.humidity = validateHumidity(event.relative_humidity);
    Serial.print("Humidity: ");
    Serial.print(sensorData.humidity);
    Serial.println(" %");
  }
  
  // Pressure
  bmp.getEvent(&event);
  if (event.pressure) {
    sensorData.pressure = validatePressure(event.pressure);
    Serial.print("Pressure: ");
    Serial.print(sensorData.pressure);
    Serial.println(" hPa");
  }
  
  // Dust particles
  sensorData.particles = dustSensor.getDustDensity();
  sensorData.avgParticles = dustSensor.getRunningAverage();
  Serial.print("Particles: ");
  Serial.print(sensorData.particles);
  Serial.print(" ug/m3 (avg: ");
  Serial.print(sensorData.avgParticles);
  Serial.println(" ug/m3)");
  
  // CO2/Vapor concentration (note: needs proper calibration for your sensor)
  sensorData.vaporConcentration = analogRead(CO2_ANALOG_PIN);
  Serial.print("Vapor/CO2: ");
  Serial.print(sensorData.vaporConcentration);
  Serial.println(" (raw ADC)");
  
  Serial.println("-----------------------------------");
  sensorData.valid = true;
}

// ==================== AIR QUALITY INDICATORS ====================
void updateAirQualityIndicators() {
  int avg = sensorData.avgParticles;
  
  if (avg >= AQ_POOR) {
    // Very Poor - Red
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    servo.write(0);
    setRGBColor(255, 0, 0);
  }
  else if (avg >= AQ_MODERATE) {
    // Poor - Cyan
    leds[0] = CRGB(0, 255, 255);
    FastLED.show();
    servo.write(50);
    setRGBColor(0, 255, 255);
  }
  else if (avg >= AQ_GOOD) {
    // Moderate - Blue
    leds[0] = CRGB(0, 0, 255);
    FastLED.show();
    servo.write(100);
    setRGBColor(0, 0, 255);
  }
  else if (avg >= AQ_EXCELLENT) {
    // Good - Yellow-Green
    leds[0] = CRGB(150, 255, 0);
    FastLED.show();
    servo.write(140);
    setRGBColor(0, 255, 0);
  }
  else {
    // Excellent - Green
    leds[0] = CRGB(0, 255, 0);
    FastLED.show();
    servo.write(180);
    setRGBColor(0, 0, 0);
  }
}

// ==================== LCD DISPLAY ====================
void updateLCD() {
  if (!sensorData.valid) {
    lcd.setCursor(0, 1);
    lcd.print("Waiting for data...");
    return;
  }
  
  char buffer[21]; // 20 chars + null terminator
  
  switch (displayMode) {
    case MODE_PARTICLES:
      lcd.setCursor(3, 0);
      lcd.print("Particles (PM2.5)");
      
      lcd.setCursor(3, 1);
      snprintf(buffer, sizeof(buffer), "Now: %d ug/m3   ", sensorData.particles);
      lcd.print(buffer);
      
      lcd.setCursor(3, 2);
      snprintf(buffer, sizeof(buffer), "Avg: %d ug/m3   ", sensorData.avgParticles);
      lcd.print(buffer);
      
      lcd.setCursor(0, 3);
      lcd.print(getAirQualityText(sensorData.avgParticles));
      break;
      
    case MODE_TEMPERATURE:
      lcd.setCursor(4, 1);
      lcd.print("Temperature:");
      lcd.setCursor(7, 2);
      snprintf(buffer, sizeof(buffer), "%.1f C   ", sensorData.temperature);
      lcd.print(buffer);
      break;
      
    case MODE_HUMIDITY:
      lcd.setCursor(4, 1);
      lcd.print("Humidity:");
      lcd.setCursor(7, 2);
      snprintf(buffer, sizeof(buffer), "%.1f %%   ", sensorData.humidity);
      lcd.print(buffer);
      break;
      
    case MODE_PRESSURE:
      lcd.setCursor(5, 1);
      lcd.print("Pressure:");
      lcd.setCursor(6, 2);
      snprintf(buffer, sizeof(buffer), "%.0f hPa   ", sensorData.pressure);
      lcd.print(buffer);
      break;
      
    case MODE_VAPOR:
      lcd.setCursor(2, 1);
      lcd.print("Vapor/Gas Conc.:");
      lcd.setCursor(7, 2);
      snprintf(buffer, sizeof(buffer), "%d (raw)   ", sensorData.vaporConcentration);
      lcd.print(buffer);
      break;
  }
}

// ==================== TIME DISPLAY ====================
void updateTimeDisplay() {
  DateTime now = rtc.now();
  int displayTime = (now.hour() * 100) + now.minute();
  display.showNumberDecEx(displayTime, 0b11100000, true);
}

// ==================== HELPER FUNCTIONS ====================
void setRGBColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

float validateTemperature(float temp) {
  if (temp < TEMP_MIN || temp > TEMP_MAX) {
    Serial.println("WARNING: Temperature out of range!");
    return 0.0;
  }
  return temp;
}

float validateHumidity(float humidity) {
  if (humidity < HUMIDITY_MIN || humidity > HUMIDITY_MAX) {
    Serial.println("WARNING: Humidity out of range!");
    return 0.0;
  }
  return humidity;
}

float validatePressure(float pressure) {
  if (pressure < PRESSURE_MIN || pressure > PRESSURE_MAX) {
    Serial.println("WARNING: Pressure out of range!");
    return 0.0;
  }
  return pressure;
}

const char* getAirQualityText(int particles) {
  if (particles >= AQ_POOR) {
    return "Very Poor Quality   ";
  } else if (particles >= AQ_MODERATE) {
    return "Poor Quality        ";
  } else if (particles >= AQ_GOOD) {
    return "Moderate Quality    ";
  } else if (particles >= AQ_EXCELLENT) {
    return "Good Quality        ";
  } else {
    return "Excellent Quality   ";
  }
}
