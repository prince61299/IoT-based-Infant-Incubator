#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[] = "b82b11fb-cbf4-4848-bfc6-376fee48c3d5";

const char SSID[] = "POCO PHONE";                  // Network SSID (name)
const char PASS[] = "Prince@61299";                // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[] = "5F6KMYELMHBV0BIGENKJ";  // Secret device password

//variable decleration
CloudHeartRate heartrate;
CloudTemperatureSensor temperature;
CloudTemperatureSensor bodyTemp;
int CO2;
CloudRelativeHumidity humidity;

void initProperties() {

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(heartrate, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(temperature, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(bodyTemp, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(CO2, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(humidity, READ, ON_CHANGE, NULL);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

//LCD library
#include <Wire.h>               // include the Wire library for I2C communication
#include <LiquidCrystal_I2C.h>  // include the LiquidCrystal_I2C library for LCD screen

// Define the I2C address for the LCD screen
#define LCD_ADDRESS 0x27

// Define the number of columns and rows for the LCD screen
#define LCD_COLUMNS 16
#define LCD_ROWS 4

// Create an instance of the LiquidCrystal_I2C class
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

//bodyTemp library
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4  // Data pin connected to the DS18B20 sensor with GPIO4 of esp32
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//include DHT library
#include "DHT.h"
#define DHTPIN 15  //Connect DHT!! data pin with the D15 of the ESP32
#define DHTTYPE DHT11

//Pulse Rate pin
const int PULSE_SENSOR_PIN = 36;  // Connect the Signal pin with VP (GPIO 36) with ESP32

//MQ135 sensor pin out
const int GAS_SENSOR_PIN = 39;  // connect the A0 pin of MQ135 with the VN (GPIO 39) with ESP32

//define the LEDs pin
const int redLEDPin = 18;
const int greenLEDPin = 5;
const int buzzerPin = 19;

//DHT11 Temperate sensor object
DHT dht(DHTPIN, DHTTYPE);


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);

  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);  //1500 milisecond

  // Initialize the I2C communication
  Wire.begin();

  // Initialize the LCD screen
  lcd.init();

  // Turn on the backlight of the LCD screen
  lcd.backlight();

  // Print a message on the LCD screen
  lcd.setCursor(0, 0);
  lcd.print("Major Project 2023");
  lcd.setCursor(0, 1);
  lcd.print("------------------------");
  lcd.setCursor(0, 2);
  lcd.print("Guided By:");
  lcd.setCursor(0, 3);
  lcd.print("Dr. Sharmila");
  delay(3000);  //delay for 3 sec
  lcd.clear();  //to clear lcd
  lcd.setCursor(0, 1);
  lcd.print("     IoT Based");
  lcd.setCursor(0, 2);
  lcd.print("  Infant Incubator  ");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1. Aditya Verma");
  lcd.setCursor(0, 1);
  lcd.print("2. Mitanshi Gaur");
  lcd.setCursor(0, 2);
  lcd.print("3. Pintu Kumar");
  lcd.setCursor(0, 3);
  lcd.print("4. Prince Kushwaha");
  delay(3000);
  lcd.clear();
  //................................................

  dht.begin();  //to start dht11 sensor

  sensors.begin();  //to start bodyTemp sensor

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {

  ArduinoCloud.update();
  // Your code here
  LCD_MESSAGE();

  CO2_SENSOR_READ();

  DHT_SENSOR_READ();

  PULSE_RATE_READ();

  BODYTEMP_SENSOR_VALUE();

  ALERT_SENSOR_VALUE();
}

// code for temp and humidity
void DHT_SENSOR_READ() {
  float h = dht.readHumidity();
  //read temperature as Celsius
  float t = dht.readTemperature();

  temperature = t;
  humidity = h;

  Serial.print("Temperature: ");
  Serial.println(t);

  Serial.print("Humidity: ");
  Serial.println(h);
}

//code for heart rate
void PULSE_RATE_READ() {
  int pulse_value = analogRead(PULSE_SENSOR_PIN);  // Read the pulse sensor value
  int p = pulse_value / 25;                        //to calibrate
  heartrate = p;
  Serial.print("Pulse Rate: ");
  Serial.println(p);  // Print the pulse sensor value to the serial monitor
}

//code for MQ135 sensor (co2 detection)
void CO2_SENSOR_READ() {
  int sensorValue = analogRead(GAS_SENSOR_PIN);
  int c = sensorValue / 5;  //to calibrate
  CO2 = c;
  Serial.print("CO2 Contn.: ");
  Serial.print(c);
  Serial.println(" PPM");
}

//body temp
void BODYTEMP_SENSOR_VALUE() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  float tempF = (tempC * 9.0 / 5.0) + 32.0;
  bodyTemp = tempF;
  Serial.print("Temperature in Celsius: ");
  Serial.print(tempC);
  Serial.print(" °C, Temperature in Fahrenheit: ");
  Serial.print(tempF);
  Serial.println(" °F");
}

//To ptint the values on LCD
void LCD_MESSAGE() {
  //print values on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temperature: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");

  lcd.setCursor(0, 2);
  lcd.print("CO2 Conctn: ");
  lcd.print(CO2);
  lcd.print(" PPM");

  delay(2000);  //delay for 2 second
  lcd.clear();  //to clear the lcd

  lcd.setCursor(0, 1);
  lcd.print("Body Temp: ");
  lcd.print(bodyTemp);
  lcd.print(" F");

  lcd.setCursor(0, 2);
  lcd.print("Pulse Rate ");
  lcd.print(heartrate);

  delay(2000); //delay for 2 second
  lcd.clear(); //to clear the lcd
}

//alert code
void ALERT_SENSOR_VALUE() {
  if (bodyTemp >= 100 || CO2 >= 700 || temperature >= 50 || heartrate > 180) {
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
  } else {
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    digitalWrite(buzzerPin, LOW);
  }
}
