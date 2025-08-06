// Usecase 6: Sistem Penyiraman Otomatis Menggunakan Aplikasi Android

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <AntaresESP32MQTT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define MAX485_RE_NEG 26
#define MAX485_DE 25
ModbusMaster node;

Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)

const int buttonPin = 35;
const int relayPin = 13;
const int ledPin = 12;

bool relayState = false;
enum DisplayState {
  STATE_SOIL,
  STATE_AIR,
  STATE_WIFI,
  NUM_STATES
};

DisplayState currentState = STATE_SOIL;
unsigned long displayChangeTime = 0;
const unsigned long displayChangeInterval = 3000;

unsigned long previousMillis = 0;
const long interval_send = 5000;

#define ACCESSKEY "nama acces key di akun antares anda"
#define WIFISSID "nama wifi "
#define PASSWORD "sandi wifi anda"
#define projectName "nama project/app di antares"
#define deviceName "nama device di app/device antares"

AntaresESP32MQTT antares(ACCESSKEY);

void callback(char topic[], byte payload[], unsigned int length)
{
  antares.get(topic, payload, length);

  DynamicJsonDocument jsonBuffer(1024);
  deserializeJson(jsonBuffer, antares.getPayload());
  String confirm_message = jsonBuffer["message"];
  if (confirm_message == "set")
  {
    String setPump = jsonBuffer["setPump"];
    if (setPump != NULL) {
      relayState = setPump.toInt();
      digitalWrite(relayPin, relayState);
      digitalWrite(ledPin, relayState);
      Serial.println();
      Serial.println("New Message Subscribe!");
      Serial.println("Topic: " + antares.getTopic());
      Serial.println("Payload: " + antares.getPayload());
      Serial.println();
    }
  }
}

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);
  Serial2.begin(4800, SERIAL_8N1, 16, 17);

  node.begin(1, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Wire.begin();
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  pinMode(buttonPin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, relayState);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(" SMART GARDEN");
  display.println("KIDI IOT - 2023");
  display.println("  Antares.id");
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to..");
  display.print("Wifi SSID : ");
  display.println(WIFISSID);
  display.display();

  antares.setDebug(true);
  antares.wifiConnection(WIFISSID, PASSWORD);
  antares.setMqttServer();
  antares.setCallback(callback);
}

void loop() {

  antares.checkMqttConnection();

  bool buttonState = digitalRead(buttonPin);
  if (buttonState == LOW)
  {
    relayState = !relayState;
    digitalWrite(relayPin, relayState);
    digitalWrite(ledPin, relayState ? HIGH : LOW);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Button Pressed..!");
    display.println();
    display.print("Pump Set: ");
    display.println(relayState ? "ON" : "OFF");
    display.display();
    Serial.println("Pump: " + String(relayState ? "ON" : "OFF"));

    while (digitalRead(buttonPin) != true) {}
    delay(3000);
  }

  float modbusTemperature = 0.0;
  float modbusHumidity = 0.0;
  float bmeTemperature = 0.0;
  float bmePressure = 0.0;
  float bmeAltitude = 0.0;
  float bmeHumidity = 0.0;

  bmeTemperature = bme.readTemperature();
  bmePressure = bme.readPressure() / 100.0F;
  bmeAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  bmeHumidity = bme.readHumidity();

  uint8_t result = node.readInputRegisters(0x00, 2);
  if (result == node.ku8MBSuccess) {
    modbusTemperature = node.getResponseBuffer(1) / 10.0;
    modbusHumidity = node.getResponseBuffer(0) / 10.0;
  }

  // ✅ Logika otomatis penyiraman berdasarkan kelembaban tanah
  if (modbusHumidity > 0.0) {  // Pastikan sensor terbaca dulu
    if (modbusHumidity < 30.0) {
      relayState = true;
    } else {
      relayState = false;
    }
    digitalWrite(relayPin, relayState);
    digitalWrite(ledPin, relayState);
  }

  if (millis() - displayChangeTime >= displayChangeInterval) {
    currentState = static_cast<DisplayState>((currentState + 1) % NUM_STATES);
    displayChangeTime = millis();

    switch (currentState) {
      case STATE_SOIL:
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Soil Temp: " + String(modbusTemperature, 1) + " C");
        display.println("Soil Humd: " + String(modbusHumidity, 1) + " %");
        display.display();
        break;

      case STATE_AIR:
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Air Temp:" + String(bmeTemperature, 1) + " C");
        display.println("Air Humd:" + String(bmeHumidity, 1) + " %");
        display.println("Air Pres:" + String(bmePressure, 1) + " hPa");
        display.println("Altitude:" + String(bmeAltitude, 1) + " mdpl");
        display.display();
        break;

      case STATE_WIFI:
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("WiFi SSID: ");
        display.println(WIFISSID);
        display.print("WiFi RSSI: ");
        display.println(WiFi.RSSI());
        display.println();
        display.print("Pump Status: ");
        display.println(relayState ? "ON" : "OFF");
        display.display();
        break;
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval_send)
  {
    Serial.println();
   
    antares.add("message", "status");
    antares.add("device_name", deviceName);
    antares.add("soil_temperature", String(modbusTemperature, 2));
    antares.add("soil_moisture", String(modbusHumidity, 2));
    antares.add("air_temperature", String(bmeTemperature, 2));
    antares.add("air_humidity", String(bmeHumidity, 2));
    antares.add("air_pressure", String(bmePressure, 0));
    antares.add("wifi_ssid", WIFISSID);
    antares.add("wifi_rssi", WiFi.RSSI());
    antares.add("pump_status", relayState ? 1 : 0);


    antares.publish(projectName, deviceName);


    previousMillis = currentMillis;
  }
}
