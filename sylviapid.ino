#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <din32-numsonly.h>
#include <din64-numsonly.h>
#include <Adafruit_MAX31865.h>
#include <AutoPID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define THERMO_CS_PIN 34
#define THERMO_RREF 430.0
#define THERMO_RNOMINAL 100.0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128
#define OLED_RESET -1
#define SCREEN_ADDR 0x3D

#define HEATER_PIN 39

//pid initial settings and gains
#define PULSE_WIDTH 200
#define KP .12
#define KI .0003
#define KD 0

// Wifi
const char *wifiSsd = "Nekotronik";
const char *wifiPass = "ireallylovecarpets";
#define HOSTNAME "spro.lan.fart.kiwi"

#define TIMEZONE "America/Chicago"


Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
Adafruit_MAX31865 thermo = Adafruit_MAX31865(THERMO_CS_PIN);

double temperature, setPoint;
bool heaterState;

AutoPIDRelay tempPid(&temperature, &setPoint, &heaterState, PULSE_WIDTH, KP, KI, KD);
AsyncWebServer server(80);

double currKp = KP;
double currKi = KI;
double currKd = KD;

void setup() {
  // set up heater
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  // set up RTD sensor
  thermo.begin(MAX31865_2WIRE);
  updateTemperature();

  // set up PID
  tempPid.setBangBang(5);
  tempPid.setTimeStep(50);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsd, wifiPass);

  delay(250);  // wait for OLED to power up
  display.begin(SCREEN_ADDR, true);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  WiFi.setHostname(HOSTNAME);
  configTzTime(TIMEZONE, "pool.ntp.org");

  setUpWebserver();
}

void loop() {
  // put your main code here, to run repeatedly:
  updateTemperature();
  tempPid.run();
  digitalWrite(HEATER_PIN, heaterState);
}

void updateTemperature() {
  temperature = thermo.temperature(THERMO_RNOMINAL, THERMO_RREF);
}

double cToF(double degC) {
  return (degC * 9.0 / 5.0) + 32.0;
}

double fToC(double degF) {
  return (degF - 32.0) * 5.0 / 9.0;
}

String getTempValuesJson() {
  return "{\"curr\":" + String(temperature, 6) + ",\"set\":" + String(setPoint, 6) + "}";
}

String getPidValuesJson() {
  return "{\"kp\":" + String(currKp, 6) + ",\"ki\":" + String(currKi, 6) + ",\"kd\":" + String(currKd, 6) + "}";
}

void updatePidValues(double kp, double ki, double kd) {
  currKp = kp;
  currKi = ki;
  currKd = kd;
  tempPid.setGains(kp, ki, kd);
}

void setUpWebserver() {
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");
  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404);
    }
  });

  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "pong");
  });

  server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getTempValuesJson());
  });

  server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getPidValuesJson());
  });

  server.on(
    "/set-temp", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(400, "text/plain", "Please send the temperature in the request body");
    },
    NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String tempStr = String((char *)data);
      double newTemp = tempStr.toDouble();

      setPoint = newTemp;

      request->send(200, "application/json", getTempValuesJson());
    });

  server.on(
    "/set-p", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(400, "text/plain", "Missing value in the request body");
    },
    NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String valStr = String((char *)data);
      double newVal = valStr.toDouble();

      updatePidValues(newVal, currKi, currKd);

      request->send(200, "application/json", getPidValuesJson());
    });

  server.on(
    "/set-i", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(400, "text/plain", "Missing value in the request body");
    },
    NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String valStr = String((char *)data);
      double newVal = valStr.toDouble();

      updatePidValues(currKp, newVal, currKd);

      request->send(200, "application/json", getPidValuesJson());
    });

  server.on(
    "/set-d", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(400, "text/plain", "Missing value in the request body");
    },
    NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String valStr = String((char *)data);
      double newVal = valStr.toDouble();

      updatePidValues(currKp, currKi, newVal);

      request->send(200, "application/json", getPidValuesJson());
    });

  server.begin();
}
