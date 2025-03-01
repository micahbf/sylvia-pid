#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "din32-numsonly.h"
#include "din16-numsonly.h"
#include <Adafruit_MAX31865.h>
#include <QuickPID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <cmath>

#define THERMO_CS_PIN 13
#define THERMO_RREF 430.0
#define THERMO_RNOMINAL 100.0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128
#define OLED_RESET -1
#define SCREEN_ADDR 0x3D

#define SS_SWITCH 24
#define SS_NEOPIX 6

#define SEESAW_ADDR 0x36

Adafruit_seesaw ss;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);
int32_t encoder_position;

#define HEATER_PIN 39

//pid initial settings and gains
const unsigned long windowSize = 2000;
const byte debounce = 50;
float pidOut, Kp = 2, Ki = 5, Kd = 1;

// Wifi
const char *wifiSsd = "Nekotronik";
const char *wifiPass = "ireallylovecarpets";
#define HOSTNAME "spro.lan.fart.kiwi"

#define TIMEZONE "America/Chicago"


Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
Adafruit_MAX31865 thermo = Adafruit_MAX31865(THERMO_CS_PIN);

int humanSetPoint = 218;

float temperature = 103, setPoint;
bool heaterState;
unsigned long windowStartTime, nextSwitchTime;
int tempOnScreen, setPointOnScreen;

QuickPID tempPID(&temperature, &pidOut, &setPoint, Kp, Ki, Kd,
                 tempPID.pMode::pOnError,
                 tempPID.dMode::dOnMeas,
                 tempPID.iAwMode::iAwClamp,
                 tempPID.Action::direct);

AsyncWebServer server(80);

int led = LED_BUILTIN;

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  setPoint = fToC(static_cast<float>(humanSetPoint));
  // set up heater
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  // set up RTD sensor
  thermo.begin(MAX31865_2WIRE);
  updateTemperature();

  // set up PID
  tempPID.SetOutputLimits(0, windowSize);
  tempPID.SetSampleTimeUs(windowSize * 1000);
  tempPID.SetMode(tempPID.Control::automatic);

  // set up rotary encoder
  if (!ss.begin(SEESAW_ADDR) || !sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while (1) delay(10);
  }

  ss.pinMode(SS_SWITCH, INPUT_PULLUP);
  encoder_position = ss.getEncoderPosition();
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();
  sspixel.setBrightness(10);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsd, wifiPass);

  delay(250);  // wait for OLED to power up
  display.begin(SCREEN_ADDR, true);

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.println("Connecting");
  display.display();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  WiFi.setHostname(HOSTNAME);
  configTzTime(TIMEZONE, "pool.ntp.org");

  setUpWebserver();
}

void loop() {
  updateTemperature();
  updateEncoder();
  runPidUpdate();
  updateEncoderPixel();
  updateScreen();
  digitalWrite(HEATER_PIN, heaterState);
}

void runPidUpdate() {
  unsigned long msNow = millis();
  if (tempPID.Compute()) windowStartTime = msNow;

  if (!heaterState && pidOut > (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      heaterState = true;
      digitalWrite(HEATER_PIN, HIGH);
    }
  } else if (heaterState && pidOut < (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      heaterState = false;
      digitalWrite(HEATER_PIN, LOW);
    }
  }
}

void updateTemperature() {
  temperature = thermo.temperature(THERMO_RNOMINAL, THERMO_RREF);
}

void updateEncoder() {
  int32_t new_position = ss.getEncoderPosition();
  if (new_position != encoder_position) {
    int32_t diff = new_position - encoder_position;
    humanSetPoint += diff;
    setPoint = fToC(static_cast<float>(humanSetPoint));
    encoder_position = new_position;
  }
}

void updateEncoderPixel() {
  if (heaterState) {
    sspixel.setPixelColor(0, sspixel.Color(255, 36, 8));
  } else {
    sspixel.setPixelColor(0, sspixel.Color(0, 0, 0));
  }
  sspixel.show();
}

double cToF(double degC) {
  return (degC * 9.0 / 5.0) + 32.0;
}

double fToC(double degF) {
  return (degF - 32.0) * 5.0 / 9.0;
}

String getTempValuesJson() {
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  float resistance = THERMO_RREF * ratio;
  uint8_t fault = thermo.readFault();
  return "{\"curr\":" + String(temperature, 6) + ",\"set\":" + String(setPoint, 6) + ",\"rtd\":" + String(rtd, 6) + ",\"resistance\":" + String(resistance, 6) + ",\"fault\":\"" + String(fault, HEX) + "\"}";
}

String getPidValuesJson() {
  return "{\"kp\":" + String(Kp, 6) + ",\"ki\":" + String(Ki, 6) + ",\"kd\":" + String(Kd, 6) + "}";
}

void updatePidValues(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  tempPID.SetTunings(kp, ki, kd);
}

void updateScreen() {
  int currTemp = static_cast<int>(std::round(cToF(temperature)));
  if (tempOnScreen != currTemp || setPointOnScreen != humanSetPoint) {
    drawScreen(currTemp, humanSetPoint);
    tempOnScreen = currTemp;
    setPointOnScreen = humanSetPoint;
  }
}

void drawScreen(int currTemp, int currSetPoint) {
  int16_t x1, y1;
  uint16_t w, h;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&DIN_Alternate_Bold32pt7b);
  String currTempStr = String(currTemp);
  display.getTextBounds(currTempStr, 0, 64, &x1, &y1, &w, &h);
  int xOffset = (SCREEN_WIDTH - w) / 2;
  display.setCursor(xOffset, 74);
  display.print(currTempStr);
  display.setFont(&DIN_Alternate_Bold16pt7b);
  String currSetPointStr = String(currSetPoint);
  display.getTextBounds(currSetPointStr, 0, 128, &x1, &y1, &w, &h);
  xOffset = (SCREEN_WIDTH - w) / 2;
  display.setCursor(xOffset, 120);
  display.print(currSetPointStr);
  display.display();
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

      updatePidValues(newVal, Ki, Kd);

      request->send(200, "application/json", getPidValuesJson());
    });

  server.on(
    "/set-i", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(400, "text/plain", "Missing value in the request body");
    },
    NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String valStr = String((char *)data);
      double newVal = valStr.toDouble();

      updatePidValues(Kp, newVal, Kd);

      request->send(200, "application/json", getPidValuesJson());
    });

  server.on(
    "/set-d", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(400, "text/plain", "Missing value in the request body");
    },
    NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String valStr = String((char *)data);
      double newVal = valStr.toDouble();

      updatePidValues(Kp, Ki, newVal);

      request->send(200, "application/json", getPidValuesJson());
    });

  server.begin();
}
