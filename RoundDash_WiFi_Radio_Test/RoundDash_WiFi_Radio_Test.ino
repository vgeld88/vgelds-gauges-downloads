/*
  RoundDash Wi-Fi radio isolation test for ESP32-WROOM-32 / GC9A01 (240x240).
  This is a diagnostic, NOT a car gauge or a Wi-Fi production firmware.
  TEST 1: Power up -> open SSID RoundDash-DIAG-OPEN (NO password).
  TEST 2: Tap BOOT (GPIO 0) -> SSID RoundDash-DIAG-LOCKED, WPA2 rounddash826.
  BOOT toggles between modes. ESP32 BT stack is never initialized.
  Connect with Samsung Wi-Fi and open http://192.168.4.1/api/status.
  Run on bench only; do not operate the vehicle while using this build.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h>

TFT_eSPI display = TFT_eSPI();
TFT_eSprite screen = TFT_eSprite(&display);
WebServer http(80);
static const char *OPEN_NAME = "RoundDash-DIAG-OPEN";
static const char *SECURE_NAME = "RoundDash-DIAG-LOCKED";
static const char *PASS = "rounddash826";
static bool secureMode = false;
static bool accessPointOK = false;
static unsigned int associations = 0;
static unsigned int disconnections = 0;
static uint32_t startTime = 0;
static uint32_t lastDraw = 0;
static bool spriteOK = false;
static bool prevButton = true;
static uint32_t buttonChanged = 0;

static void apEvent(WiFiEvent_t e, WiFiEventInfo_t info) {
  (void)info;
  if (e == ARDUINO_EVENT_WIFI_AP_STACONNECTED) associations++;
  if (e == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) disconnections++;
}

static void startAP(bool locked) {
  secureMode = locked;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  delay(50); // AP reset stabilization (not a telemetry polling loop).
  accessPointOK = locked
    ? WiFi.softAP(SECURE_NAME, PASS, 1, false, 4)
    : WiFi.softAP(OPEN_NAME, nullptr, 1, false, 4);
  startTime = millis();
  Serial.printf("AP mode=%s started=%d IP=%s\n",
      locked ? "WPA2" : "OPEN", (int)accessPointOK,
      WiFi.softAPIP().toString().c_str());
}

static void sendStatus() {
  String reply = "{\"ok\":true,\"test\":\"RoundDash WiFi diagnostic\",";
  reply += "\"mode\":\"" + String(secureMode?"WPA2":"OPEN") + "\",";
  reply += "\"ssid\":\"" + String(secureMode?SECURE_NAME:OPEN_NAME) + "\",";
  reply += "\"apStarted\":" + String(accessPointOK?"true":"false") + ",";
  reply += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  reply += "\"joins\":" + String(associations) + ",";
  reply += "\"leaves\":" + String(disconnections) + ",";
  reply += "\"heap\":" + String(ESP.getFreeHeap());
  reply += "}";
  http.send(200, "application/json", reply);
}
static void render() {
  if (!spriteOK) return;
  screen.fillSprite(TFT_BLACK);
  screen.drawCircle(120,120,115,0x5397);
  screen.drawCircle(120,120,112,0x116B);
  screen.setTextDatum(MC_DATUM);
  screen.setTextColor(0x04FF,TFT_BLACK);
  screen.drawString("WIFI DIAGNOSTIC",120,49,2);
  screen.setTextColor(TFT_WHITE,TFT_BLACK);
  screen.drawString(secureMode?"LOCKED WPA2":"OPEN - NO PIN",120,86,2);
  screen.setTextColor(accessPointOK?0x07E0:0xF800,TFT_BLACK);
  screen.drawString(accessPointOK?"AP READY":"AP START FAILED",120,111,2);
  screen.setTextColor(TFT_WHITE,TFT_BLACK);
  screen.drawString(String("PHONES: ")+WiFi.softAPgetStationNum(),120,139,2);
  screen.drawString(String("JOIN ")+associations+"   LEFT "+disconnections,120,164,2);
  screen.setTextColor(0xC638,TFT_BLACK);
  screen.drawString("PRESS BOOT TO SWITCH",120,186,2);
  screen.setTextColor(TFT_WHITE,TFT_BLACK);
  screen.drawString(secureMode?"RoundDash-DIAG-LOCKED":"RoundDash-DIAG-OPEN",120,213,1);
  screen.pushSprite(0,0);
}

void setup() {
  Serial.begin(115200);
  pinMode(0,INPUT_PULLUP);
  display.init();
  display.setRotation(0);
  display.fillScreen(TFT_BLACK);
  screen.setColorDepth(8);
  spriteOK = screen.createSprite(240,240)!=nullptr;
  WiFi.onEvent(apEvent);
  startAP(false);
  http.on("/",HTTP_GET,sendStatus);
  http.on("/api/status",HTTP_GET,sendStatus);
  http.begin();
}
void loop() {
  http.handleClient();
  const uint32_t now=millis();
  const bool button=digitalRead(0)!=LOW;
  if (button!=prevButton && now-buttonChanged>65) {
    prevButton=button;
    buttonChanged=now;
    if (!button) {
      startAP(!secureMode);
    }
  }
  if (now-lastDraw>=300) {
    lastDraw=now;
    render();
  }
}
