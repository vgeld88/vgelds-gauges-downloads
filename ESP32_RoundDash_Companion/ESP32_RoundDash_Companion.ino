/*
  ESP32-WROOM-32 + GC9A01 (240x240) + Classic Bluetooth ELM327
  Pages: RPM arc, coolant temperature, control-module voltage (PID 0142).

  NO ROTARY ENCODER REQUIRED:
   - ESP32 dev-board BOOT button (GPIO0): short press = next page,
     long press (>700 ms) = previous page (on release).
   - Pages also rotate automatically every 7 seconds.
     Disable automatic switching in the RoundDash Studio app.
   - GPIO0 is a boot strap pin: do not hold BOOT while powering up.

  TFT_eSPI's User_Setup must define:
     GC9A01_DRIVER, TFT_WIDTH 240, TFT_HEIGHT 240
     TFT_MOSI 23, TFT_SCLK 18, TFT_CS 15, TFT_DC 2, TFT_RST 4
     SPI_FREQUENCY 40000000
  TFT_eSPI owns the display pins; no pinMode() for them here.

  Display RAM (front buffer) + one full TFT_eSprite (back buffer)
  provide double-buffered, tear-minimized rendering. An 8-bit sprite
  uses ~58 KB, more reliable with Bluetooth on a WROOM-32 than 16-bit.
*/

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
  #error This sketch needs an original ESP32 with Classic Bluetooth SPP support.
#endif

// ------------------------- USER SETTINGS -------------------------
// Settings are editable from the Android application and persisted in NVS.
// WPA2 password must contain at least 8 characters. Change for your own unit.
static const char *AP_SSID = "RoundDash-ESP32";
static const char *AP_PASSWORD = "rounddash826";
// 90-second Wi-Fi-first startup avoids competing Bluetooth inquiry radio traffic
// while a phone is authenticating to the ESP32's access point.
static uint32_t wifiBootAt = 0;
static bool bluetoothStarted = false;
static volatile uint32_t apJoins = 0, apLeaves = 0;
static volatile uint8_t lastApLeaveReason = 0;
constexpr bool USE_BOOT_BUTTON = true;
constexpr uint32_t DEBOUNCE_MS = 35;
constexpr uint32_t LONG_PRESS_MS = 700;

constexpr uint32_t PID_SPACING_MS = 150;    // Minimum gap between PID sends
constexpr uint32_t PID_TIMEOUT_MS = 1250;  // Advance when ECU does not reply
constexpr uint32_t INIT_TIMEOUT_MS = 2600;
constexpr uint32_t RECONNECT_MS = 5000;
constexpr uint32_t DATA_STALE_MS = 4000;
constexpr uint32_t FRAME_MS = 50;         // Up to ~20 FPS; no delay()

constexpr int TFT_W = 240, TFT_H = 240;
constexpr int CX = 120, CY = 120;
constexpr uint8_t BUTTON_PIN = 0;


static constexpr uint16_t C_GREY = 0x8410;
static constexpr uint16_t C_DARK = 0x2104;
static constexpr uint16_t C_DIM = 0x4208;
static constexpr uint16_t C_GREEN = 0x07E0;
static constexpr uint16_t C_ORANGE = 0xFCA0;
static constexpr uint16_t C_RED = 0xF800;

struct GaugeConfig {
  uint16_t accent565 = 0xE9A7;    // #EF343B (quantized RGB565)
  uint16_t background565 = 0x0041; // #05090F
  uint16_t foreground565 = 0xFFFF;
  float rpmMax = 8000.0f;
  float redline = 6700.0f;
  float coolantMin = 40.0f;
  float coolantMax = 120.0f;
  float voltageMin = 10.0f;
  float voltageMax = 16.0f;
  int stroke = 10;
  int fontScale = 100;
  int titleY = 75;
  int valueY = 166;
  int unitY = 188;
  bool autoCycle = true;
  uint32_t cycleMs = 7000;
  String rpmTitle = "ENGINE SPEED";
  String coolantTitle = "COOLANT";
  String voltageTitle = "MODULE VOLTAGE";
  // RoundDash Pro V2 physical dial customisation
  uint8_t dialStyle = 0;   // 0 STI Red, 1 Rally, 2 GT Silver, 3 Amber, 4 Blue, 5 Carbon
  uint8_t needleStyle = 0; // 0 OEM blade, 1 silver tip, 2 fine race needle
  int needleWidth = 3;    // 1..6 pixels at hub
  int glow = 65;          // approximation on the RGB565 GC9A01
  bool numberedTicks = true;
  bool shiftLights = true;
  bool bezel = true;
  bool digitalWindow = true;
  // RoundDash Studio V3 — seven physical layout types, four editable pages
  String boostTitle = "TURBO BOOST";
  float boostMin = -15.0f;
  float boostMax = 25.0f;
  float ambientKpa = 101.3f; // Used only if 0133 barometric PID is unavailable
  uint8_t rpmLayout = 0, coolantLayout = 2, voltageLayout = 4, boostLayout = 3;
  bool turboEnabled = true, boostPeakHold = true;
  float turboThreshold = 5.0f;
  int turboIntensity = 85, turboDuration = 1200;
  // Vehicle/ROM metadata only. Live acquisition stays on verified generic Mode 01 PIDs.
  String vehiclePreset = "sti_dm";
  String vehicleName = "2018 Subaru WRX STI";
  String romId = "ZE5T201V_DM23100";
  String ecuId = "A62A907107";
  String obdName = "OBDII";
  String obdPin = "1234";
} cfg;

BluetoothSerial bt;
WebServer server(80);
Preferences preferences;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite frame = TFT_eSprite(&tft);
bool frameReady = false;
bool bluetoothReady = false;

struct Reading {
  float value = 0;
  uint32_t updatedAt = 0;
  bool valid = false;
};
Reading engineRpm, coolantC, moduleV, mapKpa, baroKpa, boostPsi;
bool boostEstimated = true;
bool turboArmed = true;
uint32_t turboTriggeredAt = 0;
float peakBoost = -1000.0f;

// ------------------------- UI INPUT ------------------------------
uint8_t currentPage = 0;
uint32_t pageChangedAt = 0;
uint32_t phonePageHoldUntil = 0;
bool lastRawPressed = false;
bool stablePressed = false;
uint32_t rawChangedAt = 0;
uint32_t pressedAt = 0;
float shownRpm = 0;
float shownCoolant = 40;
float shownVoltage = 10;
float shownBoost = -15;
uint32_t lastFrameAt = 0;

static bool elapsed(uint32_t now, uint32_t start, uint32_t duration) {
  return (uint32_t)(now - start) >= duration;
}

static float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static bool fresh(const Reading &r, uint32_t now) {
  return r.valid && !elapsed(now, r.updatedAt, DATA_STALE_MS);
}

void changePage(int direction, uint32_t now) {
  currentPage = (uint8_t)((currentPage + (direction > 0 ? 1 : 3)) % 4);
  pageChangedAt = now;
  lastFrameAt = 0;  // Force an immediate redraw
}

void updateButton(uint32_t now) {
  if (USE_BOOT_BUTTON) {
    bool raw = (digitalRead(BUTTON_PIN) == LOW);
    if (raw != lastRawPressed) {
      lastRawPressed = raw;
      rawChangedAt = now;
    }
    if (raw != stablePressed && elapsed(now, rawChangedAt, DEBOUNCE_MS)) {
      stablePressed = raw;
      if (stablePressed) {
        pressedAt = now;
      } else {
        changePage(elapsed(now, pressedAt, LONG_PRESS_MS) ? -1 : 1, now);
      }
    }
  }
  if (cfg.autoCycle && !stablePressed && (int32_t)(now - phonePageHoldUntil) >= 0 && elapsed(now, pageChangedAt, cfg.cycleMs)) {
    changePage(1, now);
  }
}

// ------------------------- BLUETOOTH -----------------------------
// connect(name) can take many seconds to discover an adapter. Performing
// it in a FreeRTOS worker keeps loop() and the display responsive.
enum class LinkState : uint8_t { RETRY_WAIT, CONNECTING, INITIALIZING, POLLING, BT_FAILURE };
LinkState linkState = LinkState::RETRY_WAIT;
volatile bool connectFinished = false;
volatile bool connectSucceeded = false;
uint32_t retryAt = 0;
uint32_t nextSendAt = 0;
uint8_t initIndex = 0;
uint8_t initFailures = 0;
uint8_t pollIndex = 0;

static const char *const INIT_COMMANDS[] = {
  "AT Z",     // reset adapter
  "AT E0",    // echo off
  "AT L0",    // linefeeds off
  "AT S0",    // spaces off (parser supports either format)
  "AT H0",    // headers off
  "AT SP 0",  // auto-select OBD protocol
  "01 00"     // supported-PID probe
};
constexpr uint8_t INIT_COUNT = sizeof(INIT_COMMANDS) / sizeof(INIT_COMMANDS[0]);

// RPM twice per cycle, so tach responsiveness is not sacrificed for boost.
static const char *const PID_COMMANDS[] = {"01 0C", "01 0B", "01 05", "01 0C", "01 33", "01 42"};
static const uint8_t PID_CODES[] = {0x0C, 0x0B, 0x05, 0x0C, 0x33, 0x42};
constexpr uint8_t PID_COUNT = sizeof(PID_CODES) / sizeof(PID_CODES[0]);

struct Exchange {
  bool waiting = false;
  uint32_t sentAt = 0;
  uint32_t timeoutMs = 0;
  char response[512] = {0};
  size_t used = 0;
};
Exchange exchange;

void connectTask(void *unused) {
  (void)unused;
  bool result = bt.connect(cfg.obdName);
  connectSucceeded = result;
  connectFinished = true;
  vTaskDelete(nullptr);
}

void scheduleReconnect(uint32_t now) {
  exchange.waiting = false;
  linkState = LinkState::RETRY_WAIT;
  retryAt = now;
  // Invalidate readings immediately so a disconnected ECU is not shown as live.
  engineRpm.valid = coolantC.valid = moduleV.valid = mapKpa.valid = baroKpa.valid = boostPsi.valid = false;
  turboArmed = true; turboTriggeredAt = 0;
}

void startConnection(uint32_t now) {
  if (!bluetoothReady) {
    linkState = LinkState::BT_FAILURE;
    return;
  }
  connectFinished = false;
  connectSucceeded = false;
  linkState = LinkState::CONNECTING;
  BaseType_t started = xTaskCreate(connectTask, "obd-connect", 6144, nullptr, 1, nullptr);
  if (started != pdPASS) scheduleReconnect(now);
}

// ------------------------- ELM327 PARSER --------------------------
// Look for the actual 41/PID payload, not merely the request echo.
// Handles: "41 0C 1A F8", "410C1AF8", "7E8 04 41 0C 1A F8",
// with extra text, CR/LF and '>' anywhere in an adapter response.
// It does not assume A/B is at a fixed offset or that headers are off.
static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  c = (char)toupper((unsigned char)c);
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static bool decodePidResponse(const char *response, uint8_t pid,
                              uint8_t neededBytes, uint8_t &A, uint8_t &B) {
  char signature[5];
  snprintf(signature, sizeof(signature), "41%02X", pid);

  // Parse each line separately: never stitch hex from different ECU lines.
  while (*response) {
    char hexLine[256];
    size_t n = 0;
    while (*response && *response != '\r' && *response != '\n') {
      if (hexDigit(*response) >= 0 && n < sizeof(hexLine) - 1)
        hexLine[n++] = (char)toupper((unsigned char)*response);
      ++response;
    }
    if (*response) ++response;
    hexLine[n] = '\0';

    // Align on 41/PID, which also works with an odd-length CAN ID prefix.
    for (size_t i = 0; i + 4 + neededBytes * 2 <= n; ++i) {
      if (memcmp(hexLine + i, signature, 4) != 0) continue;
      int a1 = hexDigit(hexLine[i + 4]);
      int a2 = hexDigit(hexLine[i + 5]);
      if (a1 < 0 || a2 < 0) continue;
      A = (uint8_t)((a1 << 4) | a2);
      B = 0;
      if (neededBytes == 2) {
        int b1 = hexDigit(hexLine[i + 6]);
        int b2 = hexDigit(hexLine[i + 7]);
        if (b1 < 0 || b2 < 0) continue;
        B = (uint8_t)((b1 << 4) | b2);
      }
      return true;
    }
  }
  return false;
}

void sendElmCommand(const char *command, uint32_t now, bool initializing) {
  if (!bt.connected(0)) return;
  exchange.used = 0;
  exchange.response[0] = '\0';
  exchange.sentAt = now;
  exchange.timeoutMs = initializing ? INIT_TIMEOUT_MS : PID_TIMEOUT_MS;
  exchange.waiting = true;
  bt.print(command);
  bt.write((uint8_t)'\r');
#if defined(OBD_DEBUG)
  Serial.printf("TX: %s\n", command);
#endif
}

// Mode 01 MAP returns absolute kPa; never claim pressure when MAP is stale.
static void refreshBoost(uint32_t now) {
  if (!fresh(mapKpa, now)) { boostPsi.valid = false; return; }
  boostEstimated = !fresh(baroKpa, now);
  const float atmospheric = boostEstimated ? cfg.ambientKpa : baroKpa.value;
  boostPsi.value = (mapKpa.value - atmospheric) * 0.145037738f;
  boostPsi.updatedAt = now;
  boostPsi.valid = true;
  if (cfg.boostPeakHold && boostPsi.value > peakBoost) peakBoost = boostPsi.value;
  if (boostPsi.value < cfg.turboThreshold - 1.5f) turboArmed = true;
  if (cfg.turboEnabled && turboArmed && boostPsi.value >= cfg.turboThreshold) {
    turboTriggeredAt = now; turboArmed = false;
  }
}

void processPid(uint8_t which, const char *response, uint32_t now) {
  uint8_t A = 0, B = 0;
  const uint8_t pid = PID_CODES[which];
  const uint8_t count = (pid == 0x0C || pid == 0x42) ? 2 : 1;
  if (!decodePidResponse(response, pid, count, A, B)) return;

  if (pid == 0x0C) {
    engineRpm.value = ((uint16_t(A) << 8) | B) / 4.0f;
    engineRpm.updatedAt = now;
    engineRpm.valid = true;
  } else if (pid == 0x05) {
    coolantC.value = (float)A - 40.0f;
    coolantC.updatedAt = now;
    coolantC.valid = true;
  } else if (pid == 0x42) {
    moduleV.value = ((uint16_t(A) << 8) | B) / 1000.0f;
    moduleV.updatedAt = now;
    moduleV.valid = true;
  } else if (pid == 0x0B) {
    mapKpa.value = A; mapKpa.updatedAt = now; mapKpa.valid = true;
    refreshBoost(now);
  } else if (pid == 0x33) {
    baroKpa.value = A; baroKpa.updatedAt = now; baroKpa.valid = true;
    if (fresh(mapKpa, now)) refreshBoost(now);
  }
}

void handleElmReply(uint32_t now, bool timedOut) {
  exchange.waiting = false;
  exchange.response[exchange.used] = '\0';
#if defined(OBD_DEBUG)
  Serial.printf("RX%s: %s\n", timedOut ? " TIMEOUT" : "", exchange.response);
#endif

  if (linkState == LinkState::INITIALIZING) {
    const bool isReset = (initIndex == 0);
    const bool isProbe = (initIndex == INIT_COUNT - 1);
    const bool badReply = timedOut || strstr(exchange.response, "?") != nullptr ||
                          strstr(exchange.response, "ERROR") != nullptr;
    // The ELM reset sometimes omits a prompt. A nonresponding vehicle can
    // make 0100 time out: continue polling so it recovers when IGN is on.
    if (badReply && !isReset && !isProbe) {
      if (++initFailures < 2) {
        nextSendAt = now + 250;
        return;
      }
      scheduleReconnect(now);
      return;
    }
    initFailures = 0;
    ++initIndex;
    if (initIndex >= INIT_COUNT) {
      pollIndex = 0;
      linkState = LinkState::POLLING;
    }
    nextSendAt = now + (isReset ? 350 : 70);
  } else if (linkState == LinkState::POLLING) {
    processPid(pollIndex, exchange.response, now);
    pollIndex = (pollIndex + 1) % PID_COUNT;
    // No PID is sent faster than 150 ms after the previous query.
    nextSendAt = exchange.sentAt + PID_SPACING_MS;
  }
}

void readElmIncoming(uint32_t now) {
  // Keep a per-loop byte budget to avoid starving the UI on noisy adapters.
  int byteBudget = 512;
  while (byteBudget-- > 0 && bt.available() > 0) {
    int byteRead = bt.read();
    if (byteRead < 0 || !exchange.waiting) continue;
    char c = (char)byteRead;
    if (c == '>') {
      handleElmReply(now, false);
      continue;
    }
    if (c == '\r' || c == '\n' || (c >= 32 && c <= 126)) {
      if (exchange.used < sizeof(exchange.response) - 1)
        exchange.response[exchange.used++] = c;
    }
  }
}

void updateObd(uint32_t now) {
  if (linkState == LinkState::BT_FAILURE) return;
  if (linkState == LinkState::RETRY_WAIT) {
    if (elapsed(now, retryAt, RECONNECT_MS)) startConnection(now);
    return;
  }
  if (linkState == LinkState::CONNECTING) {
    if (!connectFinished) return;
    if (connectSucceeded || bt.connected(0)) {
      linkState = LinkState::INITIALIZING;
      initIndex = initFailures = 0;
      nextSendAt = now;
      exchange.waiting = false;
    } else {
      scheduleReconnect(now);
    }
    return;
  }
  if (!bt.connected(0)) {
    scheduleReconnect(now);
    return;
  }

  readElmIncoming(now);
  if (exchange.waiting) {
    if (elapsed(now, exchange.sentAt, exchange.timeoutMs)) handleElmReply(now, true);
    else return;
  }

  if ((int32_t)(now - nextSendAt) < 0) return;
  if (linkState == LinkState::INITIALIZING)
    sendElmCommand(INIT_COMMANDS[initIndex], now, true);
  else if (linkState == LinkState::POLLING)
    sendElmCommand(PID_COMMANDS[pollIndex], now, false);
}

// --------------------- V2 ANALOG INSTRUMENTS ----------------------
// A real 240x240 pixel gauge is smaller than the phone preview. All effects
// here are drawn with integer primitives, no bitmaps or external fonts.
// TFT_eSPI starts its arc at 6 o'clock: 45° = lower left, 315° = lower right.
// Screen coordinates: x=CX-sin(a)*r, y=CY+cos(a)*r.
static uint16_t blend565(uint16_t a, uint16_t b, uint8_t t) {
  uint8_t r1=(a>>11)&31,g1=(a>>5)&63,b1=a&31;
  uint8_t r2=(b>>11)&31,g2=(b>>5)&63,b2=b&31;
  return (uint16_t)(((r1*(255-t)+r2*t)/255)<<11 |
                    ((g1*(255-t)+g2*t)/255)<<5 |
                    ((b1*(255-t)+b2*t)/255));
}

static void centerFont(const String &label, int y, uint8_t scale, uint16_t color) {
  frame.setTextFont(1);
  frame.setTextSize(scale);
  frame.setTextDatum(MC_DATUM);
  frame.setTextColor(color);   // transparent text: preserves dial texture
  frame.drawString(label, CX, y);
  frame.setTextSize(1);
}

static void centerFit(String label, int y, int scale, uint16_t color, int limit=164) {
  if (label.length() > 28) label.remove(28);
  scale=constrain(scale,1,5);
  while(scale>1 && (int)label.length()*6*scale>limit) --scale;
  centerFont(label,y,(uint8_t)scale,color);
}

static void dialBase() {
  frame.fillSprite(cfg.background565);
  if(cfg.bezel) {
    frame.drawCircle(CX,CY,118,blend565(C_GREY,cfg.foreground565,84));
    frame.drawCircle(CX,CY,117,blend565(C_GREY,C_DARK,190));
    frame.drawCircle(CX,CY,115,blend565(C_GREY,cfg.foreground565,50));
    frame.drawCircle(CX,CY,112,C_DARK);
    frame.drawCircle(CX,CY,110,blend565(C_GREY,cfg.accent565,42));
  } else {
    frame.drawCircle(CX,CY,115,C_DARK);
    frame.drawCircle(CX,CY,110,blend565(C_GREY,C_DARK,110));
  }
  if(cfg.dialStyle==5){
    // Sparse diagonal carbon weave, intentionally faint in 8-bit sprite.
    uint16_t weave=blend565(cfg.background565,C_GREY,40);
    for(int n=-140;n<240;n+=14) {
      for(int j=12;j<=228;j+=14){
        int x=n+j;
        int dx=x-CX,dy=j-CY;
        if(x>=12&&x<=228&&dx*dx+dy*dy<105*105)
          frame.drawLine(x,j,x+2,j,weave);
      }
    }
  }
}

static void radialLine(float a,int startR,int endR,uint16_t color,int weight) {
  float sn=sinf(a*DEG_TO_RAD),cs=cosf(a*DEG_TO_RAD);
  int x1=CX-(int)lroundf(sn*startR),y1=CY+(int)lroundf(cs*startR);
  int x2=CX-(int)lroundf(sn*endR),y2=CY+(int)lroundf(cs*endR);
  frame.drawLine(x1,y1,x2,y2,color);
  if(weight>1)frame.drawLine(x1+1,y1,x2+1,y2,color);
}

static void dialScale(float fraction, float lower,float upper,bool isRpm) {
  uint16_t dim=blend565(cfg.foreground565,cfg.background565,140);
  float f=clampFloat(fraction,0.0f,1.0f);
  const int steps = isRpm?40:(currentPage==1?32:30);
  const int divs = isRpm?8:(currentPage==1?4:6);
  const int major=steps/divs;
  const int arcRadius=105;
  // Thin illuminated perimeter, simulated phosphor halo near redline.
  frame.drawArc(CX,CY,arcRadius,arcRadius-max(3,cfg.stroke-2),45,315,C_DARK,cfg.background565,false);
  if(f>.002f) {
    int endA=45+max(1,(int)(270*f));
    frame.drawArc(CX,CY,arcRadius,arcRadius-max(3,cfg.stroke-2),45,endA,cfg.accent565,cfg.background565,false);
  }
  if(isRpm) {
    float start=45.0f+270.0f*cfg.redline/cfg.rpmMax;
    frame.drawArc(CX,CY,108,106,(int)start,315,C_RED,cfg.background565,false);
  }
  for(int i=0;i<=steps;i++) {
    float a=45.0f+270.0f*(float)i/steps;
    bool primary=(i%major==0);
    bool inRed=isRpm&&(i*cfg.rpmMax/steps>=cfg.redline);
    uint16_t c=inRed?C_RED:(i<=(int)(f*steps)?cfg.accent565:dim);
    radialLine(a,primary?85:91,99,c,primary?2:1);
    if(primary&&cfg.numberedTicks){
      float value=lower+(upper-lower)*(float)i/steps;
      String label=isRpm?String((int)lroundf(value/1000.0f)):String((int)lroundf(value));
      float ar=a*DEG_TO_RAD;
      int x=CX-(int)lroundf(sinf(ar)*76),y=CY+(int)lroundf(cosf(ar)*76);
      frame.setTextFont(1);frame.setTextDatum(MC_DATUM);frame.setTextSize(2);
      frame.setTextColor(inRed?C_RED:cfg.foreground565);
      frame.drawString(label,x,y);
      frame.setTextSize(1);
    }
  }
}

static void drawNeedle(float f,uint16_t accent) {
  float a=(45.0f+270.0f*clampFloat(f,0,1))*DEG_TO_RAD;
  float sn=sinf(a),cs=cosf(a);
  int tipX=CX-(int)lroundf(sn*85), tipY=CY+(int)lroundf(cs*85);
  int tailX=CX+(int)lroundf(sn*17),tailY=CY-(int)lroundf(cs*17);
  int w=constrain(cfg.needleWidth,1,6);
  uint16_t metal=blend565(cfg.foreground565,C_GREY,100);
  if(cfg.needleStyle==2){
    frame.drawLine(tailX,tailY,tipX,tipY,accent);
    frame.drawLine(CX+1,CY,tipX+1,tipY,accent);
  } else {
    uint16_t main=cfg.needleStyle==1?metal:accent;
    int x1=CX+(int)lroundf(cs*w),y1=CY+(int)lroundf(sn*w);
    int x2=CX-(int)lroundf(cs*w),y2=CY-(int)lroundf(sn*w);
    frame.fillTriangle(tipX,tipY,x1,y1,x2,y2,main);
    frame.drawLine(tailX,tailY,CX,CY,accent);
    if(cfg.needleStyle==1)radialLine(45.0f+270.0f*f,28,74,accent,1);
  }
  // Machined centre hub and accent pin, hardware approximation of glossy glass.
  frame.fillCircle(CX,CY,11,C_DARK);
  frame.drawCircle(CX,CY,10,metal);
  frame.fillCircle(CX,CY,6,blend565(C_DARK,metal,60));
  frame.fillCircle(CX,CY,3,accent);
}

static void shiftIndicators(float rpm) {
  if (!cfg.shiftLights || currentPage!=0) return;
  for(int i=0;i<5;i++) {
    bool on=rpm>=cfg.redline-1200.0f+i*300.0f;
    uint16_t c=i>=3?C_RED:C_GREEN;
    if(on&&cfg.glow>30)frame.drawCircle(96+i*12,27,5,blend565(c,cfg.background565,115));
    frame.fillCircle(96+i*12,27,3,on?c:C_DARK);
  }
}

static void renderInstrument(uint32_t now) {
  Reading *r=currentPage==0?&engineRpm:currentPage==1?&coolantC:currentPage==2?&moduleV:&boostPsi;
  const bool ok=fresh(*r,now);
  float raw=ok?r->value:0;
  float lo=currentPage==0?0:currentPage==1?cfg.coolantMin:currentPage==2?cfg.voltageMin:cfg.boostMin;
  float hi=currentPage==0?cfg.rpmMax:currentPage==1?cfg.coolantMax:currentPage==2?cfg.voltageMax:cfg.boostMax;
  float *shown=currentPage==0?&shownRpm:currentPage==1?&shownCoolant:currentPage==2?&shownVoltage:&shownBoost;
  if(ok) *shown+=(raw-*shown)*0.31f;
  else *shown=lo;
  float f=ok?clampFloat((*shown-lo)/max(0.1f,hi-lo),0,1):0;
  bool caution=currentPage==0?(*shown>=cfg.redline):currentPage==1?(*shown>=110):currentPage==2?(*shown<12.1f):(*shown>=cfg.turboThreshold);
  uint16_t accent=caution&&ok?C_RED:cfg.accent565;
  dialScale(f,lo,hi,currentPage==0);
  if(currentPage==0)shiftIndicators(ok?*shown:0);
  String title=currentPage==0?cfg.rpmTitle:currentPage==1?cfg.coolantTitle:currentPage==2?cfg.voltageTitle:cfg.boostTitle;
  centerFit(title,cfg.titleY,2,blend565(cfg.foreground565,C_GREY,95),140);
  if(ok)drawNeedle(f,accent);
  if(cfg.digitalWindow) {
    frame.fillRoundRect(65,cfg.valueY-17,110,34,5,C_DARK);
    frame.drawRoundRect(65,cfg.valueY-17,110,34,5,blend565(C_GREY,cfg.accent565,45));
    char buf[22];
    if(!ok)strcpy(buf,"---");
    else if(currentPage==0)snprintf(buf,sizeof(buf),"%u",(unsigned)lroundf(*shown));
    else if(currentPage==1)snprintf(buf,sizeof(buf),"%uC",(unsigned)lroundf(*shown));
    else if(currentPage==2)snprintf(buf,sizeof(buf),"%.2f",*shown);
    else snprintf(buf,sizeof(buf),"%.1f",*shown);
    centerFit(buf,cfg.valueY,cfg.fontScale>=110?4:3,ok?cfg.foreground565:C_GREY,104);
  }
  centerFit(currentPage==0?"x1000 RPM":currentPage==1?"WATER C":currentPage==2?"BATTERY V":"BOOST PSI",cfg.unitY,1,accent,125);
}


// V3 physical instrument families. All primitives are within the round 240x240 face.
static void renderV3Pods(float value, bool valid, float low, float high) {
  float f=valid?clampFloat((value-low)/max(0.1f,high-low),0,1):0;
  uint16_t pale=blend565(cfg.foreground565,cfg.background565,95);
  frame.drawArc(CX,CY,105,100,45,315,C_DARK,cfg.background565,false);
  frame.drawArc(CX,CY,105,100,45,45+(int)(270*f),cfg.accent565,cfg.background565,false);
  centerFit(currentPage==0?cfg.rpmTitle:currentPage==1?cfg.coolantTitle:currentPage==2?cfg.voltageTitle:cfg.boostTitle,40,2,pale,150);
  char number[25];
  if(!valid)strcpy(number,"--");
  else if(currentPage==0)snprintf(number,sizeof(number),"%u",(unsigned)lroundf(value));
  else if(currentPage==1)snprintf(number,sizeof(number),"%u",(unsigned)lroundf(value));
  else if(currentPage==2)snprintf(number,sizeof(number),"%.2f",value);
  else snprintf(number,sizeof(number),"%.1f",value);
  centerFit(number,117,currentPage==0?4:5,cfg.foreground565,185);
  centerFit(currentPage==0?"RPM":currentPage==1?"DEG C":currentPage==2?"VOLTS":"BOOST PSI",145,2,cfg.accent565,140);
  frame.drawFastHLine(48,167,(int)(144*f),cfg.accent565);
  frame.drawFastHLine(48,168,144,C_DIM);
  centerFit("RPM "+(fresh(engineRpm,millis())?String((int)engineRpm.value):"--")+"  PSI "+(fresh(boostPsi,millis())?String(boostPsi.value,1):"--"),185,1,pale,174);
}
static void smallDial(int x,int y,int radius,float value,float lo,float hi,uint16_t color,const char* label) {
  frame.drawCircle(x,y,radius,C_DIM);
  frame.drawCircle(x,y,radius-2,C_GREY);
  float f=clampFloat((value-lo)/max(.1f,hi-lo),0,1);
  float a=(45+270*f)*DEG_TO_RAD;
  int dx=(int)lroundf(-sinf(a)*(radius-9)),dy=(int)lroundf(cosf(a)*(radius-9));
  frame.drawLine(x,y,x+dx,y+dy,color);frame.fillCircle(x,y,3,cfg.foreground565);
  frame.setTextSize(1);frame.setTextColor(color);frame.setTextDatum(MC_DATUM);
  frame.drawString(label,x,y+radius+11);
}
static void turbineIcon(int x,int y,int radius,uint32_t now) {
  uint16_t steel=blend565(C_GREY,cfg.foreground565,80);
  frame.fillCircle(x,y,radius,C_DIM);frame.drawCircle(x,y,radius,steel);
  frame.drawCircle(x,y,radius-5,C_GREY);
  float spin=(now*0.02f + (fresh(boostPsi,now)?max(0.0f,boostPsi.value)*now*.0005f:0))*DEG_TO_RAD;
  for(int i=0;i<9;i++){
    float a=spin+(float)i*6.2831853f/9.0f;
    int ix=x+(int)(cosf(a)*(radius*.12f)),iy=y+(int)(sinf(a)*(radius*.12f));
    int tx=x+(int)(cosf(a+.52f)*(radius*.73f)),ty=y+(int)(sinf(a+.52f)*(radius*.73f));
    int ex=x+(int)(cosf(a+.81f)*(radius*.48f)),ey=y+(int)(sinf(a+.81f)*(radius*.48f));
    frame.fillTriangle(ix,iy,tx,ty,ex,ey,(i%2==0)?steel:cfg.accent565);
  }
  frame.fillCircle(x,y,5,C_DARK);frame.fillCircle(x,y,2,steel);
}
// V5 Ghost Sweep instrument: thin segmented blue/amber/red dial from the
// screenshot reference, large white digital RPM and compact internal numerals.
// Uses only integer framebuffer primitives; colour sweep is an ACTUAL layout.
static void renderGhostSweep(uint32_t now) {
  Reading *r = currentPage==0?&engineRpm:currentPage==1?&coolantC:currentPage==2?&moduleV:&boostPsi;
  const bool ok=fresh(*r,now);
  const float lo=currentPage==0?0:currentPage==1?cfg.coolantMin:currentPage==2?cfg.voltageMin:cfg.boostMin;
  const float hi=currentPage==0?cfg.rpmMax:currentPage==1?cfg.coolantMax:currentPage==2?cfg.voltageMax:cfg.boostMax;
  const float fraction=ok?clampFloat((r->value-lo)/max(.1f,hi-lo),0,1):0;
  const uint16_t ice=0x3E7F;      // bright cyan
  const uint16_t blue=0x139C;     // electric blue
  const uint16_t amber=0xFD66;
  const uint16_t hot=0xE269;
  // The old bezel (if enabled) is deliberately replaced for a black-faced,
  // nearly flush instrument with a narrow textured outer surround.
  frame.fillSprite(0x0000);
  frame.fillCircle(CX,CY,118,0x1884);
  frame.fillCircle(CX,CY,114,0x0842);
  frame.fillCircle(CX,CY,109,0x0020);
  frame.drawCircle(CX,CY,108,C_DIM);
  // Screen angle uses 0=right, 90=down, 180=left, 270=up;
  // an arc of 150..390 degrees wraps naturally across the upper dial.
  const float startDeg=150.0f, sweepDeg=240.0f;
  for(int step=0;step<=120;step++) {
    const float f=step/120.0f;
    const float a=(startDeg+sweepDeg*f)*DEG_TO_RAD;
    const int x=CX+(int)lroundf(cosf(a)*92.0f);
    const int y=CY+(int)lroundf(sinf(a)*92.0f);
    const uint16_t c=f<.10f?ice:f<.64f?blue:f<.79f?amber:hot;
    // The entire reference arc remains visible even at idle.
    frame.fillCircle(x,y,3,c);
    if(ok && fabsf(f-fraction)<=.018f)frame.fillCircle(x,y,5,blend565(c,0xFFFF,75));
  }
  const int ticks=currentPage==0?constrain((int)lroundf(cfg.rpmMax/1000.0f),5,10):6;
  frame.setTextDatum(MC_DATUM);
  frame.setTextFont(1);frame.setTextSize(2);
  for(int i=0;i<=ticks;i++){
    const float fractionTick=(float)i/ticks;
    const float angle=(startDeg+sweepDeg*fractionTick)*DEG_TO_RAD;
    const int x=CX+(int)lroundf(cosf(angle)*69.0f);
    const int y=CY+(int)lroundf(sinf(angle)*69.0f);
    const float vv=lo+(hi-lo)*fractionTick;
    const String label=currentPage==0?String((int)lroundf(vv/1000.0f)):String((int)lroundf(vv));
    frame.setTextColor(fractionTick<.64f?0x757F:fractionTick<.79f?amber:hot);
    frame.drawString(label,x,y);
  }
  frame.setTextSize(1);
  String number="--";
  if(ok){
    if(currentPage==0){number=String((int)lroundf(r->value));}
    else if(currentPage==1)number=String((int)lroundf(r->value));
    else if(currentPage==2)number=String(r->value,2);
    else number=String(r->value,1);
  }
  centerFit(number,125,5,0xFFFF,168);
  centerFit(currentPage==0?"RPM":currentPage==1?"DEG C":currentPage==2?"VOLTS":"BOOST PSI",182,3,0xEF7D,133);
  centerFit(ok?"LIVE":"NO DATA",203,1,ok?C_GREEN:C_GREY,92);
}

static void renderV3Variant(uint32_t now,int style) {
  Reading *r=currentPage==0?&engineRpm:currentPage==1?&coolantC:currentPage==2?&moduleV:&boostPsi;
  bool ok=fresh(*r,now);float v=ok?r->value:0;
  float lo=currentPage==0?0:currentPage==1?cfg.coolantMin:currentPage==2?cfg.voltageMin:cfg.boostMin;
  float hi=currentPage==0?cfg.rpmMax:currentPage==1?cfg.coolantMax:currentPage==2?cfg.voltageMax:cfg.boostMax;
  if(style==8){renderGhostSweep(now);}
  else if(style==1||style==4||style==5){renderV3Pods(v,ok,lo,hi);
    if(style==4){frame.drawArc(CX,125,99,91,225,355,cfg.accent565,cfg.background565,false);}
    if(style==5){for(int i=0;i<17;i++)frame.fillRoundRect(25,60+i*7,12,5,2,(i<17*(ok?clampFloat((v-lo)/(hi-lo),0,1):0))?cfg.accent565:C_DARK);}
  } else if(style==2){
    smallDial(73,101,40,fresh(engineRpm,now)?engineRpm.value:0,0,cfg.rpmMax,cfg.accent565,"RPM");
    smallDial(168,101,40,fresh(boostPsi,now)?boostPsi.value:cfg.boostMin,cfg.boostMin,cfg.boostMax,C_ORANGE,"BOOST");
    centerFit(ok?(currentPage==0?String((int)v):String(v,1)):"--",168,3,cfg.foreground565,125);
    centerFit(currentPage==3?"PSI":"PRIMARY",191,1,cfg.accent565,120);
  } else if(style==7){
    // ECUSTUDIO data tile instrument: simple geometry at 240x240 for 8-bit sprite.
    frame.setTextDatum(MC_DATUM);
    frame.fillRoundRect(41,62,158,128,10,C_DARK);
    frame.drawRoundRect(41,62,158,128,10,C_GREY);
    frame.drawFastVLine(120,68,115,C_DIM);
    frame.drawFastHLine(47,125,146,C_DIM);
    centerFit("ECU STUDIO",40,2,cfg.foreground565,165);
    centerFit("SUBARU / DIMEMOD",55,1,cfg.accent565,148);
    frame.setTextColor(C_GREY);
    frame.setTextSize(1);
    frame.drawString("RPM",82,81);
    frame.drawString("BOOST PSI",159,81);
    frame.drawString("COOLANT C",82,139);
    frame.drawString("BATTERY V",159,139);
    frame.setTextColor(cfg.accent565);
    frame.setTextSize(2);
    frame.drawString(fresh(engineRpm,now)?String((int)engineRpm.value):"--",82,106);
    frame.drawString(fresh(boostPsi,now)?String(boostPsi.value,1):"--",159,106);
    frame.setTextColor(cfg.foreground565);
    frame.drawString(fresh(coolantC,now)?String((int)coolantC.value):"--",82,165);
    frame.drawString(fresh(moduleV,now)?String(moduleV.value,1):"--",159,165);
    centerFit("CAN 7E0/7E8 OBD2",199,1,C_GREY,160);
  } else if(style==3){
    turbineIcon(CX,109,65,now);
    frame.fillRoundRect(68,162,104,32,5,C_DARK);
    centerFit(ok?String(v,currentPage==0?0:1):"--",177,3,cfg.foreground565,100);
    centerFit(currentPage==3?"BOOST PSI":"TURBO FORGE",200,1,cfg.accent565,130);
  } else { // Triple cluster
    smallDial(120,80,42,ok?v:lo,lo,hi,cfg.accent565,"PRIMARY");
    smallDial(72,163,31,fresh(coolantC,now)?coolantC.value:40,40,120,C_GREEN,"WATER");
    smallDial(171,163,31,fresh(boostPsi,now)?boostPsi.value:-15,-15,25,C_ORANGE,"BOOST");
    centerFit(ok?String(v,currentPage==0?0:1):"--",122,2,cfg.foreground565,100);
  }
}
static void renderTurboBurst(uint32_t now) {
  if(!cfg.turboEnabled||!fresh(boostPsi,now)||turboTriggeredAt==0||elapsed(now,turboTriggeredAt,(uint32_t)cfg.turboDuration))return;
  uint32_t t=now-turboTriggeredAt;
  // 1.2s transient overlay, clears automatically. Does not block gauge indefinitely.
  int radius=(int)((35+min(48.0f,t*.10f))*(cfg.turboIntensity/85.0f));
  radius=constrain(radius,24,82);
  frame.fillCircle(CX,111,101,cfg.background565);
  turbineIcon(CX,111,radius,now);
  frame.fillRoundRect(62,187,116,23,5,C_DARK);
  centerFit("BOOST "+String(boostPsi.value,1)+" PSI",199,1,cfg.foreground565,110);
}

static void drawFooter() {
  const char* message="BT CONNECTING";
  switch(linkState){
    case LinkState::RETRY_WAIT: message="BT RETRY";break;
    case LinkState::CONNECTING: message="BT CONNECTING";break;
    case LinkState::INITIALIZING: message="ELM SETUP";break;
    case LinkState::POLLING: message="OBD CONNECTED";break;
    case LinkState::BT_FAILURE: message="BT INIT ERROR";break;
  }
  // Show connection status even with no OBD adapter. This is useful when
  // diagnosing Samsung Wi-Fi association without a USB serial monitor.
  if (WiFi.softAPgetStationNum() > 0)
    centerFit(String("WIFI PHONE ") + WiFi.softAPgetStationNum(),211,1,C_GREEN,130);
  else if (millis() - wifiBootAt < 90000)
    centerFit("WIFI READY - JOIN",211,1,C_GREEN,130);
  else if (lastApLeaveReason != 0)
    centerFit(String("WIFI ERR ") + lastApLeaveReason,211,1,C_ORANGE,130);
  else
    centerFit("WIFI NO PHONE",211,1,C_GREY,130);
  for(int i=0;i<4;i++)frame.fillCircle(102+12*i,226,2,i==currentPage?cfg.accent565:C_DIM);
}

static void renderScreen(uint32_t now) {
  if(!frameReady)return;
  if(lastFrameAt!=0&&!elapsed(now,lastFrameAt,FRAME_MS))return;
  lastFrameAt=now;
  dialBase();
  uint8_t layout = currentPage==0?cfg.rpmLayout:currentPage==1?cfg.coolantLayout:currentPage==2?cfg.voltageLayout:cfg.boostLayout;
  if(layout==0)renderInstrument(now);
  else renderV3Variant(now,layout);
  renderTurboBurst(now);
  drawFooter();
  frame.pushSprite(0,0);
}

// ------------------ WI-FI / LIVE EDITOR PROTOCOL ------------------
// Separate Wi-Fi AP for the phone, Classic BT SPP for ELM327. No cloud.
// GET /api/status     live telemetry (null when stale / unsupported)
// GET /api/config     saved & current editable settings
// POST /api/preview   apply while editing, do NOT write flash/NVS
// POST /api/config    apply AND persist to NVS
// POST /api/page      set page=0..3 to view the edited page on hardware

void loadConfig() {
  preferences.begin("rounddash", true);
  cfg.accent565 = preferences.getUShort("accent", cfg.accent565);
  cfg.background565 = preferences.getUShort("bg", cfg.background565);
  cfg.foreground565 = preferences.getUShort("fg", cfg.foreground565);
  cfg.rpmMax = preferences.getFloat("rpmmax", cfg.rpmMax);
  cfg.redline = preferences.getFloat("redline", cfg.redline);
  cfg.coolantMin = preferences.getFloat("cmin", cfg.coolantMin);
  cfg.coolantMax = preferences.getFloat("cmax", cfg.coolantMax);
  cfg.voltageMin = preferences.getFloat("vmin", cfg.voltageMin);
  cfg.voltageMax = preferences.getFloat("vmax", cfg.voltageMax);
  cfg.stroke = preferences.getInt("stroke", cfg.stroke);
  cfg.fontScale = preferences.getInt("font", cfg.fontScale);
  cfg.titleY = preferences.getInt("titley", cfg.titleY);
  cfg.valueY = preferences.getInt("valuey", cfg.valueY);
  cfg.unitY = preferences.getInt("unity", cfg.unitY);
  cfg.autoCycle = preferences.getBool("cycle", cfg.autoCycle);
  cfg.cycleMs = preferences.getUInt("cyclems", cfg.cycleMs);
  cfg.rpmTitle = preferences.getString("rpmtitle", cfg.rpmTitle);
  cfg.coolantTitle = preferences.getString("ctitle", cfg.coolantTitle);
  cfg.voltageTitle = preferences.getString("vtitle", cfg.voltageTitle);
  cfg.dialStyle = (uint8_t)preferences.getUChar("dialstyle", cfg.dialStyle);
  cfg.needleStyle = (uint8_t)preferences.getUChar("needle", cfg.needleStyle);
  cfg.needleWidth = preferences.getInt("needlew", cfg.needleWidth);
  cfg.glow = preferences.getInt("glow", cfg.glow);
  cfg.numberedTicks = preferences.getBool("numbers", cfg.numberedTicks);
  cfg.shiftLights = preferences.getBool("shift", cfg.shiftLights);
  cfg.bezel = preferences.getBool("bezel", cfg.bezel);
  cfg.digitalWindow = preferences.getBool("digital", cfg.digitalWindow);
  cfg.rpmLayout=preferences.getUChar("rpmly",cfg.rpmLayout);
  cfg.coolantLayout=preferences.getUChar("coolly",cfg.coolantLayout);
  cfg.voltageLayout=preferences.getUChar("voltly",cfg.voltageLayout);
  cfg.boostLayout=preferences.getUChar("boostly",cfg.boostLayout);
  cfg.boostTitle=preferences.getString("btitle",cfg.boostTitle);
  cfg.boostMin=preferences.getFloat("bmin",cfg.boostMin);
  cfg.boostMax=preferences.getFloat("bmax",cfg.boostMax);
  cfg.ambientKpa=preferences.getFloat("amb",cfg.ambientKpa);
  cfg.turboEnabled=preferences.getBool("turboon",cfg.turboEnabled);
  cfg.boostPeakHold=preferences.getBool("peakh",cfg.boostPeakHold);
  cfg.turboThreshold=preferences.getFloat("ttrigger",cfg.turboThreshold);
  cfg.turboIntensity=preferences.getInt("tintens",cfg.turboIntensity);
  cfg.turboDuration=preferences.getInt("tdur",cfg.turboDuration);
  cfg.vehiclePreset = preferences.getString("vehicle", cfg.vehiclePreset);
  cfg.vehicleName = preferences.getString("vehname", cfg.vehicleName);
  cfg.romId = preferences.getString("romid", cfg.romId);
  cfg.ecuId = preferences.getString("ecuid", cfg.ecuId);
  cfg.obdName = preferences.getString("obdname", cfg.obdName);
  cfg.obdPin = preferences.getString("obdpin", cfg.obdPin);
  preferences.end();
}

void saveConfig() {
  preferences.begin("rounddash", false);
  preferences.putUShort("accent", cfg.accent565);
  preferences.putUShort("bg", cfg.background565);
  preferences.putUShort("fg", cfg.foreground565);
  preferences.putFloat("rpmmax", cfg.rpmMax);
  preferences.putFloat("redline", cfg.redline);
  preferences.putFloat("cmin", cfg.coolantMin);
  preferences.putFloat("cmax", cfg.coolantMax);
  preferences.putFloat("vmin", cfg.voltageMin);
  preferences.putFloat("vmax", cfg.voltageMax);
  preferences.putInt("stroke", cfg.stroke);
  preferences.putInt("font", cfg.fontScale);
  preferences.putInt("titley", cfg.titleY);
  preferences.putInt("valuey", cfg.valueY);
  preferences.putInt("unity", cfg.unitY);
  preferences.putBool("cycle", cfg.autoCycle);
  preferences.putUInt("cyclems", cfg.cycleMs);
  preferences.putString("rpmtitle", cfg.rpmTitle);
  preferences.putString("ctitle", cfg.coolantTitle);
  preferences.putString("vtitle", cfg.voltageTitle);
  preferences.putUChar("dialstyle", cfg.dialStyle);
  preferences.putUChar("needle", cfg.needleStyle);
  preferences.putInt("needlew", cfg.needleWidth);
  preferences.putInt("glow", cfg.glow);
  preferences.putBool("numbers", cfg.numberedTicks);
  preferences.putBool("shift", cfg.shiftLights);
  preferences.putBool("bezel", cfg.bezel);
  preferences.putBool("digital", cfg.digitalWindow);
  preferences.putUChar("rpmly",cfg.rpmLayout);
  preferences.putUChar("coolly",cfg.coolantLayout);
  preferences.putUChar("voltly",cfg.voltageLayout);
  preferences.putUChar("boostly",cfg.boostLayout);
  preferences.putString("btitle",cfg.boostTitle);
  preferences.putFloat("bmin",cfg.boostMin);
  preferences.putFloat("bmax",cfg.boostMax);
  preferences.putFloat("amb",cfg.ambientKpa);
  preferences.putBool("turboon",cfg.turboEnabled);
  preferences.putBool("peakh",cfg.boostPeakHold);
  preferences.putFloat("ttrigger",cfg.turboThreshold);
  preferences.putInt("tintens",cfg.turboIntensity);
  preferences.putInt("tdur",cfg.turboDuration);
  preferences.putString("vehicle", cfg.vehiclePreset);
  preferences.putString("vehname", cfg.vehicleName);
  preferences.putString("romid", cfg.romId);
  preferences.putString("ecuid", cfg.ecuId);
  preferences.putString("obdname", cfg.obdName);
  preferences.putString("obdpin", cfg.obdPin);
  preferences.end();
}

static String jsonQuote(const String &input) {
  String out = "\"";
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input.charAt(i);
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if ((uint8_t)c < 32) out += ' ';
    else out += c;
  }
  out += '"';
  return out;
}

static String colorHex(uint16_t rgb) {
  // Expand RGB565 to 24-bit RGB for Android canvas preview.
  unsigned int r = (rgb >> 11) & 31, g = (rgb >> 5) & 63, b = rgb & 31;
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", (r * 255 + 15) / 31,
           (g * 255 + 31) / 63, (b * 255 + 15) / 31);
  return String(buf);
}

void sendJson(int status, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(status, "application/json", body);
}

void handleStatus() {
  uint32_t now = millis();
  String out = "{\"ok\":true,\"fw\":\"5.0\",\"protocol\":\"OBD2_Mode01\",\"ssmStreaming\":false,\"bt\":";
  out += linkState == LinkState::POLLING && bt.connected(0) ? "true" : "false";
  out += ",\"page\":" + String(currentPage);
  out += ",\\\"heap\\\":" + String(ESP.getFreeHeap());
  out += ",\\\"wifiClients\\\":" + String(WiFi.softAPgetStationNum());
  out += ",\\\"wifiLeaveReason\\\":" + String((int)lastApLeaveReason);
  out += ",\\\"wifiJoins\\\":" + String((unsigned long)apJoins);
  out += ",\\\"wifiLeaves\\\":" + String((unsigned long)apLeaves);
  out += ",\"rpm\":";
  out += fresh(engineRpm, now) ? String(engineRpm.value, 0) : "null";
  out += ",\"coolant\":";
  out += fresh(coolantC, now) ? String(coolantC.value, 1) : "null";
  out += ",\"voltage\":";
  out += fresh(moduleV, now) ? String(moduleV.value, 3) : "null";
  out += ",\"boost\":";
  out += fresh(boostPsi, now) ? String(boostPsi.value, 2) : "null";
  out += ",\"boostEstimated\":";
  out += boostEstimated ? "true" : "false";
  out += ",\"mapKpa\":";
  out += fresh(mapKpa, now) ? String(mapKpa.value, 0) : "null";
  out += ",\"baroKpa\":";
  out += fresh(baroKpa, now) ? String(baroKpa.value, 0) : "null";
  out += "}";
  sendJson(200, out);
}

void handleGetConfig() {
  String out = "{\"ok\":true,";
  out += "\"accent\":" + jsonQuote(colorHex(cfg.accent565));
  out += ",\"background\":" + jsonQuote(colorHex(cfg.background565));
  out += ",\"foreground\":" + jsonQuote(colorHex(cfg.foreground565));
  out += ",\"rpmMax\":" + String(cfg.rpmMax, 0);
  out += ",\"redline\":" + String(cfg.redline, 0);
  out += ",\"coolantMin\":" + String(cfg.coolantMin, 1);
  out += ",\"coolantMax\":" + String(cfg.coolantMax, 1);
  out += ",\"voltageMin\":" + String(cfg.voltageMin, 2);
  out += ",\"voltageMax\":" + String(cfg.voltageMax, 2);
  out += ",\"stroke\":" + String(cfg.stroke);
  out += ",\"fontScale\":" + String(cfg.fontScale);
  out += ",\"titleY\":" + String(cfg.titleY);
  out += ",\"valueY\":" + String(cfg.valueY);
  out += ",\"unitY\":" + String(cfg.unitY);
  out += ",\"autoCycle\":" + String(cfg.autoCycle ? "true" : "false");
  out += ",\"cycleMs\":" + String(cfg.cycleMs);
  out += ",\"rpmTitle\":" + jsonQuote(cfg.rpmTitle);
  out += ",\"coolantTitle\":" + jsonQuote(cfg.coolantTitle);
  out += ",\"voltageTitle\":" + jsonQuote(cfg.voltageTitle);
  out += ",\"dialStyle\":" + String(cfg.dialStyle);
  out += ",\"needleStyle\":" + String(cfg.needleStyle);
  out += ",\"needleWidth\":" + String(cfg.needleWidth);
  out += ",\"glow\":" + String(cfg.glow);
  out += ",\"numberedTicks\":" + String(cfg.numberedTicks?"true":"false");
  out += ",\"shiftLights\":" + String(cfg.shiftLights?"true":"false");
  out += ",\"bezel\":" + String(cfg.bezel?"true":"false");
  out += ",\"digitalWindow\":" + String(cfg.digitalWindow?"true":"false");
  out += ",\"rpmLayout\":" + String(cfg.rpmLayout);
  out += ",\"coolantLayout\":" + String(cfg.coolantLayout);
  out += ",\"voltageLayout\":" + String(cfg.voltageLayout);
  out += ",\"boostLayout\":" + String(cfg.boostLayout);
  out += ",\"boostTitle\":" + jsonQuote(cfg.boostTitle);
  out += ",\"boostMin\":" + String(cfg.boostMin,1);
  out += ",\"boostMax\":" + String(cfg.boostMax,1);
  out += ",\"ambientKpa\":" + String(cfg.ambientKpa,1);
  out += ",\"turboEnabled\":" + String(cfg.turboEnabled?"true":"false");
  out += ",\"boostPeakHold\":" + String(cfg.boostPeakHold?"true":"false");
  out += ",\"turboThreshold\":" + String(cfg.turboThreshold,1);
  out += ",\"turboIntensity\":" + String(cfg.turboIntensity);
  out += ",\"turboDuration\":" + String(cfg.turboDuration);
  out += ",\"vehiclePreset\":" + jsonQuote(cfg.vehiclePreset);
  out += ",\"vehicleName\":" + jsonQuote(cfg.vehicleName);
  out += ",\"romId\":" + jsonQuote(cfg.romId);
  out += ",\"ecuId\":" + jsonQuote(cfg.ecuId);
  out += ",\"obdName\":" + jsonQuote(cfg.obdName);
  out += ",\"obdPin\":" + jsonQuote(cfg.obdPin);
  out += "}";
  sendJson(200, out);
}

static bool parseNumber(String s, float &value) {
  s.trim();
  if (s.isEmpty() || s.length() > 24) return false;
  char *end = nullptr;
  value = strtof(s.c_str(), &end);
  return end != s.c_str() && *end == '\0' && isfinite(value);
}

static bool numberInRange(const String &s, float minVal, float maxVal, float &out) {
  return parseNumber(s, out) && out >= minVal && out <= maxVal;
}

static bool safeLabel(String &value, size_t maxlen) {
  value.trim();
  if (value.length() > maxlen || value.isEmpty()) return false;
  for (size_t i=0; i<value.length(); ++i) {
    unsigned char c = value.charAt(i);
    if (c < 32 || c > 126) return false;
  }
  return true;
}

static bool applyItem(GaugeConfig &next, const String &key, String value) {
  float n;
  if (key == "accent565" || key == "background565" || key == "foreground565") {
    if (!numberInRange(value, 0, 65535, n)) return false;
    uint16_t v = (uint16_t)n;
    if (key == "accent565") next.accent565 = v;
    else if (key == "background565") next.background565 = v;
    else next.foreground565 = v;
  } else if (key == "rpmMax") { if (!numberInRange(value, 5000, 10000, n)) return false; next.rpmMax = n; }
  else if (key == "redline") { if (!numberInRange(value, 4500, 8500, n)) return false; next.redline = n; }
  else if (key == "coolantMin") { if (!numberInRange(value, 0, 70, n)) return false; next.coolantMin = n; }
  else if (key == "coolantMax") { if (!numberInRange(value, 90, 140, n)) return false; next.coolantMax = n; }
  else if (key == "voltageMin") { if (!numberInRange(value, 9, 12, n)) return false; next.voltageMin = n; }
  else if (key == "voltageMax") { if (!numberInRange(value, 14, 18, n)) return false; next.voltageMax = n; }
  else if (key == "stroke") { if (!numberInRange(value, 4, 16, n)) return false; next.stroke = (int)n; }
  else if (key == "fontScale") { if (!numberInRange(value, 70, 125, n)) return false; next.fontScale = (int)n; }
  else if (key == "titleY") { if (!numberInRange(value, 63, 90, n)) return false; next.titleY = (int)n; }
  else if (key == "valueY") { if (!numberInRange(value, 152, 174, n)) return false; next.valueY = (int)n; }
  else if (key == "unitY") { if (!numberInRange(value, 178, 199, n)) return false; next.unitY = (int)n; }
  else if (key == "autoCycle") { if (value != "true" && value != "false") return false; next.autoCycle = value == "true"; }
  else if (key == "cycleMs") { if (!numberInRange(value, 3000, 20000, n)) return false; next.cycleMs = (uint32_t)n; }
  else if (key == "rpmTitle") { if (!safeLabel(value, 20)) return false; next.rpmTitle = value; }
  else if (key == "coolantTitle") { if (!safeLabel(value, 20)) return false; next.coolantTitle = value; }
  else if (key == "voltageTitle") { if (!safeLabel(value, 20)) return false; next.voltageTitle = value; }
  else if (key == "dialStyle") { if (!numberInRange(value, 0, 5, n)) return false; next.dialStyle = (uint8_t)n; }
  else if (key == "needleStyle") { if (!numberInRange(value, 0, 2, n)) return false; next.needleStyle = (uint8_t)n; }
  else if (key == "needleWidth") { if (!numberInRange(value, 1, 6, n)) return false; next.needleWidth = (int)n; }
  else if (key == "glow") { if (!numberInRange(value, 0, 100, n)) return false; next.glow = (int)n; }
  else if (key == "numberedTicks") { if (value!="true"&&value!="false") return false; next.numberedTicks=value=="true"; }
  else if (key == "shiftLights") { if (value!="true"&&value!="false") return false; next.shiftLights=value=="true"; }
  else if (key == "bezel") { if (value!="true"&&value!="false") return false; next.bezel=value=="true"; }
  else if (key == "digitalWindow") { if (value!="true"&&value!="false") return false; next.digitalWindow=value=="true"; }
  else if (key == "rpmLayout" || key == "coolantLayout" || key == "voltageLayout" || key == "boostLayout") {
    if(!numberInRange(value,0,8,n))return false;
    if(key=="rpmLayout")next.rpmLayout=(uint8_t)n;
    else if(key=="coolantLayout")next.coolantLayout=(uint8_t)n;
    else if(key=="voltageLayout")next.voltageLayout=(uint8_t)n;
    else next.boostLayout=(uint8_t)n;
  }
  else if (key == "boostTitle") {if(!safeLabel(value,20))return false;next.boostTitle=value;}
  else if (key == "boostMin") {if(!numberInRange(value,-15,0,n))return false;next.boostMin=n;}
  else if (key == "boostMax") {if(!numberInRange(value,10,45,n))return false;next.boostMax=n;}
  else if (key == "ambientKpa") {if(!numberInRange(value,95,105,n))return false;next.ambientKpa=n;}
  else if (key == "turboEnabled") {if(value!="true"&&value!="false")return false;next.turboEnabled=value=="true";}
  else if (key == "boostPeakHold") {if(value!="true"&&value!="false")return false;next.boostPeakHold=value=="true";}
  else if (key == "turboThreshold") {if(!numberInRange(value,1,20,n))return false;next.turboThreshold=n;}
  else if (key == "turboIntensity") {if(!numberInRange(value,35,100,n))return false;next.turboIntensity=(int)n;}
  else if (key == "turboDuration") {if(!numberInRange(value,400,2000,n))return false;next.turboDuration=(int)n;}
  else if (key == "vehiclePreset") {
    if (value != "sti_dm" && value != "generic") return false;
    next.vehiclePreset = value;
  }
  else if (key == "vehicleName") { if (!safeLabel(value, 32)) return false; next.vehicleName = value; }
  else if (key == "romId") { if (!safeLabel(value, 32)) return false; next.romId = value; }
  else if (key == "ecuId") { if (!safeLabel(value, 32)) return false; next.ecuId = value; }
  else if (key == "obdName") { if (!safeLabel(value, 24)) return false; next.obdName = value; }
  else if (key == "obdPin") { if (!safeLabel(value, 16)) return false; next.obdPin = value; }
  else return false; // unknown fields are rejected
  return true;
}

void handleWriteConfig(bool persist) {
  if (!server.hasArg("plain")) { sendJson(400, "{\"ok\":false,\"message\":\"Missing request body\"}"); return; }
  String body = server.arg("plain");
  if (body.length() > 2000) { sendJson(413, "{\"ok\":false,\"message\":\"Too many settings\"}"); return; }
  GaugeConfig next = cfg;
  size_t at = 0;
  while (at < body.length()) {
    int eol = body.indexOf('\n', at);
    if (eol < 0) eol = body.length();
    String line = body.substring(at, eol);
    line.trim();
    if (!line.isEmpty()) {
      int eq = line.indexOf('=');
      if (eq < 1 || !applyItem(next, line.substring(0, eq), line.substring(eq + 1))) {
        sendJson(400, "{\"ok\":false,\"message\":\"Invalid setting\"}"); return;
      }
    }
    at = eol + 1;
  }
  if (next.coolantMax <= next.coolantMin || next.voltageMax <= next.voltageMin || next.boostMax <= next.boostMin || next.redline > next.rpmMax) {
    sendJson(400, "{\"ok\":false,\"message\":\"Conflicting scale limits\"}"); return;
  }
  bool obdChanged = next.obdName != cfg.obdName || next.obdPin != cfg.obdPin;
  cfg = next;
  if (persist) saveConfig(); // preview editing NEVER writes NVS/flash
  lastFrameAt = 0;
  sendJson(200, String("{\"ok\":true,\"message\":") +
    jsonQuote(obdChanged ? "Updated; restart ESP32 to use new OBD pairing" :
      (persist ? "Design saved to ESP32" : "Live preview updated")) + "}");
}

void handlePage() {
  String s = server.arg("plain"); s.trim();
  if (s == "page=0" || s == "page=1" || s == "page=2" || s == "page=3") {
    currentPage = s.charAt(5) - '0';
    pageChangedAt = millis(); phonePageHoldUntil = pageChangedAt + 60000; lastFrameAt = 0;
    sendJson(200, "{\"ok\":true}");
  } else sendJson(400, "{\"ok\":false,\"message\":\"Page must be 0..3\"}");
}

void onSoftAPEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    apJoins++;
    Serial.printf("[WIFI] Samsung/phone associated; total joins=%lu\\n", (unsigned long)apJoins);
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    apLeaves++;
    lastApLeaveReason = info.wifi_ap_stadisconnected.reason;
    Serial.printf("[WIFI] Client disconnected reason=%u leaves=%lu\\n",
                  (unsigned)lastApLeaveReason, (unsigned long)apLeaves);
  }
}
void initHttp() {
  wifiBootAt = millis();
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onSoftAPEvent);
  // Fixed 2.4GHz channel, visible WPA2 network, up to 4 phones.
  bool apOK = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 4);
  Serial.printf("[WIFI] AP start=%s ssid=%s channel=6 ip=%s\\n",
                apOK ? "YES" : "NO", AP_SSID, WiFi.softAPIP().toString().c_str());
  if (!apOK) Serial.println("ERROR: Could not start Wi-Fi AP");
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, [](){handleWriteConfig(true);});
  server.on("/api/preview", HTTP_POST, [](){handleWriteConfig(false);});
  server.on("/api/page", HTTP_POST, handlePage);
  server.onNotFound([](){sendJson(404,"{\"ok\":false,\"message\":\"Not found\"}");});
  server.begin();
  Serial.printf("RoundDash Wi-Fi: %s / IP %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

// -------------------------- ARDUINO ------------------------------
void setup() {
  Serial.begin(115200);
  loadConfig();
  // Migrate V1 numeric-only positions to the V2 analog dial layout.
  if(cfg.valueY<152 || cfg.titleY<63 || cfg.unitY<178){
    cfg.titleY=75; cfg.valueY=166; cfg.unitY=188;
  }
  if (USE_BOOT_BUTTON) pinMode(BUTTON_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(cfg.background565);
  frame.setColorDepth(8);
  frameReady = (frame.createSprite(TFT_W, TFT_H) != nullptr);
  if (!frameReady) Serial.println("ERROR: Could not allocate 240x240 sprite");

  initHttp();
  pageChangedAt = millis();
  // Bluetooth initialization is deferred until a phone authenticates to Wi-Fi
  // or the 90-second setup window expires. No functionality is removed.
  linkState = LinkState::RETRY_WAIT;
  retryAt = millis();
}

void loop() {
  uint32_t now = millis();
  server.handleClient();
  if (!bluetoothStarted && (WiFi.softAPgetStationNum() > 0 || (uint32_t)(now - wifiBootAt) >= 90000)) {
    bluetoothStarted = true;
    // Master mode is essential; most ELM327 adapters are SPP slaves.
    bluetoothReady = bt.begin("ESP32-OBD-DASH", true);
    if (bluetoothReady) {
      bt.setPin(cfg.obdPin.c_str());
      linkState = LinkState::RETRY_WAIT;
      retryAt = now - RECONNECT_MS;
    } else {
      linkState = LinkState::BT_FAILURE;
      Serial.println("ERROR: Bluetooth Classic could not start");
    }
  }
  updateButton(now);
  if (bluetoothStarted) updateObd(now);
  renderScreen(now);
  // Intentionally no delay(): polling, UI and input are millis()-driven.
}
