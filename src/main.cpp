#include <M5Dial.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <SimpleFTPServer.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#ifdef USE_CH341_TRANSPORT
#include <EspUsbHost.h>
#endif

// M5DialCubikoController — handheld controller for a Genmitsu Cubiko CNC.
//
// Joins WiFi (hardcoded SSID/password), exposes an FTP server on port 21
// (default creds cubiko/cubiko) backed by LittleFS. When a file finishes
// uploading via FTP, it becomes the "current job" — press PLAY to stream
// it to the CNC, PAUSE to feed-hold, STOP to soft-reset.
//
// The CNC link itself is not wired up yet — the streamer talks to a STUB
// transport that prints every line to USB-CDC serial and auto-acks "ok".
// Watch `pio device monitor` to see what would be sent to grbl.

// ---------------------------------------------------------------
// Config
// ---------------------------------------------------------------
// WiFi credentials are entered by the user via a first-boot captive
// portal (WiFiManager) and stored in NVS. To re-run setup: hold G0
// for the first 2s after boot — the app wipes saved creds and starts
// the portal AP named `M5DialCubiko-Setup`.
static const char* SETUP_AP_SSID = "M5DialCubiko-Setup";
static const char* FTP_USER = "cubiko";
static const char* FTP_PASS = "cubiko";

// Calibrate sequence for the Cubiko's built-in tool-length sensor
// (the gold button at the back-right, machine XY ≈ 149, 110).
// Homes, rapids over the sensor, probes down, zeroes work Z at the
// contact point, retracts. After this the tool tip sits at work Z=0
// when it touches the sensor — offset your workpiece Z separately.
// Two-stage probing (grbl best practice): fast coarse approach then
// a slow precise probe, so total time is short without giving up
// contact accuracy at the touch point.
static const char* const CALIBRATE_LINES[] = {
  "$H",                  // 0 home all axes
  "G21",                 // 1 mm units
  "G90",                 // 2 absolute
  "G53 G0 X149 Y110",    // 3 rapid to sensor XY in machine coords
  "G91",                 // 4 relative
  "G38.2 Z-50 F200",     // 5 fast probe (2x previous feed)
  "G0 Z2",               // 6 back off 2mm
  "G38.2 Z-3 F50",       // 7 slow precise probe (half previous feed)
  "G10 L20 P1 Z0",       // 8 work Z = 0 at contact (see CAL_IDX_SET_Z)
  "G0 Z10",              // 9 retract 10mm
  "G90",                 // 10 absolute
};
static constexpr int CAL_LINE_COUNT =
    sizeof(CALIBRATE_LINES) / sizeof(CALIBRATE_LINES[0]);
// Index of "G10 L20 P1 Z0" — substituted with G10 L20 P1 Z{-probeZOff}
// each time the calibrate sequence runs.
static constexpr int CAL_IDX_SET_Z = 8;

// ---------------------------------------------------------------
// Layout (240x240 round)
// ---------------------------------------------------------------
static constexpr int SCREEN = 240;
static constexpr int CENTER = SCREEN / 2;

static constexpr int AXIS_Y    = 28;
static constexpr int SPEED_Y   = 60;
static constexpr int CHIP_W    = 46;
static constexpr int CHIP_H    = 28;
static constexpr int CHIP_GAP  = 6;

// Single MENU button replaces the old CAL + UNL pair; opens the
// secondary screen with CAL / UNL / ProbeZ / Return rows.
static constexpr int MENU_BTN_W = 66;
static constexpr int MENU_BTN_H = 22;
static constexpr int MENU_BTN_Y = 92;
static constexpr int MENU_BTN_X = CENTER - MENU_BTN_W / 2;

static constexpr int STATUS_Y  = 120;

static constexpr int BTN_CY    = 152;
static constexpr int BTN_R     = 22;                 // was 18 — bigger
static constexpr int STOP_CX   = 42;
static constexpr int PLAY_CX   = 198;

// Menu screen layout — 5 rows of 28-tall buttons, 170 wide.
static constexpr int MENU_ROW_W     = 170;
static constexpr int MENU_ROW_H     = 28;
static constexpr int MENU_ROW_X     = CENTER - MENU_ROW_W / 2;
static constexpr int MENU_CAL_Y     = 40;
static constexpr int MENU_UNL_Y     = 72;
static constexpr int MENU_SPINDLE_Y = 104;
static constexpr int MENU_PROBE_Y   = 136;
static constexpr int MENU_RET_Y     = 168;

static constexpr int JOG_Y     = 188;
static constexpr int FTP_Y     = 213;

// ---------------------------------------------------------------
// State
// ---------------------------------------------------------------
enum class Axis  : uint8_t { X = 0, Y = 1, Z = 2 };
enum class Speed : uint8_t { Slow = 0, Medium = 1, Fast = 2 };
enum class JobState { Idle, Loaded, Playing, Paused, Done };

static const char* AXIS_LABELS[]  = {"X", "Y", "Z"};
static const char* SPEED_LABELS[] = {"SLOW", "MED", "FAST"};
static const float STEP_MM[]      = {0.01f, 0.10f, 1.00f};
// Feed rates for jog moves (mm/min). Small steps get low feeds so the
// motion isn't over before the visual/audible feedback lands; big steps
// get a fast feed so the head actually moves before the next detent.
static const int   JOG_FEED[]     = {100,  600,  1500};

static Axis     g_axis        = Axis::X;
static Speed    g_speed       = Speed::Medium;
static long     g_lastEncoder = 0;
static float    g_jogAccumMm  = 0.0f;
static bool     g_inMenu       = false;
static bool     g_probeZFocused = false;         // tap ProbeZ to grab the wheel
static bool     g_spindleOn    = false;          // NOT persisted — always off at boot
static int      g_spindleRpm   = 7000;
static constexpr int SPINDLE_RPM_STEP = 100;
static constexpr int SPINDLE_RPM_MIN  = 0;
static constexpr int SPINDLE_RPM_MAX  = 25000;
// Adjustment applied to the calibrate G10 L20 P1 Z line: if the probe
// button sits below the workpiece surface by this many mm, we set
// work-Z to -offset at the touch point so Z=0 lands at the surface.
// Persisted to NVS via Preferences.
static float    g_probeZOff   = 0.0f;

// Work-coordinate readout parsed from grbl `?` status reports.
// g_wcoOffset is cached (grbl only sends WCO periodically); g_workPos
// is the live X/Y/Z work position derived from MPos - WCO.
static float    g_workPos[3]   = {0.0f, 0.0f, 0.0f};
static float    g_wcoOffset[3] = {0.0f, 0.0f, 0.0f};
static bool     g_haveWorkPos  = false;
// Current grbl feed override %. During a run the encoder emits
// realtime 0x93/0x94 (±1%) commands and updates this locally;
// parseStatusLine keeps it synced with what grbl actually applies.
static int      g_feedOverride = 100;
static uint32_t g_lastStatusMs = 0;
static constexpr uint32_t STATUS_INTERVAL_MS = 200;

static FtpServer   g_ftp;
static WebServer   g_http(80);
static Preferences g_prefs;
static bool      g_wifiConnected = false;
static String    g_ipStr;

static JobState  g_jobState    = JobState::Idle;
static File      g_jobFile;
static String    g_jobName;
static String    g_jobPath;     // full LittleFS path for re-playing
static uint32_t  g_jobLineNum  = 0;

static bool      g_calibrating  = false;
static int       g_calibrateIdx = 0;

static bool      g_waitingForOk = false;

static volatile bool g_uploadDone = false;
static String        g_uploadedPath;

// Long-press on an axis chip zeroes that work coordinate. Tracks which
// chip is currently held + when the press started, so we can beep on
// press, boop + zero after HOLD_ZERO_MS, or treat as a tap on release.
static int       g_heldAxisIdx   = -1;
static uint32_t  g_heldStartMs   = 0;
static bool      g_heldZeroFired = false;
static constexpr uint32_t HOLD_ZERO_MS = 1000;

// Quadrature encoder reports 4 raw counts per physical detent click;
// jog one step per detent.
static constexpr int ENC_COUNTS_PER_DETENT = 4;

// ---------------------------------------------------------------
// CNC transport — interface + stub implementation
// ---------------------------------------------------------------
class CncTransport {
 public:
  virtual ~CncTransport() {}
  virtual void begin() = 0;
  virtual bool connected() const = 0;
  virtual void writeLine(const String& line) = 0;
  virtual void realtime(uint8_t c) = 0;
  virtual bool readLine(String& out) = 0;  // non-blocking
};

#ifndef USE_CH341_TRANSPORT
// Logs everything to USB-CDC Serial and auto-acks every line with "ok".
class StubCncTransport : public CncTransport {
  bool _pendingOk = false;
 public:
  void begin() override { Serial.println("[CNC stub] ready"); }
  bool connected() const override { return true; }
  void writeLine(const String& line) override {
    Serial.print("[TX] ");
    Serial.print(line);
    _pendingOk = true;
  }
  void realtime(uint8_t c) override {
    Serial.printf("[RT] 0x%02X\n", c);
  }
  bool readLine(String& out) override {
    if (_pendingOk) { out = "ok"; _pendingOk = false; return true; }
    return false;
  }
};
static StubCncTransport g_stubTransport;
static CncTransport*    g_cnc = &g_stubTransport;
#endif

#ifdef USE_CH341_TRANSPORT
// USB-host CDC transport. Enumerates the Cubiko's CH341 (VID 0x1A86)
// via EspUsbHost and exposes it as a Stream we feed grbl over.
class Ch341CncTransport : public CncTransport {
  EspUsbHost          _usb;
  EspUsbHostCdcSerial _cdc;
  String              _rxLine;
 public:
  Ch341CncTransport() : _cdc(_usb) {}

  void begin() override {
    _cdc.begin(115200);                    // bind stream + default baud
    _usb.begin();                          // start the host stack
  }

  bool connected() const override { return _cdc.connected(); }

  void writeLine(const String& line) override { _cdc.print(line); }

  void realtime(uint8_t c) override { _cdc.write(c); }

  bool readLine(String& out) override {
    while (_cdc.available()) {
      const int c = _cdc.read();
      if (c < 0) break;
      if (c == '\n') {
        out = _rxLine;
        out.trim();
        _rxLine = "";
        return true;
      }
      if (c == '\r') continue;
      _rxLine += (char)c;
      if (_rxLine.length() > 256) _rxLine = "";   // overflow guard
    }
    return false;
  }
};
static Ch341CncTransport g_ch341Transport;
static CncTransport*     g_cnc = &g_ch341Transport;
#endif

// ---------------------------------------------------------------
// Button-state predicates
// ---------------------------------------------------------------
static bool calEnabled() {
  // Allowed any time nothing is actively moving. After a job ends
  // (Done) or is stopped/loaded (Loaded), CAL should be usable again.
  return (g_jobState == JobState::Idle   ||
          g_jobState == JobState::Loaded ||
          g_jobState == JobState::Done) && !g_calibrating;
}
static bool playEnabled() {
  return g_jobState == JobState::Loaded ||
         g_jobState == JobState::Playing ||
         g_jobState == JobState::Paused ||
         g_jobState == JobState::Done;       // re-play same file
}
static bool stopEnabled() {
  return g_jobState == JobState::Playing ||
         g_jobState == JobState::Paused ||
         g_jobState == JobState::Done ||     // also acts as "clear job"
         g_calibrating;
}

// ---------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------
static int chipX(int idx) {
  const int rowW = 3 * CHIP_W + 2 * CHIP_GAP;
  return CENTER - rowW / 2 + idx * (CHIP_W + CHIP_GAP);
}

static void drawChip(int x, int y, int w, int h, const char* label,
                     bool selected, int textSize) {
  const uint16_t bg = selected ? TFT_ORANGE : 0x39E7;
  const uint16_t fg = selected ? TFT_BLACK  : TFT_WHITE;
  M5Dial.Display.fillRoundRect(x, y, w, h, 5, bg);
  M5Dial.Display.setTextColor(fg, bg);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(textSize);
  M5Dial.Display.drawString(label, x + w / 2, y + h / 2);
}

static void drawMenuButton() {
  const uint16_t bg = 0x39E7;
  M5Dial.Display.fillRoundRect(MENU_BTN_X, MENU_BTN_Y, MENU_BTN_W, MENU_BTN_H, 4, bg);
  M5Dial.Display.setTextColor(TFT_WHITE, bg);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(1);
  M5Dial.Display.drawString("MENU",
                            MENU_BTN_X + MENU_BTN_W / 2,
                            MENU_BTN_Y + MENU_BTN_H / 2);
}

static void drawStopButton() {
  const uint16_t ring = stopEnabled() ? 0xA000 : 0x2104;       // red / dim
  const uint16_t icon = stopEnabled() ? TFT_WHITE : 0x7BEF;
  M5Dial.Display.fillCircle(STOP_CX, BTN_CY, BTN_R, ring);
  M5Dial.Display.fillRect(STOP_CX - 7, BTN_CY - 7, 14, 14, icon);
}

static void drawPlayButton() {
  const bool isPlaying = (g_jobState == JobState::Playing);
  const bool armed    = playEnabled() && g_cnc->connected();
  uint16_t ring;
  if (!armed)               ring = 0x2104;       // dim grey
  else if (isPlaying)       ring = TFT_ORANGE;   // active
  else                      ring = 0x0500;       // dark green
  uint16_t icon = armed ? TFT_WHITE : 0x7BEF;
  if (isPlaying) icon = TFT_BLACK;

  M5Dial.Display.fillCircle(PLAY_CX, BTN_CY, BTN_R, ring);
  if (isPlaying) {
    M5Dial.Display.fillRect(PLAY_CX - 7, BTN_CY - 8, 4, 16, icon);
    M5Dial.Display.fillRect(PLAY_CX + 3, BTN_CY - 8, 4, 16, icon);
  } else {
    M5Dial.Display.fillTriangle(
        PLAY_CX - 6, BTN_CY - 9,
        PLAY_CX - 6, BTN_CY + 9,
        PLAY_CX + 9, BTN_CY,
        icon);
  }
}

static void drawJobStatus() {
  M5Dial.Display.fillRect(0, STATUS_Y - 5, SCREEN, 10, TFT_BLACK);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(1);
  char buf[64];
  if (g_calibrating) {
    snprintf(buf, sizeof(buf), "CAL %d/%d", g_calibrateIdx, CAL_LINE_COUNT);
    M5Dial.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5Dial.Display.drawString(buf, CENTER, STATUS_Y);
    return;
  }
  switch (g_jobState) {
    case JobState::Idle:
      M5Dial.Display.setTextColor(0x7BEF, TFT_BLACK);
      M5Dial.Display.drawString("JOG MODE", CENTER, STATUS_Y);
      break;
    case JobState::Loaded:
      snprintf(buf, sizeof(buf), "RDY %s", g_jobName.c_str());
      M5Dial.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
      M5Dial.Display.drawString(buf, CENTER, STATUS_Y);
      break;
    case JobState::Playing:
      snprintf(buf, sizeof(buf), "RUN %u", (unsigned)g_jobLineNum);
      M5Dial.Display.setTextColor(TFT_GREEN, TFT_BLACK);
      M5Dial.Display.drawString(buf, CENTER, STATUS_Y);
      break;
    case JobState::Paused:
      snprintf(buf, sizeof(buf), "PAUSE %u", (unsigned)g_jobLineNum);
      M5Dial.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
      M5Dial.Display.drawString(buf, CENTER, STATUS_Y);
      break;
    case JobState::Done:
      snprintf(buf, sizeof(buf), "DONE %s", g_jobName.c_str());
      M5Dial.Display.setTextColor(TFT_CYAN, TFT_BLACK);
      M5Dial.Display.drawString(buf, CENTER, STATUS_Y);
      break;
  }
}

static void drawCenter() {
  M5Dial.Display.fillRect(0, BTN_CY - BTN_R - 2, SCREEN, 2 * (BTN_R + 2),
                          TFT_BLACK);
  M5Dial.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(6);
  M5Dial.Display.drawString(AXIS_LABELS[(int)g_axis], CENTER, BTN_CY);
  drawStopButton();
  drawPlayButton();
}

static void drawJogReadout() {
  if (g_inMenu) return;                           // menu owns the screen
  M5Dial.Display.fillRect(0, JOG_Y - 8, SCREEN, 16, TFT_BLACK);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(2);
  char buf[32];
  if (g_jobState == JobState::Playing || g_jobState == JobState::Paused) {
    // During a run the wheel steers feed override — show that instead
    // of the position (position is on the CNC's own display, and the
    // number the user is actively controlling is the %).
    M5Dial.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "FEED %d%%", g_feedOverride);
  } else {
    M5Dial.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    if (g_haveWorkPos) {
      snprintf(buf, sizeof(buf), "%+8.2f mm", g_workPos[(int)g_axis]);
    } else {
      // No status from grbl yet (stub transport, disconnected, or
      // first few ms after boot). Fall back to the jog accumulator.
      snprintf(buf, sizeof(buf), "%+8.2f mm", g_jogAccumMm);
    }
  }
  M5Dial.Display.drawString(buf, CENTER, JOG_Y);
}

static void drawFtpStatus() {
  M5Dial.Display.fillRect(0, FTP_Y - 5, SCREEN, 10, TFT_BLACK);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(1);
  String s = g_wifiConnected ? String("cubiko.local") : String("offline");
  const uint16_t fg = g_wifiConnected ? TFT_GREEN : 0x7BEF;
  M5Dial.Display.setTextColor(fg, TFT_BLACK);
  M5Dial.Display.drawString(s.c_str(), CENTER, FTP_Y);
  // CNC connection dot, just right of the FTP/IP text.
  const int dotX = CENTER + (s.length() * 6) / 2 + 8;
  const uint16_t dotColor = g_cnc->connected() ? TFT_GREEN : 0x8000;
  M5Dial.Display.fillCircle(dotX, FTP_Y, 3, dotColor);
}

static void saveProbeZOff()    { g_prefs.putFloat("probeZOff", g_probeZOff); }
static void saveSpindleRpm()   { g_prefs.putInt("spRpm", g_spindleRpm); }

static bool spindleAllowed() {
  return g_jobState != JobState::Playing &&
         g_jobState != JobState::Paused  &&
         !g_calibrating;
}

static void drawMenu() {
  M5Dial.Display.fillScreen(TFT_BLACK);

  // CAL row
  const bool calArm = calEnabled();
  const uint16_t calBg = calArm ? 0x033F : 0x18E3;
  const uint16_t calFg = calArm ? TFT_WHITE : 0x7BEF;
  M5Dial.Display.fillRoundRect(MENU_ROW_X, MENU_CAL_Y, MENU_ROW_W, MENU_ROW_H, 6, calBg);
  M5Dial.Display.setTextColor(calFg, calBg);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(2);
  M5Dial.Display.drawString("CAL", CENTER, MENU_CAL_Y + MENU_ROW_H / 2);

  // UNL row
  const bool unlArm = g_cnc->connected();
  const uint16_t unlBg = unlArm ? 0x0500 : 0x18E3;
  const uint16_t unlFg = unlArm ? TFT_WHITE : 0x7BEF;
  M5Dial.Display.fillRoundRect(MENU_ROW_X, MENU_UNL_Y, MENU_ROW_W, MENU_ROW_H, 6, unlBg);
  M5Dial.Display.setTextColor(unlFg, unlBg);
  M5Dial.Display.drawString("Unlock", CENTER, MENU_UNL_Y + MENU_ROW_H / 2);

  // Spindle row: tap toggles on/off (M3 S<rpm> / M5). When on, the
  // wheel adjusts RPM and the row is highlighted orange.
  const bool spOk = spindleAllowed();
  uint16_t spBg, spFg;
  if (!spOk)             { spBg = 0x18E3;     spFg = 0x7BEF; }
  else if (g_spindleOn)  { spBg = TFT_ORANGE; spFg = TFT_BLACK; }
  else                   { spBg = 0x39E7;     spFg = TFT_WHITE; }
  M5Dial.Display.fillRoundRect(MENU_ROW_X, MENU_SPINDLE_Y, MENU_ROW_W, MENU_ROW_H, 6, spBg);
  M5Dial.Display.setTextColor(spFg, spBg);
  M5Dial.Display.setTextDatum(middle_left);
  M5Dial.Display.setTextSize(2);
  M5Dial.Display.drawString("Spindle", MENU_ROW_X + 10, MENU_SPINDLE_Y + MENU_ROW_H / 2);
  char spBuf[16];
  if (g_spindleOn) snprintf(spBuf, sizeof(spBuf), "%d", g_spindleRpm);
  else             strcpy(spBuf, "OFF");
  M5Dial.Display.setTextDatum(middle_right);
  M5Dial.Display.drawString(spBuf,
                            MENU_ROW_X + MENU_ROW_W - 10,
                            MENU_SPINDLE_Y + MENU_ROW_H / 2);

  // ProbeZ row: tap to grab the wheel (highlighted); tap again to release.
  const uint16_t pzBg = g_probeZFocused ? TFT_ORANGE : 0x39E7;
  const uint16_t pzFg = g_probeZFocused ? TFT_BLACK  : TFT_WHITE;
  M5Dial.Display.fillRoundRect(MENU_ROW_X, MENU_PROBE_Y, MENU_ROW_W, MENU_ROW_H, 6, pzBg);
  M5Dial.Display.setTextColor(pzFg, pzBg);
  M5Dial.Display.setTextDatum(middle_left);
  M5Dial.Display.setTextSize(2);
  M5Dial.Display.drawString("ProbeZ", MENU_ROW_X + 10, MENU_PROBE_Y + MENU_ROW_H / 2);
  char buf[16];
  snprintf(buf, sizeof(buf), "%+.2f", g_probeZOff);
  M5Dial.Display.setTextDatum(middle_right);
  M5Dial.Display.drawString(buf, MENU_ROW_X + MENU_ROW_W - 10, MENU_PROBE_Y + MENU_ROW_H / 2);

  // Return
  M5Dial.Display.fillRoundRect(MENU_ROW_X, MENU_RET_Y, MENU_ROW_W, MENU_ROW_H, 6, 0x6000);
  M5Dial.Display.setTextColor(TFT_WHITE, 0x6000);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.drawString("Return", CENTER, MENU_RET_Y + MENU_ROW_H / 2);
}

static void drawMain() {
  M5Dial.Display.fillScreen(TFT_BLACK);
  for (int i = 0; i < 3; i++) {
    drawChip(chipX(i), AXIS_Y,  CHIP_W, CHIP_H, AXIS_LABELS[i],
             i == (int)g_axis, 2);
    drawChip(chipX(i), SPEED_Y, CHIP_W, CHIP_H, SPEED_LABELS[i],
             i == (int)g_speed, 1);
  }
  drawMenuButton();
  drawJobStatus();
  drawCenter();
  drawJogReadout();
  drawFtpStatus();
}

static void drawAll() {
  if (g_inMenu) drawMenu();
  else          drawMain();
}

// ---------------------------------------------------------------
// Job control
// ---------------------------------------------------------------
static void loadJob(const String& path) {
  if (g_jobFile) g_jobFile.close();
  g_jobFile = LittleFS.open(path.c_str(), "r");
  if (!g_jobFile) { g_jobState = JobState::Idle; return; }
  g_jobPath = path;
  g_jobName = path;
  if (g_jobName.startsWith("/")) g_jobName.remove(0, 1);
  g_jobLineNum = 0;
  g_waitingForOk = false;
  g_jobState = JobState::Loaded;
}

static void ensureSpindleOff();

static void onPlay() {
  if (!playEnabled()) return;
  if (g_jobState == JobState::Loaded) {
    ensureSpindleOff();                         // let the file's M3 own it
    g_feedOverride = 100;                       // fresh start at 100%
    g_cnc->realtime(0x90);                      // grbl feed override reset
    g_jobState = JobState::Playing;
  } else if (g_jobState == JobState::Paused) {
    g_cnc->realtime('~');                       // grbl cycle start / resume
    g_jobState = JobState::Playing;
  } else if (g_jobState == JobState::Playing) {
    g_cnc->realtime('!');                       // grbl feed hold
    g_jobState = JobState::Paused;
  } else if (g_jobState == JobState::Done) {
    // Re-arm the same file and play it again.
    if (g_jobPath.length()) {
      ensureSpindleOff();
      loadJob(g_jobPath);
      g_feedOverride = 100;
      g_cnc->realtime(0x90);
      g_jobState = JobState::Playing;
    }
  }
}

static void onStop() {
  if (!stopEnabled()) return;
  // Only send a soft reset if there's something actually running on the CNC.
  if (g_jobState == JobState::Playing ||
      g_jobState == JobState::Paused  ||
      g_calibrating) {
    g_cnc->realtime(0x18);                      // grbl soft reset
  }
  g_calibrating  = false;
  g_calibrateIdx = 0;
  g_waitingForOk = false;
  if (g_jobFile) g_jobFile.close();
  // Keep the file path so the user can press play again to re-run from
  // the beginning. Falls back to Idle only if we have nothing loaded.
  if (g_jobPath.length()) {
    loadJob(g_jobPath);                         // re-arms to Loaded
  } else {
    g_jobState = JobState::Idle;
  }
}

static void zeroAxis(int axisIdx) {
  if (g_jobState != JobState::Idle &&
      g_jobState != JobState::Loaded &&
      g_jobState != JobState::Done) return;
  if (g_calibrating) return;
  const char* a = (axisIdx == 0) ? "X" : (axisIdx == 1) ? "Y" : "Z";
  String line = String("G10 L20 P1 ") + a + "0\n";
  g_cnc->writeLine(line);
  g_jogAccumMm = 0.0f;
}

// Never start a fresh cycle with the bit spinning — home / probe /
// job all assume the spindle is off unless the G-code turns it on.
static void ensureSpindleOff() {
  if (g_spindleOn) {
    g_cnc->writeLine("M5\n");
    g_spindleOn = false;
  }
}

static void onCalibrate() {
  if (!calEnabled()) return;
  ensureSpindleOff();
  g_calibrating  = true;
  g_calibrateIdx = 0;
  g_waitingForOk = false;
}

static void onUnlock() {
  if (!g_cnc->connected()) return;
  // $X clears grbl alarm state (e.g. after a failed probe or a limit hit).
  g_cnc->writeLine("$X\n");
}

// Extract MPos/WPos/WCO from a grbl <...> status report into globals.
static void parseStatusLine(const String& s) {
  const int wposIdx = s.indexOf("WPos:");
  const int mposIdx = s.indexOf("MPos:");
  const int wcoIdx  = s.indexOf("WCO:");
  bool updated = false;
  if (wposIdx >= 0) {
    float p[3] = {0, 0, 0};
    if (sscanf(s.c_str() + wposIdx, "WPos:%f,%f,%f", &p[0], &p[1], &p[2]) == 3) {
      for (int i = 0; i < 3; i++) g_workPos[i] = p[i];
      updated = true;
    }
  } else if (mposIdx >= 0) {
    float mp[3] = {0, 0, 0};
    if (sscanf(s.c_str() + mposIdx, "MPos:%f,%f,%f", &mp[0], &mp[1], &mp[2]) == 3) {
      if (wcoIdx >= 0) {
        float wc[3] = {0, 0, 0};
        if (sscanf(s.c_str() + wcoIdx, "WCO:%f,%f,%f", &wc[0], &wc[1], &wc[2]) == 3) {
          for (int i = 0; i < 3; i++) g_wcoOffset[i] = wc[i];
        }
      }
      for (int i = 0; i < 3; i++) g_workPos[i] = mp[i] - g_wcoOffset[i];
      updated = true;
    }
  }
  // Ov:<feed>,<rapid>,<spindle> — keep our local feed override in sync
  // in case grbl clamps it (10-200%).
  const int ovIdx = s.indexOf("Ov:");
  if (ovIdx >= 0) {
    int fo = 0;
    if (sscanf(s.c_str() + ovIdx, "Ov:%d", &fo) == 1 && fo > 0) {
      if (fo != g_feedOverride) {
        g_feedOverride = fo;
        updated = true;
      }
    }
  }
  if (updated) {
    g_haveWorkPos = true;
    drawJogReadout();
  }
}

static void streamerTick() {
  // Always drain any pending responses so ad-hoc commands (zero-axis,
  // $J= jogs, ? status polls) don't leave stale lines that the next
  // streamed line's ok handshake would consume out of order.
  {
    String r;
    while (g_cnc->readLine(r)) {
      if (r.length() > 0 && r[0] == '<') {
        parseStatusLine(r);
        continue;
      }
      if (g_waitingForOk &&
          (r.startsWith("ok") || r.startsWith("error"))) {
        g_waitingForOk = false;
      }
    }
  }
  if (g_waitingForOk) return;

  // Calibration takes priority.
  if (g_calibrating) {
    if (g_calibrateIdx >= CAL_LINE_COUNT) {
      g_calibrating = false;
      drawAll();
      return;
    }
    String line;
    if (g_calibrateIdx == CAL_IDX_SET_Z) {
      // Displace Z=0 upwards by g_probeZOff: at the probe touch point,
      // work-Z should read -offset so plate height reads 0.
      char buf[32];
      snprintf(buf, sizeof(buf), "G10 L20 P1 Z%.3f", -g_probeZOff);
      line = buf;
    } else {
      line = CALIBRATE_LINES[g_calibrateIdx];
    }
    g_cnc->writeLine(line + "\n");
    g_calibrateIdx++;
    g_waitingForOk = true;
    drawJobStatus();
    return;
  }

  if (g_jobState != JobState::Playing) return;
  if (!g_jobFile || !g_jobFile.available()) {
    if (g_jobFile) g_jobFile.close();
    g_jobState = JobState::Done;
    drawAll();
    return;
  }
  String line = g_jobFile.readStringUntil('\n');
  line.trim();
  if (line.length() == 0 || line[0] == ';' || line[0] == '(') return;
  g_cnc->writeLine(line + "\n");
  g_waitingForOk = true;
  g_jobLineNum++;
  if (g_jobLineNum % 20 == 0) drawJobStatus();
}

// ---------------------------------------------------------------
// Touch
// ---------------------------------------------------------------
static bool hitRect(int tx, int ty, int x, int y, int w, int h) {
  return tx >= x && tx < x + w && ty >= y && ty < y + h;
}
static bool hitCircle(int tx, int ty, int cx, int cy, int r) {
  const int dx = tx - cx, dy = ty - cy;
  return dx * dx + dy * dy <= r * r;
}

static void handleMenuTouch() {
  if (M5Dial.Touch.getCount() == 0) return;
  auto t = M5Dial.Touch.getDetail();
  if (!t.wasPressed()) return;
  if (t.x < MENU_ROW_X || t.x >= MENU_ROW_X + MENU_ROW_W) return;
  bool leaveMenu = false;
  bool redraw    = false;
  if (t.y >= MENU_CAL_Y && t.y < MENU_CAL_Y + MENU_ROW_H) {
    if (calEnabled()) { onCalibrate(); leaveMenu = true; }
  } else if (t.y >= MENU_UNL_Y && t.y < MENU_UNL_Y + MENU_ROW_H) {
    if (g_cnc->connected()) { onUnlock(); leaveMenu = true; }
  } else if (t.y >= MENU_SPINDLE_Y && t.y < MENU_SPINDLE_Y + MENU_ROW_H) {
    if (spindleAllowed()) {
      g_spindleOn = !g_spindleOn;
      if (g_spindleOn) {
        // Turning on takes the wheel — release any ProbeZ focus.
        g_probeZFocused = false;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "M3 S%d\n", g_spindleRpm);
        g_cnc->writeLine(cmd);
      } else {
        g_cnc->writeLine("M5\n");
      }
      redraw = true;
    }
  } else if (t.y >= MENU_PROBE_Y && t.y < MENU_PROBE_Y + MENU_ROW_H) {
    // Toggle wheel focus onto the ProbeZ value.
    g_probeZFocused = !g_probeZFocused;
    redraw = true;
  } else if (t.y >= MENU_RET_Y && t.y < MENU_RET_Y + MENU_ROW_H) {
    leaveMenu = true;
  }
  if (leaveMenu) {
    g_inMenu = false;
    g_probeZFocused = false;
    saveProbeZOff();
    saveSpindleRpm();
    drawAll();
  } else if (redraw) {
    drawMenu();
  }
}

static void handleTouch() {
  if (g_inMenu) { handleMenuTouch(); return; }
  // Finger lifted: finalise any axis-chip hold tracking.
  if (M5Dial.Touch.getCount() == 0) {
    if (g_heldAxisIdx >= 0) {
      // Short tap (no long-press fire) → switch axis.
      if (!g_heldZeroFired && g_heldAxisIdx != (int)g_axis) {
        g_axis = (Axis)g_heldAxisIdx;
        drawAll();
      }
      g_heldAxisIdx = -1;
    }
    return;
  }

  auto t = M5Dial.Touch.getDetail();

  if (t.wasPressed()) {
    // Axis chips: start hold tracking, beep, defer the action to
    // release (tap = switch axis) or the 1s mark (hold = zero axis).
    for (int i = 0; i < 3; i++) {
      if (hitRect(t.x, t.y, chipX(i), AXIS_Y, CHIP_W, CHIP_H)) {
        g_heldAxisIdx   = i;
        g_heldStartMs   = millis();
        g_heldZeroFired = false;
        M5Dial.Speaker.tone(600, 40);           // beep (press)
        return;
      }
    }
    // Speed chips: immediate switch.
    for (int i = 0; i < 3; i++) {
      if (hitRect(t.x, t.y, chipX(i), SPEED_Y, CHIP_W, CHIP_H) &&
          i != (int)g_speed) {
        g_speed = (Speed)i;
        drawAll();
        return;
      }
    }
    if (hitRect(t.x, t.y,
                MENU_BTN_X - 4, MENU_BTN_Y - 4,
                MENU_BTN_W + 8, MENU_BTN_H + 8)) {
      g_inMenu = true; drawAll(); return;
    }
    if (hitCircle(t.x, t.y, STOP_CX, BTN_CY, BTN_R + 4)) {
      onStop();       drawAll(); return;
    }
    if (hitCircle(t.x, t.y, PLAY_CX, BTN_CY, BTN_R + 4)) {
      onPlay();       drawAll(); return;
    }
  }

  // Long-press fire-event mid-hold (only for axis chips).
  if (g_heldAxisIdx >= 0 && !g_heldZeroFired &&
      millis() - g_heldStartMs >= HOLD_ZERO_MS) {
    // Higher-pitched tones sound louder on this piezo; drop the volume
    // just for the confirm beep and restore right after.
    M5Dial.Speaker.setVolume(80);
    M5Dial.Speaker.tone(1600, 100);
    M5Dial.Speaker.setVolume(130);
    if (g_heldAxisIdx != (int)g_axis) {
      g_axis = (Axis)g_heldAxisIdx;
    }
    zeroAxis(g_heldAxisIdx);
    g_heldZeroFired = true;
    drawAll();
  }
}

// ---------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------
static void handleEncoder() {
  // Snap raw counts to whole detents; remainder is left in the unread
  // (raw - g_lastEncoder) gap so partial rotations accumulate cleanly.
  const long raw = M5Dial.Encoder.read();
  const long delta_raw = raw - g_lastEncoder;
  const long detents  = delta_raw / ENC_COUNTS_PER_DETENT;
  if (detents == 0) return;
  g_lastEncoder += detents * ENC_COUNTS_PER_DETENT;

  if (g_inMenu) {
    if (g_spindleOn) {
      // Spindle owns the wheel while running: send updated S value.
      g_spindleRpm += (int)detents * SPINDLE_RPM_STEP;
      if (g_spindleRpm < SPINDLE_RPM_MIN) g_spindleRpm = SPINDLE_RPM_MIN;
      if (g_spindleRpm > SPINDLE_RPM_MAX) g_spindleRpm = SPINDLE_RPM_MAX;
      char cmd[24];
      snprintf(cmd, sizeof(cmd), "S%d\n", g_spindleRpm);
      g_cnc->writeLine(cmd);
      drawMenu();
    } else if (g_probeZFocused) {
      g_probeZOff += detents * 0.1f;
      drawMenu();
    }
    return;
  }

  // Calibration: consume detents, do nothing.
  if (g_calibrating) return;

  // During a run the wheel adjusts grbl's feed override (±1% per
  // detent) via realtime commands 0x93/0x94.
  if (g_jobState == JobState::Playing || g_jobState == JobState::Paused) {
    const uint8_t rt = detents > 0 ? 0x93 : 0x94;
    const long    n  = detents > 0 ? detents : -detents;
    for (long i = 0; i < n; i++) g_cnc->realtime(rt);
    g_feedOverride += (int)detents;
    if (g_feedOverride < 10)  g_feedOverride = 10;
    if (g_feedOverride > 200) g_feedOverride = 200;
    drawJogReadout();
    return;
  }

  const float step_mm = detents * STEP_MM[(int)g_speed];
  g_jogAccumMm  += step_mm;
  drawJogReadout();

  if (!g_cnc->connected()) return;
  const char* a = (g_axis == Axis::X) ? "X"
                : (g_axis == Axis::Y) ? "Y" : "Z";
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "$J=G91 G21 %s%.3f F%d\n",
           a, step_mm, JOG_FEED[(int)g_speed]);
  g_cnc->writeLine(cmd);
}

static void handleClick() {
  if (M5Dial.BtnA.wasPressed()) {
    g_jogAccumMm = 0.0f;
    drawJogReadout();
  }
}

// ---------------------------------------------------------------
// Networking
// ---------------------------------------------------------------
static void showSplash(const String& l1, const String& l2 = "",
                       const String& l3 = "", const String& l4 = "") {
  M5Dial.Display.fillScreen(TFT_BLACK);
  M5Dial.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5Dial.Display.setTextDatum(middle_center);
  M5Dial.Display.setTextSize(2);
  int y = CENTER - 36;
  for (const String* s : {&l1, &l2, &l3, &l4}) {
    if (s->length()) M5Dial.Display.drawString(s->c_str(), CENTER, y);
    y += 24;
  }
}

// Poll G0 for ~2s at boot; user holds to reset saved WiFi creds.
static bool wantsWifiReset() {
  pinMode(0, INPUT_PULLUP);
  showSplash("Hold G0", "to reset", "WiFi (2s)");
  const uint32_t start = millis();
  while (millis() - start < 2200) {
    if (digitalRead(0) == LOW) {
      // Require the button to stay held ~400ms so a random blip
      // during boot doesn't accidentally wipe creds.
      const uint32_t heldStart = millis();
      while (digitalRead(0) == LOW && millis() - heldStart < 500) delay(10);
      if (millis() - heldStart >= 400) return true;
    }
    delay(30);
  }
  return false;
}

static void setupWifi() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wm.setAPCallback([](WiFiManager* w) {
    showSplash("SETUP",
               "Join WiFi:",
               w->getConfigPortalSSID(),
               "open any URL");
  });

  if (wantsWifiReset()) {
    showSplash("Resetting", "WiFi...");
    wm.resetSettings();
    delay(800);
  }

  showSplash("WiFi", "connecting...");
  if (wm.autoConnect(SETUP_AP_SSID)) {
    g_wifiConnected = true;
    g_ipStr = WiFi.localIP().toString();
    if (MDNS.begin("cubiko")) {
      MDNS.addService("ftp", "tcp", 21);
    }
    return;
  }
  showSplash("NO WIFI", "setup", "timed out");
  delay(3000);
}

static void onFtpTransfer(FtpTransferOperation op, const char* name,
                          unsigned int /*transferredSize*/) {
  if (op == FTP_UPLOAD_STOP) {
    String p(name);
    if (!p.startsWith("/")) p = "/" + p;
    g_uploadedPath = p;
    g_uploadDone   = true;
  }
}

static void setupFtp() {
  if (!g_wifiConnected) return;
  if (!LittleFS.begin(/*formatOnFail=*/true)) return;
  g_ftp.setTransferCallback(onFtpTransfer);
  g_ftp.begin(FTP_USER, FTP_PASS);
}

// ---------------------------------------------------------------
// HTTP drop UI: same-origin drag-and-drop page served from the
// device itself at http://cubiko.local/.
// ---------------------------------------------------------------
static const char UPLOAD_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cubiko Pendant</title>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,system-ui,sans-serif;
max-width:520px;margin:2.5rem auto;padding:0 1rem;line-height:1.5;color:#eee;
background:#181818}
h1{font-weight:600}
.drop{border:2px dashed #555;border-radius:12px;padding:3rem 1rem;text-align:center;
color:#aaa;background:#1e1e1e;font-size:15pt;transition:all .12s}
.drop.hover{border-color:#0a7d2e;color:#eee}
input[type=file]{display:none}
.btn{background:#0a7d2e;color:#fff;border:0;padding:.6rem 1.2rem;border-radius:8px;
font-size:1rem;cursor:pointer;margin-top:1rem}
pre{background:#111;color:#0d0;padding:.8rem;border-radius:8px;min-height:2rem;
font-family:'SF Mono',Menlo,monospace;font-size:11pt;white-space:pre-wrap}
a{color:#4bd}
</style></head><body>
<h1>Cubiko Pendant</h1>
<p>Drop a G-code file here — the pendant will auto-arm it as the current job.
Press play on the pendant to run.</p>
<div id="drop" class="drop">Drop files here<br><small>or</small><br>
<button class="btn" id="pick">Pick files</button></div>
<input type="file" id="file" multiple>
<pre id="log"></pre>
<p style="text-align:center;font-size:11pt;color:#666">
<a href="/update">Update firmware</a> &middot;
<a href="https://github.com/jonphoton/cubiko-pendant">github.com/jonphoton/cubiko-pendant</a></p>
<script>
const drop=document.getElementById('drop'),file=document.getElementById('file'),
pick=document.getElementById('pick'),log=document.getElementById('log');
const say=m=>{log.textContent+=(log.textContent?'\n':'')+m};
pick.onclick=()=>file.click();
file.onchange=()=>upload([...file.files]);
drop.addEventListener('dragover',e=>{e.preventDefault();drop.classList.add('hover')});
drop.addEventListener('dragleave',()=>drop.classList.remove('hover'));
drop.addEventListener('drop',e=>{e.preventDefault();drop.classList.remove('hover');
upload([...e.dataTransfer.files])});
async function upload(files){for(const f of files){
say('→ '+f.name+' ('+f.size+' bytes)');
const fd=new FormData();fd.append('file',f);
try{const r=await fetch('/upload',{method:'POST',body:fd});
say(r.ok?'  ok':'  error '+r.status)}catch(e){say('  '+e.message)}}}
</script></body></html>)HTML";

static File g_httpUploadFile;

static void onHttpRoot() {
  g_http.send_P(200, "text/html; charset=utf-8", UPLOAD_HTML);
}

static void onHttpUpload() {
  HTTPUpload& upload = g_http.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String name = upload.filename;
    if (!name.startsWith("/")) name = "/" + name;
    if (g_httpUploadFile) g_httpUploadFile.close();
    g_httpUploadFile = LittleFS.open(name, "w");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (g_httpUploadFile) {
      g_httpUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (g_httpUploadFile) {
      String name = upload.filename;
      if (!name.startsWith("/")) name = "/" + name;
      g_httpUploadFile.close();
      g_uploadedPath = name;
      g_uploadDone   = true;
    }
  }
}

// URL of the latest firmware binary in the public repo. HTTPS via
// insecure NetworkClientSecure (no cert bundle) — we're on the user's
// home LAN and this is public firmware anyway.
static const char* FIRMWARE_URL =
    "https://raw.githubusercontent.com/jonphoton/cubiko-pendant/main/firmware/firmware.bin";

static const char UPDATE_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Update firmware</title>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,system-ui,sans-serif;
max-width:520px;margin:2.5rem auto;padding:0 1rem;line-height:1.5;color:#eee;
background:#181818}
h1{font-weight:600}
button{background:#0a7d2e;color:#fff;border:0;padding:.7rem 1.4rem;
border-radius:8px;font-size:1rem;cursor:pointer}
button:disabled{background:#333;color:#888;cursor:wait}
pre{background:#111;color:#0d0;padding:.8rem;border-radius:8px;min-height:2rem;
font-family:'SF Mono',Menlo,monospace;font-size:11pt;white-space:pre-wrap}
a{color:#4bd}
</style></head><body>
<h1>Update firmware</h1>
<p>Pull the latest <code>firmware.bin</code> straight from
<a href="https://github.com/jonphoton/cubiko-pendant">the public repo</a>
and apply it. WiFi credentials are preserved.</p>
<button id="go">Update now</button>
<pre id="log"></pre>
<p style="text-align:center;font-size:11pt;color:#666"><a href="/">back to drop UI</a></p>
<script>
document.getElementById('go').onclick=async function(){
  this.disabled=true;this.textContent='Updating...';
  const log=document.getElementById('log');
  log.textContent='Downloading + flashing (~15-30s)...';
  try{
    const r=await fetch('/update',{method:'POST'});
    const t=await r.text();
    log.textContent=t;
    if(r.ok){log.textContent+='\nReboot in progress. This page will time out.';}
  }catch(e){log.textContent='Error: '+e.message;}
};
</script></body></html>)HTML";

static void onGetUpdate() {
  g_http.send_P(200, "text/html; charset=utf-8", UPDATE_HTML);
}

static void onPostUpdate() {
  WiFiClientSecure client;
  client.setInsecure();
  httpUpdate.rebootOnUpdate(false);
  const t_httpUpdate_return ret = httpUpdate.update(client, FIRMWARE_URL);
  if (ret == HTTP_UPDATE_OK) {
    g_http.send(200, "text/plain", "Update OK. Rebooting.");
    delay(500);
    ESP.restart();
  } else if (ret == HTTP_UPDATE_NO_UPDATES) {
    g_http.send(200, "text/plain", "Already up to date.");
  } else {
    String msg = "Failed: ";
    msg += httpUpdate.getLastErrorString();
    g_http.send(500, "text/plain", msg);
  }
}

static void setupHttp() {
  if (!g_wifiConnected) return;
  g_http.on("/", HTTP_GET, onHttpRoot);
  g_http.on("/upload", HTTP_POST,
            [](){ g_http.send(200, "text/plain", "ok"); },
            onHttpUpload);
  g_http.on("/update", HTTP_GET,  onGetUpdate);
  g_http.on("/update", HTTP_POST, onPostUpdate);
  g_http.begin();
}

// ---------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5Dial.begin(cfg, /*enableEncoder=*/true, /*enableRFID=*/false);
  M5Dial.Display.setRotation(0);
  M5Dial.Display.setBrightness(180);
  M5Dial.Speaker.setVolume(130);                 // audible without being tinny

  g_cnc->begin();
  g_prefs.begin("cubiko", false);
  g_probeZOff  = g_prefs.getFloat("probeZOff", 0.0f);
  g_spindleRpm = g_prefs.getInt("spRpm", 7000);
  setupWifi();
  setupFtp();
  setupHttp();

  g_lastEncoder = M5Dial.Encoder.read();
  drawAll();
}

void loop() {
  M5Dial.update();
  handleTouch();
  handleEncoder();
  handleClick();
  if (g_wifiConnected) {
    g_ftp.handleFTP();
    g_http.handleClient();
  }

  if (g_uploadDone) {
    g_uploadDone = false;
    // Auto-load whenever no job is actively in flight. Replacing a
    // Loaded/Done job is fine; the file is still on LittleFS either way.
    if (g_jobState == JobState::Idle ||
        g_jobState == JobState::Loaded ||
        g_jobState == JobState::Done) {
      loadJob(g_uploadedPath);
      drawAll();
    }
  }

  // Repaint when CNC plug-state changes so the dot + play button reflect it.
  static bool sLastCncConnected = false;
  if (g_cnc->connected() != sLastCncConnected) {
    sLastCncConnected = g_cnc->connected();
    if (!sLastCncConnected) g_haveWorkPos = false;   // stop showing stale coords
    drawCenter();
    drawFtpStatus();
    drawJogReadout();
  }

  // Poll grbl status ~5 Hz so the work-coord readout stays live.
  if (g_cnc->connected() &&
      millis() - g_lastStatusMs >= STATUS_INTERVAL_MS) {
    g_lastStatusMs = millis();
    g_cnc->realtime('?');
  }

  streamerTick();
  delay(2);
}
