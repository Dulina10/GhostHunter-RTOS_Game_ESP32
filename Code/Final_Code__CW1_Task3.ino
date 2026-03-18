/*
------------------------------------------------------------
Ghost Hunter - Real-Time Game using FreeRTOS on ESP32
Module: 5FTC2146 - Real-Time Systems and Programming
Student ID: 23115093

Description:
This program implements a real-time ghost hunting game on ESP32
using FreeRTOS. The player is controlled using a joystick, and
an LDR sensor is used as a torch control. The TFT display shows
the game graphics, score, level, power, and warnings.

Main real-time idea:
The system is divided into multiple FreeRTOS tasks so that
different activities can run concurrently. These include:
- Joystick input task
- LDR sensor task
- Game logic task
- Display rendering task
- Verbose serial monitor task

FreeRTOS Features Used:
- Periodic tasks using vTaskDelayUntil()
- Shared queues for joystick and LDR data
- Mutex for shared game state protection
- Task priorities for scheduling control

Game Features:
- Joystick movement
- LDR based torch system
- Score and combo system
- Level progression
- Power drain and overload warning
- Pause, help, menu, and game over screens

------------------------------------------------------------
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <stdarg.h>

// ================= TFT (ST7735) =================

// ================= TFT DISPLAY SETUP =================
// ST7735 TFT display is used to show the game interface.
// A GFXcanvas16 off-screen buffer is used for smoother rendering
// and reduced flickering before drawing to the actual screen.

#define TFT_CS   5
#define TFT_DC   16
#define TFT_RST  17
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

static const int SCREEN_W = 160;
static const int SCREEN_H = 128;

GFXcanvas16 canvas(SCREEN_W, SCREEN_H);


// ================= PIN DEFINITIONS =================
// LDR_PIN  -> light sensor input for torch logic
// JOY_X    -> joystick X-axis analog input
// JOY_Y    -> joystick Y-axis analog input
// JOY_SW   -> joystick switch / button input

#define LDR_PIN   32
#define JOY_X     34
#define JOY_Y     35
#define JOY_SW    27


// ================= GAME SETTINGS =================
// DEADZONE avoids unwanted joystick movement near center position.
// SPEED controls player movement speed.
// LDR_MARGIN determines when the torch should turn ON/OFF.
// OVERLOAD_LIMIT_TICKS defines how long torch can stay ON before overload.
// Tick values define the execution rate of FreeRTOS periodic tasks.

static const int DEADZONE   = 220;
static const int SPEED      = 2;
static const int LDR_MARGIN = 150;
static const int OVERLOAD_LIMIT_TICKS = 60;

static const uint32_t INPUT_TICK_MS = 20;
static const uint32_t LDR_TICK_MS   = 40;
static const uint32_t GAME_TICK_MS  = 33;
static const uint32_t UI_TICK_MS    = 33;

// ===== Layout =====
static const int HUD_H = 30;
static const int ROOM_X_MIN = 4;
static const int ROOM_X_MAX = SCREEN_W - 5;
static const int ROOM_Y_MIN = HUD_H + 6;
static const int ROOM_Y_MAX = SCREEN_H - 16;   // leave bottom area for text + radar
static const int SPR_H = 4;
static const int FOOTER_Y = SCREEN_H - 12;
static const int RADAR_X = 118;
static const int RADAR_W = 24;

// ================= GAME SPRITES =================
// Simple bitmap sprites are used for the player and ghost.
// These are stored in program memory to save RAM.

// player sprite 
const uint8_t PROGMEM SPR_PLAYER_8[] = {
  0b00111100,
  0b01111110,
  0b11011011,
  0b11111111,
  0b11100111,
  0b01111110,
  0b01100110,
  0b11000011
};

// ghost sprite 
const uint8_t PROGMEM SPR_GHOST_8[] = {
  0b00111100,
  0b01111110,
  0b11111111,
  0b11011011,
  0b11111111,
  0b11111111,
  0b10100101,
  0b01011010
};

// ================= MAIN GAME STATE =================
// GameState stores all shared game variables in one structure.
// It includes:
// - player position
// - ghost position
// - score, health, power, level
// - combo and best combo
// - UI state (menu, game, help, pause, game over)
// - warnings and temporary visual effects
//
// This structure is shared between tasks, so access is protected
// using a FreeRTOS mutex.

typedef struct { int mx; int my; bool sw; } JoyMsg;
typedef struct { int raw; bool lightOn; } LdrMsg;

enum GhostType : uint8_t { G_SLOW=0, G_FAST=1, G_TRICK=2 };
enum UIState   : uint8_t { UI_BOOT, UI_MENU, UI_HELP, UI_PLAY, UI_PAUSE, UI_GAMEOVER };

typedef struct {
  int px, py;
  int gx, gy;

  int hp;        // 0..100
  int score;
  int power;     // 0..100
  int level;
  int combo;
  int bestCombo;
  int highScore[3];

  bool lightOn;
  GhostType gType;

  UIState ui;
  uint8_t menuIndex;
  uint8_t difficulty;
  int helpScroll;

  uint8_t shake;
  bool banishFlash;

  char statusText[18];
  uint16_t statusMs;

  bool warnOverload;
  bool warnLowPower;
  bool warnLowHP;
} GameState;


// ================= FREERTOS OBJECTS =================
// qJoy  -> queue for latest joystick input
// qLdr  -> queue for latest LDR sensor data
// stateMutex -> protects shared game state from simultaneous access
// gState -> global shared game state

QueueHandle_t qJoy;
QueueHandle_t qLdr;
SemaphoreHandle_t stateMutex;
GameState gState;

// ================= SENSOR CALIBRATION =================
// joystick center values are measured at startup so movement
// can be detected relative to the real neutral position.
// LDR baseline is also measured at startup to detect when
// the sensor is covered and light level drops.

int joyCx = 2048, joyCy = 2048;
int ldrBaseline = 2000;

// ================= COLORS =================
static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static const uint16_t C_BG     = 0x0000;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_GRAY1  = RGB565(25,25,30);
static const uint16_t C_GRAY2  = RGB565(55,55,65);
static const uint16_t C_GRAY3  = RGB565(95,95,105);
static const uint16_t C_RED    = RGB565(255,60,60);
static const uint16_t C_DRED   = RGB565(120,20,20);
static const uint16_t C_CYAN   = RGB565(80,220,220);
static const uint16_t C_YELLOW = RGB565(255,220,40);
static const uint16_t C_GREEN  = RGB565(80,255,120);

// ================= RTOS VERBOSE MONITOR =================
// This section is used for CW Task 3 evidence.
// Periodic serial printing shows:
// - task execution
// - queue values
// - system state
// - priorities and stack usage
//
// This helps demonstrate that FreeRTOS tasks are running correctly.

#define RTOS_VERBOSE 1
static const uint32_t VERBOSE_MS = 500;

TaskHandle_t hJoy  = NULL;
TaskHandle_t hLdr  = NULL;
TaskHandle_t hGame = NULL;
TaskHandle_t hUI   = NULL;

static inline void VPRINT(const char* fmt, ...) {
#if RTOS_VERBOSE
  if (!Serial) return;
  char buf[180];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
#endif
}

// ================= HELPER FUNCTIONS =================
// Utility functions are used for:
// - averaged ADC reading
// - distance calculation
// - difficulty naming
// - ghost type selection
// - ghost speed scaling
// - level calculation
// - status message update
//
// These help keep the main tasks cleaner and easier to read.

// ================= UTIL =================
int readAvgADC(int pin, int samples = 6) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(800);
  }
  return (int)(sum / samples);
}

int distSq(int x1, int y1, int x2, int y2) {
  int dx = x1 - x2;
  int dy = y1 - y2;
  return dx * dx + dy * dy;
}

// ================= GAME HELPERS =================
const char* diffName(uint8_t d) {
  if (d == 0) return "EASY";
  if (d == 1) return "NORMAL";
  return "HARD";
}

GhostType pickGhostType(int level) {
  int r = random(100);
  if (level < 3) return (r < 75) ? G_SLOW : G_FAST;
  if (level < 6) {
    if (r < 55) return G_SLOW;
    if (r < 85) return G_FAST;
    return G_TRICK;
  }
  if (r < 40) return G_SLOW;
  if (r < 75) return G_FAST;
  return G_TRICK;
}

int ghostSpeedFor(GhostType t, int level, uint8_t difficulty) {
  int diffBoost = (difficulty == 0) ? 0 : (difficulty == 1 ? 1 : 2);
  int base = 1 + (level / 3) + diffBoost;
  if (t == G_FAST) return base + 1;
  return base;
}

void respawnGhostFar(GameState &s) {
  for (int k = 0; k < 12; k++) {
    int rx = random(ROOM_X_MIN + SPR_H, ROOM_X_MAX - SPR_H);
    int ry = random(ROOM_Y_MIN + SPR_H, ROOM_Y_MAX - SPR_H);
    if (distSq(rx, ry, s.px, s.py) > 2600) { s.gx = rx; s.gy = ry; return; }
  }
  s.gx = ROOM_X_MIN + SPR_H + 4;
  s.gy = ROOM_Y_MIN + SPR_H + 4;
}

static inline void setStatus(GameState &s, const char* msg, uint16_t ms) {
  strncpy(s.statusText, msg, sizeof(s.statusText));
  s.statusText[sizeof(s.statusText)-1] = '\0';
  s.statusMs = ms;
}



uint8_t computeLevel(uint32_t elapsedMs, int score, uint8_t difficulty) {
  int lv = 1 + (elapsedMs / 20000);  // 20s per level
  lv += score / 80;                 // bonus by score
  if (difficulty == 1) lv += (elapsedMs / 60000);
  if (difficulty == 2) lv += (elapsedMs / 45000);
  if (lv < 1) lv = 1;
  if (lv > 15) lv = 15;
  return (uint8_t)lv;
}
// Level progression is mainly time-based, starting with one level
// every 20 seconds. Score and difficulty can also accelerate
// progression, making the gameplay more dynamic.


// ================= HELP SCREEN TEXT =================
const char* helpLines[] = {
  "HOW TO PLAY",
  "",
  "Move with joystick",
  "Catch the ghost",
  "Use light carefully",
  "",
  "SCORING",
  "+1 every second alive",
  "Catch ghost = +5",
  "Combo = extra points",
  "",
  "LIGHT RULE",
  "Cover LDR = light ON",
  "Hold LDR = power down",
  "No power = danger",
  "",
  "BUTTON",
  "Short press = select/back",
  "Long press = pause/menu",
  "",
  "Move up/down to scroll",
  "Press SW to go back"
};
const int helpLineCount = sizeof(helpLines) / sizeof(helpLines[0]);
const int visibleHelpLines = 8;


void drawPlayerSimple(int x, int y, int sh) {
  int px = x + sh;
  // player = simple hunter
  canvas.fillRect(px - 3, y - 3, 7, 7, C_CYAN);
  canvas.drawPixel(px - 1, y - 1, C_BG);
  canvas.drawPixel(px + 1, y - 1, C_BG);
  canvas.drawFastHLine(px - 1, y + 1, 3, C_BG);
}

void drawGhostSimple(int x, int y, int sh, GhostType t) {
  int gx = x + sh;
  uint16_t body = (t == G_TRICK) ? RGB565(255,120,120) : C_RED;
  uint16_t glow = (t == G_TRICK) ? RGB565(140,40,40) : RGB565(90,20,20);

  canvas.drawCircle(gx, y, 8, glow);
  canvas.fillCircle(gx, y - 1, 4, body);
  canvas.fillRect(gx - 4, y - 1, 9, 6, body);
  canvas.drawPixel(gx - 4, y + 5, body);
  canvas.drawPixel(gx,     y + 4, body);
  canvas.drawPixel(gx + 4, y + 5, body);

  canvas.fillRect(gx - 2, y - 1, 2, 2, C_WHITE);
  canvas.fillRect(gx + 1, y - 1, 2, 2, C_WHITE);
  canvas.drawPixel(gx - 2, y, C_BG);
  canvas.drawPixel(gx + 1, y, C_BG);
}

// ================= GRAPHICS FUNCTIONS =================
// These functions draw the visual parts of the game such as:
// - background
// - player
// - ghost
// - flashlight cone
// - health/power bars
// - warning effects
//
// The display task calls these functions to render the current frame.

void drawHorrorBackground(uint8_t shake) {
  for (int y = 0; y < SCREEN_H; y++) {
    uint8_t v = map(y, 0, SCREEN_H-1, 10, 42);
    uint16_t col = RGB565(v, v, v + 6);
    canvas.drawFastHLine(0, y, SCREEN_W, col);
  }
  for (int i = 0; i < 70; i++) {
    int x = random(0, SCREEN_W);
    int y = random(HUD_H, SCREEN_H);
    canvas.drawPixel(x, y, (random(100) < 70) ? C_GRAY1 : C_GRAY2);
  }
  uint16_t scan = RGB565(8, 8, 10);
  for (int y = 0; y < SCREEN_H; y += 2) canvas.drawFastHLine(0, y, SCREEN_W, scan);

  if (shake > 0 && random(0, 100) < 35) {
    int y = random(HUD_H, SCREEN_H);
    canvas.drawFastHLine(0, y, SCREEN_W, C_GRAY3);
  }
}

void drawFlashlightCone(int px, int py, int dirX, int dirY, int sh) {
  if (dirX == 0 && dirY == 0) { dirY = -1; }

  int vx = dirX * 7;
  int vy = dirY * 7;

  int x0 = px + sh;
  int y0 = py;

  int x1 = x0 + vx + (dirY * 14);
  int y1 = y0 + vy + (-dirX * 14);

  int x2 = x0 + vx - (dirY * 14);
  int y2 = y0 + vy - (-dirX * 14);

  canvas.drawLine(x0, y0, x1, y1, C_YELLOW);
  canvas.drawLine(x0, y0, x2, y2, C_YELLOW);
  canvas.drawLine(x1, y1, x2, y2, C_GRAY3);

  canvas.drawCircle(x0, y0, 10, C_GRAY3);
  canvas.drawCircle(x0, y0, 12, C_GRAY2);
}

void drawLowHpVignette(int hp) {
  if (hp > 20) return;
  bool pulse = ((millis() / 180) % 2) == 0;
  uint16_t col = pulse ? C_DRED : RGB565(70,10,10);
  canvas.drawRect(0, 0, SCREEN_W, SCREEN_H, col);
  canvas.drawRect(1, 1, SCREEN_W-2, SCREEN_H-2, col);
}

void drawGhostFlicker(int x, int y, int sh, GhostType t) {
  bool flick = (random(0, 100) < (t == G_TRICK ? 35 : 15));
  if (flick) canvas.drawCircle(x + sh, y, 10, C_GRAY3);
  drawGhostSimple(x, y, sh, t);
}

void drawBar(int x, int y, int w, int val100, uint16_t col) {
  val100 = constrain(val100, 0, 100);
  int fill = map(val100, 0, 100, 0, w);
  canvas.drawRect(x, y, w + 2, 6, C_GRAY3);
  canvas.fillRect(x + 1, y + 1, fill, 4, col);
}

// ================= UI SCREEN FUNCTIONS =================
// Separate functions are used for each screen:
// - boot screen
// - menu screen
// - help screen
// - pause screen
// - gameplay screen
// - game over screen
//
// This keeps UI logic modular and easier to maintain.

void drawBoot() {
  canvas.fillScreen(C_BG);
  canvas.setTextSize(2);
  canvas.setTextColor(C_GRAY3);
  canvas.setCursor(10, 18); canvas.print("GHOST HUNTER");
  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(26, 52); canvas.print("ESP32 + FreeRTOS");
  canvas.setCursor(26, 68); canvas.print("Loading...");
}

void drawMenu(const GameState &s) {
  drawHorrorBackground(0);
  canvas.setTextSize(2);
  canvas.setTextColor(C_GRAY3);
  canvas.setCursor(10, 8);
  canvas.print("GHOST HUNTER");
  canvas.drawFastHLine(8, 30, 144, C_GRAY3);

  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);

  int y = 42;
  canvas.setCursor(18, y);
  canvas.print(s.menuIndex==0 ? "> START" : "  START");

  y += 14;
  canvas.setCursor(18, y);
  canvas.print(s.menuIndex==1 ? "> HOW TO PLAY" : "  HOW TO PLAY");

  y += 14;
  canvas.setCursor(18, y);
  canvas.print(s.menuIndex==2 ? "> DIFF: " : "  DIFF: ");
  canvas.setTextColor(C_YELLOW);
  canvas.print(diffName(s.difficulty));

  y += 16;
  canvas.setCursor(18, y);
  canvas.setTextColor(C_WHITE);
  canvas.print("BEST: ");
  canvas.setTextColor(C_GREEN);
  canvas.print(s.highScore[s.difficulty]);

  canvas.setCursor(10, 112);
  canvas.setTextColor(C_GRAY3);
  canvas.print("SW=Select  Joy=Move");
}

void drawHelp(const GameState &s) {
  drawHorrorBackground(0);
  canvas.setTextSize(2);
  canvas.setTextColor(C_GRAY3);
  canvas.setCursor(52, 8);
  canvas.print("HELP");

  canvas.drawFastHLine(8, 28, 144, C_GRAY3);

  canvas.setTextSize(1);
  int y = 36;
  for (int i = 0; i < visibleHelpLines; i++) {
    int idx = s.helpScroll + i;
    if (idx >= helpLineCount) break;

    if (idx == 0) canvas.setTextColor(C_CYAN);
    else if (strcmp(helpLines[idx], "SCORING") == 0 || strcmp(helpLines[idx], "LIGHT RULE") == 0 || strcmp(helpLines[idx], "BUTTON") == 0) {
      canvas.setTextColor(C_YELLOW);
    } else if (strlen(helpLines[idx]) == 0) {
      canvas.setTextColor(C_WHITE);
    } else {
      canvas.setTextColor(C_WHITE);
    }

    canvas.setCursor(8, y);
    canvas.print(helpLines[idx]);
    y += 11;
  }

  // small scroll hint
  canvas.setTextColor(C_GRAY3);
  if (s.helpScroll > 0) {
    canvas.setCursor(145, 36);
    canvas.print("^");
  }
  if (s.helpScroll < (helpLineCount - visibleHelpLines)) {
    canvas.setCursor(145, 118);
    canvas.print("v");
  }

  canvas.setCursor(10, 118);
  canvas.print("SW=Back  Joy=Scroll");
}

void drawPause(const GameState &s) {
  drawHorrorBackground(0);
  canvas.setTextSize(2);
  canvas.setTextColor(C_YELLOW);
  canvas.setCursor(52, 14);
  canvas.print("PAUSED");

  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(24, 54); canvas.print("SW=Resume");
  canvas.setCursor(24, 68); canvas.print("Hold SW=Menu");

  canvas.setCursor(20, 96);
  canvas.setTextColor(C_GRAY3);
  canvas.print("S:"); canvas.print(s.score);
  canvas.print(" L:"); canvas.print(s.level);
  canvas.print(" C:"); canvas.print(s.combo);
}

void drawPlay(const GameState &s, int joyDirX, int joyDirY) {
  int sh = (s.shake > 0) ? random(-s.shake, s.shake + 1) : 0;

  drawHorrorBackground(s.shake);

  canvas.fillRect(0, 0, SCREEN_W, HUD_H, RGB565(6,6,8));
  canvas.drawFastHLine(0, HUD_H, SCREEN_W, C_GRAY3);

  canvas.setTextSize(1);

  canvas.setCursor(6 + sh, 6);
  canvas.setTextColor(C_WHITE);
  canvas.print("S:");
  canvas.setTextColor(C_YELLOW);
  canvas.print(s.score);

  canvas.setCursor(92 + sh, 6);
  canvas.setTextColor(C_WHITE);
  canvas.print("LV:");
  canvas.setTextColor(C_YELLOW);
  canvas.print(s.level);

  canvas.setTextColor(C_WHITE);
  canvas.setCursor(6 + sh, 16);
  canvas.print("HP");
  drawBar(26 + sh, 14, 54, s.hp, (s.hp <= 20) ? C_RED : RGB565(255,90,90));

  canvas.setTextColor(C_WHITE);
  canvas.setCursor(92 + sh, 16);
  canvas.print("PWR");
  drawBar(120 + sh, 14, 36, s.power, (s.power <= 15) ? C_YELLOW : C_CYAN);

  canvas.setCursor(6 + sh, 24);
  canvas.setTextColor(s.lightOn ? C_GREEN : C_WHITE);
  canvas.print(s.lightOn ? "LIGHT:ON" : "LIGHT:OFF");

  canvas.setCursor(108 + sh, 24);
  canvas.setTextColor(C_WHITE);
  canvas.print("C:");
  canvas.setTextColor(C_YELLOW);
  canvas.print(s.combo);

  canvas.drawRect(ROOM_X_MIN + sh, ROOM_Y_MIN - 4,
                  (ROOM_X_MAX - ROOM_X_MIN) + 1,
                  (ROOM_Y_MAX - (ROOM_Y_MIN - 4)) + 1, C_GRAY3);

  if (s.banishFlash) {
    canvas.drawRect(ROOM_X_MIN + 1 + sh, ROOM_Y_MIN - 3,
                    (ROOM_X_MAX - ROOM_X_MIN) - 1,
                    (ROOM_Y_MAX - (ROOM_Y_MIN - 4)) - 1, C_WHITE);
  }

  if (s.lightOn && s.power > 0) {
    drawFlashlightCone(s.px, s.py, joyDirX, joyDirY, sh);
  }

  drawPlayerSimple(s.px, s.py, sh);

  drawGhostFlicker(s.gx, s.gy, sh, s.gType);

  canvas.drawFastHLine(0, FOOTER_Y - 2, SCREEN_W, C_GRAY3);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(6 + sh, FOOTER_Y);

  if (s.statusMs > 0) {
    canvas.setTextColor(C_WHITE);
    canvas.print(s.statusText);
  } else if (s.warnOverload) {
    canvas.setTextColor(C_RED);
    canvas.print("OVERLOAD!");
  } else if (s.warnLowPower) {
    canvas.setTextColor(C_YELLOW);
    canvas.print("LOW POWER");
  } else if (s.warnLowHP) {
    canvas.setTextColor(C_RED);
    canvas.print("LOW HP");
  } else {
    canvas.setTextColor(C_GRAY3);
    canvas.print(s.lightOn ? "HUNT" : "HIDE");
  }

  int d2 = distSq(s.px, s.py, s.gx, s.gy);
  int radar = 100 - constrain((int)(sqrt((float)d2) * 100.0f / 120.0f), 0, 100);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(92 + sh, FOOTER_Y);
  canvas.print("RAD");
  drawBar(RADAR_X + sh, FOOTER_Y - 2, RADAR_W, radar, (radar > 70) ? C_RED : C_GREEN);

  drawLowHpVignette(s.hp);
}

void drawGameOver(const GameState &s) {
  drawHorrorBackground(2);

  canvas.setTextSize(2);
  canvas.setTextColor(C_RED);
  canvas.setCursor(30, 14);
  canvas.print("GAME OVER");

  canvas.setTextSize(1);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(22, 46); canvas.print("Score: ");
  canvas.setTextColor(C_YELLOW); canvas.print(s.score);

  canvas.setTextColor(C_WHITE);
  canvas.setCursor(22, 60); canvas.print("Best Combo: ");
  canvas.setTextColor(C_YELLOW); canvas.print(s.bestCombo);

  canvas.setTextColor(C_WHITE);
  canvas.setCursor(22, 74); canvas.print("Diff: ");
  canvas.setTextColor(C_YELLOW); canvas.print(diffName(s.difficulty));

  canvas.setTextColor(C_WHITE);
  canvas.setCursor(22, 88); canvas.print("Best: ");
  canvas.setTextColor(C_GREEN); canvas.print(s.highScore[s.difficulty]);

  canvas.setCursor(12, 112);
  canvas.setTextColor(C_GRAY3);
  if ((millis() / 400) % 2 == 0) canvas.print("SW=Restart  Hold=Menu");
}

const char* taskStateName(eTaskState s) {
  switch (s) {
    case eRunning:   return "RUN";
    case eReady:     return "READY";
    case eBlocked:   return "BLOCK";
    case eSuspended: return "SUSP";
    case eDeleted:   return "DEL";
    default:         return "UNK";
  }
}

// resetRound() restores gameplay variables to their default values
// when a new game starts or when the player restarts after game over.

void resetRound(GameState &s) {
  s.ui = UI_PLAY;
  s.hp = 100;
  s.score = 0;
  s.power = 100;
  s.combo = 0;
  s.bestCombo = 0;
  s.level = 1;
  s.lightOn = false;
  s.warnOverload = false;
  s.warnLowPower = false;
  s.warnLowHP = false;
  s.banishFlash = false;
  s.shake = 0;
  s.statusMs = 0;
  s.px = SCREEN_W/2;
  s.py = (ROOM_Y_MIN + ROOM_Y_MAX)/2;
  s.gType = pickGhostType(1);
  respawnGhostFar(s);
}


// ================= TASKS =================

// ============================================================
// JoystickTask
// Type   : Periodic FreeRTOS Task
// Period : 20 ms
//
// Function:
// - Reads joystick X and Y ADC values
// - Applies deadzone around center position
// - Converts analog values to movement directions (-1, 0, +1)
// - Reads joystick button state
// - Sends latest joystick data to qJoy queue
//
// Why this task is periodic:
// Joystick input must be sampled regularly to keep player movement
// smooth and responsive.
//
// Real-time note:
// vTaskDelayUntil() is used so this task runs at fixed intervals
// with minimal drift.
// ============================================================

void JoystickTask(void *pv) {
#if RTOS_VERBOSE
  VPRINT("[TASK] JoystickTask started on core %d\n", xPortGetCoreID());
#endif
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(INPUT_TICK_MS);
  uint32_t lastPrintMs = 0;

  while (true) {
    int x = readAvgADC(JOY_X);
    int y = readAvgADC(JOY_Y);

    int dx = x - joyCx;
    int dy = y - joyCy;

    JoyMsg msg;
    msg.mx = (abs(dx) > DEADZONE) ? ((dx > 0) ? 1 : -1) : 0;
    msg.my = (abs(dy) > DEADZONE) ? ((dy > 0) ? 1 : -1) : 0;
    msg.sw = (digitalRead(JOY_SW) == LOW);

    xQueueOverwrite(qJoy, &msg);

#if RTOS_VERBOSE
    if (millis() - lastPrintMs > 2000) {
      VPRINT("[JoystickTask] Running mx:%d my:%d sw:%d\n", msg.mx, msg.my, (int)msg.sw);
      lastPrintMs = millis();
    }
#endif
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
// LDRTask
// Type   : Periodic FreeRTOS Task
// Period : 40 ms
//
// Function:
// - Reads LDR sensor value
// - Compares raw value with baseline threshold
// - Determines whether torch should be ON or OFF
// - Sends latest LDR data to qLdr queue
//
// Why this task is periodic:
// Light sensor input does not need extremely fast sampling,
// but it must still be updated regularly for responsive torch control.
//
// Real-time note:
// vTaskDelayUntil() ensures stable sensor polling intervals.
// ============================================================

void LDRTask(void *pv) {
#if RTOS_VERBOSE
  VPRINT("[TASK] LDRTask started on core %d\n", xPortGetCoreID());
#endif
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(LDR_TICK_MS);
  uint32_t lastPrintMs = 0;

  while (true) {
    int raw = readAvgADC(LDR_PIN);
    bool on = (raw < (ldrBaseline - LDR_MARGIN));

    LdrMsg msg;
    msg.raw = raw;
    msg.lightOn = on;

    xQueueOverwrite(qLdr, &msg);

#if RTOS_VERBOSE
    if (millis() - lastPrintMs > 2000) {
      VPRINT("[LDRTask] Light raw:%d on:%d\n", msg.raw, (int)msg.lightOn);
      lastPrintMs = millis();
    }
#endif
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
// GameTask
// Type   : Periodic FreeRTOS Task
// Period : 33 ms
//
// Function:
// - Reads latest joystick and LDR values from queues
// - Handles menu, help, pause, play, and game over states
// - Updates player movement
// - Updates level progression
// - Controls torch power drain and overload behaviour
// - Moves ghost and applies AI behaviour
// - Checks collisions between player and ghost
// - Updates score, combo, health, and high score
// - Detects banish events and game over condition
//
// Why this task is periodic:
// Game logic must update continuously at a stable rate.
// 33 ms corresponds to around 30 updates per second.
//
// Real-time note:
// This is the main gameplay task, so it is given higher priority
// than display and verbose tasks.
// ============================================================

void GameTask(void *pv) {
#if RTOS_VERBOSE
  VPRINT("[TASK] GameTask started on core %d\n", xPortGetCoreID());
#endif
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(GAME_TICK_MS);

  JoyMsg joy = {0,0,false};
  LdrMsg ldr = {0,false};

  int menuMoveCooldown = 0;
  int helpMoveCooldown = 0;
  bool lastSW = false;
  int swHoldTicks = 0;

  int overloadTicks = 0;
  int banishCooldown = 0;
  int flashTicks = 0;

  uint32_t startMs = millis();
  uint32_t lastScoreMs = millis();
  uint32_t bootMs = millis();
  uint32_t lastPrintMs = 0;

  bool lastLight = false;

  while (true) {
    xQueuePeek(qJoy, &joy, 0);
    xQueuePeek(qLdr, &ldr, 0);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    GameState &s = gState;

    if (s.shake > 0) s.shake--;
    if (s.statusMs > 0) s.statusMs = (s.statusMs > GAME_TICK_MS) ? (s.statusMs - GAME_TICK_MS) : 0;

    if (s.ui == UI_BOOT) {
      if (millis() - bootMs > 1200) s.ui = UI_MENU;
      xSemaphoreGive(stateMutex);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    bool swNow = joy.sw;
    bool swPressedEdge = (swNow && !lastSW);
    lastSW = swNow;

    if (swNow) swHoldTicks++;
    else swHoldTicks = 0;

#if RTOS_VERBOSE
    if (swPressedEdge) VPRINT("[EVT] SW PRESS (ui:%u)\n", (unsigned)s.ui);
#endif

    if (s.ui == UI_MENU) {
      if (menuMoveCooldown > 0) menuMoveCooldown--;

      if (menuMoveCooldown == 0) {
        if (joy.my < 0) { s.menuIndex = (s.menuIndex == 0) ? 2 : (s.menuIndex - 1); menuMoveCooldown = 8; }
        else if (joy.my > 0) { s.menuIndex = (s.menuIndex + 1) % 3; menuMoveCooldown = 8; }
      }

      if (swPressedEdge) {
        if (s.menuIndex == 0) {
          resetRound(s);

          overloadTicks = 0; banishCooldown = 0; flashTicks = 0;
          startMs = millis(); lastScoreMs = millis();
          lastLight = false;

#if RTOS_VERBOSE
          VPRINT("[EVT] START GAME diff=%s\n", diffName(s.difficulty));
#endif

        } else if (s.menuIndex == 1) {
          s.helpScroll = 0;
          s.ui = UI_HELP;
#if RTOS_VERBOSE
          VPRINT("[EVT] OPEN HELP\n");
#endif
        } else {
          s.difficulty = (s.difficulty + 1) % 3;
#if RTOS_VERBOSE
          VPRINT("[EVT] DIFF -> %s\n", diffName(s.difficulty));
#endif
        }
      }

      xSemaphoreGive(stateMutex);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    if (s.ui == UI_HELP) {
      if (helpMoveCooldown > 0) helpMoveCooldown--;

      if (helpMoveCooldown == 0) {
        if (joy.my < 0 && s.helpScroll > 0) {
          s.helpScroll--;
          helpMoveCooldown = 6;
#if RTOS_VERBOSE
          VPRINT("[EVT] HELP SCROLL UP -> %d\n", s.helpScroll);
#endif
        } else if (joy.my > 0 && s.helpScroll < (helpLineCount - visibleHelpLines)) {
          s.helpScroll++;
          helpMoveCooldown = 6;
#if RTOS_VERBOSE
          VPRINT("[EVT] HELP SCROLL DOWN -> %d\n", s.helpScroll);
#endif
        }
      }

      if (swPressedEdge) {
        s.helpScroll = 0;
        s.ui = UI_MENU;
      }
      xSemaphoreGive(stateMutex);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    if (s.ui == UI_PAUSE) {
      if (swPressedEdge) s.ui = UI_PLAY;
      if (swHoldTicks > 20) s.ui = UI_MENU;
      xSemaphoreGive(stateMutex);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    if (s.ui == UI_GAMEOVER) {
      if (swPressedEdge) {
        resetRound(s);

        overloadTicks = 0; banishCooldown = 0; flashTicks = 0;
        startMs = millis(); lastScoreMs = millis();
        lastLight = false;

#if RTOS_VERBOSE
        VPRINT("[EVT] RESTART\n");
#endif
      }
      if (swHoldTicks > 20) s.ui = UI_MENU;

      xSemaphoreGive(stateMutex);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    if (swHoldTicks > 20) {
      s.ui = UI_PAUSE;
      xSemaphoreGive(stateMutex);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    uint32_t nowMs = millis();

    uint8_t newLv = computeLevel(nowMs - startMs, s.score, s.difficulty);
    if (newLv != s.level) {
      s.level = newLv;
      setStatus(s, "LEVEL UP!", 700);
      s.shake = 2;
#if RTOS_VERBOSE
      VPRINT("[EVT] LEVEL UP -> %d\n", s.level);
#endif
    }

    s.lightOn = ldr.lightOn;

#if RTOS_VERBOSE
    if (s.lightOn != lastLight) {
      VPRINT("[LDRTask] LDR -> LIGHT %s (raw:%d)\n", s.lightOn ? "ON" : "OFF", ldr.raw);
      lastLight = s.lightOn;
    }
#endif

    s.warnOverload = false;
    s.warnLowPower = (s.power <= 15);
    s.warnLowHP = (s.hp <= 20);

    s.px += joy.mx * SPEED;
    s.py += joy.my * SPEED;
    s.px = constrain(s.px, ROOM_X_MIN + SPR_H, ROOM_X_MAX - SPR_H);
    s.py = constrain(s.py, ROOM_Y_MIN + SPR_H, ROOM_Y_MAX - SPR_H);

    if (banishCooldown > 0) banishCooldown--;
    if (flashTicks > 0) { flashTicks--; s.banishFlash = true; }
    else s.banishFlash = false;

    if (s.lightOn) {
      overloadTicks++;
      s.power -= 2;
      if (s.power < 0) s.power = 0;

      if (overloadTicks > OVERLOAD_LIMIT_TICKS) {
        s.warnOverload = true;
        setStatus(s, "OVERLOAD!", 900);
        s.power = 0;
        overloadTicks = 0;

        s.hp -= 10;
        if (s.hp < 0) s.hp = 0;
        s.combo = 0;
        s.shake = 3;

#if RTOS_VERBOSE
        VPRINT("[EVT] OVERLOAD! hp=%d pwr=%d\n", s.hp, s.power);
#endif
      }
    } else {
      overloadTicks = 0;
      s.power += 1;
      if (s.power > 100) s.power = 100;
    }

    int sp = ghostSpeedFor(s.gType, s.level, s.difficulty);
    bool freezeGhost = (s.lightOn && s.power > 0);

    bool trickTeleport = false;
    if (s.gType == G_TRICK && s.lightOn && s.power > 0) {
      if (random(0, 100) < (2 + s.level / 2)) trickTeleport = true;
    }

    if (trickTeleport) {
      respawnGhostFar(s);
      setStatus(s, "GHOST SHIFT!", 600);
#if RTOS_VERBOSE
      VPRINT("[EVT] GHOST SHIFT!\n");
#endif
    } else if (!freezeGhost) {
      int dx = s.px - s.gx;
      int dy = s.py - s.gy;
      if (dx != 0) s.gx += (dx > 0) ? sp : -sp;
      if (dy != 0) s.gy += (dy > 0) ? sp : -sp;
      s.gx = constrain(s.gx, ROOM_X_MIN + SPR_H, ROOM_X_MAX - SPR_H);
      s.gy = constrain(s.gy, ROOM_Y_MIN + SPR_H, ROOM_Y_MAX - SPR_H);
    }

    int d2 = distSq(s.px, s.py, s.gx, s.gy);

    if (!s.lightOn) {
      if (d2 < 64) {
        int dmg = (s.gType == G_FAST) ? 4 : 3;
        s.hp -= dmg;
        s.combo = 0;
        s.shake = 2;
        setStatus(s, "HIT!", 450);
      } else if (d2 < 256) {
        int dmg = (s.gType == G_FAST) ? 2 : 1;
        s.hp -= dmg;
        s.combo = 0;
      }
      if (s.hp < 0) s.hp = 0;
    }

    if (s.lightOn && s.power > 0 && banishCooldown == 0 && d2 < 400) {
      s.score += 5 + s.combo;
      s.combo++;
      if (s.combo > s.bestCombo) s.bestCombo = s.combo;

      flashTicks = 6;
      banishCooldown = 10;

      setStatus(s, "BANISHED!", 650);
      s.shake = 1;

#if RTOS_VERBOSE
      VPRINT("[EVT] BANISH! score=%d combo=%d lv=%d\n", s.score, s.combo, s.level);
#endif

      s.gType = pickGhostType(s.level);
      respawnGhostFar(s);
    }

    if (nowMs - lastScoreMs > 1000) {
      lastScoreMs = nowMs;
      s.score += 1;
    }

    if (s.score > s.highScore[s.difficulty]) s.highScore[s.difficulty] = s.score;

#if RTOS_VERBOSE
    if (millis() - lastPrintMs > 2000) {
      VPRINT("[GameTask] Score:%d HP:%d PWR:%d LV:%d UI:%u\n", s.score, s.hp, s.power, s.level, (unsigned)s.ui);
      lastPrintMs = millis();
    }
#endif

    if (s.hp <= 0) {
      s.hp = 0;
      s.ui = UI_GAMEOVER;
      s.combo = 0;
      setStatus(s, "GAME OVER", 800);
#if RTOS_VERBOSE
      VPRINT("[EVT] GAME OVER finalScore=%d bestCombo=%d\n", s.score, s.bestCombo);
#endif
    }

    xSemaphoreGive(stateMutex);
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
// DisplayTask
// Type   : Periodic FreeRTOS Task
// Period : 33 ms
//
// Function:
// - Reads protected copy of current game state
// - Selects the correct screen to draw
// - Renders UI and gameplay graphics on off-screen canvas
// - Pushes final frame to TFT display
//
// Why this task is periodic:
// The display must refresh regularly to keep animation smooth.
//
// Real-time note:
// Rendering is separated from game logic to reduce blocking and
// improve system organisation.
// ============================================================

void DisplayTask(void *pv) {
#if RTOS_VERBOSE
  VPRINT("[TASK] DisplayTask started on core %d\n", xPortGetCoreID());
#endif
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(UI_TICK_MS);

  GameState local;
  JoyMsg joyLocal;
  uint32_t lastPrintMs = 0;

  while (true) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    local = gState;
    xSemaphoreGive(stateMutex);

    xQueuePeek(qJoy, &joyLocal, 0);

    switch (local.ui) {
      case UI_BOOT:     drawBoot(); break;
      case UI_MENU:     drawMenu(local); break;
      case UI_HELP:     drawHelp(local); break;
      case UI_PLAY:     drawPlay(local, joyLocal.mx, joyLocal.my); break;
      case UI_PAUSE:    drawPause(local); break;
      case UI_GAMEOVER: drawGameOver(local); break;
      default:          drawMenu(local); break;
    }

    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_W, SCREEN_H);
#if RTOS_VERBOSE
    if (millis() - lastPrintMs > 2000) {
      VPRINT("[DisplayTask] Screen updated ui:%u\n", (unsigned)local.ui);
      lastPrintMs = millis();
    }
#endif
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
// VerboseTask
// Type   : Periodic FreeRTOS Task
// Period : 500 ms
//
// Function:
// - Prints RTOS execution information to Serial Monitor
// - Shows queue values, game state, task priorities,
//   stack usage, and task states
//
// Why this task is periodic:
// Serial debugging is only needed at a slower rate.
//
// CW Task 3 note:
// This task provides evidence that FreeRTOS tasks are running
// and helps demonstrate task scheduling behaviour.
// ============================================================

void VerboseTask(void *pv) {
#if RTOS_VERBOSE
  VPRINT("[TASK] VerboseTask started on core %d\n", xPortGetCoreID());
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(VERBOSE_MS);

  while (true) {
    GameState s;
    JoyMsg j; LdrMsg l;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    s = gState;
    xSemaphoreGive(stateMutex);

    xQueuePeek(qJoy, &j, 0);
    xQueuePeek(qLdr, &l, 0);

    UBaseType_t prJoy  = uxTaskPriorityGet(hJoy);
    UBaseType_t prLdr  = uxTaskPriorityGet(hLdr);
    UBaseType_t prGame = uxTaskPriorityGet(hGame);
    UBaseType_t prUI   = uxTaskPriorityGet(hUI);

    UBaseType_t hwJoy  = uxTaskGetStackHighWaterMark(hJoy);
    UBaseType_t hwLdr  = uxTaskGetStackHighWaterMark(hLdr);
    UBaseType_t hwGame = uxTaskGetStackHighWaterMark(hGame);
    UBaseType_t hwUI   = uxTaskGetStackHighWaterMark(hUI);

    eTaskState stJoy  = eTaskGetState(hJoy);
    eTaskState stLdr  = eTaskGetState(hLdr);
    eTaskState stGame = eTaskGetState(hGame);
    eTaskState stUI   = eTaskGetState(hUI);

    int thr = (ldrBaseline - LDR_MARGIN);

    VPRINT("\n[RTOS %lums tick:%lu core:%d] ui:%u lv:%d diff:%s score:%d best:%d hp:%d pwr:%d combo:%d light:%d gType:%u\n",
           (unsigned long)VERBOSE_MS, (unsigned long)xTaskGetTickCount(), xPortGetCoreID(),
           (unsigned)s.ui, s.level, diffName(s.difficulty),
           s.score, s.highScore[s.difficulty], s.hp, s.power, s.combo, (int)s.lightOn, (unsigned)s.gType);

    VPRINT("  Joy(mx:%d my:%d sw:%d)  LDR(raw:%d thr:%d on:%d)\n",
           j.mx, j.my, (int)j.sw, l.raw, thr, (int)l.lightOn);

    VPRINT("  Tasks: Joy[%s pr:%u hw:%u] LDR[%s pr:%u hw:%u]\n",
           taskStateName(stJoy), (unsigned)prJoy, (unsigned)hwJoy,
           taskStateName(stLdr), (unsigned)prLdr, (unsigned)hwLdr);
    VPRINT("         Game[%s pr:%u hw:%u] UI[%s pr:%u hw:%u] Verbose[RUN]\n",
           taskStateName(stGame), (unsigned)prGame, (unsigned)hwGame,
           taskStateName(stUI), (unsigned)prUI, (unsigned)hwUI);
    VPRINT("         Running now: VerboseTask | other tasks switch READY/BLOCK by schedule\n");

    vTaskDelayUntil(&lastWake, period);
  }
#else
  vTaskDelete(NULL);
#endif
}

// ================= SETUP FUNCTION =================
// setup() performs all one-time initialization:
// - starts serial monitor
// - configures ADC and input pins
// - initializes TFT display
// - calibrates joystick and LDR baseline
// - initializes shared game state
// - creates queues and mutex
// - creates all FreeRTOS tasks
//
// After setup(), the Arduino loop is no longer used for game logic
// because FreeRTOS tasks handle the full system execution.

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(JOY_SW, INPUT_PULLUP);

  // TFT init
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  randomSeed(esp_random());

  // Calibrate joystick
  {
    long sx = 0, sy = 0;
    for (int i = 0; i < 60; i++) {
      sx += readAvgADC(JOY_X);
      sy += readAvgADC(JOY_Y);
      delay(5);
    }
    joyCx = (int)(sx / 60);
    joyCy = (int)(sy / 60);
  }

  // Calibrate LDR baseline
  {
    long sum = 0;
    int count = 0;
    uint32_t start = millis();
    while (millis() - start < 2000) {
      sum += readAvgADC(LDR_PIN);
      count++;
      delay(10);
    }
    ldrBaseline = (count > 0) ? (int)(sum / count) : 2000;
  }

  stateMutex = xSemaphoreCreateMutex();

  // Init state
  gState.px = SCREEN_W/2; gState.py = (ROOM_Y_MIN + ROOM_Y_MAX)/2;
  gState.gx = ROOM_X_MIN + 20; gState.gy = ROOM_Y_MIN + 20;

  gState.hp = 100;
  gState.score = 0;
  gState.power = 100;
  gState.level = 1;
  gState.combo = 0;
  gState.bestCombo = 0;
  gState.highScore[0] = 0;
  gState.highScore[1] = 0;
  gState.highScore[2] = 0;

  gState.lightOn = false;
  gState.gType = G_SLOW;

  gState.ui = UI_BOOT;
  gState.menuIndex = 0;
  gState.difficulty = 1;
  gState.helpScroll = 0;

  gState.shake = 0;
  gState.banishFlash = false;

  strncpy(gState.statusText, "", sizeof(gState.statusText));
  gState.statusMs = 0;

  gState.warnOverload = false;
  gState.warnLowPower = false;
  gState.warnLowHP = false;

  respawnGhostFar(gState);

  // Queues
  qJoy = xQueueCreate(1, sizeof(JoyMsg));
  qLdr = xQueueCreate(1, sizeof(LdrMsg));
  JoyMsg j0 = {0,0,false};
  LdrMsg l0 = {0,false};
  xQueueOverwrite(qJoy, &j0);
  xQueueOverwrite(qLdr, &l0);

// ================= TASK CREATION =================
// Tasks are pinned to core 1 of the ESP32.
// Priority allocation:
// - GameTask     : priority 4 (highest among game tasks)
// - JoystickTask : priority 3
// - LDRTask      : priority 3
// - DisplayTask  : priority 2
// - VerboseTask  : priority 1
//
// This priority order ensures that game logic and inputs are handled
// before display refresh and debug printing.

  xTaskCreatePinnedToCore(JoystickTask, "JoystickTask", 4096, NULL, 3, &hJoy,  1);
  xTaskCreatePinnedToCore(LDRTask,      "LDRTask",      4096, NULL, 3, &hLdr,  1);
  xTaskCreatePinnedToCore(GameTask,     "GameTask",     8192, NULL, 4, &hGame, 1);
  xTaskCreatePinnedToCore(DisplayTask,  "DisplayTask",  8192, NULL, 2, &hUI,   1);

  xTaskCreatePinnedToCore(VerboseTask,  "VerboseTask",  4096, NULL, 1, NULL,   1);

#if RTOS_VERBOSE
  VPRINT("\n[BOOT] Game running + RTOS Serial monitor ON + per-difficulty high scores\n");
#endif
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}