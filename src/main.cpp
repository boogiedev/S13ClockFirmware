// SELF MAC ADDRESS A0:B7:65:FE:7B:38
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <ESP32Time.h>
#include "Filter.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Fonts/RONIX18.h>
#include <Fonts/RONIX17.h>
#include <Fonts/RONIX14.h>
#include <Fonts/RONIX12.h>
#include <Fonts/RONIX9.h>
#include <Fonts/RONIX6.h>
#include <Fonts/RONIX5.h>
#include <Fonts/RONIX4.h>
#include <Fonts/RONIX3.h>
#include <Fonts/RONIX2.h>
#include <logos.h>
#include <logoanimation.h>
#include <logoanimationatkinson.h>

// Screen Definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define LED_BUILTIN  2 // TEST LED BLINKING TO MAKE SURE SKETCH WORKS

#define DEBUG_MODE false

#define FIRMWARE_VERSION "V:0.01"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
GFXcanvas1 bootCanvas(SCREEN_WIDTH, SCREEN_HEIGHT);
GFXcanvas1 digitalClockCanvas(SCREEN_WIDTH, SCREEN_HEIGHT);



// Display Constants
constexpr int DISPLAY_CENTER_X = 64;
constexpr int DISPLAY_CENTER_Y = 32;

// Input Pins
constexpr int CW_PIN = 33;
constexpr int CCW_PIN = 32;
constexpr int PUSH_PIN = 34;

// Filtered ADC value that signals a press. The switchgear pulls the pin to
// full 4095 when pressed; the exponential filter only reaches saturation after
// a sustained read, which gives us de-facto debounce on top of the time-based
// one below. A lower threshold here causes spurious face-switches from noise
// or partial encoder positions.
constexpr int BUTTON_PRESSED_VAL = 4095;

// Debounce windows by mode
constexpr unsigned long DEBOUNCE_FACE_MS = 150;
constexpr unsigned long DEBOUNCE_EDIT_HOUR_MS = 600;
constexpr unsigned long DEBOUNCE_EDIT_MINUTE_MS = 400;

// Input Filters
ExponentialFilter<long> CWFilter(85, 0);
ExponentialFilter<long> CCWFilter(90, 0);
ExponentialFilter<long> PUSHFilter(90, 0);

// One-shot button events, set by readButton() each loop iteration
bool cwEvent = false;
bool ccwEvent = false;
bool pushEvent = false;
unsigned long lastButtonMs = 0;

// Clock Definitions
ESP32Time rtc(3600);
int hrs = 0;
int mins = 0;
int secs = 0;

// Mode state machine — two orthogonal axes: which face is shown, and which
// time component (if any) is currently being edited.
enum class Face : uint8_t { Analog, Digital };
enum class EditState : uint8_t { None, Hour, Minute };

Face face = Face::Analog;
EditState editState = EditState::None;

// 60-step sin/cos lookup, indexed by 6° increments (matches second, minute, and
// the analog face's 60-dot ring). Avoids ~120 trig calls per frame.
constexpr int NUM_POINTS = 60;
float sinTable[NUM_POINTS];
float cosTable[NUM_POINTS];


// Function Definitions
void bootScreen();
void silviaScreen();
void initiateTime();
void readButton();
void drawAnalogBackground();
void drawAnalogThinHand(int hand_angle, int hand_length_long, int hand_length_short);
void drawAnalogBoldHand(int hand_angle, int hand_length_long, int hand_length_short, int hand_dot_size);
void displayAnalogClock(EditState edit);
void displayDigitalClock(EditState edit);

// Gear Indicator Data Struct (ESP-NOW payload from shifter board)
struct shift_data {
  int hall_1;
  int hall_2;
  int hall_3;
  int hall_4;
  int hall_5;
  int hall_6;
  int gear_position;
};

shift_data shiftData;


int CURRENT_GEAR = 0;
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&shiftData, incomingData, sizeof(shiftData));
  CURRENT_GEAR = shiftData.gear_position;

  if (DEBUG_MODE) {
    Serial.printf("ESP-NOW recv %d bytes | HALL %d %d %d %d %d %d | gear=%d\n",
                  len,
                  shiftData.hall_1, shiftData.hall_2, shiftData.hall_3,
                  shiftData.hall_4, shiftData.hall_5, shiftData.hall_6,
                  shiftData.gear_position);
  }
}

void setup() {

  // Input Pin Setup
  pinMode(CW_PIN, INPUT);
  pinMode(CCW_PIN, INPUT);
  pinMode(PUSH_PIN, INPUT);

  // Serial Out Setup
  Serial.begin(9600);
  Serial.println("Starting System");

  // Print MAC Address to Serial monitor
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);


  if (!DEBUG_MODE) {
    bootScreen();
    display.clearDisplay();
    display.display();
    delay(1000);
    silviaScreen();
    display.clearDisplay();
    display.display();
    delay(1000);
  }

  // Setup Clock
  rtc.setTime(1712484660);
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));

  // Pre-compute sin/cos for each minute/second position (6° increments)
  for (int i = 0; i < NUM_POINTS; i++) {
    float angle = radians(i * 6);
    sinTable[i] = sin(angle);
    cosTable[i] = cos(angle);
  }

  display.clearDisplay();
  display.display();
}

void loop() {
  readButton();

  // Button handling — single source of truth driving (face, editState)
  if (pushEvent) {
    // Cycle edit phase: None -> Hour -> Minute -> None
    editState = (editState == EditState::None)  ? EditState::Hour
              : (editState == EditState::Hour)  ? EditState::Minute
                                                : EditState::None;
  } else if (cwEvent || ccwEvent) {
    if (editState == EditState::None) {
      face = (face == Face::Analog) ? Face::Digital : Face::Analog;
    } else if (cwEvent) {
      // Match the original analogue-clock UX: time edits only go forward
      time_t epoch = rtc.getLocalEpoch();
      rtc.setTime(epoch + (editState == EditState::Hour ? 3600 : 60));
    }
  }

  // No explicit FPS cap — display.display() does a blocking i2c transfer that
  // naturally throttles the loop to ~30 Hz. Adding an early-return rate limit
  // here caused readButton() to run thousands of times per second instead of
  // ~30, which let the exponential ADC filter saturate from brief noise spikes
  // and triggered phantom face-switches just from touching nearby wires.
  initiateTime();
  if (face == Face::Analog) {
    displayAnalogClock(editState);
  } else {
    displayDigitalClock(editState);
  }
  display.display();
  display.clearDisplay();
}


void silviaScreen() {
  // Animated transition (~2.5s) into the silvia logo:
  //   frames 0-37 use the atkinson-dithered array — sparse dots that fill in
  //     gradually, giving a smooth "wipe" feel out of the Club 187 boot screen
  //   frames 38-50 use the clean ANIMATEDLOGOARRAY — wordmark resolves out
  //     of the dither
  // The ANIMATEDLOGOARRAY frames only cover rows 28-36 (a thin wordmark), so
  // we follow the transition with the full SILVIALOGO held for 3s.
  for (int i = 0; i < ANIMATEDLOGOARRAY_LEN; i++) {
    display.clearDisplay();
    if (i >= 38) {
      bootCanvas.drawBitmap(0, 0, ANIMATEDLOGOARRAY[i], 128, 64, WHITE, BLACK);
    } else {
      bootCanvas.drawBitmap(0, 0, ANIMATEDLOGOARRAY_ATKINSON[i], 128, 64, WHITE, BLACK);
    }
    display.drawBitmap(0, 0, bootCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, BLACK);
    display.display();
    delay(50);
  }

  // Show the outline alone briefly so the eye registers it as a starting frame
  display.clearDisplay();
  bootCanvas.fillScreen(0);
  bootCanvas.drawBitmap(0, 0, S13SILVIAOUTLINE, 128, 64, WHITE, BLACK);
  display.drawBitmap(0, 0, bootCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, BLACK);
  display.display();
  delay(250);

  // Dissolve fade: outline stays visible while SILVIALOGO pixels are revealed
  // progressively via an ordered dither. Each step adds another slice of the
  // logo's white pixels based on a per-pixel hash threshold (no native opacity
  // on a 1-bit OLED, so we fake the fade with pixel coverage).
  constexpr int FADE_STEPS = 16;
  constexpr int FADE_STEP_MS = 30;
  for (int step = 1; step <= FADE_STEPS; step++) {
    bootCanvas.fillScreen(0);
    bootCanvas.drawBitmap(0, 0, S13SILVIAOUTLINE, 128, 64, WHITE, BLACK);
    for (int y = 0; y < 64; y++) {
      for (int x = 0; x < 128; x++) {
        uint8_t b = pgm_read_byte(&SILVIALOGO[y * 16 + (x >> 3)]);
        if (!(b & (0x80 >> (x & 7)))) continue;            // pixel not lit in logo
        uint8_t threshold = (x * 31 + y * 17) & 0x0f;       // 0..15 pseudo-random
        if (threshold < step) {
          bootCanvas.drawPixel(x, y, WHITE);
        }
      }
    }
    display.clearDisplay();
    display.drawBitmap(0, 0, bootCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, BLACK);
    display.display();
    delay(FADE_STEP_MS);
  }

  // Hold the fully revealed silvia logo
  delay(2500);
}

void bootScreen() {
  // 128x64 C187 Display Screen
  display.clearDisplay(); // Always Clear display buffer
  bootCanvas.fillScreen(0);
  bootCanvas.drawBitmap(0, 0, CLUB187LOGOARRAY, 60, 61, WHITE, BLACK);
  bootCanvas.setTextSize(1);
  bootCanvas.setFont(&RONIX4);
  bootCanvas.setTextColor(WHITE);

  bootCanvas.setCursor(59, 20);
  bootCanvas.print("D");
  bootCanvas.setCursor(70, 20);
  bootCanvas.print("I");
  bootCanvas.setCursor(73, 20);
  bootCanvas.print("G");
  bootCanvas.setCursor(84, 20);
  bootCanvas.print("I");
  bootCanvas.setCursor(87, 20);
  bootCanvas.print("T");
  bootCanvas.setCursor(97, 20);
  bootCanvas.print("A");
  bootCanvas.setCursor(109, 20);
  bootCanvas.print("L");

  bootCanvas.setCursor(59, 30);
  bootCanvas.print("C");
  bootCanvas.setCursor(69, 30);
  bootCanvas.print("L");
  bootCanvas.setCursor(78, 30);
  bootCanvas.print("O");
  bootCanvas.setCursor(89, 30);
  bootCanvas.print("C");
  bootCanvas.setCursor(100, 30);
  bootCanvas.print("K");

  bootCanvas.setFont(NULL);
  bootCanvas.setCursor(59, 43);
  bootCanvas.println("CLUB 187");
  bootCanvas.setCursor(66, 52);
  bootCanvas.println(FIRMWARE_VERSION);

  display.drawBitmap(0, 0, bootCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, BLACK);
  display.display();
  delay(2000);

}




void displayAnalogClock(EditState edit) {
  // Gear indicator (top-left corner)
  display.setCursor(0, 0);
  if (CURRENT_GEAR > 0) {
    display.print(CURRENT_GEAR);
  } else if (CURRENT_GEAR < 0) {
    display.print("Reverse");
  } else if (CURRENT_GEAR == 0) {
    display.print("Neutral");
  } else {
    display.print("No");
    display.setCursor(0, 10);
    display.print("Gear");
    display.setCursor(0, 20);
    display.print("Data");
  }

  drawAnalogBackground();

  // Hands blink at 1Hz when their unit is being edited
  bool blinkOn = rtc.getMillis() > 500;
  bool showMinuteHand = (edit != EditState::Minute) || blinkOn;
  bool showHourHand = (edit != EditState::Hour) || blinkOn;

  if (showMinuteHand) drawAnalogBoldHand(mins * 6, 25, 10, 1);
  if (showHourHand) drawAnalogBoldHand(hrs * 30 + (mins / 2), 18, 10, 1);
  drawAnalogThinHand(secs * 6, 27, 22);

  // Center cap covering hand pivots
  display.fillCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 3, WHITE);
  display.fillCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 2, BLACK);
}

void drawAnalogBackground() {
  display.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 28, WHITE);

  // 60 dots around the circle, one per minute/second — LUT-driven
  for (int i = 0; i < NUM_POINTS; i++) {
    int xpos = round(DISPLAY_CENTER_X + sinTable[i] * 31);
    int ypos = round(DISPLAY_CENTER_Y - cosTable[i] * 31);
    display.drawPixel(xpos, ypos, WHITE);
  }

  // Cardinal tickmarks at 12/3/6/9 (LUT indices 0, 15, 30, 45)
  const int tickIndices[] = { 0, 15, 30, 45 };
  for (int i : tickIndices) {
    int xpos = round(DISPLAY_CENTER_X + sinTable[i] * 30);
    int ypos = round(DISPLAY_CENTER_Y - cosTable[i] * 30);
    int xpos2 = round(DISPLAY_CENTER_X + sinTable[i] * 23);
    int ypos2 = round(DISPLAY_CENTER_Y - cosTable[i] * 23);
    display.drawLine(xpos, ypos, xpos2, ypos2, WHITE);
  }

  // Hour numerals
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(80, 28); display.print("3");
  display.setCursor(62, 46); display.print("6");
  display.setCursor(44, 28); display.print("9");
  display.setCursor(60, 12); display.print("12");
}
// Thin hand (second hand) — line from a long-radius tip to a short-radius tail
// passing through the pivot. `hand_angle` is in degrees, 0 = 12 o'clock.
void drawAnalogThinHand(int hand_angle, int hand_length_long, int hand_length_short) {
  float angle_rad = radians(hand_angle);
  float s = sin(angle_rad);
  float c = cos(angle_rad);

  int xpos = round(DISPLAY_CENTER_X + s * hand_length_long);
  int ypos = round(DISPLAY_CENTER_Y - c * hand_length_long);
  int xpos2 = round(DISPLAY_CENTER_X - s * hand_length_short);  // +180° flips sign
  int ypos2 = round(DISPLAY_CENTER_Y + c * hand_length_short);

  display.drawLine(xpos, ypos, xpos2, ypos2, WHITE);
  display.fillCircle(xpos2, ypos2, 3, WHITE);
  display.fillCircle(xpos2, ypos2, 2, BLACK);
}


// Bold hand (minute and hour) — filled "lozenge" between two circles
void drawAnalogBoldHand(int hand_angle, int hand_length_long, int hand_length_short, int hand_dot_size) {
  float angle_rad = radians(hand_angle);
  float s = sin(angle_rad);
  float c = cos(angle_rad);
  // +90° rotation for the perpendicular offset that gives the lozenge width
  float s_perp = c;   // sin(angle+90)  =  cos(angle)
  float c_perp = -s;  // cos(angle+90)  = -sin(angle)

  int xpos = round(DISPLAY_CENTER_X + s * hand_length_long);
  int ypos = round(DISPLAY_CENTER_Y - c * hand_length_long);
  int xpos2 = round(DISPLAY_CENTER_X + s * hand_length_short);
  int ypos2 = round(DISPLAY_CENTER_Y - c * hand_length_short);

  int tri_xoff = round(s_perp * hand_dot_size);
  int tri_yoff = round(-c_perp * hand_dot_size);

  display.drawLine(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, xpos2, ypos2, WHITE);
  display.drawCircle(xpos, ypos, hand_dot_size, WHITE);
  display.drawCircle(xpos2, ypos2, hand_dot_size, WHITE);

  display.fillTriangle(xpos + tri_xoff, ypos + tri_yoff,
                       xpos - tri_xoff, ypos - tri_yoff,
                       xpos2 + tri_xoff, ypos2 + tri_yoff, WHITE);
  display.fillTriangle(xpos2 + tri_xoff, ypos2 + tri_yoff,
                       xpos2 - tri_xoff, ypos2 - tri_yoff,
                       xpos - tri_xoff, ypos - tri_yoff, WHITE);
}

// RONIX17 has uneven digit widths — these offsets are hand-tuned per digit so
// the time looks centered regardless of which digits are present.
static void drawDigitalHours(int y) {
  if (hrs > 9) {
    digitalClockCanvas.setCursor(0, y);
    digitalClockCanvas.print(hrs / 10);
  }
  int ones = hrs % 10;
  int x = (ones == 1) ? 25 : (ones == 0) ? 12 : 14;
  digitalClockCanvas.setCursor(x, y);
  digitalClockCanvas.print(ones);
}

static void drawDigitalMinutes(int y) {
  int tens = mins / 10;
  int ones = mins % 10;
  int x_tens = (tens == 1) ? 68 : (tens == 0) ? 55 : 56;
  int x_ones = (ones == 1) ? 93 : 90;
  digitalClockCanvas.setCursor(x_tens, y);
  digitalClockCanvas.print(tens);
  digitalClockCanvas.setCursor(x_ones, y);
  digitalClockCanvas.print(ones);
}

void displayDigitalClock(EditState edit) {
  constexpr int clockY = 42;
  bool blinkOn = rtc.getMillis() > 500;
  bool showHours = (edit != EditState::Hour) || blinkOn;
  bool showMinutes = (edit != EditState::Minute) || blinkOn;

  digitalClockCanvas.fillScreen(0);
  digitalClockCanvas.setFont(&RONIX17);
  digitalClockCanvas.setTextSize(1);
  digitalClockCanvas.setTextWrap(false);

  if (showHours) drawDigitalHours(clockY);
  if (showMinutes) drawDigitalMinutes(clockY);

  // Colon — x position depends on which adjacent digits are "1" (narrower)
  int colon_x = 52;
  if (hrs % 10 == 1 && mins / 10 != 1) {
    colon_x = 50;
  } else if (mins / 10 == 1) {
    colon_x = 54;
  }

  // Colon stays solid while editing; otherwise blinks at 1Hz
  bool showColon = (edit != EditState::None) || (secs % 2 == 0);
  if (showColon) {
    digitalClockCanvas.fillRect(colon_x, 26, 4, 4, WHITE);
    digitalClockCanvas.fillRect(colon_x, 36, 4, 4, WHITE);
  }

  display.drawBitmap(0, 0, digitalClockCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, BLACK);
}

void initiateTime() {
  hrs=rtc.getHour();
  mins=rtc.getMinute();
  secs=rtc.getSecond();
}

void readButton() {
  CWFilter.Filter(analogRead(CW_PIN));
  CCWFilter.Filter(analogRead(CCW_PIN));
  PUSHFilter.Filter(analogRead(PUSH_PIN));

  // Edit-mode tweaks the debounce so held knob rotations step at a usable pace
  unsigned long debounceMs = (editState == EditState::Hour)   ? DEBOUNCE_EDIT_HOUR_MS
                           : (editState == EditState::Minute) ? DEBOUNCE_EDIT_MINUTE_MS
                                                              : DEBOUNCE_FACE_MS;
  unsigned long now = millis();
  bool ready = (now - lastButtonMs) > debounceMs;

  pushEvent = false;
  cwEvent = false;
  ccwEvent = false;

  if (!ready) return;

  if (PUSHFilter.Current() == BUTTON_PRESSED_VAL) {
    pushEvent = true;
    lastButtonMs = now;
  } else if (CCWFilter.Current() == BUTTON_PRESSED_VAL) {
    ccwEvent = true;
    lastButtonMs = now;
  } else if (CWFilter.Current() == BUTTON_PRESSED_VAL) {
    cwEvent = true;
    lastButtonMs = now;
  }
}


