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
const int DISPLAY_CENTER_X = 64;
const int DISPLAY_CENTER_Y = 32;

// Input Pins
const int CW_PIN = 33;
const int CCW_PIN = 32;
const int PUSH_PIN = 34;

// Input VAL
int CW_VAL = 0;
int CCW_VAL = 0;
int PUSH_VAL = 0;
// Input Filters
ExponentialFilter<long> CWFilter(85, 0);
ExponentialFilter<long> CCWFilter(90, 0);
ExponentialFilter<long> PUSHFilter(90, 0);
// Input States
bool CW_STATE = false;
bool CCW_STATE = false;
bool PUSH_STATE = false;
int PUSH_EDIT_STATE = 0;
bool EDIT_HOUR = false;
bool EDIT_MINUTE = false;
// Input Debouncing
unsigned long lastDebounceTime1;
unsigned long lastDebounceClockEdit;
int debounce_time = 150;

// Clock Definitions
ESP32Time rtc(3600);
int hrs=0;
int mins=0;
int secs=0;

// Clock Display Flags
bool showAnalogClock = true;
bool showDigitalCLock = false;

const int NUM_POINTS = 60;
const int RADIUS = 28;
int pointsX[NUM_POINTS];
int pointsY[NUM_POINTS];


// Function Definitions
void bootScreen();
void silviaScreen();
void initiateTime();
void readButton(unsigned long debounce_time);
void drawAnalogBackground(bool editing);
void drawAnalogThinHand(int hand_angle, int hand_length_long, int hand_legth_short);
void drawAnalogBoldHand(int hand_angle, int hand_length_long, int hand_legth_short, int hand_dot_size);
void displayAnalogClock(bool edit_minute, bool edit_hour);
void displayDigitalClock(bool edit_minute, bool edit_hour);

// Gear Indicator Data Struct
typedef struct shift_data {
  int hall_1;
  int hall_2;
  int hall_3;
  int hall_4;
  int hall_5;
  int hall_6;
  int gear_position;
} shift_data;

// Create a structured object
shift_data shiftData;


int CURRENT_GEAR = 0;
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&shiftData, incomingData, sizeof(shiftData));
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("HALL 1: ");
  Serial.println(shiftData.hall_1);
  Serial.print("HALL 2: ");
  Serial.println(shiftData.hall_2);
  Serial.print("HALL 3: ");
  Serial.println(shiftData.hall_3);
  Serial.print("HALL 4: ");
  Serial.println(shiftData.hall_4);
  Serial.print("HALL 5: ");
  Serial.println(shiftData.hall_5);
  Serial.print("HALL 6: ");
  Serial.println(shiftData.hall_6);
  Serial.print("Gear Position: ");
  Serial.println(shiftData.gear_position);
  Serial.println();
  CURRENT_GEAR = shiftData.gear_position;
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
  for (int i = 0; i < NUM_POINTS; i++) {
    pointsX[i] = 64 + RADIUS * cos(i * 6.28 / NUM_POINTS);
    pointsY[i] = 32 + RADIUS * sin(i * 6.28 / NUM_POINTS);
  }


  display.clearDisplay();
  display.display();   
}

void loop() {

  // Clock Display Logic
  if (showAnalogClock) {
    displayAnalogClock(EDIT_MINUTE, EDIT_HOUR);
  } else if (showDigitalCLock) {
    displayDigitalClock(EDIT_MINUTE, EDIT_HOUR);
  }

  readButton(debounce_time);

  // Gotta clean this shit up eventually, there is a better way for menu control
  if (CW_STATE & (PUSH_EDIT_STATE == 0)) {
    Serial.println("Turned Clockwise");
    showAnalogClock = !showAnalogClock;
    showDigitalCLock = !showDigitalCLock;
  } else if (CCW_STATE & (PUSH_EDIT_STATE == 0)) {
    Serial.println("Turned Counter Clockwise");
    showAnalogClock = !showAnalogClock;
    showDigitalCLock = !showDigitalCLock;
  } else if ( PUSH_STATE) {
    Serial.println("Pushed Button");
    PUSH_EDIT_STATE += 1;
    if (PUSH_EDIT_STATE > 2) {
      PUSH_EDIT_STATE = 0;
    }
    if ( (PUSH_EDIT_STATE == 1)) {
      EDIT_HOUR = true;
      EDIT_MINUTE = false;
      debounce_time = 600;
      Serial.println("Editing Hour Hand");
    } else if ( (PUSH_EDIT_STATE == 2)) {
      EDIT_MINUTE = true;
      EDIT_HOUR = false;
      debounce_time = 400;
      Serial.println("Editing Minute Hand");
    } else {
      EDIT_HOUR = false;
      EDIT_MINUTE = false;
      debounce_time = 200;
    }
  }

  int currentEpoch = rtc.getLocalEpoch();
  if ((EDIT_HOUR & CW_STATE) & ((millis() - lastDebounceClockEdit) > debounce_time)) {
    rtc.setTime(currentEpoch + 3600);
    lastDebounceClockEdit = millis();
  } else if ((EDIT_MINUTE & CW_STATE) & ((millis() - lastDebounceClockEdit) > debounce_time)) {
    rtc.setTime(currentEpoch + 60);
    lastDebounceClockEdit = millis();
  }



  initiateTime();
  display.display();
  display.clearDisplay();
}


void silviaScreen() {
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
  delay(3000);
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




void displayAnalogClock(bool edit_minute, bool edit_hour) {

  bootCanvas.setTextSize(1);
  bootCanvas.setFont(&RONIX5);
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

  drawAnalogBackground(edit_minute | edit_hour); // draw the background - fullscreen circle, dots for seconds, big tickmarks, numbers

  // draw the needles with angles based on the time value
  if (edit_minute) {
    if (rtc.getMillis() > 500) {
      drawAnalogBoldHand(mins*6, 25, 10, 1); // minute hand 
    }
  } else {
    drawAnalogBoldHand(mins*6, 25, 10, 1); // minute hand 
  }

  if (edit_hour) {
    if (rtc.getMillis() > 500) {
      drawAnalogBoldHand(hrs*30 + (mins / 2), 18, 10, 1); // hour hand
    }
  } else {
    drawAnalogBoldHand(hrs*30 + (mins / 2), 18, 10, 1); // hour hand
  }


  drawAnalogThinHand(secs*6, 27, 22); // second hand

  // draw the center circle to cover the center part of the hands
  display.fillCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 3, WHITE);
  display.fillCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 2, BLACK);

}

void drawAnalogBackground(bool editing) {

  float xpos;
  float ypos;
  float xpos2;
  float ypos2;  

  display.drawCircle(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 28, WHITE); // draw fullscreen circle

  // draw 60 dots (pixels) around the circle, one for every minute/second
  for (int i=0; i<60; i++) { // draw 60 pixels around the circle
    xpos = round(DISPLAY_CENTER_X + sin(radians(i * 6)) * 31); // calculate x pos based on angle and radius
    ypos = round(DISPLAY_CENTER_Y - cos(radians(i * 6)) * 31); // calculate y pos based on angle and radius
    
    display.drawPixel(xpos, ypos, WHITE); // draw white pixel on position xpos and ypos
  }


  // drawing big tickmarks
  for (int i=0; i<12; i++) {
    if((i % 3) == 0) { // only draw tickmarks for some numbers, leave empty space for 12, 3, 6, and 9
      xpos = round(DISPLAY_CENTER_X + sin(radians(i * 90)) * 30); // calculate x pos based on angle and radius
      ypos = round(DISPLAY_CENTER_Y - cos(radians(i * 90)) * 30); // calculate y pos based on angle and radius
      xpos2 = round(DISPLAY_CENTER_X + sin(radians(i * 90)) * 23); // calculate x pos based on angle and radius
      ypos2 = round(DISPLAY_CENTER_Y - cos(radians(i * 90)) * 23); // calculate y pos based on angle and radius      
      display.drawLine(xpos, ypos, xpos2, ypos2, WHITE); // draw a line for a tickmark
    }
  }
  
  
  // Display Numbers
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(80, 28);
  display.print("3");
  display.setCursor(62, 46);
  display.print("6");
  display.setCursor(44, 28);
  display.print("9");
  display.setCursor(60, 12);
  display.print("12");
}
void drawAnalogThinHand(int hand_angle, int hand_length_long, int hand_legth_short) {

  float xpos;
  float ypos;
  float xpos2;
  float ypos2;  

  // calculate starting and ending position of the second hand
  xpos = round(DISPLAY_CENTER_X + sin(radians(hand_angle)) * hand_length_long); // calculate x pos based on angle and radius
  ypos = round(DISPLAY_CENTER_Y - cos(radians(hand_angle)) * hand_length_long); // calculate y pos based on angle and radius
  xpos2 = round(DISPLAY_CENTER_X + sin(radians(hand_angle + 180)) * hand_legth_short); // calculate x pos based on angle and radius
  ypos2 = round(DISPLAY_CENTER_Y - cos(radians(hand_angle + 180)) * hand_legth_short); // calculate y pos based on angle and radius  

  display.drawLine(xpos, ypos, xpos2, ypos2, WHITE); // draw the main line
  display.fillCircle(xpos2, ypos2, 3, WHITE); // draw small outline white circle
  display.fillCircle(xpos2, ypos2, 2, BLACK); // draw small filled black circle

}


// draw bold hand = minute hand and hour hand
void drawAnalogBoldHand(int hand_angle, int hand_length_long, int hand_legth_short, int hand_dot_size) {

  float xpos;
  float ypos;
  float xpos2;
  float ypos2;  

  float tri_xoff;
  float tri_yoff;  

  // calculate positions of the two circles
  xpos = round(DISPLAY_CENTER_X + sin(radians(hand_angle)) * hand_length_long); // calculate x pos based on angle and radius
  ypos = round(DISPLAY_CENTER_Y - cos(radians(hand_angle)) * hand_length_long); // calculate y pos based on angle and radius
  xpos2 = round(DISPLAY_CENTER_X + sin(radians(hand_angle)) * hand_legth_short); // calculate x pos based on angle and radius
  ypos2 = round(DISPLAY_CENTER_Y - cos(radians(hand_angle)) * hand_legth_short); // calculate y pos based on angle and radius  

  tri_xoff = round( sin(radians(hand_angle + 90)) * hand_dot_size);
  tri_yoff = round(-cos(radians(hand_angle + 90)) * hand_dot_size);  

  display.drawLine(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, xpos2, ypos2, WHITE); // draw the line from one circle to the center
  display.drawCircle(xpos, ypos, hand_dot_size, WHITE); // draw filled white circle
  display.drawCircle(xpos2, ypos2, hand_dot_size, WHITE); // draw filled white circle


  display.fillTriangle(xpos + tri_xoff, ypos + tri_yoff,
                    xpos - tri_xoff, ypos - tri_yoff,
                    xpos2 + tri_xoff, ypos2 + tri_yoff, WHITE);
  display.fillTriangle(xpos2 + tri_xoff, ypos2 + tri_yoff,
                    xpos2 - tri_xoff, ypos2 - tri_yoff,
                    xpos - tri_xoff, ypos - tri_yoff, WHITE);


}

void displayDigitalClock(bool edit_minute, bool edit_hour) {
  unsigned long millis = rtc.getMillis();
  int clockY = 42;
  digitalClockCanvas.fillScreen(0); 
  digitalClockCanvas.setFont(&RONIX17);
  digitalClockCanvas.setTextSize(1);
  digitalClockCanvas.setTextWrap(false);
  if (edit_hour) {
    if (millis > 500) {
      if (hrs > 9) {
        digitalClockCanvas.setCursor(0, clockY);
        digitalClockCanvas.print(hrs / 10);
      }
      if (hrs % 10 == 1) {
        digitalClockCanvas.setCursor(25, clockY);
        digitalClockCanvas.print(hrs % 10);
      } else if (hrs % 10 == 0) {
        digitalClockCanvas.setCursor(12, clockY);
        digitalClockCanvas.print(hrs % 10);
      } else {
        digitalClockCanvas.setCursor(14, clockY);
        digitalClockCanvas.print(hrs % 10);
      }
    }
  } else {
    if (hrs > 9) {
      digitalClockCanvas.setCursor(0, clockY);
      digitalClockCanvas.print(hrs / 10);
    }
    if (hrs % 10 == 1) {
      digitalClockCanvas.setCursor(25, clockY);
      digitalClockCanvas.print(hrs % 10);
    } else if (hrs % 10 == 0) {
      digitalClockCanvas.setCursor(12, clockY);
      digitalClockCanvas.print(hrs % 10);
    } else {
      digitalClockCanvas.setCursor(14, clockY);
      digitalClockCanvas.print(hrs % 10);
    }

  }

  if (edit_minute) {
    if (millis > 500) {
      if (mins / 10 == 1) {
        digitalClockCanvas.setCursor(68, clockY);
        digitalClockCanvas.print(mins / 10);
      } else if (mins / 10 == 0) {
        digitalClockCanvas.setCursor(55, clockY);
        digitalClockCanvas.print(mins / 10);
      } else {
        digitalClockCanvas.setCursor(56, clockY);
        digitalClockCanvas.print(mins / 10);
      }
      if (mins % 10 == 1) {
        digitalClockCanvas.setCursor(93, clockY);
        digitalClockCanvas.print(mins % 10);
      } else {
        digitalClockCanvas.setCursor(90, clockY);
        digitalClockCanvas.print(mins % 10);
      }
    }
  } else {
    if (mins / 10 == 1) {
        digitalClockCanvas.setCursor(68, clockY);
        digitalClockCanvas.print(mins / 10);
      } else if (mins / 10 == 0) {
        digitalClockCanvas.setCursor(55, clockY);
        digitalClockCanvas.print(mins / 10);
      } else {
        digitalClockCanvas.setCursor(56, clockY);
        digitalClockCanvas.print(mins / 10);
      }
      if (mins % 10 == 1) {
        digitalClockCanvas.setCursor(93, clockY);
        digitalClockCanvas.print(mins % 10);
      } else {
        digitalClockCanvas.setCursor(90, clockY);
        digitalClockCanvas.print(mins % 10);
      }
  }

  int colon_x = 52;

  if ((hrs % 10 == 1) & (mins / 10 != 1)) {
    colon_x = 50;
  } else if (mins / 10 == 1) {
    colon_x = 54;
  }

  // only show the colon in between every two secs
  int colon_y1 = 26;
  int colon_y2 = 36;
  if (edit_hour | edit_minute) {
    digitalClockCanvas.fillRect(colon_x, colon_y1, 4, 4, WHITE); // draw filled rectangle
    digitalClockCanvas.fillRect(colon_x, colon_y2, 4, 4, WHITE); // draw filled rectangle
  } else {
    if (secs % 2 == 0) { // the result will be 0 or 1 depending if the value is even or odd
      digitalClockCanvas.fillRect(colon_x, colon_y1, 4, 4, WHITE); // draw filled rectangle
      digitalClockCanvas.fillRect(colon_x, colon_y2, 4, 4, WHITE); // draw filled rectangle
  }
  }

  display.drawBitmap(0, 0, digitalClockCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, BLACK);
}

void initiateTime() {
  hrs=rtc.getHour();
  mins=rtc.getMinute();
  secs=rtc.getSecond();
}

void readButton(unsigned long debounce_time=debounce_time) {
  CWFilter.Filter(analogRead(CW_PIN));
  CCWFilter.Filter(analogRead(CCW_PIN));
  PUSHFilter.Filter(analogRead(PUSH_PIN));

  CW_VAL = CWFilter.Current();
  CCW_VAL = CCWFilter.Current();
  PUSH_VAL = PUSHFilter.Current();

  if (PUSH_VAL == 4095 & ((millis() - lastDebounceTime1) > debounce_time)) {
    PUSH_STATE = true;
    lastDebounceTime1 = millis();
  } else if (CCW_VAL == 4095 & ((millis() - lastDebounceTime1) > debounce_time)) {
    CCW_STATE = true;
    lastDebounceTime1 = millis();
  } else if (CW_VAL == 4095 & ((millis() - lastDebounceTime1) > debounce_time)){
    CW_STATE = true;
    lastDebounceTime1 = millis();
  } else {
    PUSH_STATE = false;
    CCW_STATE = false;
    CW_STATE = false;
  }

}


