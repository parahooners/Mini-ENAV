#include <GxEPD.h>
#include <GxDEPG0150BN/GxDEPG0150BN.h>    // 1.54" b/w 200x200
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_pm.h"

// Correct pin definitions for LilyGO E-Paper Watch
#define GPS_RX 21
#define GPS_TX 22
#define GPS_BAUD 9600

#define PIN_MOTOR 4
#define PIN_KEY 35
#define PWR_EN 5
#define Backlight 33
#define Bat_ADC 34

#define SPI_SCK 14
#define SPI_DIN 13
#define EPD_CS 15
#define EPD_DC 2
#define SRAM_CS -1
#define EPD_RESET 17
#define EPD_BUSY 16

// Display constants
#define SCREEN_WIDTH  200
#define SCREEN_HEIGHT 200
#define CENTER_X      100
#define CENTER_Y      100
#define OUTER_RADIUS  89   // Reduced by 10px
#define INNER_RADIUS  55   // Reduced by 5px
#define MAX_DISTANCE  30   // km - when icon reaches outer position

// Time constants - Optimized for faster updates
#define UPDATE_INTERVAL 100    // 0.1 seconds instead of 0.5 seconds
#define SLEEP_TIMEOUT 600000   // 10 minutes in ms
#define GPS_TIMEOUT 5000       // 5 seconds timeout for GPS data

// EEPROM addresses
#define EEPROM_SIZE 16
#define HOME_LAT_ADDR 0
#define HOME_LON_ADDR 8

// Change detection thresholds
#define SPEED_CHANGE_THRESHOLD 1.0     // km/h
#define ALT_CHANGE_THRESHOLD 5.0      // feet
#define DISTANCE_CHANGE_THRESHOLD 0.1  // km
#define HEADING_CHANGE_THRESHOLD 5.0   // degrees

// Constants for battery calculation
#define ADC_RESOLUTION 4095.0
#define ADC_REFERENCE 3.3
#define BAT_VOLTAGE_DIVIDER 2.0
#define BAT_MIN_VOLTAGE 3.0
#define BAT_MAX_VOLTAGE 4.2

// Global variables
GxIO_Class io(SPI, /*CS*/ EPD_CS, /*DC=*/EPD_DC, /*RST=*/EPD_RESET);
GxEPD_Class display(io, /*RST=*/EPD_RESET, /*BUSY=*/EPD_BUSY);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

// Home location
double homeLat = 0.0;
double homeLon = 0.0;
bool homeSet = false;

// Current data
double currentLat = 0.0;
double currentLon = 0.0;
double currentAlt = 0.0;
double currentSpeed = 0.0;
int satellites = 0;
float battery = 0.0;
double distanceToHome = 0.0;
double courseToHome = 0.0;
double currentCourse = 0.0;

// Previous values for change detection
double prevSpeed = -1.0;
double prevAlt = -1.0;
double prevDistance = -1.0;
double prevCourse = -1.0;
int prevSatellites = -1;
float prevBattery = -1.0;

// State variables
unsigned long lastUpdateTime = 0;
unsigned long lastMovementTime = 0;
unsigned long lastGPSTime = 0;
bool isMoving = false;
bool needsFullRedraw = true;
bool inDeepSleep = false;
bool waitingForGPS = true;
bool waitingForTakeoff = false;

// Previous positions for selective redraw
int prevIconX = -1;
int prevIconY = -1;
int prevTriangleX = -1;
int prevTriangleY = -1;

// Global variable for rotating dot
int rotatingDotAngle = 0;

// Function prototypes
void setCustomCpuFrequencyMhz(uint32_t frequency);
void updateBatteryLevel();
void drawBackground();
void drawRingWithGaps(int radius);
void drawArcSegment(int radius, int angle);
void updateGPSData();
void updateCenterDisplay();
void updateDirectionIndicators();
void setNewHomePoint();
void prepareForSleep();
void updateTextArea(int x, int y, int w, int h, char* text, int textX, int textY);
void drawRotatingDot();
int getBatteryPercent();

void setup() {
  Serial.begin(115200);
  
  // Debug: Indicate setup start
  Serial.println("Setup started...");
  
  // Overclock the ESP32 for better performance
  setCustomCpuFrequencyMhz(240); // Default is 160MHz
  
  // Initialize SPI for the display with the correct pins
  SPI.begin(SPI_SCK, -1, SPI_DIN, EPD_CS);
  
  // Initialize GPS with correct pins
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Set GPS to higher update rate (if supported)
  delay(100);
 // GPSSerial.println("$PMTK220,200*2C"); // Set update rate to 5Hz (200ms)

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize display with optimized settings
  display.init(0); // false = partial updates possible
  display.setRotation(0);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  
  // Clear the display at startup
  display.fillRect(0, 0, 200, 200, GxEPD_WHITE); // Draw a 200x200 white box
  display.updateWindow(0, 0, 200, 200); // Partial update for the entire screen
  
  // Debug: Indicate display initialization
  Serial.println("Display initialized...");
  
  // Enable power to peripherals
  pinMode(PWR_EN, OUTPUT);
  digitalWrite(PWR_EN, HIGH);

  
  
  // Initialize button and motor with correct pins
  pinMode(PIN_KEY, INPUT_PULLUP);
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);
  
  // Load home position from EEPROM
  EEPROM.get(HOME_LAT_ADDR, homeLat);
  EEPROM.get(HOME_LON_ADDR, homeLon);
  
  if (homeLat != 0.0 && homeLon != 0.0) {
    homeSet = true;
  }
  
  // Initial full screen draw
  display.fillScreen(GxEPD_WHITE);
  drawBackground();
  display.updateWindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); // Full update only once

  // Debug: Indicate setup completion
  Serial.println("Setup completed...");
  
  // Draw initial center display
  updateCenterDisplay(); // Ensure the center circle is drawn initially
  display.updateWindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); // Full update for initial draw

  lastUpdateTime = millis();
  lastMovementTime = millis();
  lastGPSTime = millis();
}

void loop() {
  // Debug: Indicate loop start
  Serial.println("Loop started...");

  // Continuously process GPS data
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);

    // Debug: Print raw GPS data
    Serial.print(c);

    // Update GPS time if available
    if (gps.time.isUpdated() && gps.time.isValid()) {
      Serial.print("GPS Time: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
    }
  }

  // Handle rotating dot while waiting for GPS
  if (waitingForGPS) {
    drawRotatingDot();
    delay(100); // Control the speed of rotation
    return; // Skip the rest of the loop while waiting for GPS
  }

  // Update ALT, KM, BAT, and SAT values
  display.fillScreen(GxEPD_WHITE); // Clear the screen
  drawBackground(); // Redraw the background

  // Update ALT
  if (gps.altitude.isValid()) {
    char buffer[10];
    dtostrf(gps.altitude.meters() * 3.28084, 4, 0, buffer); // Convert to feet
    display.setCursor(SCREEN_WIDTH - 40, 30);
    display.print(buffer);
    Serial.print("Altitude (ft): ");
    Serial.println(buffer);
  } else {
    display.setCursor(SCREEN_WIDTH - 40, 30);
    display.print("---");
    Serial.println("Altitude: ---");
  }

  // Update KM (distance to home)
  if (homeSet && gps.location.isValid()) {
    char buffer[10];
    dtostrf(distanceToHome, 4, 1, buffer);
    display.setCursor(0, 30);
    display.print(buffer);
    Serial.print("Distance to Home (km): ");
    Serial.println(buffer);
  } else {
    display.setCursor(0, 30);
    display.print("---");
    Serial.println("Distance to Home: ---");
  }

  // Update BAT
  char buffer[10];
  int batteryPercent = getBatteryPercent();
  sprintf(buffer, "%d", batteryPercent);
  display.setCursor(0, 177);
  display.print(buffer);
  Serial.print("Battery (%): ");
  Serial.println(buffer);

  // Update SAT
  if (gps.satellites.isValid() && gps.satellites.value() > 0) {
    sprintf(buffer, "%d", gps.satellites.value());
    display.setCursor(SCREEN_WIDTH - 20, 177);
    display.print(buffer);
    Serial.print("Satellites: ");
    Serial.println(buffer);
  } else {
    display.setCursor(SCREEN_WIDTH - 20, 177);
    display.print("---");
    Serial.println("Satellites: ---");
  }

  // Perform a partial update for the entire screen
  display.updateWindow(0, 0, 200, 200);

  // Debug: Indicate loop end
  Serial.println("Loop completed...");
}

void updateGPSData() {
  bool dataChanged = false;
  
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    waitingForGPS = false;
    
    if (gps.altitude.isValid()) {
      currentAlt = gps.altitude.meters() * 3.28084; // Convert to feet
      // Check for significant altitude change
      if (fabs(currentAlt - prevAlt) > ALT_CHANGE_THRESHOLD) {
        char buffer[10];
        dtostrf(currentAlt, 4, 0, buffer);
        updateTextArea(SCREEN_WIDTH - 70, 5, 40, 20, buffer, SCREEN_WIDTH - 70, 20);
        prevAlt = currentAlt;
        dataChanged = true;
      }
    }
    
    if (gps.speed.isValid()) {
      currentSpeed = gps.speed.kmph();
      // Check for significant speed change
      if (fabs(currentSpeed - prevSpeed) > SPEED_CHANGE_THRESHOLD) {
        char buffer[10];
        dtostrf(currentSpeed, 3, 0, buffer);
        updateTextArea(45, 5, 40, 20, buffer, 45, 20);
        prevSpeed = currentSpeed;
        dataChanged = true;
      }
    }
    
    if (gps.course.isValid()) {
      currentCourse = gps.course.deg();
    }
    
    int newSatellites = gps.satellites.value();
    if (newSatellites != prevSatellites) {
      char buffer[5];
      sprintf(buffer, "%d", newSatellites);
      updateTextArea(SCREEN_WIDTH - 70, SCREEN_HEIGHT - 25, 40, 20, buffer, SCREEN_WIDTH - 70, SCREEN_HEIGHT - 10);
      prevSatellites = newSatellites;
      satellites = newSatellites;
      dataChanged = true;
    }
    
    // Calculate distance and course to home if home is set
    if (homeSet) {
      double newDistance = TinyGPSPlus::distanceBetween(
        currentLat, currentLon, homeLat, homeLon) / 1000.0; // Convert to km
        
      double newCourse = TinyGPSPlus::courseTo(
        currentLat, currentLon, homeLat, homeLon);
      
      // Check for significant distance change
      if (fabs(newDistance - prevDistance) > DISTANCE_CHANGE_THRESHOLD) {
        distanceToHome = newDistance;
        prevDistance = newDistance;
        updateCenterDisplay();
        dataChanged = true;
      }
      
      // Check for significant heading change
      if (fabs(newCourse - prevCourse) > HEADING_CHANGE_THRESHOLD || 
          fabs(currentCourse - prevCourse) > HEADING_CHANGE_THRESHOLD) {
        courseToHome = newCourse;
        prevCourse = currentCourse;
        
        // Update direction indicators if we have sufficient satellites
        if (satellites >= 3 && !waitingForGPS && !waitingForTakeoff) {
          updateDirectionIndicators();
        }
        dataChanged = true;
      }
    }
    
    // Determine if we're waiting for takeoff
    if (!waitingForTakeoff && satellites >= 3 && !isMoving) {
      waitingForTakeoff = true;
      updateCenterDisplay();
    } else if (waitingForTakeoff && isMoving) {
      waitingForTakeoff = false;
      updateCenterDisplay();
    }
  }
}

void drawBackground() {
  // Debug: Indicate background drawing
  Serial.println("Drawing background...");

  display.fillScreen(GxEPD_WHITE);

  // Draw outer circle (3px thick)
  for (int i = 0; i < 3; i++) {
    display.drawCircle(CENTER_X, CENTER_Y, OUTER_RADIUS - i, GxEPD_BLACK);
  }

  // Draw inner circle (3px thick)
  for (int i = 0; i < 3; i++) {
    display.drawCircle(CENTER_X, CENTER_Y, INNER_RADIUS - i, GxEPD_BLACK);
  }

  // Draw mid and two-thirds rings with gaps
  int midRadius = INNER_RADIUS + (OUTER_RADIUS - INNER_RADIUS) / 3;
  int twoThirdsRadius = INNER_RADIUS + 2 * (OUTER_RADIUS - INNER_RADIUS) / 3;

  drawRingWithGaps(midRadius);
  drawRingWithGaps(twoThirdsRadius);

  // Add labels
  display.setFont(&FreeMonoBold9pt7b);

  // KM label (top left)
  display.setCursor(0, 10); // Move to line 10
  display.print("KM");
  display.setCursor(0, 30); // Number below the label
  display.print("10");

  // ALT label (top right)
  display.setCursor(SCREEN_WIDTH - 40, 10); // Move right by 10px
  display.print("ALT");
  display.setCursor(SCREEN_WIDTH - 40, 30); // Number below the label, moved right by 10px
  display.print("100");

  // BAT label (bottom left)
  display.setCursor(0, 197); // Move up by 3px
  display.print("BAT");
  display.setCursor(0, 177); // Number above the label, moved up by 3px
  display.print("75");

  // SAT label (bottom right)
  display.setCursor(SCREEN_WIDTH - 45, 197); // Move right by 5px and up by 3px
  display.print("SAT");
  display.setCursor(SCREEN_WIDTH - 20, 177); // Number above the label, moved right by 5px and up by 3px
  display.print("5");

  // Debug: Indicate background drawing completion
  Serial.println("Background drawn.");
}

void drawRingWithGaps(int radius) {
  // Draw arcs with gaps at the top, bottom, left, and right
  for (int i = 0; i < 3; i++) { // Make the ring 3px thick
    for (int angle = 30; angle <= 60; angle++) {
      drawArcSegment(radius - i, angle);
    }
    for (int angle = 120; angle <= 150; angle++) {
      drawArcSegment(radius - i, angle);
    }
    for (int angle = 210; angle <= 240; angle++) {
      drawArcSegment(radius - i, angle);
    }
    for (int angle = 300; angle <= 330; angle++) {
      drawArcSegment(radius - i, angle);
    }
  }
}

void drawArcSegment(int radius, int angle) {
  float radians = angle * PI / 180.0;
  int x = CENTER_X + int(radius * cos(radians));
  int y = CENTER_Y - int(radius * sin(radians));
  display.drawPixel(x, y, GxEPD_BLACK);
}

void updateTextArea(int x, int y, int w, int h, char* text, int textX, int textY) {
  display.updateWindow(x, y, w, h, true);
  display.fillRect(x, y, w, h, GxEPD_WHITE);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(textX, textY);
  display.print(text);
}

void updateCenterDisplay() {
  // Debug: Indicate center display update
  Serial.println("Updating center display...");

  // Clear the center circle
  display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS - 1, GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  String centerText;

  // Determine the text to display in the center circle
  if (waitingForGPS) {
    centerText = "Wait GPS";
    display.setFont(&FreeMonoBold9pt7b);
  } else if (waitingForTakeoff && satellites >= 3) {
    centerText = "Wait T/O";
    display.setFont(&FreeMonoBold9pt7b);
  } else if (homeSet) {
    // Format distance with 1 decimal place
    char buffer[10];
    dtostrf(distanceToHome, 4, 1, buffer);
    centerText = String(buffer);
    display.setFont(&FreeSansBold12pt7b); // Use a compact font for distance
  } else {
    centerText = "No Home";
    display.setFont(&FreeMonoBold9pt7b);
  }

  // Center the text within the circle
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(centerText, 0, 0, &tbx, &tby, &tbw, &tbh);
  display.setCursor(CENTER_X - tbw / 2, CENTER_Y + tbh / 2);
  display.print(centerText);

  // Debug: Print the text being displayed
  Serial.print("Center text: ");
  Serial.println(centerText);

  // Perform a full update for the center display
  display.updateWindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
}

void updateDirectionIndicators() {
  // Calculate relative heading based on current course
  float relativeBearing = courseToHome - currentCourse;
  if (relativeBearing < 0) relativeBearing += 360;
  if (relativeBearing >= 360) relativeBearing -= 360;

  float radians = relativeBearing * PI / 180.0;

  // Calculate position for triangle on inner circle
  int triangleX = CENTER_X + int((INNER_RADIUS - 5) * sin(radians));
  int triangleY = CENTER_Y - int((INNER_RADIUS - 5) * cos(radians));

  // Only redraw triangle if position changed significantly
  if (abs(triangleX - prevTriangleX) > 3 || abs(triangleY - prevTriangleY) > 3) {
    // Clear previous triangle area if it was drawn before
    if (prevTriangleX != -1) {
      display.updateWindow(CENTER_X - INNER_RADIUS - 10, CENTER_Y - INNER_RADIUS - 10, 
                           2 * (INNER_RADIUS + 10), 2 * (INNER_RADIUS + 10), true);
      display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS + 5, GxEPD_WHITE);
      display.drawCircle(CENTER_X, CENTER_Y, INNER_RADIUS, GxEPD_BLACK);

      // Redraw inner circle content
      updateCenterDisplay();
    }

    // Draw new triangle
    int triangleSize = 6;
    display.fillTriangle(
      triangleX, triangleY,
      triangleX - triangleSize * sin(radians + PI/2), triangleY + triangleSize * cos(radians + PI/2),
      triangleX - triangleSize * sin(radians - PI/2), triangleY + triangleSize * cos(radians - PI/2),
      GxEPD_BLACK
    );

    prevTriangleX = triangleX;
    prevTriangleY = triangleY;
  }

  // Calculate position for direction icon based on distance
  // At 0km -> 40px from center, at MAX_DISTANCE km -> 90px from center
  float distanceRatio = min(distanceToHome / MAX_DISTANCE, 1.0);
  int iconRadius = 40 + int(distanceRatio * 50); // 40px to 90px

  // Position the icon
  int iconX = CENTER_X + int(iconRadius * sin(radians));
  int iconY = CENTER_Y - int(iconRadius * cos(radians));

  // Only redraw icon if position changed significantly
  if (abs(iconX - prevIconX) > 3 || abs(iconY - prevIconY) > 3) {
    // Clear previous icon if it was drawn before
    if (prevIconX != -1) {
      display.updateWindow(prevIconX - 11, prevIconY - 11, 22, 22, true); // Updated size to 22px
      display.fillRect(prevIconX - 11, prevIconY - 11, 22, 22, GxEPD_WHITE);
    }

    // Draw new icon
    display.updateWindow(iconX - 11, iconY - 11, 22, 22, true); // Updated size to 22px
    display.fillCircle(iconX, iconY, 11, GxEPD_BLACK); // Updated radius to 11px

    prevIconX = iconX;
    prevIconY = iconY;
  }
}

void setNewHomePoint() {
  if (gps.location.isValid()) {
    // Vibrate to indicate button press using the correct PIN_MOTOR
    digitalWrite(PIN_MOTOR, HIGH);
    delay(200);
    digitalWrite(PIN_MOTOR, LOW);
    
    // Save new home point
    homeLat = currentLat;
    homeLon = currentLon;
    homeSet = true;
    
    // Save to EEPROM
    EEPROM.put(HOME_LAT_ADDR, homeLat);
    EEPROM.put(HOME_LON_ADDR, homeLon);
    EEPROM.commit();
    
    // Update display to show confirmation
    display.updateWindow(CENTER_X - INNER_RADIUS, CENTER_Y - INNER_RADIUS, 
                         2 * INNER_RADIUS, 2 * INNER_RADIUS, true);
    display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS - 1, GxEPD_WHITE);
    
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    
    String message = "New home";
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(message, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(CENTER_X - tbw/2, CENTER_Y - 5);
    display.print(message);
    
    message = "set";
    display.getTextBounds(message, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(CENTER_X - tbw/2, CENTER_Y + 15);
    display.print(message);
    
    display.updateWindow(0, 0, 200, 200); // Full update for new home point

    delay(2000); // Show message for 2 seconds
    
    // Reset distance and course calculations with new home
    distanceToHome = 0;
    updateCenterDisplay();
  }
}

void updateBatteryLevel() {
  // LilyGO battery reading using the correct Bat_ADC pin
  int batteryPercent = getBatteryPercent();
  
  // Only update display if battery level changed significantly
  if (fabs(batteryPercent - prevBattery) > 5.0 || prevBattery < 0) {
    char buffer[5];
    sprintf(buffer, "%d", batteryPercent);
    updateTextArea(45, SCREEN_HEIGHT - 25, 40, 20, buffer, 45, SCREEN_HEIGHT - 10);
    prevBattery = batteryPercent;
  }
  
  battery = batteryPercent;
}

void prepareForSleep() {
  // Display sleep message
  display.updateWindow(CENTER_X - INNER_RADIUS, CENTER_Y - INNER_RADIUS, 
                       2 * INNER_RADIUS, 2 * INNER_RADIUS, true);
  display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS - 1, GxEPD_WHITE);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);
  
  String message = "SLEEP";
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(message, 0, 0, &tbx, &tby, &tbw, &tbh);
  display.setCursor(CENTER_X - tbw/2, CENTER_Y + tbh/2);
  display.print(message);
  
  display.updateWindow(0, 0, 200, 200); // Full update before sleep

  delay(2000); // Show sleep message for 2 seconds
  
  // Power down peripherals to save battery
  digitalWrite(PWR_EN, LOW);
  digitalWrite(Backlight, LOW);
 
  
  // Enter deep sleep
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_KEY, 0); // Wake on button press
  esp_deep_sleep_start();
}

// Helper functions
void setCustomCpuFrequencyMhz(uint32_t frequency) {
  // Configure ESP32 CPU frequency
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = frequency;
  pm_config.min_freq_mhz = 80;  // Minimum frequency when idle
  pm_config.light_sleep_enable = false;
  esp_pm_configure(&pm_config);
}

void drawRotatingDot() {
  // Calculate the position of the rotating dot
  float radians = rotatingDotAngle * PI / 180.0;
  int dotX = CENTER_X + int((INNER_RADIUS + 15) * cos(radians)); // Moved 5px further out
  int dotY = CENTER_Y - int((INNER_RADIUS + 15) * sin(radians));

  // Redraw the entire background
  drawBackground();

  // Redraw the mid and two-thirds rings with gaps (3px thick)
  int midRadius = INNER_RADIUS + (OUTER_RADIUS - INNER_RADIUS) / 3;
  int twoThirdsRadius = INNER_RADIUS + 2 * (OUTER_RADIUS - INNER_RADIUS) / 3;
  drawRingWithGaps(midRadius);
  drawRingWithGaps(twoThirdsRadius);

  // Draw the "Wait GPS" text in the center
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  String centerText = "Wait GPS";
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(centerText, 0, 0, &tbx, &tby, &tbw, &tbh);
  display.setCursor(CENTER_X - tbw / 2, CENTER_Y + tbh / 2);
  display.print(centerText);

  // Clear and update ALT (altitude)
  display.fillRect(SCREEN_WIDTH - 40, 20, 40, 20, GxEPD_WHITE); // Clear old value
  char buffer[10];
  if (gps.altitude.isValid()) {
    dtostrf(gps.altitude.meters() * 3.28084, 4, 0, buffer); // Convert to feet
    display.setCursor(SCREEN_WIDTH - 40, 30);
    display.print(buffer);
  } else {
    display.setCursor(SCREEN_WIDTH - 40, 30);
    display.print("---");
  }

  // Clear and update KM (distance to home)
  display.fillRect(0, 20, 40, 20, GxEPD_WHITE); // Clear old value
  if (homeSet && gps.location.isValid()) {
    dtostrf(distanceToHome, 4, 1, buffer);
    display.setCursor(0, 30);
    display.print(buffer);
  } else {
    display.setCursor(0, 30);
    display.print("---");
  }

  // Clear and update BAT (battery level)
  display.fillRect(0, 167, 40, 20, GxEPD_WHITE); // Clear old value
  int batteryPercent = getBatteryPercent();
  sprintf(buffer, "%d", batteryPercent);
  display.setCursor(0, 177);
  display.print(buffer);

  // Clear and update SAT (satellite count)
  display.fillRect(SCREEN_WIDTH - 40, 167, 40, 20, GxEPD_WHITE); // Clear old value
  if (gps.satellites.isValid() && gps.satellites.value() > 0) {
    sprintf(buffer, "%d", gps.satellites.value());
    display.setCursor(SCREEN_WIDTH - 20, 177);
    display.print(buffer);
  } else {
    display.setCursor(SCREEN_WIDTH - 20, 177);
    display.print("---");
  }

  // Draw the new dot
  display.fillCircle(dotX, dotY, 11, GxEPD_BLACK); // Dot size is 22px diameter (11px radius)

  // Perform a partial update for the entire screen
  display.updateWindow(0, 0, 200, 200);

  // Increment the angle for the next frame
  rotatingDotAngle = (rotatingDotAngle + 5) % 360;
}

int getBatteryPercent() {
    int adcValue = analogRead(Bat_ADC);
    float voltage = (adcValue / ADC_RESOLUTION) * ADC_REFERENCE * BAT_VOLTAGE_DIVIDER;
    int percent = (int)((voltage - BAT_MIN_VOLTAGE) / (BAT_MAX_VOLTAGE - BAT_MIN_VOLTAGE) * 100);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}
