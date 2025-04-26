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
#include "tahoma20pt7b.h" // Include the new font file
#include "tahoma15pt7b.h" // Include the 15pt font file
#include "tahoma10pt7b.h" // Include the 10pt font file


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
#define CENTER_Y      90   // Shifted up 5 more px (was 95)
#define OUTER_RADIUS  89   // Reduced by 10px
#define INNER_RADIUS  55   // Reduced by 5px
#define MAX_DISTANCE  30   // km - when icon reaches outer position

// Time constants - Optimized for faster updates
#define UPDATE_INTERVAL 500    // Update every 0.5 seconds (500 ms)
#define SLEEP_TIMEOUT 600000   // 10 minutes in ms
#define GPS_TIMEOUT 5000       // 5 seconds timeout for GPS data

// EEPROM addresses
#define EEPROM_SIZE 16
#define HOME_LAT_ADDR 0
#define HOME_LON_ADDR 8
#define FUEL_LITRES_ADDR 12 // EEPROM address for fuelLitres (after homeLat/homeLon, so address 12)

// Change detection thresholds
#define SPEED_CHANGE_THRESHOLD 1.0     // km/h
#define ALT_CHANGE_THRESHOLD 5.0      // feet
#define DISTANCE_CHANGE_THRESHOLD 0.001 // km (Reduced to 1 meter sensitivity)
#define HEADING_CHANGE_THRESHOLD 5.0   // degrees

// Constants for battery calculation
#define ADC_RESOLUTION 4095.0
#define ADC_REFERENCE 3.3
#define BAT_VOLTAGE_DIVIDER 2.0
#define BAT_MIN_VOLTAGE 3.0
#define BAT_MAX_VOLTAGE 3.9 // Adjusted from 3.7 to 3.9 for full charge

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

// --- New Takeoff Point Variables ---
double takeoffLat = 0.0;
double takeoffLon = 0.0;
bool takeoffSet = false;
double distanceToTakeoff = 0.0;
double courseToTakeoff = 0.0;
bool initialFixProcessed = false; // Flag to process takeoff point only once
// --- End New Takeoff Point Variables ---

// Fuel variables
float fuelLitres = 12.0;
float fuelBurnRate = 4.8; // litres per hour
unsigned long lastFuelUpdate = 0;
const float FUEL_MIN = 5.0;
const float FUEL_MAX = 20.0;

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
unsigned long startTime = 0; // Track startup time
bool isMoving = false;
bool needsFullRedraw = true;
bool inDeepSleep = false;
bool waitingForGPS = true;
bool waitingForTakeoff = false;

// Previous positions for selective redraw
int prevIconX = -1;
int prevIconY = -1;

// Global variable for rotating dot
int rotatingDotAngle = 0;

// Add debounce variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce delay
bool lastButtonState = HIGH; // Assume button is not pressed initially

// Function prototypes
void setCustomCpuFrequencyMhz(uint32_t frequency);
void updateBatteryLevel();
void drawBackground();
void drawRingWithGaps(int radius);
void drawArcSegment(int radius, int angle);
void updateGPSData();
void updateCenterDisplay();
void updateNavigationIndicators();
void setNewHomePoint();
void prepareForSleep();
void updateTextArea(int x, int y, int w, int h, char* text, int textX, int textY);
void drawRotatingDot();
int getBatteryPercent();
void drawBatteryIcon(int x, int y, int width, int height, int percentage);
void drawSatelliteIcon(int x, int y, int size);
void drawJerryCan(int x, int y, int width, int height);
void enterSettingsScreen();

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
  digitalWrite(PIN_MOTOR, LOW); // Motor is explicitly set to LOW here
  
  // --- Add startup vibration ---
  Serial.println("Vibrating motor on startup...");
  digitalWrite(PIN_MOTOR, HIGH);
  delay(150); // Vibrate for 150ms
  digitalWrite(PIN_MOTOR, LOW);
  Serial.println("Startup vibration complete.");
  // --- End startup vibration ---

  // Load home position from EEPROM
  EEPROM.get(HOME_LAT_ADDR, homeLat);
  EEPROM.get(HOME_LON_ADDR, homeLon);
  
  if (homeLat != 0.0 && homeLon != 0.0) {
    homeSet = true;
  }

  // Load fuelLitres from EEPROM
  EEPROM.get(FUEL_LITRES_ADDR, fuelLitres);
  if (fuelLitres < FUEL_MIN || fuelLitres > FUEL_MAX) fuelLitres = FUEL_MAX; // Default if not set
  
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
  startTime = millis(); // Record startup time
}

void loop() {
  // Debug: Indicate loop start
  Serial.println("Loop started...");

  // Check if within 10 seconds of startup
  if (millis() - startTime <= 10000) {
      bool buttonDown = (digitalRead(PIN_KEY) == LOW);

      if (buttonDown) {
          // Transition to settings screen
          display.fillRect(0, 0, 200, 200, GxEPD_WHITE); // Draw a 200x200 white box
          display.updateWindow(0, 0, 200, 200);         // Full update
          enterSettingsScreen();
      }
  }

  // Continuously process GPS data
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    if (gps.encode(c)) { // Process new GPS sentence
        updateGPSData();
    }

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

  // Handle display based on the current state
  if (waitingForGPS) {
      drawRotatingDot();
      delay(60); // Faster refresh for better animation
      return;
  }

  // --- Button Handling Logic (Navigation Page) ---
  if (digitalRead(PIN_KEY) == LOW) {
      delay(50); // Debounce delay
      if (digitalRead(PIN_KEY) == LOW) { // Confirm button is still pressed
          unsigned long buttonPressStartTime = millis();
          bool actionTaken = false;

          Serial.println("Button press detected. Holding...");

          while (digitalRead(PIN_KEY) == LOW) {
              // Check for long press (>= 5 seconds)
              if (!actionTaken && (millis() - buttonPressStartTime >= 5000)) {
                  Serial.println("Long press detected (>= 5s). Preparing for sleep...");
                  prepareForSleep(); // Call the sleep function <--- THIS IS THE LONG PRESS ACTION
                  actionTaken = true;
                  break;
              }
              delay(20);
          }

          // Button was released
          Serial.println("Button released.");

          // If no action was taken (i.e., it wasn't a long press), treat as short press
          if (!actionTaken) {
              Serial.println("Short press action: Setting new home point.");
              setNewHomePoint(); // <--- THIS IS THE SHORT PRESS ACTION
          }
          delay(200);
      }
  }
  // --- End Button Handling Logic ---

  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      lastUpdateTime = currentTime;

      display.fillScreen(GxEPD_WHITE);
      display.setFont(&FreeMonoBold9pt7b);
      drawBackground();

      // --- Altitude Display (Bottom Center) ---
      display.setFont(&tahoma10pt7b);
      char altBuffer[10];
      if (gps.altitude.isValid()) {
          dtostrf(gps.altitude.meters() * 3.28084, 4, 0, altBuffer);
          Serial.print("Altitude (ft): "); Serial.println(altBuffer);
      } else {
          strcpy(altBuffer, "---");
          Serial.println("Altitude: ---");
      }
      int16_t alt_tbx, alt_tby; uint16_t alt_tbw, alt_tbh;
      display.getTextBounds(altBuffer, 0, 0, &alt_tbx, &alt_tby, &alt_tbw, &alt_tbh);
      int altX = CENTER_X - alt_tbw / 2;
      int altY = 197; // Bottom center - moved down 5px
      display.setCursor(altX, altY);
      display.print(altBuffer);
      // --- End Altitude Display ---

      display.setFont(&FreeMonoBold9pt7b);

      updateCenterDisplay(); // This function sets its own fonts

      if (homeSet || takeoffSet) {
          updateNavigationIndicators();
      }

      display.updateWindow(0, 0, 200, 200); // Partial update for the entire screen
  }

  // --- Fuel burn logic ---
  unsigned long now = millis();
  // Only burn fuel if speed > 5 mph (8.04672 km/h)
  if (currentSpeed > 8.04672) {
    if (lastFuelUpdate == 0) lastFuelUpdate = now;
    float hoursElapsed = (now - lastFuelUpdate) / 3600000.0f;
    if (hoursElapsed > 0) {
      fuelLitres -= fuelBurnRate * hoursElapsed;
      if (fuelLitres < 0) fuelLitres = 0;
      lastFuelUpdate = now;
    }
  } else {
    lastFuelUpdate = now; // Reset timer if not moving
  }

  // Check for sleep timeout - This should run on every loop iteration
  // Ensure updateGPSData() has run to update isMoving and lastMovementTime
  if (!isMoving && millis() - lastMovementTime > SLEEP_TIMEOUT) {
     Serial.println("Inactivity detected. Preparing for sleep...");
     prepareForSleep();
  }
}

void updateGPSData() {
  bool dataChanged = false;
  bool locationValid = gps.location.isValid(); // Check validity once

  if (gps.location.isUpdated() && locationValid) {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    Serial.print("Location Updated: Lat="); Serial.print(currentLat, 6); Serial.print(", Lon="); Serial.println(currentLon, 6);
    dataChanged = true;

    // --- Detect Takeoff Point ---
    if (!initialFixProcessed && homeSet) {
      double initialDistance = TinyGPSPlus::distanceBetween(
        currentLat, currentLon, homeLat, homeLon) / 1000.0; // Distance in KM
      if (initialDistance > 0.5) { // More than 500 meters
        takeoffLat = currentLat;
        takeoffLon = currentLon;
        takeoffSet = true;
        Serial.print("Takeoff point detected: Lat="); Serial.print(takeoffLat, 6);
        Serial.print(", Lon="); Serial.println(takeoffLon, 6);
      }
      initialFixProcessed = true;
    }
    // --- End Detect Takeoff Point ---
  } else if (gps.location.isUpdated() && !locationValid) {
      Serial.println("Location Updated but INVALID.");
  }

  if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
    double newAlt = gps.altitude.meters() * 3.28084;
    if (fabs(newAlt - currentAlt) > ALT_CHANGE_THRESHOLD) {
      currentAlt = newAlt;
      prevAlt = currentAlt;
      Serial.print("Altitude Updated: "); Serial.println(currentAlt);
      dataChanged = true;
    }
  }

  if (gps.speed.isUpdated() && gps.speed.isValid()) {
    double newSpeed = gps.speed.kmph();
    if (fabs(newSpeed - currentSpeed) > SPEED_CHANGE_THRESHOLD) {
      currentSpeed = newSpeed;
      prevSpeed = currentSpeed;
      Serial.print("Speed Updated: "); Serial.println(currentSpeed);
      dataChanged = true;
      if (currentSpeed > SPEED_CHANGE_THRESHOLD) {
          isMoving = true;
          lastMovementTime = millis();
      } else {
          isMoving = false;
      }
    }
  }

  if (gps.course.isUpdated() && gps.course.isValid()) {
     double newCourseGPS = gps.course.deg();
     if (fabs(newCourseGPS - currentCourse) > HEADING_CHANGE_THRESHOLD) {
        currentCourse = newCourseGPS;
        Serial.print("Course Updated: "); Serial.println(currentCourse);
        dataChanged = true;
     }
  }

  if (gps.satellites.isUpdated() && gps.satellites.isValid()) {
    int newSatellites = gps.satellites.value();
    if (newSatellites != satellites) {
        satellites = newSatellites;
        prevSatellites = satellites;
        Serial.print("Satellites Updated: "); Serial.println(satellites);
        dataChanged = true;

        if (satellites >= 4 && waitingForGPS) {
            waitingForGPS = false;
            waitingForTakeoff = true;
            Serial.println("Transitioning from 'Wait GPS' state.");
        }
    }
  }

  // --- Distance and Course to Home Calculation ---
  if (homeSet && locationValid) { // Only calculate if home is set AND current location is valid
    double newDistance = TinyGPSPlus::distanceBetween(
      currentLat, currentLon, homeLat, homeLon) / 1000.0; // Distance in KM

    double newCourseToHome = TinyGPSPlus::courseTo(
      currentLat, currentLon, homeLat, homeLon);

    if (fabs(newDistance - distanceToHome) > DISTANCE_CHANGE_THRESHOLD) {
      distanceToHome = newDistance;
      prevDistance = distanceToHome; // Update previous value only when a change occurs
      Serial.print(">>> Distance to Home Updated: "); Serial.println(distanceToHome, 4); // More precision
      dataChanged = true;
    }

    if (fabs(newCourseToHome - courseToHome) > HEADING_CHANGE_THRESHOLD) {
      courseToHome = newCourseToHome;
      Serial.print(">>> Course to Home Updated: "); Serial.println(courseToHome);
      dataChanged = true;
    }
  }

  // --- Distance and Course to Takeoff Calculation ---
  if (takeoffSet && locationValid) { // Only calculate if takeoff is set AND current location is valid
    double newDistanceToTakeoff = TinyGPSPlus::distanceBetween(
      currentLat, currentLon, takeoffLat, takeoffLon) / 1000.0; // Distance in KM

    double newCourseToTakeoff = TinyGPSPlus::courseTo(
      currentLat, currentLon, takeoffLat, takeoffLon);

    if (fabs(newDistanceToTakeoff - distanceToTakeoff) > DISTANCE_CHANGE_THRESHOLD) {
      distanceToTakeoff = newDistanceToTakeoff;
      Serial.print(">>> Distance to Takeoff Updated: "); Serial.println(distanceToTakeoff, 4); // More precision
    }

    if (fabs(newCourseToTakeoff - courseToTakeoff) > HEADING_CHANGE_THRESHOLD) {
      courseToTakeoff = newCourseToTakeoff;
      Serial.print(">>> Course to Takeoff Updated: "); Serial.println(courseToTakeoff);
    }
  }
}

void updateCenterDisplay() {
  String centerText = "";
  bool useDistanceFont = false; // Flag to indicate which font to use
  bool isMeters = false; // Flag to track if distance is in meters

  if (waitingForGPS) {
    // This state should ideally be handled by drawRotatingDot now
    Serial.println("Warning: updateCenterDisplay called while waitingForGPS is true.");
    centerText = "Wait GPS";
    display.setFont(&FreeMonoBold9pt7b);
  } else { // Not waiting for GPS (includes waitingForTakeoff or navigating)
    if (homeSet) {
      char buffer[10];
      if (distanceToHome < 0.5) { // Less than 500m
          int distMeters = (int)round(distanceToHome * 1000.0);
          sprintf(buffer, "%d", distMeters); // Format as integer meters
          isMeters = true;
          Serial.print("Displaying distance in meters: "); Serial.println(distMeters);
      } else { // 500m or more
          // Format distance in KM with 1 decimal place, increased width to 5
          dtostrf(distanceToHome, 5, 1, buffer); // e.g., 1.2, 12.3, 123.4
          isMeters = false;
          Serial.print("Displaying distance in KM: "); Serial.println(buffer);
      }
      centerText = String(buffer);
      // Use the Tahoma 20pt font for distance/meters
      display.setFont(&tahoma20pt7b);
      useDistanceFont = true;
    } else {
      centerText = "No Home";
      display.setFont(&FreeMonoBold9pt7b);
    }
  }

  if (centerText.length() > 0) {
      int16_t tbx, tby; uint16_t tbw, tbh;
      // Ensure the correct font is set before getting bounds
      if (useDistanceFont) display.setFont(&tahoma20pt7b);
      else display.setFont(&FreeMonoBold9pt7b);
      display.getTextBounds(centerText, 0, 0, &tbx, &tby, &tbw, &tbh);

      int textX;
      // Center vertically based on font height, shift up for distance
      int distY = CENTER_Y + tbh / 2 - 15; // Shifted up 15px

      // If using the distance font, displaying KM (not meters), and the text contains a decimal point, center on the decimal
      int decimalPos = centerText.indexOf('.');
      if (useDistanceFont && !isMeters && decimalPos != -1) {
          String beforeDecimal = centerText.substring(0, decimalPos);
          int16_t btbx, btby; uint16_t btw, bth;
          display.getTextBounds(beforeDecimal, 0, 0, &btbx, &btby, &btw, &bth);
          textX = CENTER_X - btw; // Align decimal point near center
          Serial.print("Centering distance (KM) on decimal. Width before decimal: "); Serial.println(btw);
      } else {
          // Default centering for distance (meters) or other text
          textX = CENTER_X - tbw / 2;
          Serial.print("Centering whole text block. Total width: "); Serial.println(tbw);
      }

      // --- Print Distance (or Meters) ---
      display.setCursor(textX, distY);
      display.setTextColor(GxEPD_BLACK);
      display.print(centerText);
      Serial.print("Distance/Meters text set to: "); Serial.println(centerText);
      Serial.print("Calculated position: X="); Serial.print(textX); Serial.print(", Y="); Serial.println(distY);

      // --- Add Speed Below Distance ---
      if (useDistanceFont) { // Only show speed if distance is shown (i.e., homeSet is true)
          char speedBuffer[10];
          int speedInt = (int)round(gps.speed.kmph()); // Get speed as integer KM/H
          sprintf(speedBuffer, "%d", speedInt);
          String speedText = String(speedBuffer);

          // Use the same font for speed
          display.setFont(&tahoma20pt7b);
          int16_t stbx, stby; uint16_t stbw, stbh;
          display.getTextBounds(speedText, 0, 0, &stbx, &stby, &stbw, &stbh);

          // Center speed horizontally
          int speedX = CENTER_X - stbw / 2;
          // Position speed below distance text (distY is the baseline of distance)
          // Adjusted Y: Add 10 more pixels down (original +5, now +15)
          int speedY = distY + tbh + 15; // Add height of distance text + 15px padding

          display.setCursor(speedX, speedY);
          display.print(speedText);
          Serial.print("Speed text set to: "); Serial.println(speedText);
          Serial.print("Calculated position: X="); Serial.print(speedX); Serial.print(", Y="); Serial.println(speedY);
      }

  } else {
      Serial.println("No center text to display.");
      // Optionally clear the center if needed
      // display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS - 1, GxEPD_WHITE);
  }
}

void drawBackground() {
  Serial.println("Drawing background...");

  for (int i = 0; i < 3; i++) {
    display.drawCircle(CENTER_X, CENTER_Y, OUTER_RADIUS - i, GxEPD_BLACK);
  }

  for (int i = 0; i < 3; i++) {
    display.drawCircle(CENTER_X, CENTER_Y, INNER_RADIUS - i, GxEPD_BLACK);
  }

  int midRadius = INNER_RADIUS + (OUTER_RADIUS - INNER_RADIUS) / 3;
  int twoThirdsRadius = INNER_RADIUS + 2 * (OUTER_RADIUS - INNER_RADIUS) / 3;

  drawRingWithGaps(midRadius);
  drawRingWithGaps(twoThirdsRadius);

  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK); // Ensure color is set

  // Draw battery icon (Top Left)
  drawBatteryIcon(0, 0, 40, 20, getBatteryPercent()); // Corrected size

  // Draw satellite icon (Top Right)
  drawSatelliteIcon(175, 0, 25);

  // Draw jerry can in the bottom-left corner
  drawJerryCan(0, 160, 45, 38);
}

void drawRingWithGaps(int radius) {
  for (int i = 0; i < 3; i++) {
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

void updateNavigationIndicators() {
  // --- Home Indicator ---
  float relativeBearingHome = courseToHome - currentCourse;
  if (relativeBearingHome < 0) relativeBearingHome += 360;
  if (relativeBearingHome >= 360) relativeBearingHome -= 360;

  float radiansHome = relativeBearingHome * PI / 180.0;

  // Calculate icon position - moved out 1px
  int iconRadiusHome = INNER_RADIUS + 16; // Fixed radius (was 15)
  int iconXHome = CENTER_X + int(iconRadiusHome * sin(radiansHome));
  int iconYHome = CENTER_Y - int(iconRadiusHome * cos(radiansHome));
  int dotDrawRadiusHome = 15; // Was 13

  // Draw new icon - made 2px bigger (radius +2)
  display.fillCircle(iconXHome, iconYHome, dotDrawRadiusHome, GxEPD_BLACK); // Use dotDrawRadius

  // --- Add White 'H' inside the dot ---
  display.setFont(&tahoma15pt7b); // Use larger 15pt font
  display.setTextColor(GxEPD_WHITE);   // Set text color to white

  String homeChar = "H";
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(homeChar, 0, 0, &tbx, &tby, &tbw, &tbh);

  // Calculate position to center the 'H'
  int textXHome = iconXHome - tbw / 2 - tbx; // Center horizontally
  int textYHome = iconYHome + tbh / 2 - tby / 2 - 8; // Center vertically (approx) and move up 8px (was 6)

  display.setCursor(textXHome, textYHome);
  display.print(homeChar);

  // --- Takeoff Indicator ---
  if (takeoffSet) {
    float relativeBearingTakeoff = courseToTakeoff - currentCourse;
    if (relativeBearingTakeoff < 0) relativeBearingTakeoff += 360;
    if (relativeBearingTakeoff >= 360) relativeBearingTakeoff -= 360;

    float radiansTakeoff = relativeBearingTakeoff * PI / 180.0;

    int iconRadiusTakeoff = INNER_RADIUS + 16; // Fixed radius (was 15)
    int iconXTakeoff = CENTER_X + int(iconRadiusTakeoff * sin(radiansTakeoff));
    int iconYTakeoff = CENTER_Y - int(iconRadiusTakeoff * cos(radiansTakeoff));
    int dotDrawRadiusTakeoff = 15; // Was 13

    display.fillCircle(iconXTakeoff, iconYTakeoff, dotDrawRadiusTakeoff, GxEPD_BLACK);

    // --- Add White '1' inside the dot ---
    display.setFont(&tahoma15pt7b); // Use larger 15pt font
    display.setTextColor(GxEPD_WHITE);   // Set text color to white

    String takeoffChar = "1";
    int16_t tbxTakeoff, tbyTakeoff; uint16_t tbwTakeoff, tbhTakeoff;
    display.getTextBounds(takeoffChar, 0, 0, &tbxTakeoff, &tbyTakeoff, &tbwTakeoff, &tbhTakeoff);

    int textXTakeoff = iconXTakeoff - tbwTakeoff / 2 - tbxTakeoff; // Center horizontally
    int textYTakeoff = iconYTakeoff + tbhTakeoff / 2 - tbyTakeoff / 2 - 8; // Center vertically (approx) and move up 8px (was 6)

    display.setCursor(textXTakeoff, textYTakeoff);
    display.print(takeoffChar);
  }

  // Reset text color to black for other drawing elements
  display.setTextColor(GxEPD_BLACK);
}

void setNewHomePoint() {
  if (gps.location.isValid()) {
    digitalWrite(PIN_MOTOR, HIGH);
    delay(200);
    digitalWrite(PIN_MOTOR, LOW);

    homeLat = gps.location.lat();
    homeLon = gps.location.lng();
    homeSet = true;

    // Clear takeoff indicator when new Home is set
    takeoffSet = false;

    EEPROM.put(HOME_LAT_ADDR, homeLat);
    EEPROM.put(HOME_LON_ADDR, homeLon);
    EEPROM.commit();

    Serial.print("New home point saved: ");
    Serial.print("Lat: ");
    Serial.print(homeLat, 6);
    Serial.print(", Lon: ");
    Serial.println(homeLon, 6);

    display.updateWindow(CENTER_X - INNER_RADIUS, CENTER_Y - INNER_RADIUS, 
                         2 * INNER_RADIUS, 2 * INNER_RADIUS, true);
    display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS - 1, GxEPD_WHITE);

    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);

    String message = "New home";
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(message, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(CENTER_X - tbw / 2, CENTER_Y - 5);
    display.print(message);

    message = "set";
    display.getTextBounds(message, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(CENTER_X - tbw / 2, CENTER_Y + 15);
    display.print(message);

    display.updateWindow(0, 0, 200, 200);

    delay(2000);

    distanceToHome = 0;
    updateCenterDisplay();
  } else {
    Serial.println("GPS location is invalid. Cannot set new home point.");
  }
}

void updateBatteryLevel() {
  int batteryPercent = getBatteryPercent();
  
  if (fabs(batteryPercent - prevBattery) > 5.0 || prevBattery < 0) {
    char buffer[5];
    sprintf(buffer, "%d", batteryPercent);
    updateTextArea(45, SCREEN_HEIGHT - 25, 40, 20, buffer, 45, SCREEN_HEIGHT - 10);
    prevBattery = batteryPercent;
  }
  
  battery = batteryPercent;
}

void prepareForSleep() {
  // Clear the inner circle area using partial update
  display.updateWindow(CENTER_X - INNER_RADIUS, CENTER_Y - INNER_RADIUS,
                       2 * INNER_RADIUS, 2 * INNER_RADIUS, true);
  display.fillCircle(CENTER_X, CENTER_Y, INNER_RADIUS - 1, GxEPD_WHITE);

  // Set font and color for the sleep message
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  // Prepare the "SLEEP" message
  String message = "SLEEP";
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(message, 0, 0, &tbx, &tby, &tbw, &tbh);

  // Calculate position and print the message
  display.setCursor(CENTER_X - tbw/2, CENTER_Y + tbh/2);
  display.print(message);

  // Update the full display to show the message - Use blocking update
  display.update(); // Changed from updateWindow(0, 0, 200, 200)

  // Wait for 2 seconds to show the message
  delay(2000);

  // Explicitly power down the display controller to retain the image
  display.powerDown(); // Add this line

  // Power down peripherals
  digitalWrite(PWR_EN, LOW);
  digitalWrite(Backlight, LOW); // Assuming Backlight pin is defined and used

  // Enable wakeup source (button press)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_KEY, 0); // Wake on low level

  // Enter deep sleep
  esp_deep_sleep_start();
}

void setCustomCpuFrequencyMhz(uint32_t frequency) {
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = frequency;
  pm_config.min_freq_mhz = 240;
  pm_config.light_sleep_enable = false;
  esp_pm_configure(&pm_config);
}

void drawRotatingDot() {
  if (!waitingForGPS) {
    return;
  }

  // Limit update rate for this screen
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < 200) { // Update approx 5 times/sec
      return;
  }
  lastUpdateTime = currentTime;

  float radians = rotatingDotAngle * PI / 180.0;
  int dotX = CENTER_X + int((INNER_RADIUS + 15) * cos(radians));
  int dotY = CENTER_Y - int((INNER_RADIUS + 15) * sin(radians));

  display.fillScreen(GxEPD_WHITE); // Full clear for this animation state
  drawBackground(); // Redraw static background elements (rings, labels)

  // Draw "Wait GPS" text in the center
  display.setFont(&FreeMonoBold9pt7b); // Set font for "Wait GPS"
  display.setTextColor(GxEPD_BLACK);
  String centerText = "Wait GPS";
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(centerText, 0, 0, &tbx, &tby, &tbw, &tbh);
  int waitGpsY = CENTER_Y + tbh / 2 - 5; // Slightly shift up Wait GPS text
  display.setCursor(CENTER_X - tbw / 2, waitGpsY);
  display.print(centerText);

  // --- Battery Voltage Below "Wait GPS" (same font and color) ---
  int adcValue = analogRead(Bat_ADC);
  float voltage = (adcValue / ADC_RESOLUTION) * ADC_REFERENCE * BAT_VOLTAGE_DIVIDER;
  char voltageBuffer[8];
  dtostrf(voltage, 3, 1, voltageBuffer); // e.g., "3.7"
  strcat(voltageBuffer, "V");
  int16_t vtbx, vtby; uint16_t vtbw, vtbh;
  display.getTextBounds(voltageBuffer, 0, 0, &vtbx, &vtby, &vtbw, &vtbh);
  int voltageY = waitGpsY + tbh + 8; // 8px below Wait GPS text
  display.setCursor(CENTER_X - vtbw / 2, voltageY);
  display.print(voltageBuffer);
  // --- End Battery Voltage Display ---

  // --- Altitude Display (Bottom Center) ---
  display.setFont(&tahoma10pt7b);
  char altBuffer[10];
  strcpy(altBuffer, "---");
  int16_t alt_tbx, alt_tby; uint16_t alt_tbw, alt_tbh;
  display.getTextBounds(altBuffer, 0, 0, &alt_tbx, &alt_tby, &alt_tbw, &alt_tbh);
  int altX = CENTER_X - alt_tbw / 2;
  int altY = 197; // Bottom center - moved down 5px
  display.setCursor(altX, altY);
  display.print(altBuffer);
  // --- End Altitude Display ---

  display.setFont(&FreeMonoBold9pt7b);

  // Draw battery icon (Top Left)
  drawBatteryIcon(0, 0, 40, 20, getBatteryPercent()); // Corrected size

  // Draw satellite icon (Top Right)
  drawSatelliteIcon(175, 0, 25);

  // Draw the moving dot
  display.fillCircle(dotX, dotY, 11, GxEPD_BLACK);

  display.updateWindow(0, 0, 200, 200);

  rotatingDotAngle = (rotatingDotAngle + 10) % 360;
}

int getBatteryPercent() {
    int adcValue = analogRead(Bat_ADC);
    float voltage = (adcValue / ADC_RESOLUTION) * ADC_REFERENCE * BAT_VOLTAGE_DIVIDER;
    int percent = (int)((voltage - BAT_MIN_VOLTAGE) / (BAT_MAX_VOLTAGE - BAT_MIN_VOLTAGE) * 100);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}

void drawBatteryIcon(int x, int y, int width, int height, int percentage) {
  // Draw the rounded rectangle outline of the battery
  display.drawRoundRect(x, y, width, height, 4, GxEPD_BLACK); // Rounded corners

  // Draw the positive terminal (now a bit smaller and offset)
  display.fillRect(x + width + 1, y + height / 3, 2, height / 3, GxEPD_BLACK); // Smaller terminal

  // Calculate the fill width based on the percentage
  int fillWidth = (int)((float)(width - 6) * (float)percentage / 100.0); // Leave space for rounded corners

  // Draw the fill rectangle (horizontal)
  display.fillRect(x + 3, y + 3, fillWidth, height - 6, GxEPD_BLACK); // Fill with offset for rounded corners
}

void drawSatelliteIcon(int x, int y, int size) {
  // Simple satellite representation (you can customize this)
  int centerX = x + size / 2;
  int centerY = y + size / 2;

  // Body
  display.drawCircle(centerX, centerY, size / 4, GxEPD_BLACK);

  // Solar panels
  display.fillRect(x, centerY - 2, size / 3, 4, GxEPD_BLACK);
  display.fillRect(x + size * 2 / 3, centerY - 2, size / 3, 4, GxEPD_BLACK);

  // Antenna
  display.drawLine(centerX, centerY, centerX, y, GxEPD_BLACK);

  // Display satellite count below the icon
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  char satBuffer[4];
  if (gps.satellites.isValid()) {
    sprintf(satBuffer, "%d", gps.satellites.value());
  } else {
    strcpy(satBuffer, "---");
  }
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(satBuffer, 0, 0, &tbx, &tby, &tbw, &tbh);
  int textX = x + (size - tbw) / 2 - 10; // Move 10px to the left
  int textY = y + size + tbh;
  display.setCursor(textX, textY);
  display.print(satBuffer);
}

void drawJerryCan(int x, int y, int width, int height) {
  // Ensure the can fits on screen
  if (y + height > SCREEN_HEIGHT) {
    height = SCREEN_HEIGHT - y;
  }

  // Body with diagonal top-right cut
  int cut = width / 4;
  display.drawLine(x, y + height, x, y, GxEPD_BLACK); // left
  display.drawLine(x, y, x + width - cut, y, GxEPD_BLACK); // top left
  display.drawLine(x + width - cut, y, x + width, y + cut, GxEPD_BLACK); // diagonal cut
  display.drawLine(x + width, y + cut, x + width, y + height, GxEPD_BLACK); // right
  display.drawLine(x + width, y + height, x, y + height, GxEPD_BLACK); // bottom

  // Spout (angled, top-right)
  int spoutLen = width / 4;
  display.drawLine(x + width, y + cut, x + width + spoutLen / 2, y + cut - spoutLen / 2, GxEPD_BLACK);
  display.drawLine(x + width + spoutLen / 2, y + cut - spoutLen / 2, x + width + spoutLen, y + cut, GxEPD_BLACK);

  // Handle (set back from front edge)
  int handleW = width / 3;
  int handleH = height / 7;
  int handleX = x + width - cut - handleW - 2;
  int handleY = y + 2;
  display.drawRoundRect(handleX, handleY, handleW, handleH, 3, GxEPD_BLACK);

  // Small triangle in top-left
  int tri = width / 5;
  display.drawLine(x + 2, y + 2, x + tri, y + 2, GxEPD_BLACK);
  display.drawLine(x + 2, y + 2, x + 2, y + tri, GxEPD_BLACK);
  display.drawLine(x + tri, y + 2, x + 2, y + tri, GxEPD_BLACK);

  // Draw the current fuel value in the can using tahoma10pt7b
  char buf[8];
  dtostrf(fuelLitres, 4, 1, buf); // e.g. "12.0"
  display.setFont(&tahoma10pt7b);
  display.setTextColor(GxEPD_BLACK);

  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);

  int textX = x + (width - tbw) / 2 - tbx;
  int textY = y + (height + tbh) / 2 - tby - 10; // Move up 10px

  display.setCursor(textX, textY);
  display.print(buf);
}

void enterSettingsScreen() {
    bool adjustingLitres = true;
    unsigned long lastInteractionTime = millis();
    bool processed = true;

    // Initial screen draw
    display.setFont(&tahoma15pt7b);
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(20, 50);
    display.print("Litres:");
    display.setCursor(120, 50);
    display.print(fuelLitres, 1);
    display.setCursor(20, 100);
    display.print("Burn:");
    display.setCursor(120, 100);
    display.print(fuelBurnRate, 1);
    display.drawRect(110, 30, 70, 30, GxEPD_BLACK);
    display.updateWindow(0, 0, 200, 200);
    
    while (true) {
        // Button handling with minimal overhead
        if (!digitalRead(PIN_KEY)) {
            if (processed) {
                processed = false;
                lastInteractionTime = millis();

                // Update values
                if (adjustingLitres) {
                    fuelLitres += 0.5;
                    if (fuelLitres > FUEL_MAX) fuelLitres = FUEL_MIN;
                } else {
                    fuelBurnRate += 0.1;
                    if (fuelBurnRate > 5.5) fuelBurnRate = 3.0;
                }

                // Quick vibration
                digitalWrite(PIN_MOTOR, HIGH);
                delayMicroseconds(30000); // 30ms vibration
                digitalWrite(PIN_MOTOR, LOW);

                // Update display only on button press
                display.fillScreen(GxEPD_WHITE);
                display.setCursor(20, 50);
                display.print("Litres:");
                display.setCursor(120, 50);
                display.print(fuelLitres, 1);
                display.setCursor(20, 100);
                display.print("Burn:");
                display.setCursor(120, 100);
                display.print(fuelBurnRate, 1);
                
                if (adjustingLitres) {
                    display.drawRect(110, 30, 70, 30, GxEPD_BLACK);
                } else {
                    display.drawRect(110, 80, 70, 30, GxEPD_BLACK);
                }
                display.updateWindow(0, 0, 200, 200);
            }
        } else {
            processed = true;
        }

        // Check timeout with no delay
        if (millis() - lastInteractionTime > 5000) {
            if (adjustingLitres) {
                adjustingLitres = false;
                lastInteractionTime = millis();
                
                // Draw new selection box
                display.fillScreen(GxEPD_WHITE);
                display.setCursor(20, 50);
                display.print("Litres:");
                display.setCursor(120, 50);
                display.print(fuelLitres, 1);
                display.setCursor(20, 100);
                display.print("Burn:");
                display.setCursor(120, 100);
                display.print(fuelBurnRate, 1);
                display.drawRect(110, 80, 70, 30, GxEPD_BLACK);
                display.updateWindow(0, 0, 200, 200);
            } else {
                // Save and exit
                EEPROM.put(FUEL_LITRES_ADDR, fuelLitres);
                EEPROM.put(FUEL_LITRES_ADDR + 4, fuelBurnRate);
                EEPROM.commit();
                ESP.restart();
            }
        }
    }
}
