#include <GxEPD.h>
#include <GxDEPG0150BN/GxDEPG0150BN.h>    // 1.54" b/w 200x200
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include "time.h"
#include <AceButton.h>
#include "driver/adc.h"
#include "esp_sleep.h"

#define PIN_MOTOR 4
#define PIN_KEY 35
#define PWR_EN 5
#define Backlight 33
#define BAT_ADC 34
#define BUZZER_PIN 27  // Define the buzzer pin

#define SPI_SCK 14
#define SPI_DIN 13
#define EPD_CS 15
#define EPD_DC 2
#define SRAM_CS -1
#define EPD_RESET 17
#define EPD_BUSY 16

#define GPS_RX_PIN 21
#define GPS_TX_PIN 22
#define GPS_RES 23

using namespace ace_button;
AceButton button(PIN_KEY);

double homeLatitude = 0.0;
double homeLongitude = 0.0;
bool homePointSet = false;
bool isHomePointScreen = true; // Start on the home point screen
bool isDataScreen = false;
bool isWaitingForSatsScreen = true;
bool newHomePointSet = false;

volatile bool buttonPressed = false; // Flag for button press
unsigned long lastDebounceTime = 0;  // Last debounce time
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds

// Add these variables for direct button state handling
unsigned long buttonPressStartTime = 0;
bool buttonCurrentlyPressed = false;
bool buttonWasProcessed = false;

// Timezone and NTP server settings
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// WiFi credentials
const char *ssid = "ESPNETWORK";
const char *password = "12345678";

// GPS settings
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Create an instance of the display class for your specific ePaper display
GxIO_Class io(SPI, /*CS=*/EPD_CS, /*DC=*/EPD_DC, /*RST=*/EPD_RESET);
GxEPD_Class display(io, /*RST=*/EPD_RESET, /*BUSY=*/EPD_BUSY);

unsigned long lastRefreshTime = 0;  // Track the last refresh time
unsigned long lastSerialOutputTime = 0; // Track the last time serial output was done
unsigned long lastButtonPressTime = 0; // Track the last button press on data screen
const unsigned long dataScreenTimeout = 10000; // 10 seconds timeout for data screen
const unsigned int BUZZER_FREQUENCY = 1000; // Frequency of the buzzer tone
const unsigned long BUZZER_DURATION = 250; // Duration of the buzzer tone in milliseconds

// Add a global variable to track the fuel level
double fuelLevel = 12.0; // Initial fuel level in liters

void IRAM_ATTR handleButtonInterrupt() {
  buttonPressed = true; // Set the flag when the button is pressed
}

void handleEvent(AceButton* /*button*/, uint8_t eventType, uint8_t /*buttonState*/);
void displayWelcomeScreen();
void displayWaitingForSatsScreen();
void displaySetHomePointScreen();
void displayHomePointScreen();
void displayDataScreen();
void displayCountdownScreen(int seconds);
void printLocalTime();
void printGPSData();
void printGPSTime();
void saveHomePoint();
void loadHomePoint();
double calculateDirectionToHome();
int calculateBatteryStatus();
void drawNavigationDisplay(int centerX, int centerY, double homeBearing, double pilotHeading);
void drawSpeedometer(int centerX, int centerY, int speed);
void updateDisplay();
void handleButtonPress();
void buzz(unsigned int frequency, unsigned long duration);
void vibrateMotor(unsigned long duration);
void displaySleepScreen();
void enterSleepMode();
void handleWakeUp();

void handleEvent(AceButton* /*button*/, uint8_t eventType, uint8_t /*buttonState*/) {
  // Keep this for long press detection only
  switch (eventType) {
    case AceButton::kEventLongPressed:
      Serial.println("Button long pressed - entering sleep mode");
      // Don't try to set GPS_RES low here, as it's handled properly in enterSleepMode
      enterSleepMode(); // Enter sleep mode on long press
      break;
  }
}

void displayWelcomeScreen() {
  display.fillScreen(GxEPD_WHITE);

  // Draw more stars in the sky
  display.fillCircle(20, 20, 2, GxEPD_BLACK);
  display.fillCircle(50, 30, 2, GxEPD_BLACK);
  display.fillCircle(80, 10, 2, GxEPD_BLACK);
  display.fillCircle(120, 40, 2, GxEPD_BLACK);
  display.fillCircle(160, 20, 2, GxEPD_BLACK);
  display.fillCircle(30, 50, 2, GxEPD_BLACK);
  display.fillCircle(70, 60, 2, GxEPD_BLACK);
  display.fillCircle(150, 10, 2, GxEPD_BLACK);
  display.fillCircle(180, 50, 2, GxEPD_BLACK);

  // Draw a static half-moon
  display.fillCircle(200, 30, 10, GxEPD_BLACK); // Full circle
  display.fillCircle(195, 30, 8, GxEPD_WHITE); // Overlapping white circle to create a half-moon effect

  // Draw a river
  display.drawLine(0, 180, 50, 200, GxEPD_BLACK);
  display.drawLine(50, 200, 100, 190, GxEPD_BLACK);
  display.drawLine(100, 190, 200, 200, GxEPD_BLACK);

  // Draw more mountains
  display.drawLine(10, 130, 40, 80, GxEPD_BLACK); // Far left mountain
  display.drawLine(40, 80, 70, 130, GxEPD_BLACK);
  display.drawLine(20, 120, 60, 60, GxEPD_BLACK); // Left mountain
  display.drawLine(60, 60, 100, 120, GxEPD_BLACK);
  display.drawLine(120, 120, 160, 40, GxEPD_BLACK); // Right mountain
  display.drawLine(160, 40, 200, 120, GxEPD_BLACK);
  display.drawLine(140, 130, 180, 70, GxEPD_BLACK); // Far right mountain
  display.drawLine(180, 70, 210, 130, GxEPD_BLACK);

  // Draw a volcano with lava flowing down
  display.drawLine(40, 120, 70, 90, GxEPD_BLACK); // Volcano base
  display.drawLine(70, 90, 100, 120, GxEPD_BLACK);
  display.drawLine(70, 90, 75, 100, GxEPD_RED);   // Lava flow left
  display.drawLine(75, 100, 80, 110, GxEPD_RED); // Lava flow middle
  display.drawLine(80, 110, 85, 120, GxEPD_RED); // Lava flow right

  // Add smoke coming out of the volcano
  display.drawCircle(70, 80, 5, GxEPD_BLACK); // Smoke puff 1
  display.drawCircle(75, 70, 6, GxEPD_BLACK); // Smoke puff 2
  display.drawCircle(80, 60, 7, GxEPD_BLACK); // Smoke puff 3

  // Draw a house
  display.drawRect(110, 100, 30, 20, GxEPD_BLACK); // House base
  display.drawLine(110, 100, 125, 85, GxEPD_BLACK); // Roof left
  display.drawLine(125, 85, 140, 100, GxEPD_BLACK); // Roof right

  // Add windows to the house
  display.drawRect(115, 105, 5, 5, GxEPD_BLACK); // Left window
  display.drawRect(130, 105, 5, 5, GxEPD_BLACK); // Right window

  // Add a door to the house
  display.drawRect(122, 110, 6, 10, GxEPD_BLACK); // Door

  // Add a chimney to the house
  display.drawRect(135, 85, 5, 10, GxEPD_BLACK); // Chimney

  // Add smoke coming out of the chimney
  display.drawCircle(137, 75, 3, GxEPD_BLACK); // Smoke puff 1
  display.drawCircle(140, 70, 4, GxEPD_BLACK); // Smoke puff 2
  display.drawCircle(143, 63, 5, GxEPD_BLACK); // Smoke puff 3

  // Draw a campfire
  display.fillTriangle(90, 160, 100, 140, 110, 160, GxEPD_RED); // Fire
  display.drawLine(85, 165, 115, 165, GxEPD_BLACK); // Bottom log
  display.drawLine(95, 170, 105, 160, GxEPD_BLACK); // Cross log

  // Draw people dancing around the campfire
  display.drawCircle(70, 150, 5, GxEPD_BLACK); // Person 1 head
  display.drawLine(70, 155, 70, 165, GxEPD_BLACK); // Person 1 body
  display.drawLine(70, 160, 65, 165, GxEPD_BLACK); // Person 1 left leg
  display.drawLine(70, 160, 75, 165, GxEPD_BLACK); // Person 1 right leg
  display.drawLine(70, 157, 65, 152, GxEPD_BLACK); // Person 1 left arm
  display.drawLine(70, 157, 75, 152, GxEPD_BLACK); // Person 1 right arm

  display.drawCircle(130, 150, 5, GxEPD_BLACK); // Person 2 head
  display.drawLine(130, 155, 130, 165, GxEPD_BLACK); // Person 2 body
  display.drawLine(130, 160, 125, 165, GxEPD_BLACK); // Person 2 left leg
  display.drawLine(130, 160, 135, 165, GxEPD_BLACK); // Person 2 right leg
  display.drawLine(130, 157, 125, 152, GxEPD_BLACK); // Person 2 left arm
  display.drawLine(130, 157, 135, 152, GxEPD_BLACK); // Person 2 right arm

  // Text "ICELAND"
  display.setTextSize(2);
  display.setCursor(70, 170);
  display.print("ICELAND");

  // Add screen number
  display.setTextSize(1);
  display.setCursor(5, display.height() - 10);
  display.print("1");

  display.update();
  Serial.println("Screen 1: Welcome Screen");

  // Wait for 1 second before moving on
  delay(1000);
}

void displayWaitingForSatsScreen() {
  display.fillScreen(GxEPD_WHITE);

  // Draw a box around the edge of the screen (200x200)
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);

  // Use a shorter title that fits on one line
  display.setTextSize(2);
  int titleWidth = 12 * 10; // Approximate width of "GPS SEARCH" at text size 2
  int titleX = (200 - titleWidth) / 2;
  display.setCursor(titleX, 15);
  display.print("GPS SEARCH");

  // Draw a horizontal line below the title
  display.drawLine(10, 35, 190, 35, GxEPD_BLACK);

  // Show satellite information in a cleaner layout
  display.setTextSize(2);
  int satellitesTextWidth = 12 * 10; // Approximate width of "Satellites:" at text size 2
  int satellitesTextX = (200 - satellitesTextWidth) / 2;
  display.setCursor(satellitesTextX, 55);
  display.print("Satellites:");

  // Show satellite count in larger text - centered
  display.setTextSize(3);
  int satelliteCount = gps.satellites.value();
  int satelliteCountWidth = (satelliteCount < 10) ? 18 : 36; // Approximate width of the number
  int satelliteCountX = (200 - satelliteCountWidth) / 2;
  display.setCursor(satelliteCountX, 80);
  display.print(satelliteCount);

  // Draw a separator line
  display.drawLine(10, 110, 190, 110, GxEPD_BLACK);

  // Show GPS time in a clean format
  display.setTextSize(2);
  display.setCursor(10, 125);
  display.print("GPS Time:");

  if (gps.time.isValid()) {
    char timeString[10];
    sprintf(timeString, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    display.setCursor(65, 150);
    display.print(timeString);
  } else {
    display.setCursor(40, 150);
    display.print("Waiting...");
  }

  // Add battery percentage on the bottom right
  int batteryPercentage = calculateBatteryStatus();
  display.setTextSize(1);
  display.setCursor(150, 185);
  display.print(batteryPercentage);
  display.print("%");

  // Add screen number
  display.setTextSize(1);
  display.setCursor(5, 185);
  display.print("2");

  // Partial update
  display.updateWindow(0, 0, display.width(), display.height(), false);
  Serial.println("Screen 2: Waiting for Satellites Screen");
}

void displaySetHomePointScreen() {
  // Screen 3 is disabled, so this function is no longer used.
}

void displayHomePointScreen() {
  display.fillScreen(GxEPD_WHITE);

  // Calculate direction to home and pilot's heading
  double homeBearing = gps.location.isValid() && homePointSet
                           ? TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), homeLatitude, homeLongitude)
                           : 0.0;
  double pilotHeading = gps.course.isValid() ? gps.course.deg() : 0.0;

  // Draw the navigation display
  drawNavigationDisplay(100, 100, homeBearing, pilotHeading);

  // Draw the speedometer
  int speed = gps.speed.isValid() ? (int)gps.speed.kmph() : 0; // Get speed in km/h
  drawSpeedometer(100, 100, speed);

  // Remove the MPH display (no longer needed)

  // Display battery percentage on the top left
  int batteryPercentage = calculateBatteryStatus();
  display.setCursor(0, 0);
  display.print(batteryPercentage);
  display.print("%");

  // Add screen number
  display.setTextSize(1);
  display.setCursor(5, display.height() - 10);
  display.print("5");

  display.updateWindow(0, 0, display.width(), display.height(), false);
  Serial.println("Screen 5: Home Point Screen");
}

void displayDataScreen() {
  display.fillScreen(GxEPD_WHITE);
  
  // Draw a box around the edge of the screen (200x200)
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);
  
  // Title at the top center - keep this size as the user likes it
  display.setTextSize(2);
  display.setCursor(50, 10);
  display.print("GPS DATA");
  
  // Draw a horizontal line below the title
  display.drawLine(10, 30, 190, 30, GxEPD_BLACK);
  
  // Use text size 2 for data fields - matches the title size
  display.setTextSize(2);
  
  // Display coordinate data with labels - using shorter labels
  // We'll need to adjust spacing to fit everything with the larger font
  int yPos = 45; // Starting y position
  int labelX = 10;
  int valueX = 70;
  
  // Lat/Lon with shortened decimal places
  display.setCursor(labelX, yPos);
  display.print("Lat:");
  display.setCursor(valueX, yPos);
  if (gps.location.isValid()) {
    display.print(gps.location.lat(), 5); // Limit to 5 decimal places
  } else {
    display.print("N/A");
  }
  
  yPos += 25; // Increased spacing for larger font
  display.setCursor(labelX, yPos);
  display.print("Lon:");
  display.setCursor(valueX, yPos);
  if (gps.location.isValid()) {
    display.print(gps.location.lng(), 5); // Limit to 5 decimal places
  } else {
    display.print("N/A");
  }
  
  yPos += 25;
  display.setCursor(labelX, yPos);
  display.print("Alt:");
  display.setCursor(valueX, yPos);
  if (gps.altitude.isValid()) {
    display.print(gps.altitude.meters(), 0); // No decimal places to save space
    display.print("m");
  } else {
    display.print("N/A");
  }
  
  yPos += 25;
  display.setCursor(labelX, yPos);
  display.print("Spd:");
  display.setCursor(valueX, yPos);
  if (gps.speed.isValid()) {
    display.print((int)gps.speed.kmph()); // Integer to save space
    display.print("km/h");
  } else {
    display.print("N/A");
  }
  
  yPos += 25;
  display.setCursor(labelX, yPos);
  display.print("Sats:");
  display.setCursor(valueX, yPos);
  display.print(gps.satellites.value());
  
  // Add time at the bottom
  yPos += 25;
  display.setCursor(labelX, yPos);
  display.print("Time:");
  display.setCursor(valueX, yPos);
  if (gps.time.isValid()) {
    char timeString[10];
    sprintf(timeString, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    display.print(timeString);
  } else {
    display.print("N/A");
  }
  
  // Add screen number at the bottom right
  display.setTextSize(1); // Smaller size just for the screen number
  display.setCursor(185, 190);
  display.print("6");

  display.updateWindow(0, 0, display.width(), display.height(), false);
  Serial.println("Screen 6: Data Screen");
}

void displayCountdownScreen(int seconds) {
  display.fillScreen(GxEPD_WHITE);

  // Draw a 200x200 box
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);

  // Move the "Press button" text to the top of the screen
  display.setTextSize(2);
  int textWidth = 12 * 11; // Approximate width of "Press button" at text size 2
  int textX = (200 - textWidth) / 2;
  display.setCursor(textX, 10); // Start closer to the top
  display.print("Press button");

  textWidth = 12 * 7; // Approximate width of "to set new"
  textX = (200 - textWidth) / 2;
  display.setCursor(textX, 40); // Adjusted position
  display.print("to set new");

  textWidth = 12 * 10; // Approximate width of "home point"
  textX = (200 - textWidth) / 2;
  display.setCursor(textX, 70); // Adjusted position
  display.print("home point");

  // Center the countdown number
  display.setTextSize(5);
  char secondsText[3];
  sprintf(secondsText, "%d", seconds);
  textWidth = 24 * strlen(secondsText); // Approximate width of the countdown number
  textX = (200 - textWidth) / 2;
  display.setCursor(textX, 100); // Adjusted position
  display.print(seconds);

  // Increase the font size of latitude and longitude
  display.setTextSize(2);
  display.setCursor(10, 140); // Adjusted position for latitude
  display.print("Lat: ");
  display.print(homeLatitude, 6);
  display.setCursor(10, 170); // Adjusted position for longitude
  display.print("Lon: ");
  display.print(homeLongitude, 6);

  display.updateWindow(0, 0, display.width(), display.height(), false);

  if (seconds == 10) {
    Serial.println("Screen 4: Press Button Countdown Screen");
  }
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void printGPSData()
{
  if (gps.location.isValid())
  {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    int satellites = gps.satellites.value();
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
    Serial.print("Satellites: ");
    Serial.println(satellites);
  }
  else
  {
    Serial.println("Waiting for GPS signal...");
  }
}

void printGPSTime()
{
  if (gps.time.isValid())
  {
    char timeString[10];
    sprintf(timeString, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.print("GPS Time: ");
    Serial.println(timeString);
  }
  else
  {
    Serial.println("Waiting for GPS time...");
  }
}

void saveHomePoint() {
  EEPROM.begin(512);
  EEPROM.put(0, homeLatitude);
  EEPROM.put(sizeof(homeLatitude), homeLongitude);
  EEPROM.commit();
  Serial.println("Home point saved");
}

void loadHomePoint() {
  EEPROM.begin(512);
  EEPROM.get(0, homeLatitude);
  EEPROM.get(sizeof(homeLatitude), homeLongitude);
  Serial.println("Home point loaded");
}

double calculateDirectionToHome() {
  if (gps.location.isValid() && homePointSet) {
    double currentLatitude = gps.location.lat();
    double currentLongitude = gps.location.lng();
    double bearing = TinyGPSPlus::courseTo(currentLatitude, currentLongitude, homeLatitude, homeLongitude);
    return bearing;
  }
  return 0.0;
}

int calculateBatteryStatus() {
  int bat = 0;
  for (uint8_t i = 0; i < 25; i++) {
    bat += analogRead(BAT_ADC);
  }
  bat /= 25;
  float volt = (bat * 3.3 / 4096);
  return constrain(map(volt * 1000, 1630, 1850, 0, 100), 0, 100);
}

double calculateRelativeBearing(double homeBearing, double pilotHeading) {
  // Calculate the relative bearing
  double relativeBearing = homeBearing - pilotHeading;

  // Normalize to the range -180° to 180°
  if (relativeBearing > 180) {
    relativeBearing -= 360;
  } else if (relativeBearing < -180) {
    relativeBearing += 360;
  }

  return relativeBearing;
}

void drawNavigationDisplay(int centerX, int centerY, double homeBearing, double pilotHeading) {
  // Draw a 200x200 box around the screen
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);

  // Reduce the size of the filled black circle by 30%
  display.fillCircle(centerX, centerY, 28, GxEPD_BLACK);  // Inner circle (reduced size)
  display.drawCircle(centerX, centerY, 70, GxEPD_BLACK); // Middle circle
  display.drawCircle(centerX, centerY, 95, GxEPD_BLACK); // Outer circle (made smaller to fit the screen)
  display.drawCircle(centerX, centerY, 96, GxEPD_BLACK); // Thicker outer circle

  // Draw the fixed arrow (triangle) pointing forward
  int arrowTipX = centerX;
  int arrowTipY = centerY - 28; // Adjusted for smaller inner circle
  int arrowBaseLeftX = centerX - 10;
  int arrowBaseLeftY = centerY - 10;
  int arrowBaseRightX = centerX + 10;
  int arrowBaseRightY = centerY - 10;
  display.fillTriangle(arrowTipX, arrowTipY, arrowBaseLeftX, arrowBaseLeftY, arrowBaseRightX, arrowBaseRightY, GxEPD_BLACK);

  // Calculate the position of the "H" circle based on the relative bearing
  double relativeBearingRad = calculateRelativeBearing(homeBearing, pilotHeading) * DEG_TO_RAD; // Convert to radians
  int hCircleX = centerX + 80 * sin(relativeBearingRad);    // Move inward slightly
  int hCircleY = centerY - 80 * cos(relativeBearingRad);

  // Draw the "H" circle filled with black
  display.fillCircle(hCircleX, hCircleY, 12, GxEPD_BLACK); // Larger circle for "H"

  // Draw the "H" in bold white text
  display.setTextColor(GxEPD_WHITE);
  display.setTextSize(2); // Make the "H" bolder
  display.setCursor(hCircleX - 6, hCircleY - 8); // Center the "H" inside the circle
  display.print("H");
  display.setTextColor(GxEPD_BLACK); // Reset text color to black

  // Display the fuel level in the middle solid circle (bold)
  char fuelText[5];
  sprintf(fuelText, "%.0f", fuelLevel); // Convert fuel level to integer-like format
  display.setTextColor(GxEPD_WHITE);
  display.setTextSize(3); // Make the fuel number bolder
  display.setCursor(centerX - 15, centerY - 10); // Adjust position to match the moved circle
  display.print(fuelText);
  display.setTextColor(GxEPD_BLACK); // Reset text color to black

  // Display distance slightly below the previous position and reduce font size
  double distanceKm = homePointSet ? gps.distanceBetween(gps.location.lat(), gps.location.lng(), homeLatitude, homeLongitude) / 1000.0 : 0.0;
  display.setTextSize(2); // Reduce the font size for the distance number
  char distanceText[10];
  if (distanceKm > 10) {
    sprintf(distanceText, "%.0f", distanceKm); // No decimal places for values above 10
  } else if (distanceKm > 1) {
    sprintf(distanceText, "%.1f", distanceKm); // 1 decimal place for values above 1
  } else {
    sprintf(distanceText, "%.2f", distanceKm); // 2 decimal places for values below 1
  }
  int textWidth = strlen(distanceText) * 12; // Approximate width of text at size 2
  display.setCursor(centerX - (textWidth / 2), centerY + 35); // Center the text and move it slightly down
  display.print(distanceText);
}

void drawSpeedometer(int centerX, int centerY, int speed) {
  // Adjusted radius to fit the new layout
  int radius = 90; // Radius of the speedometer
  int labelOffset = 35; // Adjusted offset to position numbers along the curve
  int needleLength = radius - 20; // Ensure the needle does not touch the circle

  // Draw tick marks and labels for every 10 km/h
  for (int i = 0; i <= 60; i += 10) {
    double angle = (180 + (i * 180.0 / 60)) * DEG_TO_RAD; // Map 0-60 to 180° to 0°
    int labelX = centerX + (radius - labelOffset) * cos(angle);
    int labelY = centerY + (radius - labelOffset) * sin(angle);

    // Rotate the text to follow the curve
    display.setTextSize(2); // Larger font size for numbers
    display.setCursor(labelX - 8, labelY - 10); // Adjust position to center the text
    display.print(i);
  }

  // Draw the needle (shortened and thicker)
  double needleAngle = (180 + (speed * 180.0 / 60)) * DEG_TO_RAD; // Map speed to angle
  int needleX = centerX + needleLength * cos(needleAngle);
  int needleY = centerY + needleLength * sin(needleAngle);
  display.drawLine(centerX, centerY, needleX, needleY, GxEPD_BLACK); // First line
  display.drawLine(centerX + 1, centerY, needleX + 1, needleY, GxEPD_BLACK); // Thicker line
}

void updateDisplay() {
  // Ensure the correct screen is displayed based on the flags
  if (!isWaitingForSatsScreen) {
    if (isDataScreen) {
      displayDataScreen(); // Ensure Screen 6 is displayed when the flag is set
    } else if (isHomePointScreen) {
      displayHomePointScreen(); // Ensure Screen 5 is displayed when the flag is set
    }
  }
}

// Function to generate a tone on the buzzer pin
void buzz(unsigned int frequency, unsigned long duration) {
  tone(BUZZER_PIN, frequency, duration);
  delay(duration); // Add a small delay to ensure the tone is played
  noTone(BUZZER_PIN); // Stop the tone
}

// Function to vibrate the motor
void vibrateMotor(unsigned long duration) {
  digitalWrite(PIN_MOTOR, HIGH);
  delay(duration);
  digitalWrite(PIN_MOTOR, LOW);
}

void displaySleepScreen() {
  display.fillScreen(GxEPD_WHITE);

  // Draw a happy face with closed eyes
  display.drawCircle(100, 100, 30, GxEPD_BLACK); // Face outline
  display.fillCircle(90, 90, 5, GxEPD_BLACK);    // Left eye
  display.fillCircle(110, 90, 5, GxEPD_BLACK);   // Right eye
  display.drawLine(85, 95, 95, 95, GxEPD_WHITE); // Left eye closed
  display.drawLine(105, 95, 115, 95, GxEPD_WHITE); // Right eye closed

  // Draw a smile using a series of lines
  display.drawLine(85, 110, 90, 115, GxEPD_BLACK);
  display.drawLine(90, 115, 100, 118, GxEPD_BLACK);
  display.drawLine(100, 118, 110, 115, GxEPD_BLACK);
  display.drawLine(110, 115, 115, 110, GxEPD_BLACK);

  // Display "Going to sleep" text
  display.setTextSize(2);
  display.setCursor(50, 150);
  display.print("Going to sleep");

  display.update();
}

void enterSleepMode() {
  displaySleepScreen();
  delay(2000); // Show the sleep screen for 2 seconds
  
  // Disable the button interrupt before sleep to prevent spurious wakeups
  detachInterrupt(digitalPinToInterrupt(PIN_KEY));
  
  // Power down GPS module
  Serial.println("Powering down GPS module");
  gpsSerial.end(); // Close the GPS serial port
  
  // Set GPS_RES pin to LOW and hold it during sleep
  //pinMode(GPS_RES, OUTPUT);
  digitalWrite(GPS_RES, LOW); // Turn off GPS module
  //gpio_hold_en((gpio_num_t)GPS_RES); // Hold the pin state during sleep
  
  // Turn off the display backlight
  Serial.println("Turning off backlight");
  digitalWrite(Backlight, LOW);
  
  // Power down all external components
  Serial.println("Turning off power to peripherals");
  digitalWrite(PWR_EN, LOW); // Assuming this controls power to peripherals including GPS
  digitalWrite(GPS_RES, LOW); 
  // Ensure motor is off
  digitalWrite(PIN_MOTOR, LOW);
  
  // Make sure we have a clean state for deep sleep
  Serial.println("Configuring deep sleep mode");
  
  // Disable WiFi and BT explicitly
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  digitalWrite(GPS_RES, LOW); 
  
  // Reconfigure button pin with stronger pull-up to prevent spurious wakeups
  pinMode(PIN_KEY, INPUT_PULLUP);
  
  // More aggressive power-down of peripherals
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  digitalWrite(GPS_RES, LOW); 
  // Disable all wakeup sources first 
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  digitalWrite(GPS_RES, LOW); 
  // Use EXT1 wakeup which is more robust than EXT0 for a single GPIO
  uint64_t pin_mask = (1ULL << PIN_KEY);
  esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ALL_LOW);
  digitalWrite(GPS_RES, LOW); 
  // Add significant delay to ensure everything settles
  Serial.println("Entering deep sleep in 1 second...");
  Serial.flush();
  delay(1000);
  digitalWrite(GPS_RES, LOW); 
  // Put the board into deep sleep
  esp_deep_sleep_start();
}

void handleWakeUp() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    uint64_t wakeup_pins = esp_sleep_get_ext1_wakeup_status();
    
    if (wakeup_pins & (1ULL << PIN_KEY)) {
      Serial.println("Woke up from external button press");
      
      // Wait a bit to ensure button is fully released
      delay(100);
      
      // Release any held pins after wakeup
      gpio_hold_dis((gpio_num_t)GPS_RES);
      
      // Power up necessary components
      pinMode(PWR_EN, OUTPUT);
      digitalWrite(PWR_EN, HIGH);  // Turn on power to peripherals
      
      pinMode(Backlight, OUTPUT);
      digitalWrite(Backlight, HIGH);
      
      // Initialize GPS power pin and set it HIGH
      pinMode(GPS_RES, OUTPUT);
      digitalWrite(GPS_RES, HIGH);
      delay(200); // Give GPS module time to initialize
      
      gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
      
      // Vibrate briefly to indicate wake-up
      pinMode(PIN_MOTOR, OUTPUT);
      vibrateMotor(200);
    } else {
      Serial.println("Spurious wakeup detected - going back to sleep");
      // Go back to sleep immediately if wakeup wasn't from our button
      enterSleepMode();
    }
  } else if (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    // Handle unexpected wakeup reasons
    Serial.print("Unexpected wakeup cause: ");
    Serial.println(wakeup_reason);
    // We could choose to go back to sleep here as well if desired
  }
}

void setup()
{
  Serial.begin(115200);
  
  // Call handleWakeUp at the beginning to properly restore state if waking from sleep
  handleWakeUp();
  
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Initialize GPS serial

  delay(10);
  Serial.println("ESP32 Send Image test");
  SPI.begin(SPI_SCK, -1, SPI_DIN, EPD_CS);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PWR_EN, OUTPUT);
  pinMode(PIN_KEY, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  pinMode(Backlight, OUTPUT);
  pinMode(BAT_ADC, INPUT); // Set battery ADC pin as input
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output

  // Initialize GPS reset pin
  pinMode(GPS_RES, OUTPUT);
  digitalWrite(GPS_RES, HIGH); // Enable GPS module

  digitalWrite(PWR_EN, HIGH);
  digitalWrite(PIN_MOTOR, HIGH);
  delay(200);
  digitalWrite(PIN_MOTOR, LOW);
  delay(100);
  digitalWrite(PIN_MOTOR, HIGH);
  delay(200);
  digitalWrite(PIN_MOTOR, LOW);

  // Turn on backlight
  digitalWrite(Backlight, HIGH);
  
  display.init();
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  // Display welcome screen
  displayWelcomeScreen();

  // Start GPS
  Serial.println("GPS STARTED");

  // Turn off WiFi initially
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // Display waiting for sats screen
  displayWaitingForSatsScreen();

  // Configure the ButtonConfig with the event handler - only for long press now
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  // We're only using AceButton for long press detection now
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setLongPressDelay(3000); 
  buttonConfig->setDebounceDelay(20);

  // Attach interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(PIN_KEY), handleButtonInterrupt, FALLING);

  lastButtonPressTime = millis(); // Initialize the last button press time
}

void loop() {
  static unsigned long lastFuelUpdateTime = 0; // Track the last time fuel was updated
  const double fuelConsumptionRate = 4.8 / 3600.0; // Fuel consumption rate in liters per second

  // Direct button state monitoring for more reliable detection
  int buttonState = digitalRead(PIN_KEY);
  
  // Button is active LOW with pull-up resistor
  if (buttonState == LOW && !buttonCurrentlyPressed) {
    // Button just pressed
    buttonCurrentlyPressed = true;
    buttonPressStartTime = millis();
    buttonWasProcessed = false;
    Serial.println("Button pressed directly detected");
  } 
  else if (buttonState == HIGH && buttonCurrentlyPressed) {
    // Button just released
    buttonCurrentlyPressed = false;
    
    // Only process short presses here (less than long press threshold)
    if ((millis() - buttonPressStartTime < 3000) && !buttonWasProcessed && homePointSet && !isWaitingForSatsScreen) {
      Serial.println("Button short press detected - switching screens");
      
      // Toggle between screens
      if (isHomePointScreen) {
        isHomePointScreen = false;
        isDataScreen = true;
        Serial.println("Switching to Screen 6: Data Screen");
        displayDataScreen();
      } else {
        isDataScreen = false;
        isHomePointScreen = true;
        Serial.println("Switching to Screen 5: Home Point Screen");
        displayHomePointScreen();
      }
      
      buttonWasProcessed = true;
    }
  }
  
  // Still keep AceButton library for long press detection
  button.check();
  
  // Clear interrupt flag if set
  if (buttonPressed) {
    buttonPressed = false;
  }

  // Process GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (!homePointSet) {
    // Print GPS data to Serial every 2 seconds
    if (millis() - lastSerialOutputTime >= 2000) {
      Serial.println("Printing GPS data...");
      printGPSData();
      printGPSTime();
      lastSerialOutputTime = millis();
    }

    // Refresh the WAITING FOR SATS screen every 5 seconds
    if (millis() - lastRefreshTime >= 5000) {
      Serial.println("Refreshing screen...");
      displayWaitingForSatsScreen();
      lastRefreshTime = millis();
    }

    if (gps.satellites.value() >= 3) {
      Serial.println("More than 3 satellites found.");
      loadHomePoint();  // Load the last saved home point after 3 satellites are found

      // Skip "Set Home Point Screen" and go directly to the countdown screen
      for (int i = 10; i >= 1; i--) {
        displayCountdownScreen(i); // Show the "Press Button" screen
        // Check for button press without blocking
        unsigned long startTime = millis();
        while (millis() - startTime < 1000) {
          if (buttonPressed && (millis() - lastDebounceTime) > debounceDelay) {
            Serial.println("Home point set during countdown.");
            newHomePointSet = true;
            homeLatitude = gps.location.lat();
            homeLongitude = gps.location.lng();
            saveHomePoint();
            Serial.println("New home point set.");

            // Immediately break out of the countdown loop and set flags for main loop
            homePointSet = true;
            isWaitingForSatsScreen = false;
            isHomePointScreen = true;
            updateDisplay(); // Update the display immediately
            buttonPressed = false; // Clear the flag
            lastDebounceTime = millis();
            break; // Break out of the while loop
          }
          yield(); // Allow other tasks to run
        }
        if (newHomePointSet) break; // Break out of the for loop if button was pressed
      }

      if (!newHomePointSet) {
        Serial.println("Using last saved home point.");
      }

      homePointSet = true;
      isWaitingForSatsScreen = false;
      displayHomePointScreen();
      isHomePointScreen = true;
    }
  }

  // Update fuel level if speed is above 5 MPH
  if (gps.speed.isValid() && gps.speed.mph() > 5) {
    unsigned long currentTime = millis();
    if (currentTime - lastFuelUpdateTime >= 1000) { // Update every second
      fuelLevel -= fuelConsumptionRate; // Decrease fuel level
      if (fuelLevel < 0) {
        fuelLevel = 0; // Ensure fuel level does not go below 0
      }
      lastFuelUpdateTime = currentTime;
    }
  }

  // Update display less frequently to avoid excessive refreshing
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 2000) {  
    // Only update display every 2 seconds unless screen just changed
    if (!buttonWasProcessed) {
      updateDisplay();
    }
    lastDisplayUpdate = millis();
  }

  // Very short delay to make button detection more responsive
  delay(20);
}