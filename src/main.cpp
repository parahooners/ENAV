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
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEBUG_ENABLED 0 // Set to 1 to enable debug output, 0 to disable

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

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

// Define app version
#define APP_VERSION "V2.00"

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

// Define BLE service and characteristic UUIDs
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe2-0000-1000-8000-00805f9b34fb"

// Define an additional characteristic UUID for receiving data
#define RECEIVE_CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

// BLE variables
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLECharacteristic *pReceiveCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long bleConnectionTime = 0;
const unsigned long bleTimeout = 300000; // 5 minutes in milliseconds

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        bleConnectionTime = millis(); // Reset the timeout timer
        DEBUG_PRINTLN("Device connected, resetting BLE timeout timer");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        DEBUG_PRINTLN("Device disconnected");
    }
};

// Add these global variables for improved button handling
#define BUTTON_DEBOUNCE_MS 50       // Debounce time in milliseconds
#define BUTTON_SHORT_PRESS_MAX 800  // Maximum time for a short press in milliseconds
#define BUTTON_LONG_PRESS_MIN 2000  // Minimum time for long press recognition in milliseconds
volatile unsigned long lastButtonInterruptTime = 0;
volatile bool buttonStateChanged = false;

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long interruptTime = millis();
  // Ignore interrupts within debounce period
  if (interruptTime - lastButtonInterruptTime > BUTTON_DEBOUNCE_MS) {
    buttonPressed = true;
    buttonStateChanged = true;
    lastButtonInterruptTime = interruptTime;
  }
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
double calculateRelativeBearing(double homeBearing, double pilotHeading);
void drawNavigationDisplay(int centerX, int centerY, double homeBearing, double pilotHeading);
void drawSpeedometer(int centerX, int centerY, int speed);
void updateDisplay();
void handleButtonPress();
void buzz(unsigned int frequency, unsigned long duration);
void vibrateMotor(unsigned long duration);
void displaySleepScreen();
void enterSleepMode();
void handleWakeUp();
void setupBLE();
void sendBLEData();
void handleBLECommand(const std::string &command);

// Add variables for the POI (for backward compatibility)
double poiLatitude = 0.0;
double poiLongitude = 0.0;
bool legacyPoiEnabled = false; // Renamed from poiEnabled to avoid conflict

// Add variables for fuel burn rate
double fuelBurnRate = 4.8; // Default burn rate in L/h

// Add global variables for mode selection and POIs at the top
#define MODE_FLYING 1
#define MODE_WALKING 2
uint8_t operationMode = MODE_FLYING; // Default to flying mode
bool doubleTapDetected = false;
unsigned long lastTapTime = 0;
const unsigned long doubleTapThreshold = 300; // ms between taps to count as double-tap

// Add global variables for POIs
#define MAX_POIS 3
double poiLatitudes[MAX_POIS] = {0.0};
double poiLongitudes[MAX_POIS] = {0.0};
bool poiEnabled[MAX_POIS] = {false};

// Add more screens
bool isScreen6 = false;
bool isScreen7 = false;
bool isScreen8 = false;
bool isScreen9 = false;

// For auto-sleep
unsigned long lastSpeedAboveThresholdTime = 0;
unsigned long lastWalkingModeRefresh = 0;
const unsigned long flyingSleepTimeout = 600000;  // 10 minutes in ms
const unsigned long walkingSleepTimeout = 600000; // 10 minutes in ms
const unsigned long walkingModeRefreshInterval = 180000; // 3 minutes in ms

// Forward declarations for new functions
void displayPOIScreen(int poiIndex);
void displayCoordinatesScreen();
void displayAutoPowerOff(bool isFlying);
float getBatteryVoltage();
void saveOperationMode();
void loadOperationMode();
void savePOIs();
void loadPOIs();

// Function to calculate the direction to the POI
double calculateDirectionToPOI() {
    if (gps.location.isValid() && legacyPoiEnabled) {
        return TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), poiLatitude, poiLongitude);
    }
    return 0.0;
}

void handleEvent(AceButton* /*button*/, uint8_t eventType, uint8_t /*buttonState*/) {
  // Keep this for long press detection only
  switch (eventType) {
    case AceButton::kEventLongPressed:
      DEBUG_PRINTLN("Button long pressed - entering sleep mode");
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
  DEBUG_PRINTLN("Screen 1: Welcome Screen");

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
  DEBUG_PRINTLN("Screen 2: Waiting for Satellites Screen");
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

  // Display battery percentage on the top left
  int batteryPercentage = calculateBatteryStatus();
  display.setCursor(0, 0);
  display.print(batteryPercentage);
  display.print("%");

  // Add "H" label instead of screen number 5
  display.setTextSize(3); // Larger and bolder
  display.setCursor(5, display.height() - 22); // Adjusted position
  display.print("H");

  display.updateWindow(0, 0, display.width(), display.height(), false);
  DEBUG_PRINTLN("Screen 5: Home Point Screen");
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
  DEBUG_PRINTLN("Screen 6: Data Screen");
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
    DEBUG_PRINTLN("Screen 4: Press Button Countdown Screen");
  }
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    DEBUG_PRINTLN("Failed to obtain time");
    return;
  }
  DEBUG_PRINTF("%s\n", asctime(&timeinfo)); // Use DEBUG_PRINTF for formatted output
}

void printGPSData()
{
  if (gps.location.isValid())
  {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    int satellites = gps.satellites.value();
    DEBUG_PRINTF("Latitude: %.6f\n", latitude);
    DEBUG_PRINTF("Longitude: %.6f\n", longitude);
    DEBUG_PRINTF("Satellites: %d\n", satellites);
  }
  else
  {
    DEBUG_PRINTLN("Waiting for GPS signal...");
  }
}

void printGPSTime()
{
  if (gps.time.isValid())
  {
    char timeString[10];
    sprintf(timeString, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    DEBUG_PRINT("GPS Time: ");
    DEBUG_PRINTLN(timeString);
  }
  else
  {
    DEBUG_PRINTLN("Waiting for GPS time...");
  }
}

void saveHomePoint() {
  EEPROM.begin(512);
  EEPROM.put(0, homeLatitude);
  EEPROM.put(sizeof(homeLatitude), homeLongitude);
  EEPROM.commit();
  DEBUG_PRINTLN("Home point saved");
}

void loadHomePoint() {
  EEPROM.begin(512);
  EEPROM.get(0, homeLatitude);
  EEPROM.get(sizeof(homeLatitude), homeLongitude);
  DEBUG_PRINTLN("Home point loaded");
}

void savePOI() {
    EEPROM.begin(512);
    EEPROM.put(sizeof(homeLatitude) + sizeof(homeLongitude), poiLatitude);
    EEPROM.put(sizeof(homeLatitude) + sizeof(homeLongitude) + sizeof(poiLatitude), poiLongitude);
    EEPROM.put(sizeof(homeLatitude) + sizeof(homeLongitude) + sizeof(poiLatitude) + sizeof(poiLongitude), legacyPoiEnabled);
    EEPROM.commit();
    DEBUG_PRINTLN("POI saved to EEPROM");
}

void loadPOI() {
    EEPROM.begin(512);
    EEPROM.get(sizeof(homeLatitude) + sizeof(homeLongitude), poiLatitude);
    EEPROM.get(sizeof(homeLatitude) + sizeof(homeLongitude) + sizeof(poiLatitude), poiLongitude);
    EEPROM.get(sizeof(homeLatitude) + sizeof(homeLongitude) + sizeof(poiLatitude) + sizeof(poiLongitude), legacyPoiEnabled);
    DEBUG_PRINTF("POI loaded from EEPROM: Lat=%.6f, Lon=%.6f, Enabled=%d\n", poiLatitude, poiLongitude, legacyPoiEnabled);
}

void saveFuelData() {
    // Initialize EEPROM with enough size
    EEPROM.begin(512);
    
    // Calculate the memory addresses where to store the values
    int fuelLevelOffset = sizeof(homeLatitude) + sizeof(homeLongitude) + sizeof(poiLatitude) + sizeof(poiLongitude) + sizeof(legacyPoiEnabled);
    int fuelBurnRateOffset = fuelLevelOffset + sizeof(double);
    
    DEBUG_PRINTF("DEBUG: Saving fuel data - Level=%.2f L, Rate=%.2f L/h at offsets %d and %d\n", 
                  fuelLevel, fuelBurnRate, fuelLevelOffset, fuelBurnRateOffset);
    
    // Save the values to EEPROM
    EEPROM.put(fuelLevelOffset, fuelLevel);
    EEPROM.put(fuelBurnRateOffset, fuelBurnRate);
    
    // Important: commit the changes to EEPROM
    bool success = EEPROM.commit();
    DEBUG_PRINTF("EEPROM commit %s\n", success ? "successful" : "failed");
    
    // Verify what was actually saved
    double storedLevel, storedRate;
    EEPROM.get(fuelLevelOffset, storedLevel);
    EEPROM.get(fuelBurnRateOffset, storedRate);
    DEBUG_PRINTF("Verification - Read from EEPROM: Level=%.2f L, Rate=%.2f L/h\n", storedLevel, storedRate);
}

void loadFuelData() {
    EEPROM.begin(512);
    
    // Calculate the same memory addresses
    int fuelLevelOffset = sizeof(homeLatitude) + sizeof(homeLongitude) + sizeof(poiLatitude) + sizeof(poiLongitude) + sizeof(legacyPoiEnabled);
    int fuelBurnRateOffset = fuelLevelOffset + sizeof(double);
    
    DEBUG_PRINTF("DEBUG: Loading fuel data from offsets %d and %d\n", fuelLevelOffset, fuelBurnRateOffset);
    
    // Load the values from EEPROM
    double storedLevel, storedRate;
    EEPROM.get(fuelLevelOffset, storedLevel);
    EEPROM.get(fuelBurnRateOffset, storedRate);
    
    DEBUG_PRINTF("Raw fuel data from EEPROM: Level=%.2f L, Rate=%.2f L/h\n", storedLevel, storedRate);
    
    // Only update the global variables if the stored values seem valid
    if (!isnan(storedLevel) && storedLevel > 0 && storedLevel <= 100) {
        fuelLevel = storedLevel;
    } else {
        fuelLevel = 12.0; // Default
        DEBUG_PRINTLN("Using default fuel level: 12.0 L");
    }
    
    if (!isnan(storedRate) && storedRate > 0 && storedRate <= 10) {
        fuelBurnRate = storedRate;
    } else {
        fuelBurnRate = 4.5; // Default
        DEBUG_PRINTLN("Using default burn rate: 4.5 L/h");
    }
    
    DEBUG_PRINTF("Final fuel data: Level=%.2f L, Rate=%.2f L/h\n", fuelLevel, fuelBurnRate);
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

  // Replace the inner circle with a jerry can icon
  // Jerry can outline
  int canLeft = centerX - 32;
  int canTop = centerY - 20;
  int canWidth = 64;
  int canHeight = 50;
  
  // Draw the main body of the jerry can
  display.drawRect(canLeft, canTop, canWidth, canHeight, GxEPD_BLACK);
  display.drawLine(canLeft + 5, canTop, canLeft + 5, canTop + canHeight, GxEPD_BLACK); // Left vertical ridge
  display.drawLine(canLeft + canWidth - 5, canTop, canLeft + canWidth - 5, canTop + canHeight, GxEPD_BLACK); // Right vertical ridge
  
  // Draw the cap/lid on top that appears open
  display.drawLine(canLeft + 20, canTop - 10, canLeft + 44, canTop - 10, GxEPD_BLACK); // Cap top
  display.drawLine(canLeft + 20, canTop - 10, canLeft + 24, canTop, GxEPD_BLACK); // Cap left side
  display.drawLine(canLeft + 44, canTop - 10, canLeft + 40, canTop, GxEPD_BLACK); // Cap right side
  
  // Draw fuel dripping/pouring from the can
  display.drawLine(canLeft + 32, canTop - 10, canLeft + 32, canTop - 15, GxEPD_BLACK); // Fuel stream
  display.drawLine(canLeft + 33, canTop - 12, canLeft + 35, canTop - 18, GxEPD_BLACK); // Drip 1
  display.drawLine(canLeft + 30, canTop - 14, canLeft + 28, canTop - 19, GxEPD_BLACK); // Drip 2

  // Draw handle
  display.drawLine(canLeft + canWidth, canTop + 10, canLeft + canWidth + 10, canTop + 10, GxEPD_BLACK);
  display.drawLine(canLeft + canWidth, canTop + 40, canLeft + canWidth + 10, canTop + 40, GxEPD_BLACK);
  display.drawLine(canLeft + canWidth + 10, canTop + 10, canLeft + canWidth + 10, canTop + 40, GxEPD_BLACK);

  // Keep the middle and outer circles for navigation display
  display.drawCircle(centerX, centerY, 70, GxEPD_BLACK); // Middle circle
  display.drawCircle(centerX, centerY, 95, GxEPD_BLACK); // Outer circle
  display.drawCircle(centerX, centerY, 96, GxEPD_BLACK); // Thicker outer circle

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

  // Display current speed in digital style with increased size and moved further left
  int speed = gps.speed.isValid() ? (int)gps.speed.kmph() : 0; // Get speed in km/h
  char speedText[4];
  sprintf(speedText, "%3d", speed); // Format speed with leading spaces
  display.setTextSize(4); // Increase text size (was 3)
  display.setCursor(centerX - 70, centerY - 65); // Move much further left, almost touching the circle edge

  display.print(speedText);

  // Display the fuel level in the middle of the jerry can with black text - moved up a bit
  char fuelText[5];
  sprintf(fuelText, "%.0f", fuelLevel); // Convert fuel level to integer-like format
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(3);
  // Center the text inside the jerry can
  int textWidth = strlen(fuelText) * 18; // Approximate width at text size 3
  display.setCursor(centerX - (textWidth / 2), centerY - 5); // Moved up a bit from centerY + 3
  display.print(fuelText);

  // Display distance to home at the bottom - moved down a bit
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
  textWidth = strlen(distanceText) * 12; // Approximate width of text at size 2
  display.setCursor(centerX - (textWidth / 2), centerY + 45); // Moved down from centerY + 35
  display.print(distanceText);
  
  // Draw mode icon in top right corner - MAKE AIRPLANE ICON LARGER
  if (operationMode == MODE_WALKING) {
    // Draw walking person icon
    int iconX = 180;
    int iconY = 20;
    // Head
    display.fillCircle(iconX, iconY - 8, 3, GxEPD_BLACK);
    // Body
    display.drawLine(iconX, iconY - 5, iconX, iconY + 5, GxEPD_BLACK);
    // Arms
    display.drawLine(iconX, iconY, iconX - 4, iconY - 2, GxEPD_BLACK);
    display.drawLine(iconX, iconY, iconX + 4, iconY + 2, GxEPD_BLACK);
    // Legs
    display.drawLine(iconX, iconY + 5, iconX - 4, iconY + 12, GxEPD_BLACK);
    display.drawLine(iconX, iconY + 5, iconX + 4, iconY + 12, GxEPD_BLACK);
  } else { // MODE_FLYING - Make airplane icon larger
    int iconX = 180;
    int iconY = 20;
    // Fuselage - make longer
    display.drawLine(iconX - 12, iconY, iconX + 12, iconY, GxEPD_BLACK);
    display.drawLine(iconX - 12, iconY+1, iconX + 12, iconY+1, GxEPD_BLACK); // Make thicker
    
    // Wings - make wider
    display.drawLine(iconX, iconY - 9, iconX, iconY + 9, GxEPD_BLACK);
    display.drawLine(iconX+1, iconY - 9, iconX+1, iconY + 9, GxEPD_BLACK); // Make thicker
    
    display.drawLine(iconX - 8, iconY - 3, iconX + 8, iconY - 3, GxEPD_BLACK);
    display.drawLine(iconX - 8, iconY - 2, iconX + 8, iconY - 2, GxEPD_BLACK); // Make thicker
    
    // Tail
    display.drawLine(iconX - 8, iconY + 6, iconX + 2, iconY + 6, GxEPD_BLACK);
    display.drawLine(iconX - 8, iconY + 5, iconX + 2, iconY + 5, GxEPD_BLACK); // Make thicker
  }
}

// This function is now empty - it's kept for compatibility but does nothing
// since we've removed the speedometer needle functionality
void drawSpeedometer(int centerX, int centerY, int speed) {
  // The speed gauge line has been removed completely
  // This function is left empty for compatibility with existing code
}

void updateDisplay() {
    if (!isWaitingForSatsScreen) {
        if (isHomePointScreen) {
            displayHomePointScreen();
        }
        else if (isScreen6) {
            displayPOIScreen(0);
        }
        else if (isScreen7) {
            displayPOIScreen(1);
        }
        else if (isScreen8) {
            displayPOIScreen(2);
        }
        else if (isScreen9) {
            displayCoordinatesScreen();
        }
        else if (isDataScreen) {
            // For backward compatibility
            displayHomePointScreen();
            isDataScreen = false;
            isHomePointScreen = true;
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
  // Modified to use the isFlying parameter
  displayAutoPowerOff(operationMode == MODE_FLYING);
  delay(2000); // Show the auto power off screen for 2 seconds
  
  displaySleepScreen();
  delay(2000); // Show the sleep screen for 2 seconds
  
  // Disable the button interrupt before sleep to prevent spurious wakeups
  detachInterrupt(digitalPinToInterrupt(PIN_KEY));
  
  // Power down GPS module
  DEBUG_PRINTLN("Powering down GPS module");
  gpsSerial.end(); // Close the GPS serial port
  
  // Set GPS_RES pin to LOW and hold it during sleep
  //pinMode(GPS_RES, OUTPUT);
  digitalWrite(GPS_RES, LOW); // Turn off GPS module
  //gpio_hold_en((gpio_num_t)GPS_RES); // Hold the pin state during sleep
  
  // Turn off the display backlight
  DEBUG_PRINTLN("Turning off backlight");
  digitalWrite(Backlight, LOW);
  
  // Power down all external components
  DEBUG_PRINTLN("Turning off power to peripherals");
  digitalWrite(PWR_EN, LOW); // Assuming this controls power to peripherals including GPS
  digitalWrite(GPS_RES, LOW); 
  // Ensure motor is off
  digitalWrite(PIN_MOTOR, LOW);
  
  // Make sure we have a clean state for deep sleep
  DEBUG_PRINTLN("Configuring deep sleep mode");
  
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
  DEBUG_PRINTLN("Entering deep sleep in 1 second...");
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
      DEBUG_PRINTLN("Woke up from external button press");
      
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
      DEBUG_PRINTLN("Spurious wakeup detected - going back to sleep");
      // Go back to sleep immediately if wakeup wasn't from our button
      enterSleepMode();
    }
  } else if (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    // Handle unexpected wakeup reasons
    DEBUG_PRINT("Unexpected wakeup cause: ");
    DEBUG_PRINTLN(wakeup_reason);
    // We could choose to go back to sleep here as well if desired
  }
}

void setupBLE() {
    BLEDevice::init("ENAV_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create the characteristic for sending data
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->addDescriptor(new BLE2902()); // Enable notifications

    // Create the characteristic for receiving data
    pReceiveCharacteristic = pService->createCharacteristic(
        RECEIVE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pReceiveCharacteristic->addDescriptor(new BLE2902()); // Add descriptor for compatibility

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    DEBUG_PRINTLN("BLE started");
}

void sendBLEData() {
    if (deviceConnected) {
        char bleString[512]; // Increased buffer size for more data
        float voltage = getBatteryVoltage();
        
        // Format the string with all POIs and battery voltage
        char poiData[256] = "";
        for (int i = 0; i < MAX_POIS; i++) {
            char poiStr[80];
            snprintf(poiStr, sizeof(poiStr), "POI%d: Lat=%.6f, Lon=%.6f, En=%d | ", 
                    i+1, poiLatitudes[i], poiLongitudes[i], poiEnabled[i]);
            strcat(poiData, poiStr);
        }
        
        snprintf(bleString, sizeof(bleString),
                 "Home Lat: %.6f, Lon: %.6f | %sMode: %d | Fuel: %.2f, Burn Rate: %.2f | Batt: %.2fV",
                 homeLatitude, homeLongitude, poiData, operationMode,
                 (double)fuelLevel, (double)fuelBurnRate, voltage);

        pCharacteristic->setValue(bleString);
        pCharacteristic->notify();
        DEBUG_PRINTLN("BLE Data Sent:");
        DEBUG_PRINTLN(bleString);
    }

    // Handle BLE disconnection or timeout
    if (!deviceConnected && oldDeviceConnected) {
        delay(400); // Give the Bluetooth stack time to process disconnection events
        pServer->startAdvertising(); // Restart advertising
        DEBUG_PRINTLN("BLE disconnected, restarting advertising...");
    }
    oldDeviceConnected = deviceConnected;

    if (!deviceConnected && millis() - bleConnectionTime > bleTimeout && bleConnectionTime != 0) {
        DEBUG_PRINTLN("BLE timeout reached, turning off BLE and WiFi...");
        BLEDevice::deinit(true); // Turn off BLE
        WiFi.disconnect(true);   // Ensure WiFi is off
        WiFi.mode(WIFI_OFF);
        bleConnectionTime = 0;   // Reset the timeout timer
    } else if (deviceConnected) {
        bleConnectionTime = millis(); // Update the connection time if a device is connected
    }
}

// Add these diagnostic functions for verification
void verifyPOIStorage() {
    // First read what was stored
    double storedLats[MAX_POIS];
    double storedLons[MAX_POIS];
    bool storedEnabled[MAX_POIS];
    
    EEPROM.begin(512);
    int poiOffset = sizeof(homeLatitude) + sizeof(homeLongitude);
    
    EEPROM.get(poiOffset, storedLats);
    EEPROM.get(poiOffset + sizeof(poiLatitudes), storedLons);
    EEPROM.get(poiOffset + sizeof(poiLatitudes) + sizeof(poiLongitudes), storedEnabled);
    
    // Print verification results
    DEBUG_PRINTLN("POI EEPROM Storage Verification:");
    DEBUG_PRINTLN("Current POI values in memory:");
    for (int i = 0; i < MAX_POIS; i++) {
        DEBUG_PRINTF("POI %d: Lat=%.6f, Lon=%.6f, Enabled=%d\n", 
                     i+1, poiLatitudes[i], poiLongitudes[i], poiEnabled[i]);
    }
    
    DEBUG_PRINTLN("Values read from EEPROM:");
    for (int i = 0; i < MAX_POIS; i++) {
        DEBUG_PRINTF("POI %d: Lat=%.6f, Lon=%.6f, Enabled=%d\n", 
                     i+1, storedLats[i], storedLons[i], storedEnabled[i]);
        
        // Check if values match
        bool matches = (storedLats[i] == poiLatitudes[i]) && 
                      (storedLons[i] == poiLongitudes[i]) && 
                      (storedEnabled[i] == poiEnabled[i]);
        DEBUG_PRINTF("Values match: %s\n", matches ? "YES" : "NO");
    }
}

void verifyFuelStorage() {
    double storedLevel, storedRate;
    
    EEPROM.begin(512);
    int fuelLevelOffset = sizeof(homeLatitude) + sizeof(homeLongitude) + 
                          sizeof(poiLatitude) + sizeof(poiLongitude) + 
                          sizeof(legacyPoiEnabled);
    int fuelBurnRateOffset = fuelLevelOffset + sizeof(double);
    
    EEPROM.get(fuelLevelOffset, storedLevel);
    EEPROM.get(fuelBurnRateOffset, storedRate);
    
    DEBUG_PRINTLN("Fuel EEPROM Storage Verification:");
    DEBUG_PRINTF("Current values in memory: Level=%.2f L, Rate=%.2f L/h\n", fuelLevel, fuelBurnRate);
    DEBUG_PRINTF("Values read from EEPROM: Level=%.2f L, Rate=%.2f L/h\n", storedLevel, storedRate);
    
    bool levelsMatch = (fabs(storedLevel - fuelLevel) < 0.01); // Compare with small tolerance
    bool ratesMatch = (fabs(storedRate - fuelBurnRate) < 0.01);
    
    DEBUG_PRINTF("Level values match: %s\n", levelsMatch ? "YES" : "NO");
    DEBUG_PRINTF("Rate values match: %s\n", ratesMatch ? "YES" : "NO");
}

// Modify handleBLECommand() to add verification after POI and fuel updates
void handleBLECommand(const std::string &command) {
    DEBUG_PRINTF("Received BLE command: %s\n", command.c_str());

    // Handle POI commands in format "POI:<index>:<lat>:<lon>:<enabled>"
    if (command.rfind("POI:", 0) == 0) {
        int index;
        double lat, lon;
        int enabled;

        if (sscanf(command.c_str(), "POI:%d:%lf:%lf:%d", &index, &lat, &lon, &enabled) == 4) {
            if (index >= 1 && index <= MAX_POIS) {
                index--; // Convert to 0-based index
                poiLatitudes[index] = lat;
                poiLongitudes[index] = lon;
                poiEnabled[index] = (enabled == 1);

                // Update the first POI for backward compatibility
                if (index == 0) {
                    poiLatitude = lat;
                    poiLongitude = lon;
                    legacyPoiEnabled = (enabled == 1);
                }

                savePOIs();
                DEBUG_PRINTF("Updated POI %d: Lat=%.6f, Lon=%.6f, Enabled=%d\n",
                              index + 1, poiLatitudes[index], poiLongitudes[index], poiEnabled[index]);

                sendBLEData();
            } else {
                DEBUG_PRINTF("Invalid POI index: %d\n", index);
            }
        } else {
            DEBUG_PRINTLN("Invalid POI command format. Expected POI:<index>:<lat>:<lon>:<enabled>");
        }
        return;
    }

    // Parse the command in the format <latitude>:<longitude>:<enabled>  (legacy format)
    double lat, lon;
    int enabled;
    if (sscanf(command.c_str(), "%lf:%lf:%d", &lat, &lon, &enabled) == 3) {
        poiLatitude = lat;
        poiLongitude = lon;
        legacyPoiEnabled = (enabled == 1);

        // Update first POI in the array for new structure
        poiLatitudes[0] = lat;
        poiLongitudes[0] = lon;
        poiEnabled[0] = (enabled == 1);

        // Save the POI to EEPROM and confirm via Serial
        savePOI();
        savePOIs();
        DEBUG_PRINTF("BLE SET_POI received: Lat=%.6f, Lon=%.6f, Enabled=%d\n", 
                     poiLatitude, poiLongitude, legacyPoiEnabled);
        sendBLEData(); // Send confirmation data back to the client
    } 
    // Special command for retrieving data
    else if (command == "GET_DATA") {
        DEBUG_PRINTLN("Received GET_DATA command, sending current data");
        sendBLEData();
    }
    // Handle fuel update command
    else if (command.compare(0, 5, "FUEL:") == 0) {
        double newFuelLevel, newBurnRate;
        // Debug the FUEL command parsing
        DEBUG_PRINTF("Parsing FUEL command: %s\n", command.c_str());
        
        if (sscanf(command.c_str(), "FUEL:%lf:%lf", &newFuelLevel, &newBurnRate) == 2) {
            DEBUG_PRINTF("Parsed values: Level=%.2f L, Burn Rate=%.2f L/h\n", newFuelLevel, newBurnRate);
            
            // Track if values were updated
            bool valuesChanged = false;
            
            // Validate and update fuel level
            if (newFuelLevel > 0 && newFuelLevel <= 100) {
                if (fuelLevel != newFuelLevel) {
                    fuelLevel = newFuelLevel;
                    valuesChanged = true;
                    DEBUG_PRINTF("Updated fuel level to %.2f L\n", fuelLevel);
                }
            } else {
                DEBUG_PRINTF("Invalid fuel level: %.2f L (must be between 0-100)\n", newFuelLevel);
            }
            
            // Validate and update burn rate
            if (newBurnRate > 0 && newBurnRate <= 10) {
                if (fuelBurnRate != newBurnRate) {
                    fuelBurnRate = newBurnRate;
                    valuesChanged = true;
                    DEBUG_PRINTF("Updated burn rate to %.2f L/h\n", fuelBurnRate);
                }
            } else {
                DEBUG_PRINTF("Invalid burn rate: %.2f L/h (must be between 0-10)\n", newBurnRate);
            }
            
            // Only save if values changed
            if (valuesChanged) {
                // Save to EEPROM and confirm
                saveFuelData();
                DEBUG_PRINTF("Fuel data after update: Level=%.2f L, Burn Rate=%.2f L/h\n", fuelLevel, fuelBurnRate);
                
                // Add verification after saving
                verifyFuelStorage();
                
                // Force an update to the display if we're on the home point screen
                if (isHomePointScreen) {
                    displayHomePointScreen();
                }
                
                // Send updated data back to the web app
                sendBLEData();
            } else {
                DEBUG_PRINTLN("No fuel values were changed");
            }
        } else {
            DEBUG_PRINTLN("Error: Could not parse FUEL command format. Expected FUEL:<level>:<rate>");
        }
    } else if (command.rfind("MODE:", 0) == 0) {
        int mode;
        if (sscanf(command.c_str(), "MODE:%d", &mode) == 1) {
            if (mode == MODE_FLYING || mode == MODE_WALKING) {
                operationMode = mode;
                saveOperationMode();
                DEBUG_PRINTF("Mode updated to: %s\n", mode == MODE_FLYING ? "Flying" : "Walking");
                sendBLEData(); // Send confirmation back
            } else {
                DEBUG_PRINTF("Invalid mode: %d\n", mode);
            }
        } else {
            DEBUG_PRINTLN("Invalid MODE command format. Expected MODE:<mode>");
        }
        return;
    } else {
        DEBUG_PRINTLN("Error: Unrecognized BLE command format");
    }
}

// Add a test verification function call to the setup function (optional)
void setup() {
    Serial.begin(115200);
  
    // Call handleWakeUp at the beginning to properly restore state if waking from sleep
    handleWakeUp();
  
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Initialize GPS serial

    delay(10);
    DEBUG_PRINTLN("ESP32 Send Image test");
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
    DEBUG_PRINTLN("GPS STARTED");

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

    // Remove the interrupt attachment for the button as we'll poll it directly
    // attachInterrupt(digitalPinToInterrupt(PIN_KEY), handleButtonInterrupt, FALLING);

    lastButtonPressTime = millis(); // Initialize the last button press time

    setupBLE(); // Initialize BLE
    loadHomePoint(); // Load home point from EEPROM
    loadPOI();       // Load POI from EEPROM
    loadPOIs();      // Load all POIs from EEPROM
    loadFuelData();  // Load fuel data from EEPROM or set defaults
    loadOperationMode(); // Load operation mode from EEPROM
    
    // Verify that saved data was loaded correctly
    DEBUG_PRINTLN("\n=== EEPROM Data Verification at Startup ===");
    verifyPOIStorage();
    verifyFuelStorage();
    DEBUG_PRINTLN("==========================================\n");
}

void loop() {
    static unsigned long lastFuelUpdateTime = 0;
    const double fuelConsumptionRate = fuelBurnRate / 3600.0;
    
    // Simple button state variables
    static bool lastButtonState = HIGH;
    static unsigned long buttonPressTime = 0;
    static unsigned long buttonReleaseTime = 0;
    static bool buttonHandled = false;
    
    // Direct button polling - simplest approach
    bool buttonState = digitalRead(PIN_KEY);
    
    // Button press detection (transition from HIGH to LOW)
    if (buttonState == LOW && lastButtonState == HIGH) {
        buttonPressTime = millis();
        buttonHandled = false;
        DEBUG_PRINTLN("Button pressed");
    }
    
    // Button release detection (transition from LOW to HIGH)
    if (buttonState == HIGH && lastButtonState == LOW) {
        buttonReleaseTime = millis();
        unsigned long pressDuration = buttonReleaseTime - buttonPressTime;
        
        // Process button press if not already handled
        if (!buttonHandled && pressDuration < 1000) {  // Short press (< 1 second)
            // Check for double-tap
            if (buttonReleaseTime - lastTapTime < 500) {  // 500ms for double-tap detection
                DEBUG_PRINTLN("Double-tap detected - showing coordinates");
                if (!isWaitingForSatsScreen && homePointSet) {
                    isHomePointScreen = false;
                    isScreen6 = false;
                    isScreen7 = false;
                    isScreen8 = false;
                    isDataScreen = false;
                    isScreen9 = true;
                    displayCoordinatesScreen();
                    buttonHandled = true;
                }
                lastTapTime = 0;  // Reset to prevent triple-tap detection
            } else {
                lastTapTime = buttonReleaseTime;
                
                // Handle regular short press for screen cycling
                if (!isWaitingForSatsScreen && homePointSet) {
                    DEBUG_PRINTLN("Short press - cycling screens");
                    if (isHomePointScreen) {
                        if (poiEnabled[0]) {
                            isHomePointScreen = false;
                            isScreen6 = true;
                            DEBUG_PRINTLN("Switching to POI 1 screen");
                            displayPOIScreen(0);
                            buttonHandled = true;
                        } else if (poiEnabled[1]) {
                            isHomePointScreen = false;
                            isScreen7 = true;
                            DEBUG_PRINTLN("Switching to POI 2 screen");
                            displayPOIScreen(1);
                            buttonHandled = true;
                        } else if (poiEnabled[2]) {
                            isHomePointScreen = false;
                            isScreen8 = true;
                            DEBUG_PRINTLN("Switching to POI 3 screen");
                            displayPOIScreen(2);
                            buttonHandled = true;
                        }
                    } else if (isScreen6) {
                        if (poiEnabled[1]) {
                            isScreen6 = false;
                            isScreen7 = true;
                            DEBUG_PRINTLN("Switching to POI 2 screen");
                            displayPOIScreen(1);
                        } else if (poiEnabled[2]) {
                            isScreen6 = false;
                            isScreen8 = true;
                            DEBUG_PRINTLN("Switching to POI 3 screen");
                            displayPOIScreen(2);
                        } else {
                            isScreen6 = false;
                            isHomePointScreen = true;
                            DEBUG_PRINTLN("Switching to home screen");
                            displayHomePointScreen();
                        }
                        buttonHandled = true;
                    } else if (isScreen7) {
                        if (poiEnabled[2]) {
                            isScreen7 = false;
                            isScreen8 = true;
                            DEBUG_PRINTLN("Switching to POI 3 screen");
                            displayPOIScreen(2);
                        } else {
                            isScreen7 = false;
                            isHomePointScreen = true;
                            DEBUG_PRINTLN("Switching to home screen");
                            displayHomePointScreen();
                        }
                        buttonHandled = true;
                    } else if (isScreen8 || isScreen9 || isDataScreen) {
                        // From any other screen, go back to home screen
                        isScreen8 = false;
                        isScreen9 = false;
                        isDataScreen = false;
                        isHomePointScreen = true;
                        DEBUG_PRINTLN("Switching to home screen");
                        displayHomePointScreen();
                        buttonHandled = true;
                    }
                }
                
                // Handle button press during countdown to set new home point
                if (isWaitingForSatsScreen && gps.satellites.value() >= 3) {
                    DEBUG_PRINTLN("Button press during satellite search - setting home point");
                    homeLatitude = gps.location.lat();
                    homeLongitude = gps.location.lng();
                    saveHomePoint();
                    homePointSet = true;
                    isWaitingForSatsScreen = false;
                    isHomePointScreen = true;
                    displayHomePointScreen();
                    buttonHandled = true;
                }
            }
        }
    }
    
    // Check for long press (button still held down)
    if (buttonState == LOW && !buttonHandled && (millis() - buttonPressTime > 2000)) {
        DEBUG_PRINTLN("Long press detected - entering sleep mode");
        buttonHandled = true;
        enterSleepMode();
    }
    
    lastButtonState = buttonState;
    
    // Process GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    // Handle waiting for satellites
    if (!homePointSet) {
        // Print GPS data to Serial every 2 seconds
        if (millis() - lastSerialOutputTime >= 2000) {
            DEBUG_PRINTLN("Printing GPS data...");
            printGPSData();
            printGPSTime();
            lastSerialOutputTime = millis();
        }

        // Refresh the WAITING FOR SATS screen every 5 seconds
        if (millis() - lastRefreshTime >= 5000) {
            DEBUG_PRINTLN("Refreshing screen...");
            displayWaitingForSatsScreen();
            lastRefreshTime = millis();
        }

        if (gps.satellites.value() >= 3) {
            DEBUG_PRINTLN("More than 3 satellites found.");
            loadHomePoint();  // Load the last saved home point after 3 satellites are found

            // Show the countdown screen
            for (int i = 10; i >= 1; i--) {
                displayCountdownScreen(i);
                
                // Check for button press without blocking
                unsigned long startTime = millis();
                while (millis() - startTime < 1000) {
                    if (digitalRead(PIN_KEY) == LOW) {  // Direct button reading
                        DEBUG_PRINTLN("Button pressed during countdown - setting new home point");
                        delay(50);  // Simple debounce
                        homeLatitude = gps.location.lat();
                        homeLongitude = gps.location.lng();
                        saveHomePoint();
                        DEBUG_PRINTLN("New home point set");
                        
                        // Exit the countdown and show the home screen
                        homePointSet = true;
                        isWaitingForSatsScreen = false;
                        isHomePointScreen = true;
                        displayHomePointScreen();
                        i = 0;  // Force exit from the countdown loop
                        break;
                    }
                    delay(10);  // Small delay while checking
                }
            }

            if (isWaitingForSatsScreen) {  // If we didn't set a new home point
                DEBUG_PRINTLN("Using last saved home point");
                homePointSet = true;
                isWaitingForSatsScreen = false;
                isHomePointScreen = true;
                displayHomePointScreen();
            }
        }
    }

    // Rest of the loop (fuel updates, display refresh, etc.)
    // ...existing code...
}

float getBatteryVoltage() {
  int bat = 0;
  for (uint8_t i = 0; i < 25; i++) {
    bat += analogRead(BAT_ADC);
  }
  bat /= 25;
  float volt = (bat * 3.3 / 4096) * 2; // Voltage divider factor
  return volt;
}

// Save operation mode to EEPROM
void saveOperationMode() {
    EEPROM.begin(512);
    int modeOffset = sizeof(homeLatitude) + sizeof(homeLongitude) + 
                     sizeof(poiLatitudes) + sizeof(poiLongitudes) + 
                     sizeof(poiEnabled) + sizeof(fuelLevel) + sizeof(fuelBurnRate);
    EEPROM.put(modeOffset, operationMode);
    EEPROM.commit();
    DEBUG_PRINTF("Operation mode saved to EEPROM: %d\n", operationMode);
}

// Load operation mode from EEPROM
void loadOperationMode() {
    EEPROM.begin(512);
    int modeOffset = sizeof(homeLatitude) + sizeof(homeLongitude) + 
                     sizeof(poiLatitudes) + sizeof(poiLongitudes) + 
                     sizeof(poiEnabled) + sizeof(fuelLevel) + sizeof(fuelBurnRate);
    EEPROM.get(modeOffset, operationMode);
    
    // Validate mode
    if (operationMode != MODE_FLYING && operationMode != MODE_WALKING) {
        operationMode = MODE_FLYING; // Default to flying mode if invalid
    }
    
    DEBUG_PRINTF("Operation mode loaded from EEPROM: %d\n", operationMode);
}

// Save all POIs to EEPROM
void savePOIs() {
    EEPROM.begin(512);
    int poiOffset = sizeof(homeLatitude) + sizeof(homeLongitude);
    
    // Save POI coordinates and enabled flags
    EEPROM.put(poiOffset, poiLatitudes);
    EEPROM.put(poiOffset + sizeof(poiLatitudes), poiLongitudes);
    EEPROM.put(poiOffset + sizeof(poiLatitudes) + sizeof(poiLongitudes), poiEnabled);
    
    EEPROM.commit();
    DEBUG_PRINTLN("All POIs saved to EEPROM");
}

// Load all POIs from EEPROM
void loadPOIs() {
    EEPROM.begin(512);
    int poiOffset = sizeof(homeLatitude) + sizeof(homeLongitude);
    
    // Load POI coordinates and enabled flags
    EEPROM.get(poiOffset, poiLatitudes);
    EEPROM.get(poiOffset + sizeof(poiLatitudes), poiLongitudes);
    EEPROM.get(poiOffset + sizeof(poiLatitudes) + sizeof(poiLongitudes), poiEnabled);
    
    // For backward compatibility
    poiLatitude = poiLatitudes[0];
    poiLongitude = poiLongitudes[0];
    legacyPoiEnabled = poiEnabled[0];
    
    DEBUG_PRINTLN("All POIs loaded from EEPROM");
    for (int i = 0; i < MAX_POIS; i++) {
        DEBUG_PRINTF("POI %d: Lat=%.6f, Lon=%.6f, Enabled=%d\n", 
                     i+1, poiLatitudes[i], poiLongitudes[i], poiEnabled[i]);
    }
}

// Add new POI screens (6, 7, 8)
void displayPOIScreen(int poiIndex) {
  if (poiIndex < 0 || poiIndex >= MAX_POIS || !poiEnabled[poiIndex]) {
    // Invalid POI or not enabled
    return;
  }
  
  display.fillScreen(GxEPD_WHITE);
  
  // Calculate direction to POI and pilot's heading
  double poiBearing = gps.location.isValid() ?
    TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), poiLatitudes[poiIndex], poiLongitudes[poiIndex]) : 0.0;
  double pilotHeading = gps.course.isValid() ? gps.course.deg() : 0.0;
  
  // Draw the basic navigation display but use poiBearing instead of homeBearing
  // Start with a white inner circle with black outline
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK); // Box around screen
  
  // Replace the inner circle with a jerry can icon
  // Jerry can outline
  int canLeft = 100 - 32;
  int canTop = 100 - 20;
  int canWidth = 64;
  int canHeight = 50;
  
  // Draw the main body of the jerry can
  display.drawRect(canLeft, canTop, canWidth, canHeight, GxEPD_BLACK);
  display.drawLine(canLeft + 5, canTop, canLeft + 5, canTop + canHeight, GxEPD_BLACK); // Left vertical ridge
  display.drawLine(canLeft + canWidth - 5, canTop, canLeft + canWidth - 5, canTop + canHeight, GxEPD_BLACK); // Right vertical ridge
  
  // Draw the cap/lid on top that appears open
  display.drawLine(canLeft + 20, canTop - 10, canLeft + 44, canTop - 10, GxEPD_BLACK); // Cap top
  display.drawLine(canLeft + 20, canTop - 10, canLeft + 24, canTop, GxEPD_BLACK); // Cap left side
  display.drawLine(canLeft + 44, canTop - 10, canLeft + 40, canTop, GxEPD_BLACK); // Cap right side
  
  // Draw fuel dripping/pouring from the can
  display.drawLine(canLeft + 32, canTop - 10, canLeft + 32, canTop - 15, GxEPD_BLACK); // Fuel stream
  display.drawLine(canLeft + 33, canTop - 12, canLeft + 35, canTop - 18, GxEPD_BLACK); // Drip 1
  display.drawLine(canLeft + 30, canTop - 14, canLeft + 28, canTop - 19, GxEPD_BLACK); // Drip 2

  // Draw handle
  display.drawLine(canLeft + canWidth, canTop + 10, canLeft + canWidth + 10, canTop + 10, GxEPD_BLACK);
  display.drawLine(canLeft + canWidth, canTop + 40, canLeft + canWidth + 10, canTop + 40, GxEPD_BLACK);
  display.drawLine(canLeft + canWidth + 10, canTop + 10, canLeft + canWidth + 10, canTop + 40, GxEPD_BLACK);
  
  // Keep the middle and outer circles for navigation display
  display.drawCircle(100, 100, 70, GxEPD_BLACK); // Middle circle
  display.drawCircle(100, 100, 95, GxEPD_BLACK); // Outer circle
  display.drawCircle(100, 100, 96, GxEPD_BLACK); // Thicker outer circle
  
  // Display current speed in digital style with increased size and moved further left
  int speed = gps.speed.isValid() ? (int)gps.speed.kmph() : 0; // Get speed in km/h
  char speedText[4];
  sprintf(speedText, "%3d", speed); // Format speed with leading spaces
  display.setTextSize(4); // Increase text size further
  display.setCursor(100 - 70, 100 - 65); // Move much further left, almost touching the circle edge
  display.print(speedText);
  
  // Display the fuel level in the middle of the jerry can with black text - moved up a bit
  char fuelText[5];
  sprintf(fuelText, "%.0f", fuelLevel);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(3);
  // Center the text inside the jerry can
  int textWidth = strlen(fuelText) * 18; // Approximate width at text size 3
  display.setCursor(100 - (textWidth / 2), 100 - 5); // Moved up a bit from 100 + 3
  display.print(fuelText);
  
  // Draw the POI number circle
  double relativeBearingRad = calculateRelativeBearing(poiBearing, pilotHeading) * DEG_TO_RAD;
  int poiCircleX = 100 + 80 * sin(relativeBearingRad);
  int poiCircleY = 100 - 80 * cos(relativeBearingRad);
  
  display.fillCircle(poiCircleX, poiCircleY, 12, GxEPD_BLACK);
  display.setTextColor(GxEPD_WHITE);
  display.setTextSize(2);
  display.setCursor(poiCircleX - 6, poiCircleY - 8);
  display.print(poiIndex + 1);
  
  // Draw the home point
  double homeBearing = gps.location.isValid() && homePointSet ?
    TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), homeLatitude, homeLongitude) : 0.0;
  double homeRelativeBearing = calculateRelativeBearing(homeBearing, pilotHeading);
  double homeRelativeBearingRad = homeRelativeBearing * DEG_TO_RAD;
  
  int homeCircleX = 100 + 80 * sin(homeRelativeBearingRad);
  int homeCircleY = 100 - 80 * cos(homeRelativeBearingRad);
  
  display.fillCircle(homeCircleX, homeCircleY, 12, GxEPD_BLACK);
  display.setTextColor(GxEPD_WHITE);
  display.setTextSize(2);
  display.setCursor(homeCircleX - 6, homeCircleY - 8);
  display.print("H");
  display.setTextColor(GxEPD_BLACK); // Reset text color
  
  // Display battery percentage on the top left
  int batteryPercentage = calculateBatteryStatus();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(batteryPercentage);
  display.print("%");
  
  // Draw mode icon in top right corner
  if (operationMode == MODE_WALKING) {
    // Walk icon - keep as is
    int iconX = 180;
    int iconY = 20;
    display.fillCircle(iconX, iconY - 8, 3, GxEPD_BLACK);
    display.drawLine(iconX, iconY - 5, iconX, iconY + 5, GxEPD_BLACK);
    display.drawLine(iconX, iconY, iconX - 4, iconY - 2, GxEPD_BLACK);
    display.drawLine(iconX, iconY, iconX + 4, iconY + 2, GxEPD_BLACK);
    display.drawLine(iconX, iconY + 5, iconX - 4, iconY + 12, GxEPD_BLACK);
    display.drawLine(iconX, iconY + 5, iconX + 4, iconY + 12, GxEPD_BLACK);
  } else {
    // Airplane icon - make bigger
    int iconX = 180;
    int iconY = 20;
    // Fuselage - make longer
    display.drawLine(iconX - 12, iconY, iconX + 12, iconY, GxEPD_BLACK);
    display.drawLine(iconX - 12, iconY+1, iconX + 12, iconY+1, GxEPD_BLACK); // Make thicker
    
    // Wings - make wider
    display.drawLine(iconX, iconY - 9, iconX, iconY + 9, GxEPD_BLACK);
    display.drawLine(iconX+1, iconY - 9, iconX+1, iconY + 9, GxEPD_BLACK); // Make thicker
    
    display.drawLine(iconX - 8, iconY - 3, iconX + 8, iconY - 3, GxEPD_BLACK);
    display.drawLine(iconX - 8, iconY - 2, iconX + 8, iconY - 2, GxEPD_BLACK); // Make thicker
    
    // Tail
    display.drawLine(iconX - 8, iconY + 6, iconX + 2, iconY + 6, GxEPD_BLACK);
    display.drawLine(iconX - 8, iconY + 5, iconX + 2, iconY + 5, GxEPD_BLACK); // Make thicker
  }
  
  // Add P1, P2, or P3 label instead of screen number
  display.setTextSize(3); // Larger and bolder
  display.setCursor(5, display.height() - 22); // Adjusted position for larger text
  display.print("P");
  display.print(poiIndex + 1); // Will print P1, P2, or P3
  
  // Display distance to POI at the bottom instead of distance to home
  double distanceKm = gps.location.isValid() ?
    gps.distanceBetween(gps.location.lat(), gps.location.lng(), poiLatitudes[poiIndex], poiLongitudes[poiIndex]) / 1000.0 : 0.0;
  
  display.setTextSize(2);
  char distanceText[10];
  if (distanceKm > 10) {
    sprintf(distanceText, "%.0f", distanceKm);
  } else if (distanceKm > 1) {
    sprintf(distanceText, "%.1f", distanceKm);
  } else {
    sprintf(distanceText, "%.2f", distanceKm);
  }
  
  // Fix: Use the existing textWidth variable instead of redeclaring it
  textWidth = strlen(distanceText) * 12;  // Reuse the existing textWidth variable
  display.setCursor(100 - (textWidth / 2), 145); // Moved down from 135
  display.print(distanceText);
  
  display.updateWindow(0, 0, display.width(), display.height(), false);
  DEBUG_PRINTF("Screen %d: POI %d Screen\n", 6 + poiIndex, poiIndex + 1);
}

// Add new Screen 9 for coordinates display
void displayCoordinatesScreen() {
  display.fillScreen(GxEPD_WHITE);
  
  // Draw a box around the edge of the screen
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);
  
  // Title at the top
  display.setTextSize(2);
  display.setCursor(25, 20);
  display.print("GPS LOCATION");
  
  // Draw a horizontal line below the title
  display.drawLine(10, 40, 190, 40, GxEPD_BLACK);
  
  // Display latitude label
  display.setTextSize(2);
  display.setCursor(10, 60);
  display.print("Lat:");
  
  // Display latitude value in bold
  display.setTextSize(3);
  if (gps.location.isValid()) {
    char latText[12];
    dtostrf(gps.location.lat(), 8, 5, latText);
    display.setCursor(10, 90);
    display.print(latText);
  } else {
    display.setCursor(10, 90);
    display.print("N/A");
  }
  
  // Display longitude label
  display.setTextSize(2);
  display.setCursor(10, 130);
  display.print("Lon:");
  
  // Display longitude value in bold
  display.setTextSize(3);
  if (gps.location.isValid()) {
    char lonText[12];
    dtostrf(gps.location.lng(), 8, 5, lonText);
    display.setCursor(10, 160);
    display.print(lonText);
  } else {
    display.setCursor(10, 160);
    display.print("N/A");
  }
  
  // Add screen number
  display.setTextSize(1);
  display.setCursor(5, display.height() - 10);
  display.print("9");
  
  display.updateWindow(0, 0, display.width(), display.height(), false);
  DEBUG_PRINTLN("Screen 9: Coordinates Screen");
}

// Display auto power off screen
void displayAutoPowerOff(bool isFlying) {
  display.fillScreen(GxEPD_WHITE);
  
  // Draw a box around the edge of the screen
  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);
  
  // Title at the top
  display.setTextSize(2);
  display.setCursor(30, 30);
  display.print("AUTO POWER OFF");
  
  // Draw horizontal line below title
  display.drawLine(20, 60, 180, 60, GxEPD_BLACK);
  
  // Draw the appropriate icon based on mode
  if (isFlying) {
    // Draw a larger airplane icon
    int planeX = 100;
    int planeY = 110;
    
    // Fuselage
    display.drawLine(planeX - 20, planeY, planeX + 20, planeY, GxEPD_BLACK);
    // Wings
    display.drawLine(planeX, planeY - 15, planeX, planeY + 15, GxEPD_BLACK);
    display.drawLine(planeX - 15, planeY - 5, planeX + 15, planeY - 5, GxEPD_BLACK);
    // Tail
    display.drawLine(planeX - 15, planeY + 10, planeX + 5, planeY + 10, GxEPD_BLACK);
  } else {
    // Draw a larger walking person icon
    int personX = 100;
    int personY = 110;
    
    // Head
    display.fillCircle(personX, personY - 25, 10, GxEPD_BLACK);
    // Body
    display.drawLine(personX, personY - 15, personX, personY + 15, GxEPD_BLACK);
    display.drawLine(personX + 1, personY - 15, personX + 1, personY + 15, GxEPD_BLACK);
    // Arms
    display.drawLine(personX, personY - 5, personX - 15, personY - 15, GxEPD_BLACK);
    display.drawLine(personX, personY - 5, personX + 15, personY + 5, GxEPD_BLACK);
    // Legs
    display.drawLine(personX, personY + 15, personX - 15, personY + 35, GxEPD_BLACK);
    display.drawLine(personX, personY + 15, personX + 15, personY + 35, GxEPD_BLACK);
  }
  
  // Display message at the bottom
  display.setTextSize(1);
  display.setCursor(20, 160);
  display.print("Press button to wake up");
  
  display.update(); // Full update for this screen
  DEBUG_PRINTLN("Auto Power Off Screen displayed");
}