# ENAV - Electronic Navigation Aid

## Description

ENAV is an electronic navigation aid for pilots, built using an ESP32, GPS module, and e-paper display. It provides essential navigation information, including bearing to a home point, distance, speed, and fuel level.

## Features

*   Displays bearing and distance to a saved home point.
*   Shows current speed and altitude.
*   Tracks fuel level.
*   Uses an e-paper display for low power consumption.
*   Includes a sleep mode to conserve battery.

## Hardware Requirements

*   ESP32 development board
*   GPS module (e.g., NEO-6M)
*   e-paper display (e.g., GxEPD 1.54" 200x200)
*   Button
*   Buzzer (optional)
*   Vibration motor (optional)

## Software Requirements

*   Arduino IDE or PlatformIO
*   Required libraries (install via Library Manager):
    *   GxEPD
    *   AceButton
    *   TinyGPSPlus
    *   ArduinoJson

## Installation

1.  Clone this repository:

    ```bash
    git clone https://github.com/your_username/ENAV.git
    ```

2.  Open the project in the Arduino IDE or PlatformIO.
3.  Install the required libraries using the Library Manager.
4.  Configure the `platformio.ini` file (if using PlatformIO) with your board and upload settings.
5.  Upload the code to your ESP32.

## Usage

1.  Power on the device.
2.  Wait for the GPS to acquire a satellite fix.
3.  The device will display the bearing and distance to the saved home point.
4.  Press the button to switch between the home point screen and the data screen.
5.  Long press the button to enter sleep mode.

## Contributing

Contributions are welcome! Please submit a pull request with your changes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Credits

*   Uses the [GxEPD](https://github.com/ZinggJM/GxEPD) library for e-paper display.
*   Uses the [AceButton](https://github.com/bxparks/AceButton) library for button handling.
*   Uses the [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) library for GPS data parsing.

## Contact

Matty
