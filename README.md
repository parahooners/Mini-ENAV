# 🧭 Mini ENAV - Your Pocket Crow-Flies Navigator! 🧭


![Mini ENAV Display](https://github.com/parahooners/Mini-ENAV/blob/main/include/Photos/image0.png?raw=true)
![Mini ENAV Watch](https://github.com/parahooners/Mini-ENAV/blob/main/include/Photos/image1.jpeg?raw=true)

Ever wandered off the beaten path and wished you had a super simple way to point back home? Whether you're hiking a mountain trail 🌲, paddling across a lake 🛶, backpacking through the wilderness 🎒, walking the dog in a new park 🐕, or even flying your paramotor 🪂, the Mini ENAV has got your back!

<div align="center">
  <a href="https://parahooners.github.io/Mini-ENAV/" target="_blank">
    <img src="https://img.shields.io/badge/🌐_Live_Web_Interface-Visit_Mini_ENAV_Online-blue?style=for-the-badge&color=0366d6" alt="Mini ENAV Web Interface">
  </a>
</div>

---

## What's the Big Idea? 🤔

This project turns a nifty little LilyGO E-Paper Watch into a minimalist navigation device. Forget complex maps! Mini ENAV focuses on one core mission: **getting you back to where you started.**

---

## Features

- **Navigation Indicators**:
  - Displays distance and heading to a home point, takeoff point, or any active waypoint/location.
  - Updates dynamically based on GPS data.
  - Support for up to 20 waypoints and 5 regular locations, all managed via BLE and the web interface.
  - Cycles through navigation targets (Home, Takeoff, Waypoints, Locations) on the device.
- **Navigation Modes**:
  - Three navigation modes: Off, Location, and Waypoint, selectable via device settings or web interface.
- **Bluetooth LE Connectivity**:
  - Connect with the web interface (see badge/link above) through your phone or computer browser to manage waypoints and locations.
  - Save, activate, and deactivate custom locations and waypoints remotely.
  - View all saved locations and waypoints when connecting to the device.
  - BLE auto-shutdown after inactivity, re-enabled by button press.
- **Fuel Tracking**:
  - Tracks remaining fuel in liters.
  - Displays estimated flight time based on fuel burn rate.
- **Flight Hours Tracking**:
  - Accumulates total flight hours and stores them persistently in EEPROM.
  - Displays total flight hours on the "Wait GPS" screen.
- **Settings Screen**:
  - Allows adjustment of:
    - **Fuel quantity**: Set the current fuel level in liters.
    - **Fuel burn rate**: Adjust the fuel consumption rate in liters per hour.
    - **Fuel display visibility**: Toggle the visibility of the fuel display.
    - **Navigation mode**: Select Off, Location, or Waypoint mode.
  - Displays **estimated flight time** based on the current fuel and burn rate.
- **Takeoff Point Logic**:
  - Takeoff point is set automatically when moving >500m from Home.
- **Energy Efficiency**:
  - CPU frequency reduced to **40 MHz** to maximize power efficiency.
  - E-paper display refresh rate optimized to 0.8 seconds.
  - Deep sleep mode implemented for inactivity or long button press.
- **Improved UI**:
  - Rotating dot animation while waiting for GPS.
  - Compass rose, battery, satellite, and fuel indicators.
  - Partial display refreshes for efficiency.
- **Haptic Feedback**:
  - Vibration motor for tactile feedback during key actions.

---

## Hardware

- LilyGO E-Paper Watch (ESP32)
- 1.54" GxEPD e-paper display
- GPS module (connected to RX/TX)
- Vibration motor
- Battery (LiPo)
- Button for setting home point

---

## 3D-Printed Case

A 3D-printable case is available for this project.  
**[Download STL files here (GitHub)](https://github.com/parahooners/Mini-ENAV/tree/main/include/STL%20files%20for%20case)**

> **Note:** To fit the watch into the provided 3D case, you will need to use a **half-size GPS antenna**. The standard antenna that comes with the watch is too large.

---

## Build & Setup (VSCode + PlatformIO)

1. **Install [Visual Studio Code](https://code.visualstudio.com/)**
2. **Install [PlatformIO extension](https://platformio.org/install/ide?install=vscode)**
3. **Clone this repository**
   ```
   git clone https://github.com/parahooners/Mini-ENAV.git
   ```
4. **Open the project folder in VSCode**
5. **Connect your ESP32 device via USB**
6. **Select the correct serial port in PlatformIO (bottom bar)**
7. **Build and upload the firmware**
   - Click the "PlatformIO: Upload" button (checkmark/arrow icon)
   - Or run:
     ```
     pio run --target upload
     ```
8. **Monitor serial output (optional)**
   - Click "PlatformIO: Monitor" or run:
     ```
     pio device monitor
     ```

---

## How to Operate the Mini ENAV

1. **Power On:**  
   Device starts and waits for GPS fix ("Wait GPS" message).
2. **Set Home Point:**  
   Short press the button to set your current location as "Home". The device validates GPS accuracy to ensure reliable home points.
3. **Takeoff Point Detection:**  
   After moving >500m from Home, the device automatically marks your takeoff location with a "T" indicator.
4. **Display Overview:**
   - **Center:** Distance to Home (in meters if you're close, kilometers if you're further out). Speed shown below.
   - **Top Right:** Altitude (feet)
   - **Bottom Left:** Battery percentage
   - **Bottom Right:** Number of satellites
   - **Ring Indicators:**  
     - "H" = Home direction  
     - "T" = Takeoff point direction
     - "1-5" = Custom waypoint directions (when set via BLE)
5. **Long Press (5 seconds):**  
   Long-press the button for 5 seconds to enter sleep mode.
6. **Sleep Mode:**  
   Device also enters sleep after 10 minutes of inactivity. Press the button to wake.
7. **Charging:**  
   Charge via USB as per your ESP32 board's instructions.

---

## Settings Page

The settings page is accessible only within the first 10 seconds after the unit starts up by pressing the button. On the settings page, you can:

- Adjust the starting fuel litres (increments by 0.5).
- Adjust the fuel burn rate (increments by 0.1).
- Toggle the visibility of the Jerry Can and fuel litres display on the main navigation and GPS wait screens.
- Select navigation mode (Off, Location, Waypoint).

To change a value, press the button while the selection box is on the desired setting. After 5 seconds of inactivity, the selection box moves to the next setting. When all settings are configured, the values are saved to EEPROM and the unit restarts.

---

## Web Interface (Bluetooth LE)

The Mini ENAV now includes Bluetooth LE connectivity with a web interface for managing waypoints and locations:

1. **Accessing the Web Interface:**
   - Use a Web Bluetooth compatible browser (Chrome, Edge, or Opera on Android/Windows/Mac)
   - Visit the [Live Web Interface](https://parahooners.github.io/Mini-ENAV/) (recommended) or open the `docs/index.html` file locally
   - Click "Connect to Mini ENAV" to establish a connection

2. **Managing Waypoints and Locations:**
   - **View Saved Locations/Waypoints:** When connected, all saved waypoints and locations are displayed automatically
   - **Add/Update Waypoint or Location:**
     - Select a location or waypoint number
     - Set status (ON/OFF)
     - Enter coordinates manually or click on the map
     - Click "Send Location"
   - **Modify Waypoint/Location:** Click "Load" on an existing entry to edit it
   - **Switch Navigation Modes:** Use the web interface to change between Off, Location, and Waypoint navigation modes

3. **Navigation Interface:**
   - The map displays all active waypoints and locations
   - Each appears as a numbered marker
   - The device cycles through and shows directions to active targets on the display

4. **Notes:**
   - The web interface works best on mobile phones for on-the-go management
   - Waypoints and locations persist through device reboots and power cycles
   - Up to 20 waypoints and 5 locations can be stored
   - BLE will auto-disable after 2 minutes of inactivity or disconnect; press the device button to re-enable

---

## Recent Updates

### May 2025 Updates

- Expanded BLE support: Now supports up to 20 waypoints and 5 locations, all managed via the web interface
- Added navigation modes (Off, Location, Waypoint) and cycling through navigation targets
- Improved settings screen: Now includes navigation mode selection and better persistence
- Takeoff point logic: Automatically sets takeoff point when moving >500m from Home
- Flight hours tracking: Tracks and displays total flight hours, saved in EEPROM
- BLE auto-shutdown and re-enable logic
- Improved UI: Rotating dot animation, compass rose, battery, satellite, and fuel indicators, partial refreshes
- Power management: Deep sleep after inactivity or long button press, lower CPU frequency

### April 2025 Updates

- Added Bluetooth LE connectivity for managing waypoints via a web interface
- Implemented a web-based interface for saving, viewing and managing waypoints
- Added support for up to 5 custom Points of Interest that appear on the navigation display

### Previous Updates

- Labels are now aligned to the far left for improved readability.
- Added the ability to adjust:
  - **Fuel quantity**: Set the current fuel level in liters.
  - **Fuel burn rate**: Adjust the fuel consumption rate in liters per hour.
  - **Fuel display visibility**: Toggle the visibility of the fuel display.
- Displays **estimated flight time** based on the current fuel and burn rate.
- Total flight hours are now displayed on the "Wait GPS" screen.
- Flight hours are stored persistently in EEPROM and updated periodically during flight.
- CPU frequency reduced to **40 MHz** to conserve power.
- E-paper display refresh rate optimized to match its 0.8-second refresh limitation.
- All unnecessary `Serial.print` debugging statements have been removed to save power.
- Wi-Fi and Bluetooth are disabled throughout the program.

---

## Notes

- Ensure GPS has a clear view of the sky for best accuracy.
- Battery voltage calibration may be needed for your hardware.
- For 3D case printing, adjust STL scaling as needed for your printer and hardware tolerances.
- Web Bluetooth is not supported on iOS devices (iPhone/iPad).

---

## About the Author

This project was written by **Matty Austen**, a lifelong coder who started programming at a very young age. Matty is passionate about making technology accessible to everyone, not just a select few. His love for paramotoring inspired this project, and he hopes it will be useful to the paramotoring community and beyond.

Matty is also the creator of the **Parahooners YouTube channel**, where he shares his adventures in paramotoring. Be sure to check it out for exciting content and insights into the world of paramotoring!

---

## Support the Project 🙏

If you find Mini ENAV useful and want to show your appreciation, you can buy me a coffee! It helps fuel late-night coding sessions and future adventures (both digital and real-world!).

<a href="https://buymeacoffee.com/hooners" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>

---

## License

MIT License. See [LICENSE](./LICENSE) for details.
