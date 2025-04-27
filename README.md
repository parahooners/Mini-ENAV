# 🧭 Mini ENAV - Your Pocket Crow-Flies Navigator! 🧭


![Mini ENAV Display](https://github.com/parahooners/Mini-ENAV/blob/main/include/Photos/image0.png?raw=true)
![Mini ENAV Watch](https://github.com/parahooners/Mini-ENAV/blob/main/include/Photos/image1.jpeg?raw=true)

Ever wandered off the beaten path and wished you had a super simple way to point back home? Whether you're hiking a mountain trail 🌲, paddling across a lake 🛶, backpacking through the wilderness 🎒, walking the dog in a new park 🐕, or even flying your paramotor 🪂, the Mini ENAV has got your back!

---

## What's the Big Idea? 🤔

This project turns a nifty little LilyGO E-Paper Watch into a minimalist navigation device. Forget complex maps! Mini ENAV focuses on one core mission: **getting you back to where you started.**

---

## Features

- E-paper display (1.54" 200x200)
- GPS navigation (TinyGPS++)
- Return-to-home and takeoff point tracking
- Battery and satellite status
- Haptic feedback (vibration motor)
- 3D-printable case

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
   Press and hold the button until vibration. The current GPS location is saved as "Home".
3. **Takeoff Point:**  
   After moving >500m from Home, the device marks your takeoff location automatically.
4. **Display Overview:**
   - **Center:** Distance to Home (in meters if you're close, kilometers if you're further out). Speed shown below.
   - **Top Right:** Altitude (feet)
   - **Bottom Left:** Battery percentage
   - **Bottom Right:** Number of satellites
   - **Ring Dots:**  
     - "H" = Home direction  
     - "1" = Takeoff point direction
5. **Follow the Dot:**  
   Walk (or fly, or paddle!) in the direction the dot is pointing. As you get closer, the distance number will drop.
6. **Sleep Mode:**  
   Device sleeps after 10 minutes of inactivity. Press the button to wake.
7. **Charging:**  
   Charge via USB as per your ESP32 board's instructions.

---

## Settings Page

The settings page is accessible only within the first 10 seconds after the unit starts up by pressing the button. On the settings page, you can:

- Adjust the starting fuel litres (increments by 0.5).
- Adjust the fuel burn rate (increments by 0.1).
- Toggle the visibility of the Jerry Can and fuel litres display on the main navigation and GPS wait screens.

To change a value, press the button while the selection box is on the desired setting. After 5 seconds of inactivity, the selection box moves to the next setting. When all settings are configured, the values are saved to EEPROM and the unit restarts.

---

## Notes

- Ensure GPS has a clear view of the sky for best accuracy.
- Battery voltage calibration may be needed for your hardware.
- For 3D case printing, adjust STL scaling as needed for your printer and hardware tolerances.

---

## Support the Project 🙏

If you find Mini ENAV useful and want to show your appreciation, you can buy me a coffee! It helps fuel late-night coding sessions and future adventures (both digital and real-world!).

<a href="https://buymeacoffee.com/hooners" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>

---



## License

MIT License. See [LICENSE](./LICENSE) for details.
