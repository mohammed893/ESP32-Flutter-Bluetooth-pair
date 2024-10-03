
# FlexiScan ESP32 MPU-6050 with Bluetooth Communication

## Overview

This project demonstrates how to use an **ESP32** microcontroller to communicate with a **Flutter** app via **Bluetooth**, along with MPU-6050 sensor integration for accelerometer and gyroscope data. The project includes basic authentication, real-time sensor data transmission, and LED indicators for connection status, authentication, and data transmission.

## Features

- **MPU-6050 Integration**: Reads accelerometer and gyroscope data.
- **Bluetooth Communication**: Uses ESP32 Bluetooth to send sensor data to a Flutter app.
- **Authentication**: Ensures secure communication using a predefined secret key.
- **LED Indicators**:
  - **Green LED**: Indicates successful MPU initialization and authentication.
  - **Red LED**: Indicates Bluetooth connection status.
  - **Blue LED**: Blinks during data transmission.
  
## Hardware Requirements

- **ESP32**
- **MPU-6050 Sensor**
- **4 LEDs** (Green/Red/Blue for various status indicators)
- **Resistors** (for LEDs, typically 220Ω)
- **Jumper Wires**
- **Breadboard (optional)**

### Pin Connections:

| Component  | ESP32 Pin |
|------------|-----------|
| Red/Green LED (for MPU Init) | D14       |
| Green LED (for Authentication) | D27      |
| Red LED (for Bluetooth) | D26       |
| Blue LED (for Data Transmission) | D25       |
| MPU-6050 SDA | GPIO 21  |
| MPU-6050 SCL | GPIO 22  |

## Software Requirements

- **Arduino IDE**
- **BluetoothSerial Library**
- **Adafruit MPU6050 Library**

## Setup

1. **Install Arduino IDE** and add the **ESP32 board** by following [this guide](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/).
2. Install the required libraries:
    - **BluetoothSerial**: This is included with ESP32 boards support.
    - **Adafruit MPU6050**: Can be installed via Arduino IDE Library Manager.
3. **Wiring**: Connect the MPU-6050 sensor and LEDs according to the pin connections in the table above.
4. **Upload Code**: Upload the provided sketch to your ESP32 using Arduino IDE.
5. **Flutter App**: Ensure that the Flutter app communicates via Bluetooth and uses the same `secretKey` for authentication.

## Code Explanation

- **Bluetooth Setup**: Initializes Bluetooth communication using `SerialBT` and registers a callback function for connection events.
- **MPU-6050 Initialization**: Uses `mpu.begin()` to initialize the MPU sensor. The LED connected to pin D14 will turn green when successful.
- **Authentication**: The ESP32 authenticates a client by checking the received Bluetooth message against the predefined `secretKey`.
- **Sensor Data Transmission**: After authentication, the ESP32 sends accelerometer and gyroscope data from the MPU-6050 to the Flutter app every second. The blue LED blinks to indicate data transmission.


