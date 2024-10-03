#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

// Pin definitions for the LEDs
const int redGreenLED = 14;  // Combined Red/Green LED for MPU initialization (D14)
const int greenLED = 27;     // Green LED for Authorization (D27)
const int redLED2 = 26;      // Red LED for Bluetooth connection (D26)
const int blueLED = 25;      // Blue LED for Data transmission (D25)

const String secretKey = "MY_SECRET_KEY"; // Shared key between ESP32 and Flutter app
bool isAuthenticated = false; // Authentication status
String receivedMessage = "";

// Function to get MPU-6050 sensor readings
void getMPUData(float &accX, float &accY, float &accZ, float &gyroX, float &gyroY, float &gyroZ) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Store accelerometer data
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  // Store gyroscope data
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
}

void setup() {
  Serial.begin(115200);

  // Initialize LED pins
  pinMode(redGreenLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED2, OUTPUT);
  pinMode(blueLED, OUTPUT);

  // Initialize Bluetooth
  if (!SerialBT.begin("FlexiScan Device")) {
    Serial.println("Error initializing Bluetooth");
    digitalWrite(redLED2, HIGH); // Turn on Red LED2 to indicate Bluetooth initialization error
    ESP.restart();
  } else {
    Serial.println("Bluetooth Initialized");
  }

  SerialBT.register_callback(btCallBack);
  Serial.println("The device is ready to connect!");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050. Check your connections!");
    while (1);
  }
  Serial.println("MPU6050 Initialized");
  digitalWrite(redGreenLED, LOW);   // Ensure Red is OFF
  digitalWrite(redGreenLED, HIGH);  // Turn on Green LED for successful MPU initialization
}

void btCallBack(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
    digitalWrite(redLED2, HIGH); // Turn on Red LED2 when a device is connected
    isAuthenticated = false;    // Reset authentication status on new connection
    digitalWrite(greenLED, LOW); // Turn off Green LED (D27) for new connection
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client Disconnected");
    digitalWrite(redLED2, LOW);  // Turn off Red LED2 when a device is disconnected
    isAuthenticated = false;    // Reset authentication status on disconnection
    digitalWrite(greenLED, LOW); // Turn off Green LED when disconnected
  }
}

void loop() {
  if (SerialBT.available()) {
    receivedMessage = SerialBT.readString();  // Read incoming message
    Serial.print("Received message: ");
    Serial.println(receivedMessage);

    // Authentication handling
    if (!isAuthenticated) {
      // Blink Green LED (D27) during authentication
      digitalWrite(greenLED, HIGH);  
      delay(300);
      digitalWrite(greenLED, LOW);
      delay(300);

      if (receivedMessage == secretKey) {
        isAuthenticated = true;
        Serial.println("Authenticated successfully!");
        digitalWrite(greenLED, HIGH);  // Keep Green LED ON when authenticated
        SerialBT.println("Authentication successful");
      } else {
        Serial.println("Authentication failed! Disconnecting...");
        SerialBT.println("Authentication failed");
        SerialBT.disconnect();
        digitalWrite(greenLED, LOW);  // Turn OFF Green LED if authentication fails
      }
    }
  }

  // After authentication, send MPU-6050 data every second
  if (isAuthenticated) {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    getMPUData(accX, accY, accZ, gyroX, gyroY, gyroZ);

    // Send the MPU data to the Flutter app
    String dataMessage = "ACC: X=" + String(accX) + 
                         " Y=" + String(accY) + 
                         " Z=" + String(accZ) +
                         " | GYRO: X=" + String(gyroX) + 
                         " Y=" + String(gyroY) + 
                         " Z=" + String(gyroZ);

    SerialBT.println(dataMessage); // Send data over Bluetooth
    Serial.println(dataMessage);   // Print to serial monitor
    
    // Blink Blue LED to indicate data transmission
    digitalWrite(blueLED, HIGH);
    delay(100);
    digitalWrite(blueLED, LOW);

    delay(1000); // Wait for 1 second before sending the next reading
  }
}
