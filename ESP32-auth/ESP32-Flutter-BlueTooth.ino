/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
static BLEUUID BLE_DATA_SERVICE_UUID           ("82afce6c-9638-493b-9be0-d2aebc45f5af");
static BLEUUID BLE_AUTH_SERVICE_UUID           ("b7037e40-4dbe-4e49-9652-c5d11a0e7bbf");

#define CHARACTERISTIC_AUTH_WRITE_UUID          "c8744281-4e46-4b49-99c2-4a1d0a4ad2a4"

#define CHARACTERISTIC_LED_UUID                 "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define CHARACTERISTIC_ACCEL_X_UUID             "dc2ddfce-07a1-4cd9-b53a-0c9461dd2e4d"
#define CHARACTERISTIC_ACCEL_Y_UUID             "2434c5cd-9af9-4c20-9f6c-ed569283adb5"
#define CHARACTERISTIC_ACCEL_Z_UUID             "442f145b-195d-4c97-b601-731700c36aa5"

#define CHARACTERISTIC_GYRO_X_UUID              "4c77c0a5-dbd2-44fd-8a8a-655fa880039b"
#define CHARACTERISTIC_GYRO_Y_UUID              "4dda858f-1ee8-49f4-b7a3-b4fba86fab5a"
#define CHARACTERISTIC_GYRO_Z_UUID              "9186d23f-cd46-476c-bdcc-cba3536cc490"


//-----------------CONFIG----------------

// MUST be a 6-digit number, no more, no less
const uint32_t PASSWORD = 123456;

// set to true to get simulated MPU data
const bool simulateMPU = false;

// whether to print readings to the serial or not
const bool printReadingsToSerial = false;

// the pin that sends the esp to deep sleep and back
const uint8_t POWER_PIN = 14;

// the four pins that the mpu uses
const uint8_t MPU_VCC_PIN = 26;
const uint8_t MPU_GND_PIN = 25;
const uint8_t MPU_SCL_PIN = 33;
const uint8_t MPU_SDA_PIN = 32;


//-----------------GLOBAL VARIABLES----------------
BLEServer *pServer;
BLEService *pDataService;

struct characteristicAccelStruct {
  BLECharacteristic *X = nullptr;
  BLECharacteristic *Y = nullptr;
  BLECharacteristic *Z = nullptr;
};

characteristicAccelStruct characteristicAccel;

struct characteristicGyroStruct {
  BLECharacteristic *X = nullptr;
  BLECharacteristic *Y = nullptr;
  BLECharacteristic *Z = nullptr;
};

characteristicAccelStruct characteristicGyro;

bool device_connected = false;
bool mpu_connected = false;
bool device_authenticated = false;
Adafruit_MPU6050 mpu;

//-----------------FUNCTION DEFINITIONS---------------------

void ToggleDeepSleep();
void power_down();

void init_BLEDataService();
void init_BLESecurity();
void init_BLE();
void DataServiceStart();
void DataServiceStop();

void init_MPU6050();


//-----------------CLASSES---------------------

class ServerCallBacks : public BLEServerCallbacks {
  
  void onConnect(BLEServer *pServer) {
    Serial.println("connected to someone");
    device_authenticated = false;
    device_connected = true;
  }
  void onDisconnect(BLEServer *pServer) {
    device_authenticated = false;
    device_connected = false;
    Serial.println("disconnected from someone");
    DataServiceStop();
    pServer->getAdvertising()->start();
  }
};


class SecurityCallback : public BLESecurityCallbacks {

  uint32_t onPassKeyRequest(){
    Serial.println("key requested");
    return PASSWORD;
  }

  void onPassKeyNotify(uint32_t pass_key){}

  bool onConfirmPIN(uint32_t pass_key){
    Serial.println("on confirmPIN before");
    vTaskDelay(5000);
    Serial.println("on confirmPIN after");

    return pass_key == PASSWORD;
  }

  bool onSecurityRequest(){
    Serial.println("onSecurityRequest");
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
    if(cmpl.success){
      Serial.println("   - SecurityCallback - Authentication Success");
      device_authenticated = true;
      DataServiceStart();
    }else{
      Serial.println("   - SecurityCallback - Authentication Failure*");
      device_authenticated = false;
      pServer->removePeerDevice(pServer->getConnId(), true);
      DataServiceStop();
    }
    BLEDevice::startAdvertising();
  }
};


//-----------------DEEP SLEEP------------------

void IRAM_ATTR ToggleDeepSleep() {
  static int64_t lMillis = 0;

  if((millis() - lMillis) < 5) return;

  lMillis = millis();
  disableInterrupt(POWER_PIN);
  
  Serial.println("going to sleep...");
  detachInterrupt(POWER_PIN);
  attachInterrupt(POWER_PIN, power_down, FALLING);
}


void power_down() {
  Serial.println("now can enable again");
  detachInterrupt(POWER_PIN);
  esp_sleep_enable_ext0_wakeup(gpio_num_t(POWER_PIN), RISING);
  esp_deep_sleep_start();
}

//-----------------BLE SET UP-----------------

void init_BLEDataService() {
  pDataService = pServer->createService(BLE_DATA_SERVICE_UUID, 30, 0);


  characteristicAccel.X = pDataService->createCharacteristic(CHARACTERISTIC_ACCEL_X_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristicAccel.Y = pDataService->createCharacteristic(CHARACTERISTIC_ACCEL_Y_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristicAccel.Z = pDataService->createCharacteristic(CHARACTERISTIC_ACCEL_Z_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  characteristicGyro.X  = pDataService->createCharacteristic(CHARACTERISTIC_GYRO_X_UUID,  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristicGyro.Y  = pDataService->createCharacteristic(CHARACTERISTIC_GYRO_Y_UUID,  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristicGyro.Z  = pDataService->createCharacteristic(CHARACTERISTIC_GYRO_Z_UUID,  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  
  
  characteristicAccel.X->setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  characteristicAccel.Y->setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  characteristicAccel.Z->setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  characteristicGyro.X-> setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  characteristicGyro.Y-> setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  characteristicGyro.Z-> setAccessPermissions(ESP_GATT_PERM_READ_AUTHORIZATION);
  


  BLEDescriptor *pDesciptorAccelX = new BLEDescriptor((uint16_t) 0x2901);
  pDesciptorAccelX->setValue("Acceleration X Axis");
  BLEDescriptor *pDesciptorAccelY = new BLEDescriptor((uint16_t) 0x2901);
  pDesciptorAccelY->setValue("Acceleration Y Axis");
  BLEDescriptor *pDesciptorAccelZ = new BLEDescriptor((uint16_t) 0x2901);
  pDesciptorAccelZ->setValue("Acceleration Z Axis");

  BLEDescriptor *pDesciptorGyroX = new BLEDescriptor((uint16_t) 0x2901);
  pDesciptorGyroX->setValue("Gyro X Axis");
  BLEDescriptor *pDesciptorGyroY = new BLEDescriptor((uint16_t) 0x2901);
  pDesciptorGyroY->setValue("Gyro Y Axis");
  BLEDescriptor *pDesciptorGyroZ = new BLEDescriptor((uint16_t) 0x2901);
  pDesciptorGyroZ->setValue("Gyro Z Axis");

  (characteristicAccel.X)->addDescriptor(pDesciptorAccelX);
  (characteristicAccel.Y)->addDescriptor(pDesciptorAccelY);
  (characteristicAccel.Z)->addDescriptor(pDesciptorAccelZ);

  (characteristicGyro.X)->addDescriptor(pDesciptorGyroX);
  (characteristicGyro.Y)->addDescriptor(pDesciptorGyroY);
  (characteristicGyro.Z)->addDescriptor(pDesciptorGyroZ);

  pDataService->start();
}


void init_BLESecurity() {
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;          
  uint8_t key_size = 16;     
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = PASSWORD;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  Serial.println("init_BLESecurity");



}


void init_BLE() {
  BLEDevice::init("ESP-32 slave");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallBacks());

  init_BLEDataService();

  pServer->getAdvertising()->start();

  Serial.println("began advertising");

  init_BLESecurity();
}


void DataServiceStart() {
  //pDataService->start();
}

void DataServiceStop() {
  //pDataService->stop();
}

//-----------------MPU SET UP-----------------

void init_MPU6050() {

  if (simulateMPU) return;

  pinMode(MPU_VCC_PIN, OUTPUT);
  pinMode(MPU_GND_PIN, OUTPUT);

  digitalWrite(MPU_VCC_PIN, HIGH);
  digitalWrite(MPU_GND_PIN, LOW);

  if(!Wire.setPins(MPU_SDA_PIN, MPU_SCL_PIN)) {
    return;
  }


  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    mpu_connected = false;
    return;
  } else {
    Serial.println("MPU6050 Found!");
    mpu_connected = true;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  //esp_sleep_enable_ext0_wakeup((gpio_num_t) 14, HIGH);
  
  attachInterrupt(POWER_PIN, ToggleDeepSleep, HIGH);
  //esp_deep_sleep_start();

  init_BLE();
  init_MPU6050();


}


void loop() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  double accelX;
  double accelY;
  double accelZ;

  double gyroX;
  double gyroY;
  double gyroZ;

  if (!simulateMPU) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;

    gyroX= g.gyro.x;
    gyroY= g.gyro.y;
    gyroZ= g.gyro.z;
  } else {
    accelX = random(-1000, 1000) / 100.0; // Random values between -10.00 and 10.00
    accelY = random(-1000, 1000) / 100.0;
    accelZ = random(-1000, 1000) / 100.0;

    gyroX = random(-500, 500) / 100.0; // Random values between -5.00 and 5.00
    gyroY = random(-500, 500) / 100.0;
    gyroZ = random(-500, 500) / 100.0;
  }

  if (device_connected && device_authenticated) {
    char tx_string[8];

    dtostrf(accelX, 8, 4, tx_string);
    (characteristicAccel.X)->setValue(tx_string);

    dtostrf(accelY, 8, 4, tx_string);
    (characteristicAccel.Y)->setValue(tx_string);
    
    dtostrf(accelZ, 8, 4, tx_string);
    (characteristicAccel.Z)->setValue(tx_string);

    dtostrf(gyroX, 8, 4, tx_string);
    (characteristicGyro.X)->setValue(tx_string);
    
    dtostrf(gyroY, 8, 4, tx_string);
    (characteristicGyro.Y)->setValue(tx_string);
    
    dtostrf(gyroZ, 8, 4, tx_string);
    (characteristicGyro.Z)->setValue(tx_string);


    characteristicAccel.X->notify();
    characteristicAccel.Y->notify();
    characteristicAccel.Z->notify();

    characteristicGyro.X->notify();
    characteristicGyro.Y->notify();
    characteristicGyro.Z->notify();


  }

  if (printReadingsToSerial) {
    Serial.printf("Acceleration X: %f   Y: %f   Z: %f \n", accelX, accelY, accelZ);
    Serial.printf("Gyro         X: %f   Y: %f   Z: %f \n", gyroX, gyroY, gyroZ);
  }

  

  delay(500);
}

