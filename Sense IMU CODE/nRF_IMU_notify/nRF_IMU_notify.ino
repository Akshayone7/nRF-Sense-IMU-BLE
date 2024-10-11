#include <Wire.h>
#include <LSM6DS3.h>
#include <bluefruit.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>

// Define UUIDs for the custom service and characteristic
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef1"

#define DATA_SIZE 240
#define NUM_SETS 20

bool is_connect = false;

BLEService customService(SERVICE_UUID);                       // Define the custom service
BLECharacteristic customCharacteristic(CHARACTERISTIC_UUID);  // Define the custom characteristic

Adafruit_FlashTransport_QSPI flashTransport;
LSM6DS3 lsm6ds3;

uint8_t count = 1;
unsigned long lastSendTime = 0;

#define SEND_INTERVAL 100

// Variables for sending data
uint8_t message[DATA_SIZE];
int16_t dataBuffer[NUM_SETS * 6];  // Buffer to store 20 datasets of 6-axis data (int16_t)

void QSPIF_sleep(void) {
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delayMicroseconds(5);
  flashTransport.end();
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(customService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // Advertising interval: 20 ms to 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);    // Fast advertising for 30 seconds
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising
}

// Callback for connection
void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected");

  // Request for a higher MTU size
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);

  // Request for a better connection interval
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(6, 8);  // 7.5 - 10 ms connection interval  6*1.25=7.5   8*1.25=10ms
}

// Callback for disconnection
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}

void imuSetup() {
  Serial.println("Initializing IMU...");
  lsm6ds3.begin();

  uint8_t dataToWrite = 0;

  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
  lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
  lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);  // High-performance operating mode disabled for accelerometer

  Serial.println("IMU initialized.");
}

void setup(void) {
  Serial.begin(115200);
  Serial.println("Setup starting...");

  NRF_POWER->DCDCEN = 1;
  QSPIF_sleep();

  imuSetup();

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Set transmission power to maximum
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);


  customService.begin();
  customCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  customCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);  // No security required
  customCharacteristic.setFixedLen(DATA_SIZE);                     // Fixed length of 20 bytes
  customCharacteristic.begin();


  startAdv();
}

void loop(void) {
  unsigned long currentTime = millis();

  // Check if BLE is connected and it's time to send data
  if (Bluefruit.connected() && (currentTime - lastSendTime >= SEND_INTERVAL)) {
    lastSendTime = currentTime;

    for (int set = 0; set < NUM_SETS; set++) {
      uint16_t ax = lsm6ds3.readRawAccelX();
      uint16_t ay = lsm6ds3.readRawAccelY();
      uint16_t az = lsm6ds3.readRawAccelZ();
      uint16_t gx = lsm6ds3.readRawGyroX();
      uint16_t gy = lsm6ds3.readRawGyroY();
      uint16_t gz = lsm6ds3.readRawGyroZ();

      // Store the raw data in the message buffer
      message[set * 12 + 0] = ax & 0xFF;
      // Serial.println( message[set * 12 + 0],HEX);
      // char str[100];
      // snprintf(str, sizeof(str), "%u",  message[set * 12 + 0]);
      // Serial.println("Value of String: ");
      // Serial.println(str);
      message[set * 12 + 1] = (ax >> 8) & 0xFF;
      message[set * 12 + 2] = ay & 0xFF;
      message[set * 12 + 3] = (ay >> 8) & 0xFF;
      message[set * 12 + 4] = az & 0xFF;
      message[set * 12 + 5] = (az >> 8) & 0xFF;
      message[set * 12 + 6] = gx & 0xFF;
      message[set * 12 + 7] = (gx >> 8) & 0xFF;
      message[set * 12 + 8] = gy & 0xFF;
      message[set * 12 + 9] = (gy >> 8) & 0xFF;
      message[set * 12 + 10] = gz & 0xFF;
      message[set * 12 + 11] = (gz >> 8) & 0xFF;
    }
    // Update the characteristic value
    customCharacteristic.notify(message, sizeof(message));
  }
}
