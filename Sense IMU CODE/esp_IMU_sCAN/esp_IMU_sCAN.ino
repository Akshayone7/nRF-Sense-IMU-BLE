/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
static int ledPin = 2;
int scanTime = 5;  //In seconds
static boolean doConnect = false;
static boolean doScan = false;
static boolean connected = false;
BLEScan* pBLEScan;
static BLERemoteCharacteristic* pTPVCharacteristic;
static BLEAdvertisedDevice* myDevice;
String targetMac = "d1:d6:24:02:61:2e";

static BLEUUID SERVICE_UUID("12345678-1234-5678-1234-56789abcdef0");
static BLEUUID CHAR_UUID("abcdef01-1234-5678-1234-56789abcdef1");


class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    if (advertisedDevice.getAddress().toString() == targetMac) {
      Serial.print("Found BLE Advertised Device: ");
      Serial.println(advertisedDevice.toString().c_str());
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};




bool connectToServer() {
  Serial.print("Connecting to BLE Server: ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println("Failed to connect to server. Reconnecting...");
    delay(1000);
    return false;
  }
  Serial.println("Connected to server.");

  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service UUID: ");
    Serial.println(SERVICE_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found service.");

  pTPVCharacteristic = pRemoteService->getCharacteristic(CHAR_UUID);
  if (pTPVCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic UUID: ");
    Serial.println(CHAR_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found characteristic.");

  delay(1000);

  connected = true;
  return true;
}


void readCharacteristicValue() {
  if (pTPVCharacteristic->canRead()) {
    String value = pTPVCharacteristic->readValue();
    Serial.print("Characteristic value: ");
    Serial.print(" = ");
    // Serial.print("The characteristic value was: ");
    for (int i = 0; i < value.length(); i++) {
      Serial.print(value[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  } else {
    Serial.println("Failed to read characteristic value!");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");
  pinMode(ledPin, OUTPUT);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  Serial.println("In CAllback");
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);  // less or equal setInterval value
  pBLEScan->start(0, false);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to BLE Server");
    } else {
      Serial.println("Failed to Connect");
    }
    doConnect = false;
  }

  if (connected) {
    readCharacteristicValue();
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
  delay(1000);
}
