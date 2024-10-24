/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-web-bluetooth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// Servo control using ESP32's LEDC
const int servoPin = D1;       // GPIO pin connected to the servo signal
const int pwmChannel = 0;      // PWM channel to use for the servo
const int pwmFrequency = 50;   // Frequency for the servo (50 Hz is typical for servos)
const int pwmResolution = 10;  // Resolution of PWM signal (10-bit for finer control)

// Pulse width calculations for a 10-bit resolution at 50Hz
const int minDutyCycle = 26;   // Corresponds to 500us pulse width (~0 degrees)
const int maxDutyCycle = 128;  // Corresponds to 2500us pulse width (~180 degrees)

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10010-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10012-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10011-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string value = pLedCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0]));  // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {
        Serial.println("Press once");

        // Map the value of 180 degrees to the duty cycle range
        int dutyCycle = map(180, 0, 180, minDutyCycle, maxDutyCycle);
        ledcWrite(pwmChannel, dutyCycle);
        delay(500);
        dutyCycle = map(90, 0, 180, minDutyCycle, maxDutyCycle);
        ledcWrite(pwmChannel, dutyCycle);
      } else if (receivedValue == 0){
        Serial.println("Setting servo to 180 degrees...");

        // Map the value of 180 degrees to the duty cycle range
        int dutyCycle = map(180, 0, 180, minDutyCycle, maxDutyCycle);
        ledcWrite(pwmChannel, dutyCycle);
        delay(2000);
        dutyCycle = map(90, 0, 180, minDutyCycle, maxDutyCycle);
        ledcWrite(pwmChannel, dutyCycle);
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  
  // Configure the PWM channel for the servo
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(servoPin, pwmChannel);

  // Initialize the servo to its original position (90 degrees)
  int initialDutyCycle = map(90, 0, 180, minDutyCycle, maxDutyCycle);
  ledcWrite(pwmChannel, initialDutyCycle);
  Serial.println("Servo initialized to 90 degrees.");

  // Create the BLE Device
  BLEDevice::init("Flash");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
    LED_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    value++;
    Serial.print("New value notified: ");
    Serial.println(value);
    delay(3000);  // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}
