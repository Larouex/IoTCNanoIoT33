/* ==========================================================================
    File:     main.cpp
    Author:   Larry W Jordan Jr (larouex@gmail.com)
    Purpose:  Arduino Nano IoT 33 example for connecting to Azure IoT Central.
              The scenario enables Bluetooth Connectivity to a Nano BLE 33
              device to connect and collect telemetry. That data is sent
              Azure IoT Central acting as a transparent IoT Gateway.

    Online:   www.hackinmakin.com

    (c) 2020 Larouex Software Design LLC
    This code is licensed under MIT license (see LICENSE.txt for details)    
  ==========================================================================*/
#include <Arduino.h>
#undef max
#undef min

#include "config.h"
//#include "az_iot_helpers.h"

#include <WiFiNINA.h>
#include <PubSubClient.h>

#include "WiFiUdp.h"
#include "NTP.h"

#include <az_json.h>
#include <az_result.h>
#include <az_span.h>
#include <az_iot_hub_client.h>

//#include "mbedtls/md.h"
//#include "mbedtls/base64.h"
//#include "mbedtls/sha256.h"

#include <SPI.h>
#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <string>
#include <ArduinoBLE.h>

// MACROS for Reading Build Flags
#define XSTR(x) #x
#define STR(x) XSTR(x)


/* --------------------------------------------------------------------------
    WiFi Details
   -------------------------------------------------------------------------- */
char ssid[] = IOT_CONFIG_WIFI_SSID;
char pass[] = IOT_CONFIG_WIFI_PASSWORD;
int status = WL_IDLE_STATUS;
WiFiSSLClient client;

/* --------------------------------------------------------------------------
    We use this to delay the start when the BLE Central application/gateway
    is interogating the device...
   -------------------------------------------------------------------------- */
unsigned long bleStartDelay   = 0;
bool          bleDelayActive  = false;

/* --------------------------------------------------------------------------
    Frequency of Simulation on Battery Level
   -------------------------------------------------------------------------- */
unsigned long   telemetryStartDelay     = 0;
bool            telemetryDelayActive    = false;
unsigned long   telemetryFrequency      = 1500;

/* --------------------------------------------------------------------------
    We flash the RTGB to show ready to connect Green/Blue...
   -------------------------------------------------------------------------- */
unsigned long readyToConnectDelay       = 0;
bool          readyToConnectDelayActive = false;

/* --------------------------------------------------------------------------
    Configure IMU
   -------------------------------------------------------------------------- */
const int IMU_HZ = 119;
Madgwick filter;
unsigned long msecsPerReading, msecsPrevious;

/* --------------------------------------------------------------------------
    Leds we manipulate for Status, etc.
   -------------------------------------------------------------------------- */
#define ONBOARD_LED     13

/* --------------------------------------------------------------------------
    Characteristic Mappers
   -------------------------------------------------------------------------- */
enum CHARACTERISTICS: int {
  VERSION_CHARACTERISTIC = 1,
  BATTERYCHARGED_CHARACTERISTIC,
  TELEMETRYFREQUENCY_CHARACTERISTIC,
  ACCELEROMETER_CHARACTERISTIC,
  GYROSCOPE_CHARACTERISTIC,
  DCM_CHARACTERISTIC
};

/* --------------------------------------------------------------------------
    Previous Battery Level Monitors
   -------------------------------------------------------------------------- */
int oldBatteryLevel = 0;

/* --------------------------------------------------------------------------
    Broadcast Version for BLE Device
   -------------------------------------------------------------------------- */
const unsigned char SEMANTIC_VERSION[14] = STR(VERSION);

/* --------------------------------------------------------------------------
    Broadcast of the Device Capability Model for BLE Device
   -------------------------------------------------------------------------- */
const unsigned char DEVICE_CAPABILITY_MODEL[256] = STR(DCM);

/* --------------------------------------------------------------------------
    BLE Service Definition UUID and DEVICE NAME
   -------------------------------------------------------------------------- */
BLEService blePeripheral(STR(SERVICE_UUID)); 

// ************************* BEGIN CHARACTERISTICS **************************

/* --------------------------------------------------------------------------
    BLE Peripheral Characteristics - Readable by Central/Gateway
   -------------------------------------------------------------------------- */
// Version
BLECharacteristic       versionCharacteristic("1001", BLERead | BLEWrite, sizeof(SEMANTIC_VERSION));
BLEDescriptor           versionCharacteristicDesc ("2901", "Version");

// Battery Charged
BLEFloatCharacteristic  batteryChargedCharacteristic("2001", BLERead);
BLEDescriptor           batteryChargedCharacteristicDesc ("2901", "Battery Charged");

// Telemetry Frequency
BLEIntCharacteristic    telemetryFrequencyCharacteristic("3001", BLERead | BLEWrite);
BLEDescriptor           telemetryFrequencyCharacteristicDesc ("2901", "Telemetry Frequency");

// Accelerometer
BLECharacteristic       accelerometerCharacteristic("4001", BLENotify, 3 * sizeof(float));
BLEDescriptor           accelerometerCharacteristicDesc ("2901", "Accelerometer");

// Gyroscope
BLECharacteristic       gyroscopeCharacteristic("5001", BLENotify, 3 * sizeof(float));
BLEDescriptor           gyroscopeCharacteristicDesc ("2901", "Gyroscope");


// Device Capability Model
BLECharacteristic       dcmCharacteristic("9901", BLERead, sizeof(DEVICE_CAPABILITY_MODEL));
BLEDescriptor           dcmCharacteristicDesc("2901", "Device Capability Model");



// ************************** END CHARACTERISTICS ***************************

/* --------------------------------------------------------------------------
    Function to set the onboard Nano RGB LED
   -------------------------------------------------------------------------- 
void SetBuiltInRGB(
  PinStatus redLightPinValue,
  PinStatus blueLightPinValue,
  PinStatus greenLightPinValue)
{
     digitalWrite(RED_LIGHT_PIN, redLightPinValue);
     digitalWrite(BLUE_LIGHT_PIN, blueLightPinValue);
     digitalWrite(GREEN_LIGHT_PIN, greenLightPinValue);    
    return;
}

*/

/* --------------------------------------------------------------------------
    Function to set the RGB LED to the color of the battery charge
      * Green >=50%
      * Yellow <= 49% && >=20%
      * Red <=19%
   -------------------------------------------------------------------------- */
void BatteryCheck(int level) {
  if (level >=5 )
  {
    Serial.println("BatteryCheck Set Green");
    //SetBuiltInRGB(HIGH, HIGH, LOW);
  }
  else if (level >=2 and level <= 4 )
  {
    Serial.println("BatteryCheck Set Yellow");
    //SetBuiltInRGB(LOW, HIGH, LOW);
  }
  else
  {
    Serial.println("BatteryCheck Set Red");
    //SetBuiltInRGB(LOW, HIGH, HIGH);
  }
  return;
}

/* --------------------------------------------------------------------------
    Function to read the following IMU capabilities
      Accelerometer
      Gyroscope
      MagneticField
      Orientation
   -------------------------------------------------------------------------- */
void UpdateIMU() {
  
  float acceleration[3];
  float gyroDPS[3];

  if (IMU.accelerationAvailable()) {
    
    // read the Accelerometer
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    acceleration[0] = x;
    acceleration[1] = y;
    acceleration[2] = z;

    if (accelerometerCharacteristic.subscribed()) {
      accelerometerCharacteristic.writeValue(acceleration, sizeof(acceleration));
      Serial.print("[IMU] Acceleration(X): ");
      Serial.println(acceleration[0]);
      Serial.print("[IMU] Acceleration(Y): ");
      Serial.println(acceleration[1]);
      Serial.print("[IMU] Acceleration(Z): ");
      Serial.println(acceleration[2]);
    }
    #ifdef DEBUG
      if (!accelerometerCharacteristic.subscribed())
      {
        Serial.println("[IMU] Please Subscribe to Acceleration for Notifications");
      }
    #endif
  }

  if (IMU.gyroscopeAvailable()) {
    // read the Gyro
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    gyroDPS[0] = x;
    gyroDPS[1] = y;
    gyroDPS[2] = z;

    if (gyroscopeCharacteristic.subscribed()) {
      gyroscopeCharacteristic.writeValue(gyroDPS, sizeof(gyroDPS));
      Serial.print("[IMU] Gyroscope(X): ");
      Serial.println(gyroDPS[0]);
      Serial.print("[IMU] Gyroscope(Y): ");
      Serial.println(gyroDPS[1]);
      Serial.print("[IMU] Gyroscope(Z): ");
      Serial.println(gyroDPS[2]);
    }

    #ifdef DEBUG
      if (!gyroscopeCharacteristic.subscribed())
      {
        Serial.println("[IMU] Please Subscribe to Gyroscope for Notifications");
      }
    #endif
  }

}

/* --------------------------------------------------------------------------
    Read the current voltage level on the A0 analog input pin.
    This is used here to simulate the charge level of a battery.
   -------------------------------------------------------------------------- */
void UpdateBatteryLevel() {
  
  int batteryLevel = 1 + rand() % 10;

  // only if the battery level has changed
  if (batteryLevel != oldBatteryLevel) {      
    Serial.print("Battery Level % is now: ");
    Serial.println(batteryLevel);
    // and update the battery level characteristic to BLE
    batteryChargedCharacteristic.writeValue(batteryLevel);
    oldBatteryLevel = batteryLevel;
    BatteryCheck(batteryLevel);
  }

  // Enable a Pause to Show this Write
  telemetryStartDelay = millis();
  telemetryDelayActive = true;
}

// ************************* BEGIN EVENT HANDLERS ***************************

/* --------------------------------------------------------------------------
    TELEMETRY_FREQUENCY (write) Event Handler from Central/Gateway
   -------------------------------------------------------------------------- */
void onTelemetryFrequencyCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  
  unsigned long val = telemetryFrequencyCharacteristic.value();
  
  if (val >= 500 && val <= 30000) {
    telemetryFrequency = val;
    Serial.print("[EVENT] telemetryFrequency: ");
    Serial.println(telemetryFrequency);
  } else {
    Serial.print("[EVENT] telemetryFrequency MUST BE BETWEEN 500 AND 30000: ");
    Serial.println(telemetryFrequency);
  }

}


// ************************** END EVENT HANDLERS ****************************

/* --------------------------------------------------------------------------
    This Function acts as a positive indicater (true) when the 
    characteristic is setup properly
   -------------------------------------------------------------------------- */
bool SetUpCharacteristic(int whichCharacteristic)
{
  bool result = false;
  switch (whichCharacteristic) {

    case VERSION_CHARACTERISTIC:
      blePeripheral.addCharacteristic(versionCharacteristic); 
      versionCharacteristic.addDescriptor(versionCharacteristicDesc);
      versionCharacteristic.setValue(SEMANTIC_VERSION, 14);
      result = true;
      break;

    case BATTERYCHARGED_CHARACTERISTIC:
      blePeripheral.addCharacteristic(batteryChargedCharacteristic); 
      batteryChargedCharacteristic.addDescriptor(batteryChargedCharacteristicDesc);
      batteryChargedCharacteristic.writeValue(oldBatteryLevel);
      batteryChargedCharacteristic.broadcast();
      result = true;
      break;

    case TELEMETRYFREQUENCY_CHARACTERISTIC:
      blePeripheral.addCharacteristic(telemetryFrequencyCharacteristic); 
      telemetryFrequencyCharacteristic.addDescriptor(telemetryFrequencyCharacteristicDesc);
      telemetryFrequencyCharacteristic.setValue(telemetryFrequency);
      telemetryFrequencyCharacteristic.setEventHandler(BLEWritten, onTelemetryFrequencyCharacteristicWrite);
      result = true;
      break;
  
    case ACCELEROMETER_CHARACTERISTIC:
      blePeripheral.addCharacteristic(accelerometerCharacteristic); 
      accelerometerCharacteristic.addDescriptor(accelerometerCharacteristicDesc);
      result = true;
      break;

    case GYROSCOPE_CHARACTERISTIC:
      blePeripheral.addCharacteristic(gyroscopeCharacteristic); 
      gyroscopeCharacteristic.addDescriptor(gyroscopeCharacteristicDesc);
      result = true;
      break;

    case DCM_CHARACTERISTIC:
      blePeripheral.addCharacteristic(dcmCharacteristic); 
      dcmCharacteristic.addDescriptor(dcmCharacteristicDesc);
      dcmCharacteristic.setValue(DEVICE_CAPABILITY_MODEL, 256);
      result = whichCharacteristic;
      break;

    default:
      break;
  }
  return result;

}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP(); // Device IP address
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void connectToAP() {
  // Try to connect to Wifi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);

    // wait 1 second for connection:
    delay(1000);
    Serial.println("Connected...");
  }
}

/* --------------------------------------------------------------------------
    Standard Sketch Setup
   -------------------------------------------------------------------------- */
void setup() {
  
  // Setup our Pins
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize serial communication
  Serial.begin(9600);    
  //while (!Serial);
    
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  connectToAP();    // Connect to Wifi Access Point
  printWifiStatus();
  
  // begin IMU initialization
  if (!IMU.begin()) {
    Serial.println("[FAILED] Starting IMU");
    while (1);
  } else {
    Serial.println("[SUCCESS] Starting IMU");
    
    // start the MadgwickAHRS filter to run at the IMU sample rate
    filter.begin(IMU_HZ);
    msecsPerReading = 1000000 / IMU_HZ;
    msecsPrevious = micros();
  }

  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("[FAILED] Starting BLE");
    while (1);
  } else {
    Serial.println("[SUCCESS] Starting BLE");
  }
  
  // Setup the Characteristics
  if (!SetUpCharacteristic(VERSION_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->VERSION_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->VERSION_CHARACTERISTIC");
  }
  
  if (!SetUpCharacteristic(BATTERYCHARGED_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->BATTERYCHARGED_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->BATTERYCHARGED_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(TELEMETRYFREQUENCY_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->TELEMETRYFREQUENCY_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->TELEMETRYFREQUENCY_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(ACCELEROMETER_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->ACCELEROMETER_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->ACCELEROMETER_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(GYROSCOPE_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->GYROSCOPE_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->GYROSCOPE_CHARACTERISTIC");
  }

  if (!SetUpCharacteristic(DCM_CHARACTERISTIC)) {
    Serial.println("[FAILED] SetUpCharacteristic->DCM_CHARACTERISTIC");
  } else {
    Serial.println("[SUCCESS] SetUpCharacteristic->DCM_CHARACTERISTIC");
  }

  
  /* 
    Set a local name for the BLE device
    This name will appear in advertising packets
    and can be used by remote devices to identify this BLE device
    The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName(STR(DEVICE_NAME));
  BLE.setDeviceName(STR(DEVICE_NAME));
  byte data[5] = { 0x01, 0x02, 0x03, 0x04, 0x05};
  BLE.setManufacturerData(data, 5);
  BLE.setAdvertisedServiceUuid(STR(SERVICE_UUID));
  BLE.setAdvertisedService(blePeripheral);
  BLE.addService(blePeripheral);

    /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  BLE.advertise();

  readyToConnectDelay = millis();
  readyToConnectDelayActive = true;

  // Set leds to idle
  digitalWrite(LED_BUILTIN, HIGH);
  //SetBuiltInRGB(HIGH, LOW, HIGH);

  Serial.println("[READY] Bluetooth device active, waiting for connections...");

  return;
}

/* --------------------------------------------------------------------------
    Standard Sketch Loop
   -------------------------------------------------------------------------- */
void loop() {

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  
  if (readyToConnectDelayActive && ((millis() - readyToConnectDelay) >= 1000)) {
    //SetBuiltInRGB(HIGH, HIGH, LOW);
    readyToConnectDelay = millis();
    readyToConnectDelayActive = false;
  }
  else if (!readyToConnectDelayActive && ((millis() - readyToConnectDelay) >= 1000)) {
    //SetBuiltInRGB(LOW, HIGH, HIGH);
    readyToConnectDelay = millis();
    readyToConnectDelayActive = true;
  }

  // if a central is connected to peripheral:
  if (central) {

    bleStartDelay = millis();
    bleDelayActive = true;
    telemetryStartDelay = millis();
    telemetryDelayActive = true;

    digitalWrite(ONBOARD_LED, HIGH);

    // print the central's MAC address:
    Serial.print("[STARTED] Connected to central: ");
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      
      if (bleDelayActive && ((millis() - bleStartDelay) >= 5000)) {
        bleDelayActive = false;
      }

      if (!bleDelayActive) {
        if (telemetryDelayActive && ((millis() - telemetryStartDelay) >= telemetryFrequency)) {
          UpdateBatteryLevel();
          UpdateIMU();
          telemetryStartDelay = millis();
        }
      }

    }

    // when the central disconnects, print it out:
    digitalWrite(ONBOARD_LED, LOW);
    //SetBuiltInRGB(LOW, LOW, LOW);

    Serial.print(F("[STOPPED] Disconnected from central: "));
    Serial.println(central.address());
  }
}
