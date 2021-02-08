/**
   A sketch that searches for a compatible Airthings device and 
   publishes the radon level, temperature, and humidity to an MQTT 
   server.

   The sketch was created with the intention to allow Airthings devices
   to cheaply integrate with Home Assistant.

   To use:
   (1) Set up your Airthings following the manufacter's instructions.
   (2) Install the PunSubClient library (https://pubsubclient.knolleary.net/).
   (3) Set your WiFi credentials below.
   (4) Set your MQTT server/credentials below.
   (5) Update the published topics below (if desired).
   (6) Flash to any ESP32 board.
   (7) Watch the Serial output to make sure it works.

   * The library runs once an hour to take a reading and deep sleeps in 
     between, so feasibly this could run on a battery for a very long time.
   * The library will attempt to find any airthings device to read from, 
     picking the first it finds.  The Airthings BLE API is unauthenticated 
     so no device configuration or pairing is necessary on the Airthings.
   * The library will not interfere with your Airthings' normal upload to a 
     phone/cloud.
   * If it fails to read, it will attempt again after 30 seconds instead.
   * I only have an Airthings Wave to test this with, though presumably it 
     would also work with the Wave Plus.
   * The ESP32's bluetooth stack is a little unstable IMHO so expect this to
     hang for a few minutes, restart prematurely, and report errors often.
*/
#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials.
#define WIFI_SSID "YOUR SSID"
#define WIFI_PASS "YOUR PASSWORD"

// MQTT Settings.
#define MQTT_HOST "YOUR HOST"
#define MQTT_PORT 1883
#define MQTT_USER "YOUR USERNAME"
#define MQTT_PASS "YOUR PASSWORD"
#define MQTT_CLIENT "radon_client"

// The MQTT topic to publish a 24 hour average of radon levels to.
#define TOPIC_RADON_24HR "stat/airthings/radon24hour"
// The MQTT topic to publish the lifetime radon average to.  Documentation
// says this will be the average ever since the batteries were removed.
#define TOPIC_RADON_LIFETIME "stat/airthings/radonLifetime"
// Topics for temperature and humidity.
#define TOPIC_TEMPERATURE "stat/airthings/temperature"
#define TOPIC_HUMIDITY "stat/airthings/humidity"
#define TOPIC_PRESSURE "stat/airthings/pressure"
#define TOPIC_VOC "stat/airthings/voc"
#define TOPIC_CO2 "stat/airthings/co2"

// Unlikely you'll need to change any of the settings below.

// The time to take between readings.  One hour has worked pretty well for me.  
// Since the device only gives us the 24hr average, more frequent readings 
// probably wouldn't be useful, run the airthings battery down, and risk 
// interfering with the "normal" mechanism Airthings uses to publish info
// to your phone.
#define READ_WAIT_SECONDS 60*60 // One hour

// If taking a reading fails for any reason (BLE is pretty flaky...) then
// the ESP will sleep for this long before retrying.
#define READ_WAIT_RETRY_SECONDS 30

// How long the ESP will wait to connect to WiFi, scan for 
// Airthings devices, etc.
#define CONNECT_WAIT_SECONDS 30

// Some useful constants.
#define uS_TO_S_FACTOR 1000000
#define SECONDS_TO_MILLIS 1000
#define BECQUERELS_M2_TO_PICOCURIES_L 37.0
#define DOT_PRINT_INTERVAL 50

// The hard-coded uuid's airthings uses to advertise itself and its data.
static BLEUUID serviceUUID("b42e1c08-ade7-11e4-89d3-123b93f75cba");             // (Found in ??)
static BLEUUID currentValuesUUID("b42e2a68-ade7-11e4-89d3-123b93f75cba");       // (Found in https://github.com/Airthings/waveplus-reader/blob/master/read_waveplus.py)

int unpack(int data1, int data2) {
  int value = data2;
  value = value<<8;
  value += data1;
  return value;
}

bool getAndRecordReadings(BLEAddress pAddress) {
  Serial.println();
  Serial.println("Connecting...");
  BLEClient* client = BLEDevice::createClient();

  // Connect to the remove BLE Server.
  if (!client->connect(pAddress)) {
    Serial.println("Failed to connect.");
    return false;
  }

  Serial.println("Connected!");
  // Obtain a reference to the service we are after in the remote BLE server.
  Serial.println("Retrieving service reference...");
  BLERemoteService* pRemoteService = client->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Airthings refused its service UUID.");
    client->disconnect();
    return false;
  }

  // Get references to our characteristics
  Serial.println("Reading radon/temperature/humidity/pressure/CO2/VOC...");
  BLERemoteCharacteristic* currentValuesCharacteristic = pRemoteService->getCharacteristic(currentValuesUUID);

  if (currentValuesCharacteristic == nullptr) {
    Serial.print("Failed to read from the device!");
    return false;
  }

  std::string data = currentValuesCharacteristic->readValue();

  // Data format BBBBHHHHHHHH

  //  0       - byte dataVersion;
  //  1       - byte humidity;
  //  2       - byte ambientLight;
  //  3       - byte waves;
  //  4 +  5  - int radon;
  //  6 +  7  - int radonLongTerm;
  //  8 +  9  - int temperature;
  // 10 + 11  - int pressure;
  // 12 + 13  - int co2;
  // 14 + 15  - int voc;

  float val;
  Serial.print("Data version: ");
  Serial.println(data[0], DEC);
  
  Serial.print("Humidity: ");
  val = data[1];
  val /= 2.0;
  int humidity = val;
  Serial.println(humidity, DEC);

  Serial.print("Radon: ");
  int radon = unpack(data[4], data[5]);

  if ((radon <= 0) || (radon > 16383)) {
     radon = 0;
  }

  Serial.println(radon, DEC);

  Serial.print("Radon long term: ");
  int radonLongterm = unpack(data[6], data[7]);

  if ((radonLongterm <= 0) || (radonLongterm > 16383)) {
     radonLongterm = 0;
  }

  Serial.println(radonLongterm, DEC);

  Serial.print("Temperature: ");
  val = unpack(data[8], data[9]);
  val /= 100;
  int temperature = val;
  Serial.println(temperature, DEC);

  Serial.print("Pressure: ");
  val = unpack(data[10], data[11]);
  val /= 50;
  int pressure = val;
  Serial.println(pressure, DEC);

  Serial.print("CO2: ");
  int co2 = unpack(data[12], data[13]);
  Serial.println(co2, DEC);

  Serial.print("VOC: ");
  int voc = unpack(data[14], data[15]);
  Serial.println(voc, DEC);

  client->disconnect();

  Serial.printf("So far so good");
  //Serial.printf("Temperature: %f\n", temperature);
  //Serial.printf("Humidity: %f\n", humidity);
  //Serial.printf("Radon 24hr average: %f\n", radon);
  //Serial.printf("Radon Lifetime average: %f\n", radonLongterm);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() < start + CONNECT_WAIT_SECONDS * SECONDS_TO_MILLIS) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to wifi");
    return false;
  }

  // Connect and publish to MQTT.
  WiFiClient espClient;
  PubSubClient mqtt(espClient);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  if (!mqtt.connect("RADON_CLIENT", MQTT_USER, MQTT_PASS) ||
      !mqtt.publish(TOPIC_RADON_24HR, String(radon).c_str()) ||
      !mqtt.publish(TOPIC_RADON_LIFETIME, String(radonLongterm).c_str()) ||
      !mqtt.publish(TOPIC_TEMPERATURE, String(temperature).c_str()) ||
      !mqtt.publish(TOPIC_HUMIDITY, String(humidity).c_str()) ||
      !mqtt.publish(TOPIC_PRESSURE, String(pressure).c_str()) ||
      !mqtt.publish(TOPIC_VOC, String(voc).c_str()) ||
      !mqtt.publish(TOPIC_CO2, String(co2).c_str())) {
    Serial.println("Unable to connect/publish to mqtt server.");
    return false;
  }
  return true;
}

// The bluetooth stack takes a callback when scannign for devices.  The first Airthings device it finds it will record in pServerAddress.
class FoundDeviceCallback: public BLEAdvertisedDeviceCallbacks {
  public: 
  BLEAddress* address;
  bool found = false;
  bool foundAirthings() {
    return found;
  }
  BLEAddress getAddress() {
    return *address;
  }
  void onResult(BLEAdvertisedDevice device) {
    // We have found a device, see if it has the Airthings service UUID
    if (device.haveServiceUUID() && device.getServiceUUID().equals(serviceUUID)) {
      Serial.print("Found our device: ");
      Serial.println(device.toString().c_str());
      device.getScan()->stop();
      address = new BLEAddress(device.getAddress());
      found = true;
    }
  }
};

void setup() {
  Serial.begin(115200);

  // Start up WiFi early so it'll probably be ready by the time 
  // we're reading from Airthings.
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Scan for an Airthings device.
  Serial.println("Scanning for airthings devices");
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  FoundDeviceCallback* callback = new FoundDeviceCallback();
  pBLEScan->setAdvertisedDeviceCallbacks(callback);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);

  unsigned long timeToSleep = 0;
  if (!callback->foundAirthings()) {
    // We timed out looking for an Airthings.
    Serial.printf("\nFAILED to find any Airthings devices. Sleeping for %i seconds before retrying.\n", READ_WAIT_RETRY_SECONDS);
    timeToSleep = READ_WAIT_RETRY_SECONDS;
  } else if (getAndRecordReadings(callback->getAddress())) {
    Serial.printf("\nReading complete. Sleeping for %i seconds before taking another reading.\n", READ_WAIT_SECONDS);
    timeToSleep = READ_WAIT_SECONDS;
  } else {
    Serial.printf("\nReading FAILED. Sleeping for %i seconds before retrying.\n", READ_WAIT_RETRY_SECONDS);
    timeToSleep = READ_WAIT_RETRY_SECONDS;
  }
  Serial.flush();
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // We should never reach here.
  delay(1);
}
