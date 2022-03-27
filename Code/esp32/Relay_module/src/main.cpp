#include <Arduino.h>

#include "BLEDevice.h"
#include "EspMQTTClient.h"
//#include "BLEScan.h"

#include <EEPROM.h>

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <esp_task_wdt.h>

uint32_t succes_count = 0;

AsyncWebServer server(80);

boolean websiteEnd = false;
boolean isInit = false;
boolean relay_on = false;
boolean temperature_received = false;

const char* ssid     = "Bridge config";
const char* password = "123456789";

String variable1 = "wifi_ssid"; 
String variable2 = "wifi_passwd"; 
String variable3 = "mqtt_ip"; 
String variable4 = "mqtt_login"; 
String variable5 = "mqtt_passwd"; 
String variable6 = "mqtt_name";

String wifi_ssid = ""; 
String wifi_passwd = ""; 
String mqtt_ip = ""; 
String mqtt_login = ""; 
String mqtt_passwd = ""; 
String mqtt_name = "";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Bridge config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get">
    Wi-Fi SSID: <input type="text" name="wifi_ssid"><br><br>
    Wi-Fi password: <input type="text" name="wifi_passwd"><br><br>
    MQTT broker IP: <input type="text" name="mqtt_ip"><br><br>
    MQTT login: <input type="text" name="mqtt_login"><br><br>
    MQTT password: <input type="text" name="mqtt_passwd"><br><br>
    MQTT client name: <input type="text" name="mqtt_name"><br><br>
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

EspMQTTClient client;

// The remote service we wish to connect to.
static BLEUUID    serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
// The characteristics of the remote services we are interested in.
static BLEUUID    RxUUID("0000ffe4-0000-1000-8000-00805f9b34fb");
static BLEUUID    TxUUID("0000ffe9-0000-1000-8000-00805f9b34fb");

static boolean doConnectBLE = false; 
static boolean connectedBLE = false;
static boolean doScanBLE = true;

static BLERemoteCharacteristic* pRemoteCharacteristicNotify;
static BLERemoteCharacteristic* pRemoteCharacteristicWrite;
static BLEAdvertisedDevice* myDevice;

String mqtt_message;

float temperature;
float target_temp;
float temp_hist;

uint8_t relay_pin = 27;
uint8_t button_pin = 0; 

boolean reset_eeprom = false;


uint32_t button_start_time; 
uint32_t button_stop_time;

void IRAM_ATTR button_isr()
{
  if(digitalRead(0) == 0)
  {
    button_start_time = millis();
    relay_on = !relay_on;
  }
  else
  {
    button_stop_time = millis();
    if(button_stop_time - button_start_time >= 5000)
    {
      reset_eeprom = true;
    }
    button_start_time = 0;
    button_stop_time = 0;
  }
}

//website
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void server_setup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {

    if (request->params() == 6) {
      wifi_ssid = request->getParam(variable1)->value();
      wifi_passwd = request->getParam(variable2)->value();
      mqtt_ip = request->getParam(variable3)->value();
      mqtt_login = request->getParam(variable4)->value();
      mqtt_passwd = request->getParam(variable5)->value();
      mqtt_name = request->getParam(variable6)->value();

      request->send(200, "text/html", "Data applied");
      delay(1000);
      server.end();
      WiFi.softAPdisconnect();
      websiteEnd = true;
    }
    else {
      request->send(200, "text/html", "Something went wrong.<br><a href=\"/\">Return to Home Page</a>");
    }

  });
  server.onNotFound(notFound);
  server.begin();
}
//website end

//MQTT
// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  client.subscribe("Thermostat/Settings/TargetTemp", [](const String & payload) {
    target_temp = payload.toFloat();
  });

  client.subscribe("Thermostat/Settings/TempHist", [](const String & payload) {
    temp_hist = payload.toFloat();
  });
}
//MQTT end


//BLE
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

    memcpy(&temperature, pData, length);
    
    if (client.isMqttConnected())
    {
      if (temperature < target_temp - temp_hist)
      {
        relay_on = true;
      }
      else if (temperature > target_temp + temp_hist)
      {
        relay_on = false;
      }
    }

    temperature_received = true;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connectedBLE = false;
    doScanBLE = true;
    Serial.println("Disconnected");
    
    mqtt_message = String(temperature, 1);

    delay(500);

    if (client.isMqttConnected())
    {
      if (temperature_received)
      {
        client.publish("Thermostat/Temperature", mqtt_message);
        if (relay_on)
        {
          client.publish("Thermostat/Relay", "ON");
        }
        else
        {
          client.publish("Thermostat/Relay", "OFF");
        }
      }
      temperature_received = false;
    }

    
    delay(2000);
  }
};

bool connectToServer() {
    esp_task_wdt_init(5000, true);
    esp_task_wdt_add(NULL);
    
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    if(!pClient->connect(myDevice))
    {
      Serial.println("Failed to connect");

      return false;
    }
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicNotify = pRemoteService->getCharacteristic(RxUUID);
    if (pRemoteCharacteristicNotify == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(RxUUID.toString().c_str());
      pClient->disconnect();
      
      return false;
    }
    Serial.println(" - Found Notify characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristicNotify->canRead()) {
      std::string value = pRemoteCharacteristicNotify->readValue();
      Serial.print("The notify characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristicNotify->canNotify())
      pRemoteCharacteristicNotify->registerForNotify(notifyCallback);

    pRemoteCharacteristicWrite = pRemoteService->getCharacteristic(TxUUID);
    if (pRemoteCharacteristicWrite == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(TxUUID.toString().c_str());
      pClient->disconnect();
      
      return false;
    }
    Serial.println(" - Found Write characteristic");

    connectedBLE = true;
    
    esp_task_wdt_reset();
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    
    if (advertisedDevice.getName() == "TempSens") {
      BLEDevice::getScan()->stop();
      doScanBLE = false;
      doConnectBLE = true;
      Serial.print("Found our BLE device: ");
      Serial.println(advertisedDevice.toString().c_str());
      myDevice = new BLEAdvertisedDevice(advertisedDevice);    
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks
//BLE end

void setup() {
  uint8_t len;

  Serial.begin(115200);

  EEPROM.begin(122);

  pinMode(0, INPUT_PULLUP);
  attachInterrupt(0, button_isr, CHANGE);
  pinMode(relay_pin, OUTPUT);

  isInit = EEPROM.readBool(0);

  delay(500);

  Serial.println(isInit);

  if(!isInit)
  {
    len = 0;
    isInit = true;

    server_setup();

    while(!websiteEnd)
    {
      delay(1000);
    }

    len += EEPROM.writeBool(0, isInit); //1 byte
    len += EEPROM.writeString(len, wifi_ssid); //max 21 bytes
    len++;
    len += EEPROM.writeString(len, wifi_passwd); //max 21 bytes
    len++;
    len += EEPROM.writeString(len, mqtt_ip); //max 16 bytes
    len++;
    len += EEPROM.writeString(len, mqtt_login); //max 21 bytes
    len++;
    len += EEPROM.writeString(len, mqtt_passwd); //max 21 bytes
    len++;
    len += EEPROM.writeString(len, mqtt_name); //max 21 bytes

    EEPROM.commit();
  }
  else
  {
    len = 1;

    wifi_ssid = EEPROM.readString(len);
    len+=wifi_ssid.length();
    len++;
    wifi_passwd = EEPROM.readString(len);
    len+=wifi_passwd.length();
    len++;
    mqtt_ip = EEPROM.readString(len);
    len+=mqtt_ip.length();
    len++;
    mqtt_login = EEPROM.readString(len);
    len+=mqtt_login.length();
    len++;
    mqtt_passwd = EEPROM.readString(len);
    len+=mqtt_passwd.length();
    len++;
    mqtt_name = EEPROM.readString(len);
  }

  client.setMqttClientName(mqtt_name.c_str());
  client.setWifiCredentials(wifi_ssid.c_str(), "67070909316");
  //client.setWifiCredentials(wifi_ssid.c_str(), wifi_passwd.c_str());
  //client.setWifiCredentials(wifi_ssid.c_str(), "Yk5kwjfcHamh");
  //client.setWifiCredentials(wifi_ssid.c_str(), "j32Vfvubdkza");
  client.setMqttServer(mqtt_ip.c_str(), mqtt_login.c_str(), mqtt_passwd.c_str());

  //MQTT
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword.
  client.setMqttReconnectionAttemptDelay(5000);
  //end MQTT

  //BLE
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device. Specify that we want active scanning. 
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  //end BLE 
} // End of setup.

// This is the Arduino main loop function.
void loop() {
  client.loop();

  if (client.isMqttConnected())
  {
    if (doConnectBLE == true) {
      if (connectToServer()) {
        Serial.println("We are now connected to the BLE Server.");
        succes_count++;
        Serial.print("succes count: ");
        Serial.println(succes_count);
      } else {
        Serial.println("We have failed to connect to the server.");
        ESP.restart();
      }
      doConnectBLE = false;  
    }

    if(doScanBLE){
      delay(1000);
      Serial.println("Scan start");
      BLEDevice::getScan()->start(1);
    }
  }
  
  if(relay_on)
  {
    digitalWrite(relay_pin, HIGH);
  }
  else
  {
    digitalWrite(relay_pin, LOW);
  }

  if(reset_eeprom)
  {
    Serial.println("Button pressed for 5 sec. Resetting...");
    isInit = false;
    EEPROM.writeBool(0, isInit);
    EEPROM.commit();
    reset_eeprom = false;
    delay(1000);
    ESP.restart();
  }
} // End of loop
