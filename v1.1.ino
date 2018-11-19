/*
  Thanks much to @bruhautomation and @corbanmailloux for providing a great framework for implementing flash/fade with HomeAssistant https://github.com/corbanmailloux/esp-mqtt-rgb-led
  To use this code you will need the following dependancies: 
  
  - Support for the ESP8266 boards. 
        - You can add it to the board manager by going to File -> Preference and pasting http://arduino.esp8266.com/stable/package_esp8266com_index.json into the Additional Board Managers URL field.
        - Next, download the ESP8266 dependancies by going to Tools -> Board -> Board Manager and searching for ESP8266 and installing it.
  
  - You will also need to download the follow libraries by going to Sketch -> Include Libraries -> Manage Libraries
      - DHT sensor library 
      - Adafruit unified sensor
      - ArduinoJSON
      - WebSockets for Arduino
      - AWS SDK ESP8266
    
  UPDATE 16 MAY 2017 by Knutella - Fixed MQTT disconnects when wifi drops by moving around Reconnect and adding a software reset of MCU
             
  UPDATE 23 MAY 2017 - The MQTT_MAX_PACKET_SIZE parameter may not be setting appropriately do to a bug in the PubSub library. If the MQTT messages are not being transmitted as expected please you may need to change the MQTT_MAX_PACKET_SIZE parameter in "PubSubClient.h" directly.
  
  UPDATE 6-22-18 - Adam Youngblood  - Removed all LED functions and optimized code to save battery / power / reduce ping for AWS IoT use cases
                                    - Added AWS IoT Core Connection
                                    - Updates Notes above
                                    - Added Secrets File - arduino_secrets.h                                   
*/

#include "arduino_secrets.h"
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <DHT.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#include <Arduino.h>
#include <Stream.h>
//AWS
#include "sha256.h"
#include "Utils.h"


//WEBSockets
#include <Hash.h>
#include <WebSocketsClient.h>

//MQTT PUBSUBCLIENT LIB 
#include <PubSubClient.h>

//AWS MQTT Websocket
#include "Client.h"
#include "AWSWebSocketClient.h"
#include "CircularByteBuffer.h"

extern "C" {
  #include "user_interface.h"
}


/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
char wifi_ssid[] = SECRET_WIFISSID; //type your WIFI information in arduino_secrets.h
char wifi_password[] = SECRET_WIFIPW;
#define static_or_dhcp "static" //Select Static or DHCP. DHCP is default
#define static_ip {192, 168, 1, 55}     //Only needed if you selected 'static' above
#define static_dns {8, 8, 4, 4}       //Only needed if you selected 'static' above
#define static_gateway {192, 168, 1, 1}   //Only needed if you selected 'static' above
#define static_mask {255, 255, 255, 0}    //Only needed if you selected 'static' above

char aws_key[]= SECRET_AWSKEY;
char aws_secret[] = SECRET_AWSSECRET;
char aws_region[] = SECRET_AWSREGION; 
char aws_endpoint[] = SECRET_AWSEP;
#define mqtt_port 443 //Default 1883, AWS is 8883, 443 for HTTPS

/************* MQTT TOPICS (change these topics as you wish)  **************************/
#define aws_topic  "homeIoT/Multisensors/hallway"
#define SENSORNAME "hallway"
const char* on_cmd = "ON";
const char* off_cmd = "OFF";



/**************************** FOR OTA **************************************************/
char OTApassword[] = SECRET_OTAPASSWORD; // change this variable to whatever password you want to use when you upload OTA
int OTAport = 8266;


//MQTT config
const int maxMQTTpackageSize = 512;
const int maxMQTTMessageHandlers = 1;


/**************************** PIN DEFINITIONS ********************************************/
#define PIRPIN    D5
#define DHTPIN    D7
#define DHTTYPE   DHT22
#define LDRPIN    A0



/**************************** SENSOR DEFINITIONS *******************************************/
float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 50;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

int pirValue;
int pirStatus;
String motionStatus;

char message_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512


WiFiClient espClient;
// PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

ESP8266WiFiMulti WiFiMulti;

AWSWebSocketClient awsWSclient(1000);

PubSubClient client(awsWSclient);

//# of connections
long connection = 0;

// Startup loop init
long startup_loop = 0;

//generate random mqtt clientID
char* generateClientID () {
  char* cID = new char[23]();
  for (int i=0; i<22; i+=1)
    cID[i]=(char)random(1, 256);
  return cID;
}

//connects to websocket layer and mqtt layer
bool connect () {
   
    if (client.connected()) {    
        client.disconnect ();
    }  
    //delay is not necessary... it just help us to get a "trustful" heap space value
    delay (1000);
    Serial.print (millis ());
    Serial.print (" - conn: ");
    Serial.print (++connection);
    Serial.print (" - (");
    Serial.print (ESP.getFreeHeap ());
    Serial.println (")");
    startup_loop = 0;

    //creating random client id
    char* clientID = generateClientID ();
    
    client.setServer(aws_endpoint, mqtt_port);
    if (client.connect(clientID)) {
      Serial.println("connected");
      //client.subscribe(aws_topic);
      sendState();     
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      return false;
    }  
}
/********************************** START SETUP*****************************************/
void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(1);
  pinMode(PIRPIN, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(LDRPIN, INPUT);

  Serial.begin(115200);
  delay(10);

  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setHostname(SENSORNAME);
  ArduinoOTA.setPassword((const char *)OTApassword);

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Starting Node named " + String(SENSORNAME));

  setup_wifi();
  
  client.setCallback(callback);
  
  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IPess: ");
  Serial.println(WiFi.localIP());
  //reconnect();

  //AWS parameters    
  awsWSclient.setAWSRegion(aws_region);
  awsWSclient.setAWSDomain(aws_endpoint);
  awsWSclient.setAWSKeyID(aws_key);
  awsWSclient.setAWSSecretKey(aws_secret);
  awsWSclient.setUseSSL(true);

  if (connect ()){
    //Debug
    //subscribe ();
    //sendmessage ();
  }
 
}

/********************************** START SETUP WIFI*****************************************/
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  
  if (static_or_dhcp == "static") {
      WiFi.config(static_ip, static_dns, static_gateway, static_mask); 
    }
    
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}



/********************************** START CALLBACK*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (!processJson(message)) {
    return;
  }

  sendState();
}



/********************************** START PROCESS JSON*****************************************/
bool processJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  return true;
}



/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["motion"] = (String)motionStatus;
  root["ldr"] = (String)LDR;
  root["humidity"] = (String)humValue;
  root["temperature"] = (String)tempValue;
  root["heatIndex"] = (String)calculateHeatIndex(humValue, tempValue);
  root["sensorName"] = (String)SENSORNAME;
  root["codeVersion"] = SECRET_VERSION;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  /* This loop is so we dont send "empty" data. The first couple of times it runs, it will send zeros. We dont want to do that.*/
  
  if ( startup_loop < 5) {
    //Debug
    //Serial.print("Collecting Info... ");
    startup_loop++;
    } else {
      Serial.print("Sending message to topic: ");
      Serial.print(aws_topic);
      client.publish(aws_topic, buffer);    
      Serial.println(" - Message Sent");
    }
}


/*
 * Calculate Heat Index value AKA "Real Feel"
 * NOAA heat index calculations taken from
 * http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
 */
float calculateHeatIndex(float humidity, float temp) {
  float heatIndex= 0;
  if (temp >= 80) {
    heatIndex = -42.379 + 2.04901523*temp + 10.14333127*humidity;
    heatIndex = heatIndex - .22475541*temp*humidity - .00683783*temp*temp;
    heatIndex = heatIndex - .05481717*humidity*humidity + .00122874*temp*temp*humidity;
    heatIndex = heatIndex + .00085282*temp*humidity*humidity - .00000199*temp*temp*humidity*humidity;
  } else {
     heatIndex = 0.5 * (temp + 61.0 + ((temp - 68.0)*1.2) + (humidity * 0.094));
  }

  if (humidity < 13 && 80 <= temp <= 112) {
     float adjustment = ((13-humidity)/4) * sqrt((17-abs(temp-95.))/17);
     heatIndex = heatIndex - adjustment;
  }

  return heatIndex;
}


/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


/********************************** START MAIN LOOP***************************************/
void loop() {

  ArduinoOTA.handle();
  
  if (!client.connected()) {
    connect();
    software_Reset();
  }
  client.loop();

  float newTempValue = dht.readTemperature(true); //to use celsius remove the true text inside the parentheses  
  float newHumValue = dht.readHumidity();

  //PIR CODE
  pirValue = digitalRead(PIRPIN); //read state of the

  if (pirValue == LOW && pirStatus != 1) {
    motionStatus = "standby";
    sendState();
    pirStatus = 1;
  }

  else if (pirValue == HIGH && pirStatus != 2) {
    motionStatus = "motion detected";
    sendState();
    pirStatus = 2;
  }

  delay(100);

  if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
    tempValue = newTempValue;
    sendState();
  }

  if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
    humValue = newHumValue;
    sendState();
  }


  int newLDR = analogRead(LDRPIN);

  if (checkBoundSensor(newLDR, LDR, diffLDR)) {
    LDR = newLDR;
    sendState();
    }
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
Serial.print("resetting");
ESP.reset(); 
}
