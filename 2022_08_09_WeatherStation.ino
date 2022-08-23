/* ESP8266 AWS IoT
 *  
 * Simplest possible example (that I could come up with) of using an ESP8266 with AWS IoT.
 * No messing with openssl or spiffs just regular pubsub and certificates in string constants
 * 
 * This is working as at 7th Aug 2021 with the current ESP8266 Arduino core release 3.0.2
 * 
 * Author: Anthony Elder 
 * License: Apache License v2
 * 
 * 
 * 
 */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <WiFiClient.h> 
#include <ESP8266HTTPClient.h>
#include "Adafruit_PM25AQI.h"
#include "Adafruit_SHT4x.h"

TinyGPSPlus gps;
SoftwareSerial GPSPort(D6, D7); //RX, TX
SoftwareSerial AQSensor(D5, D8); //Do not actually connect to D8 - causes boot to fail. D7 also cannot read data

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

#define NAME ########
#define publish_topic "weatherStation/pub"
#define subscribe_topic "weatherStation/sub"

int pm;
float latitude, longitude;
PM25_AQI_Data data;

const char* ssid = #############;
const char* password = ##########;

// Find this awsEndpoint in the AWS Console: Manage - Things, choose your thing
// choose Interact, its the HTTPS Rest endpoint 
const char* awsEndpoint = "a25w67mn0bb0hm-ats.iot.us-east-1.amazonaws.com";

// For the two certificate strings below paste in the text of your AWS 
// device certificate and private key:

// xxxxxxxxxx-certificate.pem.crt
static const char certificatePemCrt[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
==
-----END CERTIFICATE-----
)EOF";

// xxxxxxxxxx-private.pem.key
static const char privatePemKey[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
==
-----END RSA PRIVATE KEY-----
)EOF";

// This is the AWS IoT CA Certificate from: 
// https://docs.aws.amazon.com/iot/latest/developerguide/managing-device-certs.html#server-authentication
// This one in here is the 'RSA 2048 bit key: Amazon Root CA 1' which is valid 
// until January 16, 2038 so unless it gets revoked you can leave this as is:
static const char caPemCrt[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

BearSSL::X509List client_crt(certificatePemCrt);
BearSSL::PrivateKey client_key(privatePemKey);
BearSSL::X509List rootCert(caPemCrt);

WiFiClientSecure WiFiClient;

void msgReceived(char* topic, byte* payload, unsigned int length);
//PubSubClient pubSubClient(awsEndpoint, 8883, msgReceived, WiFiClient); 
PubSubClient client(WiFiClient);


void connectAWS()
{
  delay(3000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
 
  Serial.println(String("Attempting to connect to SSID: ") + String(ssid));
 
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
 
  setCurrentTime();
 
  WiFiClient.setTrustAnchors(&rootCert);
  WiFiClient.setClientRSACert(&client_crt, &client_key);
 
  client.setServer(awsEndpoint, 8883);
  client.setCallback(msgReceived);
 
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(NAME))
  {
    Serial.print(".");
    delay(1000);
  }
 
  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }
  // Subscribe to a topic
  client.subscribe(subscribe_topic);
 
  Serial.println("AWS IoT Connected!");
}
 


bool listenToGPS(){
  
  GPSPort.listen();

  smartDelayGPS(1000); //Very important, gives time for serial messages to be read into buffer
  
  if(gps.location.isValid()){
    Serial.println("GPS location valid");
    latitude=gps.location.lat();
    longitude = gps.location.lng();
    return true;
    } 
  else {
    return false;
       }
}

bool listenToAQSensor(){
  AQSensor.listen(); // Switch Software Serial connection to Air Quality Sensor

  delay(500); //Very important, gives time for serial messages to be read into buffer
 
  if(AQSensor.available()>0){
    Serial.println("AQ sensor available");
    if (! aqi.read(&data)) {
      Serial.println("Could not read from AQI");
      delay(500);
      return false;
    } 
    else{
      Serial.println("AQI reading success");
      return true;
    }
  } 
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("Serial Connection Established");

  GPSPort.begin(9600);
  AQSensor.begin(9600);

  connectAWS();
  
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  if (! aqi.begin_UART(&AQSensor)) { // connect to the sensor over software serial 
    Serial.println("Could not find PM 2.5 sensor!");
  }
}

unsigned long lastPublish;
int msgCount;

//int messageTimer=1000*60*5;
int messageTimer = 1000*15;


bool readAQSensor = false;
bool readGPS = false;
bool readAQSensorSinceLast = false;
bool readGPSSinceLast = false;

sensors_event_t humidity, temp;

void loop() {

  readGPS = listenToGPS();
  if (readGPS){
    readGPSSinceLast = true;
  }
  
  readAQSensor = listenToAQSensor();
  if (readAQSensor){
    readAQSensorSinceLast = true;
  }

  if (!client.connected()){
    connectAWS();
  }else{
    client.loop();
  }

   if (millis() - lastPublish > messageTimer) {
    String payload = "{";

    bool need_comma = false;

    if(sht4.getEvent(&humidity, &temp)){

      payload+=String("\"temp\":\"")+temp.temperature+String("\",") +String("\"humidity\":\"")+humidity.relative_humidity+String("\"");
      need_comma = true;
    }
    if(readGPSSinceLast){

      if(need_comma){
        payload+=String(",");
      }

      payload += String("\"lat\":\"")+latitude+String("\",") +String("\"long\":\"")+longitude+String("\"");
      need_comma = true;
    }


    if(readAQSensorSinceLast){

      if(need_comma){
        payload+=String(",");
      }

      payload+=String("\"PM25\":\"")+ data.pm25_standard+String("\",") +String("\"PM10\":\"")+data.pm10_standard+String("\"");
      
    }

    payload+=String("}");
      
    client.publish(publish_topic, payload.c_str());
    Serial.print("Published: "); Serial.println(payload);
    lastPublish = millis();
    readAQSensorSinceLast = false;
    readGPSSinceLast = false;
  }
 }



void msgReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

static void smartDelayGPS(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPSPort.available())
      gps.encode(GPSPort.read());
//      Serial.print("Reading GPS");
  } while (millis() - start < ms);
}

void setCurrentTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: "); Serial.print(asctime(&timeinfo));
}

//bool readWeatherData(float &t, float &h){
//  t=dht.readTemperature();
//  h=dht.readHumidity();
//  if (isnan(h) || isnan(t)) {                                                // Check if any reads failed and exit early (to try again).
//    Serial.println("Failed to read from DHT sensor!");
//    return false;
//  }else{
//    return true;
//  }
//}
