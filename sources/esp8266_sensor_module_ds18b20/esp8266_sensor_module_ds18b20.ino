/***************************************************
 * Themostat d'ambiance
 * Connexion Wifi 
 * Connexion MQTT (IO Adafruit)
 * Connexion NTP 
 * Capteur DS18B20
 * gnd
 * -
 * vcc
 * rx - green
 * tx - white
 * -
 ****************************************************/

// Libraries
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#include <ntp-client.h>

#define WIFITIMEOUT 50000

#define WIFI_ERROR 3
#define MQTT_ERROR 4
#define SENSOR_ERROR 2

// Config WiFi parameters
//#define WLAN_SSID       "Livebox-79ca"
//#define WLAN_PASS       "AD12566717FADDD9D4E9545412"

//#define WLAN_SSID       "nirshnarfnow"
//#define WLAN_PASS       "deboutlesdamnesdelaterre"


//#define WLAN_SSID       "LAX"
//#define WLAN_PASS       "deboutlesdamnesdelaterre"

#define WLAN_SSID       "DOMO"
#define WLAN_PASS       "domo1234"

// Config NTP  server 
ntp* ntpServer; 
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(167,114,231,173); // ntp.midway.ovh

//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server

// Config Adafruit IO
/*
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "jpinon"
#define AIO_KEY         "78f2cc8dde492223bcb1d15fc24b0d4da0df66e0"
*/
#define AIO_SERVER      "mare.pinon-hebert.fr"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "jpinon"
#define AIO_KEY         "f588rmp"
#define AIO_ID          "MOBILE"

#define BLUE_LED 2
#define PROG_PIN 0
#define ERROR_LED 0
#define ONE_WIRE_BUS 14  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// Functions
void connect();

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);/****************************** Feeds ***************************************/

// Setup feeds for temperature & humidity
//const char TEMPERATURE_FEED[] PROGMEM = AIO_USERNAME "/feeds/"AIO_ID"/temp";
const char TEMPERATURE_FEED[] PROGMEM = "/feeds/"AIO_ID"/temp";
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

//const char TIME_FEED[] PROGMEM = AIO_USERNAME "/feeds/"AIO_ID"/time";
const char TIME_FEED[] PROGMEM = "/feeds/"AIO_ID"/time";
Adafruit_MQTT_Publish mqtime = Adafruit_MQTT_Publish(&mqtt, TIME_FEED);

/*************************** Sketch Code ************************************/

void error(int code) {
  int i;
  Serial.print("ERROR (code=");
  Serial.print(code);
  Serial.println(")");
  pinMode(ERROR_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(ERROR_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);    
  delay (100);
  for (;;) {
    for (i=0;i<code; i++) {
      digitalWrite(ERROR_LED, LOW);  
      delay (300);
      digitalWrite(ERROR_LED, HIGH);  
      delay (300);    
    }
    delay(1000);
  }
}

void warning(int code) {
  int i;
  Serial.print("WARNING (code=");
  Serial.print(code);
  Serial.println(")");
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, HIGH);  
  digitalWrite(BLUE_LED, HIGH);    
  delay (1000);
  for (i=0;i<code; i++) {
      digitalWrite(ERROR_LED, LOW);  
      delay (300);
      digitalWrite(ERROR_LED, HIGH);  
      delay (300);    
  }
  delay (1000);
}

void setup() {

  // Init sensor
  pinMode(PROG_PIN,INPUT);
  Serial.begin(115200);
  delay (500);
  if (digitalRead(PROG_PIN)==LOW) {
    Serial.println ("\n\n---PROGRAM---\n");
    while (digitalRead(PROG_PIN)==LOW) {delay (500);}
    Serial.println("ENTER SSID: ");
    for (;;) {
      delay (1000);
    }
  }
  unsigned int markTime;
  DS18B20.setResolution(12);
  delay(1000);
  Serial.println(F("Wifi IoT MQTT sensor"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  markTime=millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
    if ((millis()-markTime) > WIFITIMEOUT) error(WIFI_ERROR);
  }
  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.println("Request for Internet time");
  ntpServer = new ntp(localPort,timeServer);
  Serial.println (ntpServer->epochTimeString());
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);  
  
  connect();
  digitalWrite(BLUE_LED, HIGH);
  
}

unsigned int t=1000000; // index of mesures
// log every (in millis)
#define INTERVAL (10*1000) 


void loop() {
  Adafruit_MQTT_Subscribe *subscription;
  // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io    if(! mqtt.connected())
      connect();
  }
    
  // every interval acquire time for logs purposes
  if ((millis()/INTERVAL)!=t) {
    Serial.print (ntpServer->epochTimeString());
    Serial.print (": ");
    t=millis()/INTERVAL;

    float temperature_data;
    // Grab the current state of the sensor
    digitalWrite(BLUE_LED, LOW);  
    do {
      DS18B20.requestTemperatures(); 
      temperature_data = DS18B20.getTempCByIndex(0);
      Serial.print("Temperature 1: ");
      Serial.println(temperature_data,4);
      if (temperature_data == (-127.0)) warning(SENSOR_ERROR);
      if (temperature_data == (85.0)) warning(SENSOR_ERROR);
    } while (temperature_data == 85.0 || temperature_data == (-127.0));
    digitalWrite(BLUE_LED, HIGH);  
    Serial.print(F(" Temperature:"));
    Serial.print(temperature_data);
    char bufferstr[30];
    delay (100);
    digitalWrite(BLUE_LED, LOW);  
    ntpServer->epochTimeString().toCharArray(bufferstr, 30);
    mqtime.publish(bufferstr);
    if ((temperature_data>-20) && (temperature_data<100)) {
      // Publish data
      if (! temperature.publish(temperature_data))
        Serial.print(F(" Failed to publish temperature"));
      }    
    Serial.println();
    digitalWrite(BLUE_LED, HIGH);  
  }
}

// connect to adafruit io via MQTT
void connect() {
  int maxcount = 10;
  Serial.print(F("Connecting MQTT server ("AIO_SERVER") ... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {
    warning(ret);
    
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
    maxcount--;  
  }
  Serial.println(F("MQTT server Connected!"));
}
