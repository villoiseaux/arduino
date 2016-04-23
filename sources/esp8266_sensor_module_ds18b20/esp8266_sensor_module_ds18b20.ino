/***************************************************
 * Themostat d'ambiance
 * Connexion Wifi 
 * Connexion MQTT (IO Adafruit)
 * Connexion NTP 
 * Capteur DS18B20
 * 
 * 17/04/2016 Ajout de la connectifvte descendant MQTT (subscribe) 
 * 18/04/2016 gestion le Relais
 * 22/04/2016 programation (hold button down during the first sec)
 * 23/04/2016 Gestion multi capteur
 * 
 * gnd
 * -
 * vcc
 * rx - green
 * tx - white
 * -
 ****************************************************/

// Standards Libraries
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
// My libraries
#include <ntp-client.h>

// STRINGSIZE size of strint to store ssid & pass
#define STRINGSIZE 40
#define WIFITIMEOUT 50000

#define WIFI_ERROR 3
#define MQTT_ERROR 4
#define SENSOR_ERROR 2
#define SENSOR_MISSING 5

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
#define AIO_SERVER      "mqtt.pinon-hebert.fr"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "jpinon"
#define AIO_KEY         "f588rmp"
#define AIO_ID          "IoT"

#define BLUE_LED 2
#define PROG_PIN 0
#define ERROR_LED 0
#define RELAY_PIN 12

// Dallas One Wire
#define ONE_WIRE_BUS 14  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
byte maxsensors = 0;

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
//const char TEMPERATURE_FEED[] PROGMEM = "/feeds/"AIO_ID"/temp";
//Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

//const char TIME_FEED[] PROGMEM = AIO_USERNAME "/feeds/"AIO_ID"/time";
const char TIME_FEED[] PROGMEM = "/feeds/"AIO_ID"/time";
Adafruit_MQTT_Publish mqtime = Adafruit_MQTT_Publish(&mqtt, TIME_FEED);


// Setup a feed called 'downlink' for subscribing to changes.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char DOWNLINK_FEED[] PROGMEM = "/feeds/"AIO_ID"/downlink";
Adafruit_MQTT_Subscribe downlink = Adafruit_MQTT_Subscribe(&mqtt, DOWNLINK_FEED);


/*************************** Sketch Code ************************************/

void flashled(void)
{
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
  delay(1);
  digitalWrite(BLUE_LED, LOW);
}

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
  for (i=0;i<code; i++) {
      digitalWrite(ERROR_LED, LOW);  
      delay (300);
      digitalWrite(ERROR_LED, HIGH);  
      delay (300);    
  }
}

#define ON LOW
#define OFF HIGH

void programDevice(){
  char ssid[STRINGSIZE]="";
  char pass[STRINGSIZE]="";
  unsigned p=0;
  char ch;
  int pstep=0;
  boolean echo=ON;
  pinMode (BLUE_LED, OUTPUT);
  pinMode (ERROR_LED, OUTPUT);
  Serial.println("Entering program mode");
  Serial.print("SSID:");
  while (pstep<2) {
    if (Serial.available() > 0) {
      ch=Serial.read();
      if (ch==10) continue;
      if (echo==ON) Serial.print(ch);
      if (ch==13) {// CR is end of string
        pstep++;
        p=0;        
        switch (pstep) {        
          case 1: Serial.print("\nPASS:"); echo=OFF; break;
          case 2: Serial.print("DONE"); break;
        }
        
      } else {
        switch (pstep) {
          case 0: ssid[p++]=ch; ssid[p]=0;break;
          case 1: pass[p++]=ch; pass[p]=0;break;
        }
      }
    }
    // blink leds while entering parameters
    if ((millis()%1000)>500){
      digitalWrite(BLUE_LED,LOW);
      digitalWrite(ERROR_LED,HIGH);
    } else {
      digitalWrite(BLUE_LED,HIGH);
      digitalWrite(ERROR_LED,LOW);
    }
  }
  Serial.print("Programming SSID:"); Serial.println(ssid);
  for (p=0; p<STRINGSIZE; p++)
    EEPROM.write(p, ssid[p]);    
  Serial.print("Programming PASS:"); Serial.println(pass);
  for (p=0; p<STRINGSIZE; p++)
    EEPROM.write(p+STRINGSIZE, pass[p]);
  Serial.println("new SSID in EEPROM\n");
  EEPROM.commit();
  digitalWrite(BLUE_LED,HIGH);
  digitalWrite(ERROR_LED,HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nWifi IoT MQTT sensor\nHold push button to enter program mode.");
  EEPROM.begin(512);
  delay (5000);
  // TEST PROGRAMM MODE
  pinMode(PROG_PIN,INPUT);
  if (digitalRead(PROG_PIN)==LOW) 
    programDevice();
  Serial.println("Starting...");
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,HIGH);
  unsigned int markTime;
  DS18B20.setResolution(12);
  DS18B20.begin();
  // Get WIFI values from EEPROM
  char eSSID[STRINGSIZE];
  char ePASS[STRINGSIZE];
  unsigned p;
  for (p=0; p<STRINGSIZE; p++)
    eSSID[p]=EEPROM.read(p);
  for (p=0; p<STRINGSIZE; p++)
    ePASS[p]=EEPROM.read(STRINGSIZE+p);
    
  // Connect to WiFi access point.
  Serial.print("Connecting to AP '");
  //Serial.println(WLAN_SSID);
  Serial.print(eSSID);
  Serial.println("'");
  //WiFi.begin(WLAN_SSID, WLAN_PASS);
  WiFi.begin(eSSID, ePASS);

  markTime=millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if ((millis()-markTime) > WIFITIMEOUT) error(WIFI_ERROR);
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // get GMT time
  Serial.println("Request for Internet time");
  ntpServer = new ntp(localPort,timeServer);
  Serial.println (ntpServer->epochTimeString());
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);  

  // listen for events
  int k;    // counter variable
  char myChar;
  Serial.println("Wait message on MQTT topic:");
  int len = strlen_P(DOWNLINK_FEED);
  for (k = 0; k < len; k++)
  {
    myChar =  pgm_read_byte_near(DOWNLINK_FEED + k);
    Serial.print(myChar);
  }  

  mqtt.subscribe(&downlink);
  // connect to adafruit io
  
  connect();
  digitalWrite(BLUE_LED, HIGH);
  
}

unsigned int t=1000000; // index of mesures
// log every (in millis)
#define INTERVAL (10*1000) 

char getHexDigit(byte n) {
  char ch;
  if ((n>=0) && (n<10)) 
    ch=48+n;
  else
    ch=65+(n-10);
  return ch;
}

void address2string(char* buf, byte addr[8]) {
  // assume buf in 17 bytes length
  buf[16]=0;
  for (int i=0; i<8; i++) {
    buf[i*2]=getHexDigit(addr[i]/16);
    buf[(i*2)+1]=getHexDigit(addr[i]%16);
  }
}

void loop() {
  int i;
  byte addr[8];
  char asciiaddr[17];

  Adafruit_MQTT_Subscribe *subscription;
  // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io    if(! mqtt.connected())
      warning (MQTT_ERROR);
      connect();
  }

  // this is our 'wait for incoming subscription packets' busy subloop
  while (subscription = mqtt.readSubscription(1000)) {
     Serial.println("Message received.");
     // we only care about the downlink events
     if (subscription == &downlink) {
       Serial.println ("This is a downlink message:\n  ");
       // convert mqtt ascii payload to int
       Serial.println((char *)downlink.lastread);
     }

  }  
  // every interval acquire time for logs purposes
  if ((millis()/INTERVAL)!=t) {
    Serial.print (ntpServer->epochTimeString());
    Serial.print (": ");
    t=millis()/INTERVAL;

    float temperature_data;
    // Grab the current state of the sensor
    uint8_t n = DS18B20.getDeviceCount();
    Serial.print("getDeviceCount: ");
    Serial.println(n,HEX);
    for (i=0; i<n; i++) {
      do {        
        digitalWrite(BLUE_LED, LOW);  
        DS18B20.requestTemperaturesByIndex(i); 
        DS18B20.getAddress(addr, i);
        address2string(asciiaddr,addr);
        char TEMPERATURE_FEED[50];
        strcpy(TEMPERATURE_FEED, "/feeds/"AIO_ID"/temp/");
        strcat(TEMPERATURE_FEED,asciiaddr);
        Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);
        temperature_data = DS18B20.getTempCByIndex(i);
        Serial.print("Temperature [");
        Serial.print(asciiaddr);
        Serial.print("] ");
        Serial.print(temperature_data,4);
        if (temperature_data == (-127.0)) warning(SENSOR_MISSING);
        if (temperature_data == (85.0)) warning(SENSOR_ERROR);
        digitalWrite(BLUE_LED, HIGH);  
        digitalWrite(BLUE_LED, LOW);  
        if ((temperature_data>-20) && (temperature_data<100)) {
          // Publish data
          if (! temperature.publish(temperature_data))
            Serial.print(" Failed to publish temperature");
          }    
        } while (temperature_data == 85.0 || temperature_data == (-127.0));
        Serial.println();
        digitalWrite(BLUE_LED, HIGH);  

    }
  }
}

// connect to via MQTT
void connect() {
  int maxcount = 10;
  Serial.print("\nConnecting MQTT server ("AIO_SERVER") ... ");

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {
    warning(MQTT_ERROR);
    
    switch (ret) {
      case 1: Serial.println("Wrong protocol"); break;
      case 2: Serial.println("ID rejected"); break;
      case 3: Serial.println("Server unavail"); break;
      case 4: Serial.println("Bad user/pass"); break;
      case 5: Serial.println("Not authed"); break;
      case 6: Serial.println("Failed to subscribe"); break;
      default: Serial.println("Connection failed"); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println("Retrying connection...");
    delay(10000);
    maxcount--;  
  }
  Serial.println("MQTT server Connected!");
}





