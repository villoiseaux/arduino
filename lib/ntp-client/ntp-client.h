/*
 Udp NTP Client
 Get the time from a Network Time Protocol (NTP) time server
 Based on Tom Igoe and Ivan Grokhotkov code
 created 24 Jan 2016
 by Jean Pinon
 This code is in the public domain.
 */
#ifndef ntpClient_h
#define ntpClient_h

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <ESP8266WiFi.h>

#include <WiFiUdp.h>


#define NTP_PACKET_SIZE 48

class ntp {
 public:
   ntp (int newPort, IPAddress newTimeServer);
   unsigned long getArduinoTime();
   unsigned long getInternetTime();
   int  getLocalPort();
   String epochTimeString(unsigned long e=0);
   int epochHours(unsigned long e=0);
   int epochMinutes(unsigned long e=0);
   int epochSeconds(unsigned long e=0);
   int epochDayOfWeek(unsigned long e=0);
 private:
   unsigned long internetEpoch;
   unsigned long arduinoShift;
   WiFiUDP udp;
   void sendNTPpacket();
   unsigned long receiveTime();
   int localPort;
   IPAddress timeServer;
   byte packetBuffer[ NTP_PACKET_SIZE];
};

#endif