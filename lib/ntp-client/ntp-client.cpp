/*
 Udp NTP Client
 Get the time from a Network Time Protocol (NTP) time server
 Based on Tom Igoe and Ivan Grokhotkov code
 created 24 Jan 2016
 by Jean Pinon
 This code is in the public domain.
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ntp-client.h>


ntp::ntp (int newPort, IPAddress newTimeServer) {
  localPort=newPort;
  timeServer=newTimeServer;
  udp.begin(localPort);
  getInternetTime();
}

void ntp::sendNTPpacket() {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(timeServer, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();  
}

unsigned long ntp::receiveTime(){  
  int cb = udp.parsePacket();
  if (!cb) {
    return (-1);
  }
  else {
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;

    return (epoch);
  }
}

unsigned long ntp::getInternetTime(){
  unsigned long epoch;
  int i=0;
  int t=0;
  sendNTPpacket();
  delay(500);
  epoch=receiveTime();
  t=0; // count sent tries
  while ((epoch == 0xFFFFFFFF) && (t<4)){  
    i=0; // count receve tries
    while ((epoch == 0xFFFFFFFF) && (i<4)){
      delay(500);
      epoch=receiveTime();
      i++;
    } 
    if (epoch == 0xFFFFFFFF) {    
      sendNTPpacket();
      t++;
    }
  }

   internetEpoch=epoch;
   arduinoShift=millis()/1000;

  return (epoch);
}

unsigned long ntp::getArduinoTime(){
  if (((millis()/1000)<arduinoShift)){
    // millis has lapped so refresh with internet time
    getInternetTime();
  }
  return (internetEpoch+((millis()/1000)-arduinoShift));
}

int ntp::epochHours(unsigned long e){
  if (e==0) 
     return ((getArduinoTime()  % 86400L) / 3600);
  else
     return ((e  % 86400L) / 3600);
}

int ntp::epochMinutes(unsigned long e){
  if (e==0) 
     return ((getArduinoTime()  % 3600) / 60);
  else
     return ((e  % 3600) / 60);
}

int ntp::epochSeconds(unsigned long e){
  if (e==0) 
     return (getArduinoTime()  % 60);
  else
     return (e % 60);
}

int ntp::epochDayOfWeek(unsigned long e){
  int day;
  if (e==0) 
    day=(getArduinoTime() % 604800L) / 86400L; // Friday is 0
  else
    day=(e % 604800L) / 86400L; // Friday is 0
  
  if (day <3)
    day+=4;
  else
    day-=3;
  return (day);
}

String ntp::epochTimeString(unsigned long e){
  if (e==0)
	e=getArduinoTime();
  int h=epochHours(e);
  int m=epochMinutes(e);
  int s=epochSeconds(e);
  String str="";
  if (h<10) 
    str+="0";
  str+=h;
  str+=":";
  if (m<10)
    str+="0";
  str+=m;
  str+=":";
  if (s<10)
    str+="0";
  str+=s;  
  return (str);
}
int  ntp::getLocalPort(){
  return (localPort);
}
