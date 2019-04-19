#line 1 "/home/pirana/Repos/Self-Balancing-Inverted-Pendulum/esp8266/main/main.ino"
#line 1 "/home/pirana/Repos/Self-Balancing-Inverted-Pendulum/esp8266/main/main.ino"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

#ifndef STASSID
#define STASSID "Selba"
#define STAPSK  "Selba123"
#endif

int i= 0;

IPAddress local_IP(192,168,4,20);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

IPAddress broadcastIp(192,168,4,255);
unsigned int localPort = 8888;      // local port to listen on

unsigned int broadcastPort = 5005;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "Hello \r\n";       // a string to send back

WiFiUDP Udp;

#line 27 "/home/pirana/Repos/Self-Balancing-Inverted-Pendulum/esp8266/main/main.ino"
void setup();
#line 43 "/home/pirana/Repos/Self-Balancing-Inverted-Pendulum/esp8266/main/main.ino"
void loop();
#line 27 "/home/pirana/Repos/Self-Balancing-Inverted-Pendulum/esp8266/main/main.ino"
void setup() {
  Serial.begin(115200);

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  Serial.println(WiFi.softAP(STASSID,STAPSK) ? "Ready" : "Failed!");
  Serial.print("ESP8266 on IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);

  Serial.print("Broadcasting on port: ");
  Serial.println(broadcastPort);

}

void loop() {
  // if there's data available, read a packet
  // int packetSize = Udp.parsePacket();
  // if (packetSize) {
  //   Serial.print("Received packet of size ");
  //   Serial.println(packetSize);
  //   Serial.print("From ");
  //   IPAddress remote = Udp.remoteIP();
  //   for (int i = 0; i < 4; i++) {
  //     Serial.print(remote[i], DEC);
  //       if (i < 3) {
  //       Serial.print(".");
  //     }
  //   }
  //   Serial.print(", port ");
  //   Serial.println(Udp.remotePort());
  //     // read the packet into packetBufffer
  //   Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
  //   Serial.println("Contents:");
  //   Serial.println(packetBuffer);
  //     // send a reply, to the IP address and port that sent us the packet we received
  //   Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  //   Udp.write(ReplyBuffer);
  //   Udp.endPacket();
  // }

  sprintf(ReplyBuffer,"Hello %d\r\n",i);
  i++;
  // Broadcast data
  Udp.beginPacket(broadcastIp,broadcastPort);
  Udp.write(ReplyBuffer);
  Udp.endPacket();

  delay(100);
}
  /*
    test (shell/netcat):
    --------------------
  	  nc -kluvw 0  *PORT*
  */

