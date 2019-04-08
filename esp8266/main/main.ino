#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
#define STASSID "Selba"
#define STAPSK  "Selba123"
#endif

IPAddress local_IP(192,168,4,20);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  Serial.println(WiFi.softAP(STASSID,STAPSK) ? "Ready" : "Failed!");

  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
        if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
      // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  delay(10);
}
  /*
    test (shell/netcat):
    --------------------
  	  nc -u 192.168.esp.address 8888
  */
