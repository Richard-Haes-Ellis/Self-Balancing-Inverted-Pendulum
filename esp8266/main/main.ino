#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Selba";
const char* password = "Selba123";

IPAddress local_IP(192,168,4,20);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

IPAddress broadcastIp;

WiFiUDP udp;
unsigned int localUdpPort = 4200;
char replyPacket[] = "Hi there! Got the message :-)";


void setup()
{
  Serial.begin(115200);
  Serial.println();


  while(!Serial){}

  Serial.println("Starting...");

  delay(1000);

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  Serial.println(WiFi.softAP(ssid,password) ? "Ready" : "Failed!");

  Serial.println("Starting UDP ... ");
  udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), localUdpPort);

  // set broadcast and gateway ip
  IPAddress ipBroadcast(192,168,4,255);
}

void loop() {
  // broadcast udp package
  udp.beginPacket(ipBroadcast, 5005);
  udp.write("{ \"sensor\" : \"esp8266\", \"value\": \"hello from esp8266\" }");
  udp.endPacket();
  // wait 1  seconds
  delay(1000);

}
