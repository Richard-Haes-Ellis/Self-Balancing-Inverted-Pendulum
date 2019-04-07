#include <ESP8266WiFi.h>

void setup()
{
  Serial.begin(115200);
  Serial.println();


  while(!Serial){}

  Serial.println("Starting...");

  WiFi.begin("MiFibra-ADCD", "5DhSTp6U");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {}
