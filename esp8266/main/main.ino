#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

#define STASSID "Selba"
#define STAPSK  "Selba123"



IPAddress local_IP(192,168,4,20);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);
IPAddress broadcastIp(192,168,4,255);
unsigned int broadcastPort = 5005;

char  ReplyBuffer[100];  // Buffers forsending data 

WiFiUDP Udp;


uint8_t encoderPin_x = 0; // D3 == 0
uint32_t tOn_x = 0;
uint32_t period_x = 0;
uint32_t startTime_x = 0;
uint32_t deltaT = 0;
uint32_t prv_Time = 0;
float phi_x = 0;
float phi_x_dot = 0;
float prv_phi_x = 0;
float k = 0;



void encoder_x_Handler(){
  uint8_t currState = 0;
  if(digitalRead(encoderPin_x)){
    period_x = micros() - startTime_x;
    startTime_x = micros();
  }else{
    tOn_x = micros() - startTime_x;
  }
}

void setup() {
  Serial.begin(115200);

  delay(5);

  pinMode(encoderPin_x,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin_x),encoder_x_Handler,CHANGE);


  // NETWORK AND UDP CONFIG // 
  // WiFi.softAPConfig(local_IP, gateway, subnet);
  // Serial.println(WiFi.softAP(STASSID,STAPSK) ?  "Soft-AP Ready" : "soft-AP Failed!");
  // Udp.begin(localPort);
  // Serial.printf("Wifi: %s \nPassword: %s\n",STASSID,STAPSK);
  // Serial.print("Broadcasting on port: ");
  // Serial.println(broadcastPort);

}

void loop() {
  if(period_x!=0){
    phi_x =2*PI*k + 2*PI*((float)tOn_x/(float)period_x);
  }

  
  deltaT = micros()-prv_Time;
  if(deltaT>0){
    phi_x_dot = (prv_phi_x - phi_x)/deltaT;
  }
  prv_Time = micros();
  prv_phi_x = phi_x;

  Serial.print(phi_x);
  Serial.print(" ");
  Serial.printf("%f \n\r",phi_x_dot);



  // Broadcast data
  // Udp.beginPacket(broadcastIp,broadcastPort);
  // Udp.write(ReplyBuffer);
  // Udp.endPacket();

}
  /*
    test (shell/netcat):
    --------------------
  	  nc -kluvw 0  *PORT*
  */
