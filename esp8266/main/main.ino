#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

#define STASSID "Selba"
#define STAPSK  "Selba123"

#define doubleLength 8 
#define tickToRad 0.006135923

#define serialTelemetry
 

IPAddress local_IP(192,168,4,20);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);
IPAddress broadcastIp(192,168,4,255);
unsigned int broadcastPort = 5005;

char  ReplyBuffer[100];  // Buffers forsending data 

WiFiUDP Udp;


uint8_t x_ChannelAPin = 5; 
uint8_t x_ChannelBPin = 4; 
int32_t xEnconderTick = 0;
float x_theta = 0;
double x_theta_dot = 0;
float x_theta_prv = 0;

uint32_t deltaT = 0;
uint32_t timer0 = 0;
uint32_t timer1 = 0;

float phi_x = 0;
float phi_x_dot = 0;
float prv_phi_x = 0;
float ratio_x = 0;

byte buffer4[4];




void setup() {
  delay(5000);
  Serial.begin(115200);
  #ifdef debug
    Serial.println("\nStarting...");
  #endif
  delay(2000);

  #ifdef debug
    Serial.println("Configuring system.. ");
  #endif
  pinMode(x_ChannelAPin,INPUT);
  pinMode(x_ChannelBPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(x_ChannelAPin),x_channelAInterruptHandler,CHANGE);
  attachInterrupt(digitalPinToInterrupt(x_ChannelBPin),x_channelBInterruptHandler,CHANGE);

    
  // NETWORK AND UDP CONFIG // 
  // WiFi.softAPConfig(local_IP, gateway, subnet);
  // Serial.println(WiFi.softAP(STASSID,STAPSK) ?  "Soft-AP Ready" : "soft-AP Failed!");
  // Udp.begin(localPort);
  // Serial.printf("Wifi: %s \nPassword: %s\n",STASSID,STAPSK);
  // Serial.print("Broadcasting on port: ");
  // Serial.println(broadcastPort);
  #ifdef debug
    Serial.println("Setup done!");
  #endif
}

void loop() {

  x_theta = xEnconderTick*tickToRad;

  if(micros()-timer1>100)
  {
    timer1 = micros();
    x_theta_dot = (x_theta_prv-x_theta)/0.0001;
    x_theta_prv = x_theta;
    #ifdef serialTelemetry
      printDouble(x_theta_dot,100000000);
    #endif
  }

  
  if(micros()-timer0>10000) // Every 10000 microseconds OR 0.01 milliseconds
  {
    #ifdef matlab
      // MatlabSendInt(xEnconderTick);
      // MatlabSendDouble((double)PI);
    #endif
    timer0 = micros();
  }


  // Broadcast data
  // Udp.beginPacket(broadcastIp,broadcastPort);
  // Udp.write(ReplyBuffer);
  // Udp.endPacket();
  /*
    test (shell/netcat):
    --------------------
  	  nc -kluvw 0  *PORT*
  */

}

// Low pass bessel filter order=1 alpha1=0.01 
// http://www.schwietering.com/jayduino/filtuino/
class  FilterBeLp1
{
	public:
		FilterBeLp1()
		{
			v[0]=0.0;
		}
	private:
		double v[2];
	public:
		double step(double x) //class II 
		{
			v[0] = v[1];
			v[1] = (3.046874709125380054e-2 * x) + (0.93906250581749239892 * v[0]);
			return (v[0] + v[1]);
		}
};


void x_channelAInterruptHandler()
{
  if(digitalRead(x_ChannelAPin)){     // If A is rising
    if(!digitalRead(x_ChannelBPin)){  // and B is LOW
      xEnconderTick++;                 // then disc is going CW
    }else{                            // or if B is High
      xEnconderTick--;                 // then disc is going CCW
    }
  }else{                              // If is A falling
    if(digitalRead(x_ChannelBPin)){   // and B is HIGH
      xEnconderTick++;                 // then disc is going CW
    }else{                            // or if B is LOW
      xEnconderTick--;                 // then disc is going CCW
    }
  }
}

void x_channelBInterruptHandler()
{
  if(digitalRead(x_ChannelBPin)){     // If B is rising
    if(digitalRead(x_ChannelAPin)){   // and A is LOW
      xEnconderTick++;                 // then disc is going CW
    }else{                            // or A is HIGH
      xEnconderTick--;                 // then disc is going CCW
    }
  }else{                              // If its falling
    if(!digitalRead(x_ChannelAPin)){  // Going CW
      xEnconderTick++;
    }else{                            // Going CCW
      xEnconderTick--;
    }
  }
}
  

void MatlabSendInt(int32_t val){
  union {
    int32_t variable;
    byte temp_array[4];
  } u;
  
  u.variable = val;

  for(int i=0;i<4;i++){
    Serial.write(u.temp_array[i]);
  }
  Serial.print("\n");
}

void MatlabSendDouble(double val){
  union {
    double variable;
    byte temp_array[8];
  } u;
  
  u.variable = 2536.12523;

  for(int i=0;i<8;i++){
    Serial.write(u.temp_array[8]);
  }
  Serial.print("\n");
}


void printDouble( double val, unsigned int precision){
   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
       frac = (val - int(val)) * precision;
   else
       frac = (int(val)- val ) * precision;
   Serial.println(frac,DEC) ;
} 

