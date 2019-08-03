#include <AS5040.h>

// CHANGE THESE AS APPROPRIATE
#define CSpin   26
#define CLKpin  33
#define DOpin   13
#define PROGpin 18




AS5040 enc (CLKpin, CSpin, DOpin, PROGpin) ;

// Set mode to quadrature (A + B + index), monitor via serial IF
void setup ()
{
  Serial.begin (9600) ;   // NOTE BAUD RATE
  delay(3000);
  Serial.println("Arduino ready");
  if (!enc.begin (AS5040_QUADRATURE, true, 0x200))  // example setting reverse sense and an offset
    Serial.println ("Error setting up AS5040") ;
}

void loop ()
{
  Serial.print (enc.read (), HEX) ;
  Serial.print ("   ") ;
  Serial.print (enc.status (), BIN) ;
  Serial.print ("   ") ;
  Serial.print (enc.valid () ? "OK" : "Fault") ;
  Serial.print ("   ") ;
  Serial.println (enc.Zaxis ()) ;
  delay (400) ;
}