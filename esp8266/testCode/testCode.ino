

const uint8_t encdr_X_channel_A_Pin = 18; // Encoder imputs
const uint8_t encdr_X_channel_B_Pin = 26;
const uint8_t encdr_Y_channel_A_Pin = 33;
const uint8_t encdr_Y_channel_B_Pin = 13;
// Outputs
const uint8_t motorAdir = 4; // Digital direction pins
const uint8_t motorBdir = 27;
const uint8_t motorAPWM = 25; // Digital PWM pins
const uint8_t motorBPWM = 32;

void setup() {
  delay(5000);
  Serial.begin(9600);
  pinMode(motorAdir,OUTPUT);
  pinMode(motorBPWM,OUTPUT);
  pinMode(motorBdir,OUTPUT);
  pinMode(motorAPWM,OUTPUT);
  pinMode(encdr_X_channel_A_Pin,INPUT_PULLUP);
  pinMode(encdr_X_channel_B_Pin,INPUT_PULLUP);
  pinMode(encdr_Y_channel_A_Pin,INPUT_PULLUP);
  pinMode(encdr_Y_channel_B_Pin,INPUT_PULLUP);
}

void loop() {
  if(digitalRead(encdr_X_channel_A_Pin)==LOW){
    Serial.println("encoder pin 1A");
  }
  if(digitalRead(encdr_X_channel_B_Pin)==LOW){
    Serial.println("encoder pin 1B");
  }
  if(digitalRead(encdr_Y_channel_A_Pin)==LOW){
    Serial.println("encoder pin 2A");
  }
  if(digitalRead(encdr_Y_channel_B_Pin)==LOW){
    Serial.println("encoder pin 2B");
  }  
  digitalWrite(motorAdir,HIGH);
  digitalWrite(motorBdir,HIGH);
  digitalWrite(motorAPWM,HIGH);
  digitalWrite(motorAPWM,HIGH);
  delay(1000);
  digitalWrite(motorAdir,HIGH);
  digitalWrite(motorBdir,HIGH);
  digitalWrite(motorAPWM,HIGH);
  digitalWrite(motorAPWM,HIGH);
  delay(1000);
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

