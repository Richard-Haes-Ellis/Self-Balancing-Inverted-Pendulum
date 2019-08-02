

const uint8_t encdr_X_channel_A_Pin = 18; // Encoder imputs
const uint8_t encdr_X_channel_B_Pin = 26;
const uint8_t encdr_Y_channel_A_Pin = 33;
const uint8_t encdr_Y_channel_B_Pin = 13;
// Outputs
const uint8_t motorAdir = 4; // Digital direction pins
const uint8_t motorBdir = 27;
const uint8_t motorAPWM = 25; // Digital PWM pins
const uint8_t motorBPWM = 32;

const int32_t channel0 = 0;
const int32_t channel1 = 1;
const int32_t channel2 = 2;
const int32_t channel3 = 3;

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

  ledcSetup(channel0, 5000, 8);
  ledcSetup(channel1, 5000, 8);
  ledcSetup(channel2, 5000, 8);
  ledcSetup(channel3, 5000, 8);
  ledcAttachPin(motorAPWM,channel0);
  ledcAttachPin(motorBPWM,channel1);
  ledcAttachPin(motorAdir,channel2);
  ledcAttachPin(motorBdir,channel3);
}

void loop() {
  
  ledcWrite(channel0,20);
  ledcWrite(channel1,20);
  ledcWrite(channel2,20);
  ledcWrite(channel3,20);
  delay(100);
  ledcWrite(channel0,40);
  ledcWrite(channel1,40);
  ledcWrite(channel2,40);
  ledcWrite(motorBPWM,40);
  delay(100);
  ledcWrite(channel0,80);
  ledcWrite(channel1,80);
  ledcWrite(channel2,80);
  ledcWrite(channel3,80);
  delay(100);
  ledcWrite(channel0,120);
  ledcWrite(channel1,120);
  ledcWrite(channel2,120);
  ledcWrite(channel3,120);
  delay(100);
  ledcWrite(channel0,180);
  ledcWrite(channel1,180);
  ledcWrite(channel2,180);
  ledcWrite(channel3,180);
  delay(100);
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

