

#include <Wire.h>

#define M_PI 3.14159265359
#define dt 0.01 // 10 ms sample rate!
#define SIZEFILT 21
#define LOOPTIME 10000
uint32_t timer0;

// Test variables
uint8_t stopTest = 0;
uint8_t testCycle = 0;
uint32_t timer1;

// Pin definitions
// Inputs
const uint8_t scl = SCL; // SDA and SCL pins for I2C communication
const uint8_t sda = SDA;
const uint8_t encdr_X_channel_A_Pin = 18; // Encoder imputs
const uint8_t encdr_X_channel_B_Pin = 26;
const uint8_t encdr_Y_channel_A_Pin = 33;
const uint8_t encdr_Y_channel_B_Pin = 13;
// Outputs
const uint8_t motor_Y_PWM_Pin  =  4;
const uint8_t motor_Y_dir1_Pin = 16; 
const uint8_t motor_Y_dir2_Pin = 17;
const uint8_t motor_X_PWM_Pin  = 27;
const uint8_t motor_X_dir1_Pin = 25;
const uint8_t motor_X_dir2_Pin = 32;

// AS5040 Encoder variables
int32_t encdr_X_tick = 0;
int32_t encdr_Y_tick = 0;
float sensorValuesA[SIZEFILT];
float sensorValuesB[SIZEFILT];

// Accelerometer configuration values
#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 65.131
int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
short accelData[3];
short gyroData[3];
float x_Angle;
float y_Angle;
float x_Offset = 3.95;
float y_Offset = -2.65;

// Info pulled from the MPU6050 datasheet
const uint8_t MPU6050SlaveAddress = 0x68;         // MPU6050 Slave Device Address
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19; // MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

// PWM variables
const int32_t x_PWM_Channel0 = 0;
const int32_t y_PWM_Channel1 = 1;


// ADC configuration values
uint32_t m_conversionDelay = 5; // 8ms for conversion time
int16_t adc2, adc3;
float volts_Sensor_x;
float volts_Sensor_y;
// Info pulled from ADC1015 datasheet
const uint8_t ADS1015SlaveAddress = 0x48; // ADS1015 Slave Device Address
const uint8_t ADS1015_REG_POINTER_CONVERT = 0x00;
const uint8_t ADS1015_REG_POINTER_CONFIG = 0x01;
/*
    config mask = OS(1) MUX(3) PGA(3) MODE(1) DR(3) COMP_MODE(1) COMP_POL(1) COMP_LATCH(1) COMP_QUE(2)

    Config mask used is [1 100 000 1 100 0 0 0 11] = 0xC183 for channel 0
    Config mask used is [1 101 000 1 100 0 0 0 11] = 0xD183 for channel 1
    Config mask used is [1 110 000 1 100 0 0 0 11] = 0xE183 for channel 2
    Config mask used is [1 111 000 1 100 0 0 0 11] = 0xF183 for channel 3

    OS:         Operation Status
                    When writing:
                    0 : No effect
                    1 : Start a single conversion (when in power-down state)
                    When reading:
                    0 : Device is currently performing a conversion
                    1 : Device is not currently performing a conversion
    MUX:        Channel choice
                    100 for channel 0 to GND
                    101 for channel 1 to GND
                    110 for channel 2 to GND
                    111 for channel 3 to GND
                    there are more options... 
    PGA:        Programmable gain amplifier
                    000 = +- 6.144V Range
                    001 = +- 4.096V Range
                    010 = +- 2.048V Range
                    011 = +- 1.024V Range
                    100 = +- 0.512V Range
                    101 = +- 0.256V Range
                    110 = +- 0.256V Range
                    111 = +- 0.256V Range
    MODE:       Device operating mode    
                    0 = Continous   conversion
                    1 = Single shot conversion
    DR:         Data rate
                    000 : 8 SPS
                    001 : 16 SPS
                    010 : 32 SPS
                    011 : 64 SPS
                    100 : 128 SPS (default)
                    101 : 250 SPS
                    110 : 475 SPS
                    111 : 860 SPS
    COMP_MODE:  Comparator mode
                    0 : Traditional comparator
                    1 : Window comparator
    COMP_POL:   Comparator polarity
                    0 : Active low (default)
                    1 : Active high
    COMP_LATCH: Latching comparator
                    0 : Nonlatching comparator (default).
                    1 : Latching comparator
    COMP_QUE:   Comparator queue and disable
                    00 : Assert after one conversion
                    01 : Assert after two conversions
                    10 : Assert after four conversions
                    11 : Disable comparator and set ALERT/RDY pin to high-impedance (default)
*/

// The IRAM_ATTR atribute is to store the function in ram so its faster
void IRAM_ATTR encdr_X_channel_A_InterruptHandler();
void IRAM_ATTR encdr_X_channel_B_InterruptHandler();
void IRAM_ATTR encdr_Y_channel_A_InterruptHandler();
void IRAM_ATTR encdr_Y_channel_B_InterruptHandler();
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);
static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg);
void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll);
float modeFilter(float val, float *vec);
int16_t get_ADS115_Channel(uint8_t channel);
void MPU6050_Init();
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress);
void printDouble( double val, unsigned int precision);
void MatlabSendDouble(double val);
void MatlabSendInt(int32_t val);
uint8_t motorSignal(uint8_t motor,int32_t controlSignal);

/* GENERAL SETUP */
void setup()
{
  // Safety wait
  delay(3000);
  // Start the serial protocol
  Serial.begin(9600);
  Serial.print("Seting up peripherals.");
  // We declare the ecoder pins as inputs
  pinMode(encdr_X_channel_A_Pin, INPUT);
  pinMode(encdr_X_channel_B_Pin, INPUT);
  pinMode(encdr_Y_channel_A_Pin, INPUT);
  pinMode(encdr_Y_channel_B_Pin, INPUT);
  // We declare the motor pins as Outputs
  pinMode(motor_X_dir1_Pin,OUTPUT);
  pinMode(motor_X_dir2_Pin,OUTPUT);
  pinMode(motor_Y_dir1_Pin,OUTPUT);
  pinMode(motor_Y_dir2_Pin,OUTPUT);
  pinMode(motor_X_PWM_Pin,OUTPUT);
  pinMode(motor_Y_PWM_Pin,OUTPUT);
  // We setup the pwm channels
  ledcSetup(x_PWM_Channel0, 20000, 16);
  ledcSetup(y_PWM_Channel1, 20000, 16);
  // We attach the motor pwm pins to each channel 
  ledcAttachPin(motor_X_PWM_Pin,x_PWM_Channel0);
  ledcAttachPin(motor_Y_PWM_Pin,y_PWM_Channel1);
  // We attach encoder pins to their interrupt handlers
  attachInterrupt(digitalPinToInterrupt(encdr_X_channel_A_Pin), encdr_X_channel_A_InterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encdr_X_channel_B_Pin), encdr_X_channel_B_InterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encdr_Y_channel_A_Pin), encdr_Y_channel_A_InterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encdr_Y_channel_B_Pin), encdr_Y_channel_B_InterruptHandler, CHANGE);
  // We start the I2C wire interface
  Wire.begin(sda, scl);
  // We initialize the MPU6050
  MPU6050_Init();
  // Send a Arduino Ready
  Serial.println("Done. Arduino Ready");
  // Small 1s pause before start
  delay(1000);
}

/* MAIN LOOP */
void loop()
{
  // If the indicated time in ms has elapsed we execute a cycle
  if (micros() - timer0 > LOOPTIME) // 20 ms time sample
  {
    uint32_t loopTime = micros() - timer0;
    /* ############### Read Sensors ############### */

    // From accelerometer
    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
    accelData[0] = AccelX;
    accelData[1] = AccelY;
    accelData[2] = AccelZ;
    gyroData[0] = GyroX;
    gyroData[1] = GyroY;
    gyroData[2] = GyroZ;

    // From ADC (Current sensors)
    adc2 = get_ADS115_Channel(2);
    adc3 = get_ADS115_Channel(3);

    /* ############### Process data ############### */

    // Filering raw angle data from accelerometer
    ComplementaryFilter(accelData, gyroData, &x_Angle, &y_Angle);

    // Calculate Voltage from ADC
    volts_Sensor_x = (float)(12.288/65536)*adc2;
    volts_Sensor_y = (float)(12.288/65536)*adc3;

    /* ############### Display data ############### */
    
    Serial.print(" Angle a: ");
    Serial.print(x_Angle - x_Offset);
    Serial.print(" \tAngle b: ");
    Serial.print(y_Angle - y_Offset);
    Serial.print(" \tV_1: ");
    Serial.print(volts_Sensor_x);
    Serial.print(" \tV_2: ");
    Serial.print(volts_Sensor_y);
    Serial.print(" \tX_Tick: ");
    Serial.print(encdr_X_tick);
    Serial.print(" \tY_Tick: ");
    Serial.print(encdr_Y_tick);
    Serial.print(" \tLoop_T: ");
    Serial.println(loopTime);
    
    // Restart the timer
    timer0 = micros();
  }
  if(micros()-timer1 > 500000 && stopTest == 0) // 1 second
  {
    timer1 = micros();
    switch(testCycle){
      case 0:
      motorSignal(0,0);
      motorSignal(1,0);
      break;
      case 1:
      motorSignal(1,10000);
      motorSignal(0,10000);
      break;
      case 2:
      motorSignal(1,0);
      motorSignal(0,0);
      break;
      case 3:
      motorSignal(1,-10000);
      motorSignal(0,-10000);
      break;
      case 4:
      motorSignal(1,0);
      motorSignal(0,0);
      break;
    }
    testCycle++;
    if(testCycle == 5){
      stopTest = 0;
      testCycle = 0;
    }
  }
  
}

/* FOR WRITING DATA TO I2C LINES */
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

/* READ ALL 14 REGISTERS FROM MPU6050 */
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

/* CONFIGURATION FOR MPU6050 */
void MPU6050_Init()
{
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);  //set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00); // set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

/* ADS1015 CONFIGURATION AND COMUNICATION FOR READING A ADC CHANNEL */
int16_t get_ADS115_Channel(uint8_t channel)
{
  uint16_t config = 0;
  switch (channel)
  {
  case 2:
    config = 0xE183; // Pulled form the datasheet
    break;
  case 3:
    config = 0xF183; // Pulled form the datasheet
    break;
  default:
    return -1;
    break;
  }

  // Ask for conversion
  Wire.beginTransmission(ADS1015SlaveAddress);     // We write a byte with the slave address
  Wire.write((uint8_t)ADS1015_REG_POINTER_CONFIG); // We write a byte with the address of the config register
  Wire.write((uint8_t)(config >> 8));              // We write the last  bytes
  Wire.write((uint8_t)(config & 0xFF));            // We write the first byte
  Wire.endTransmission();

  // We wait a litle for the conversion to take place
  delay(m_conversionDelay);

  // Retrieve conversion
  return readRegister(ADS1015SlaveAddress, ADS1015_REG_POINTER_CONVERT) >> 0;
}

/* READS A REGISTER ON THE I2C LINE */
static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((Wire.read() << 8) | Wire.read());
}

/* COMPLEMETARY FILTER FOR ANGLE ESTIMATION ON x_ AND BETA ANGLES */
void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;  // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;
  }
}

/* ENCODER X INTERRUPT HANDLER CHANNEL A */
void IRAM_ATTR encdr_X_channel_A_InterruptHandler()
{
  if(digitalRead(encdr_X_channel_A_Pin)){     // If A is rising
    if(!digitalRead(encdr_X_channel_B_Pin)){  // and B is LOW
      encdr_X_tick++;                         // then disc is going CW
    }else{                                    // or if B is High
      encdr_X_tick--;                         // then disc is going CCW
    }
  }else{                                      // If is A falling
    if(digitalRead(encdr_X_channel_B_Pin)){   // and B is HIGH
      encdr_X_tick++;                         // then disc is going CW
    }else{                                    // or if B is LOW
      encdr_X_tick--;                         // then disc is going CCW
    }
  }
}

/* ENCODER X INTERRUPT HANDLER CHANNEL B */
void IRAM_ATTR encdr_X_channel_B_InterruptHandler()
{
  if(digitalRead(encdr_X_channel_B_Pin)){     // If B is rising
    if(digitalRead(encdr_X_channel_A_Pin)){   // and A is LOW
      encdr_X_tick++;                         // then disc is going CW
    }else{                                    // or A is HIGH
      encdr_X_tick--;                         // then disc is going CCW
    }
  }else{                                      // If its falling
    if(!digitalRead(encdr_X_channel_A_Pin)){  // Going CW
      encdr_X_tick++;
    }else{                                    // Going CCW
      encdr_X_tick--;
    }
  }
}

/* ENCODER Y INTERRUPT HANDLER CHANNEL A */
void IRAM_ATTR encdr_Y_channel_A_InterruptHandler()
{
  if(digitalRead(encdr_Y_channel_A_Pin)){     // If A is rising
    if(!digitalRead(encdr_Y_channel_B_Pin)){  // and B is LOW
      encdr_Y_tick++;                         // then disc is going CW
    }else{                                    // or if B is High
      encdr_Y_tick--;                         // then disc is going CCW
    }
  }else{                                      // If is A falling
    if(digitalRead(encdr_Y_channel_B_Pin)){   // and B is HIGH
      encdr_Y_tick++;                         // then disc is going CW
    }else{                                    // or if B is LOW
      encdr_Y_tick--;                         // then disc is going CCW
    }
  }
}

/* ENCODER Y INTERRUPT HANDLER CHANNEL B */
void IRAM_ATTR encdr_Y_channel_B_InterruptHandler()
{
  if(digitalRead(encdr_Y_channel_B_Pin)){     // If B is rising
    if(digitalRead(encdr_Y_channel_A_Pin)){   // and A is LOW
      encdr_Y_tick++;                         // then disc is going CW
    }else{                                    // or A is HIGH
      encdr_Y_tick--;                         // then disc is going CCW
    }
  }else{                                      // If its falling
    if(!digitalRead(encdr_Y_channel_A_Pin)){  // Going CW
      encdr_Y_tick++;
    }else{                                    // Going CCW
      encdr_Y_tick--;
    }
  }
}

float modeFilter(float val, float *vec)
{
  int i, j;
  float a, copyVect[SIZEFILT];

  // We make a copy of the sensor stream
  for (i = 0; i < SIZEFILT - 1; ++i)
  {
    copyVect[i] = vec[i];
  }
  copyVect[i + 1] = val; // We insert the new sensed value

  // We sort the array in ascending order
  for (i = 0; i < SIZEFILT; ++i)
  {
    for (j = i + 1; j < SIZEFILT; ++j)
    {
      if (copyVect[i] > copyVect[j])
      {
        a = copyVect[i];
        copyVect[i] = copyVect[j];
        copyVect[j] = a;
      }
    }
  }

#if SIZEFILT % 2 != 0
  // If it's odd
  return copyVect[SIZEFILT / 2 + 1];
#else
  // If is even we pick the mean value of the two
  return (copyVect[SIZEFILT / 2 - 1] + copyVect[SIZEFILT / 2]) / 2;
#endif
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

uint8_t motorSignal(uint8_t motor,int32_t controlSignal){
  // We saturate the input 
  if(controlSignal<-10000){
    controlSignal = -10000;
  }else if(controlSignal > 10000){
    controlSignal = 10000;
  }
  switch(motor){
    case 0: // beeing 0 the x motor
      if(controlSignal<0){  // If is negative CCW
        digitalWrite(motor_X_dir1_Pin,HIGH);
        digitalWrite(motor_X_dir2_Pin,LOW);
        ledcWrite(x_PWM_Channel0,(int)controlSignal*-6.5536);
      }else if(controlSignal>0){  // If its positive CW
        digitalWrite(motor_X_dir1_Pin,LOW);
        digitalWrite(motor_X_dir2_Pin,HIGH);
        ledcWrite(x_PWM_Channel0,(int)controlSignal*6.5536);
      }else if(controlSignal == 0){ // Thats a full stop (BRAKE)
        // H-Brige interpetes same inputs as full stop 
        digitalWrite(motor_X_dir1_Pin,HIGH);
        digitalWrite(motor_X_dir2_Pin,HIGH);
        ledcWrite(x_PWM_Channel0,0);
      }
    break;
    case 1: // beeing 1 the y motor
      if(controlSignal<0){  // If is negative CCW
        digitalWrite(motor_Y_dir1_Pin,HIGH);
        digitalWrite(motor_Y_dir2_Pin,LOW);
        ledcWrite(y_PWM_Channel1,(int)controlSignal*-6.5536);
      }else if(controlSignal>0){  // If its positive CW
        digitalWrite(motor_Y_dir1_Pin,LOW);
        digitalWrite(motor_Y_dir2_Pin,HIGH);
        ledcWrite(y_PWM_Channel1,(int)controlSignal*6.5536);
      }else if(controlSignal == 0){ // Thats a full stop (BRAKE)
        // H-Brige interpetes same inputs as full stop 
        digitalWrite(motor_Y_dir1_Pin,HIGH);
        digitalWrite(motor_Y_dir2_Pin,HIGH);
        ledcWrite(y_PWM_Channel1,0);
      }
    break;
    default:
    return -1;
  }
  return 0;
}