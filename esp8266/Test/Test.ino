

#include <Wire.h>

#define M_PI 3.14159265359
#define dt 0.01 // 10 ms sample rate!
#define SIZEFILT 21
#define LOOPTIME 10000
uint32_t timer0;
uint32_t timer1;
uint32_t timer2;

// Pin definitions
// Inputs
const uint8_t scl = D1; // SDA and SCL pins for I2C communication
const uint8_t sda = D2;
const uint8_t encoderApin = D5; // Encoder PMW inputs
const uint8_t encoderBpin = D6;
// Outputs
const uint8_t motorAdir = D3; // Digital direction pins
const uint8_t motorBdir = D4;
const uint8_t motorAPWM = D7; // Digital PWM pins
const uint8_t motorBPWM = D8;

// AS5040 Encoder variables
float motorAlfaAngle = 0;
float motorBetaAngle = 0;
uint32_t tonA;
uint32_t tonB;
uint32_t periodA;
uint32_t periodB;
float sensorValuesA[SIZEFILT];
float sensorValuesB[SIZEFILT];

// Accelerometer configuration values
#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 65.131
int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
short accelData[3];
short gyroData[3];
float alfaAngle;
float betaAngle;
float alfaOffset = 3.95;
float betaOffset = -2.65;

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

// ADC configuration values
uint32_t m_conversionDelay = 8; // 8ms for conversion time
int16_t adc2, adc3;
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

/* GENERAL SETUP */
void setup()
{
  // Safety wait
  delay(3000);
  // Start the serial protocol
  Serial.begin(9600);
  // We declare the ecoder pins as inputs
  pinMode(encoderApin, INPUT);
  pinMode(encoderBpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderApin), alfaMotorEncoder_interriptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBpin), betaMotorEncoder_interriptHandler, CHANGE);
  // We start the I2C wire interface
  Wire.begin(sda, scl);
  // We initialize the MPU6050
  MPU6050_Init();
  // Send a Arduino Ready
  Serial.println("Arduino Ready");
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

    // From AS5040

    motorAlfaAngle = getMotorAngle(0);
    motorBetaAngle = getMotorAngle(1);

    /* ############### Filter data ############### */

    // Filering raw angle data from accelerometer
    ComplementaryFilter(accelData, gyroData, &alfaAngle, &betaAngle);
    // motorAlfaAngle = modeFilter(motorAlfaAngle,sensorValuesA);
    // motorBetaAngle = modeFilter(motorBetaAngle,sensorValuesB);

    /* ############### Display data ############### */
     
    Serial.print(" Angle a: ");
    Serial.print(alfaAngle - alfaOffset);
    Serial.print(" \tAngle b: ");
    Serial.print(betaAngle - betaOffset);
    Serial.print(" \tMA angle: ");
    Serial.print(motorAlfaAngle);
    Serial.print(" \tMB angle: ");
    Serial.print(motorBetaAngle);
    Serial.print(" \tADC1: ");
    Serial.print(adc2);
    Serial.print(" \tADC2: ");
    Serial.print(adc3);
    Serial.print(" \tLoop_T: ");
    Serial.println(loopTime);

    // Restart the timer
    timer0 = micros();
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

/* COMPLEMETARY FILTER FOR ANGLE ESTIMATION ON ALFA AND BETA ANGLES */
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

/* GET THE ANGLE OF MOTORS */
float getMotorAngle(uint8_t motor)
{
  float angle = 0;
  switch (motor)
  {
  case 0: // Alfa motor
          // Some conditions to be met for the sensed value to be valid
    if ((tonA < periodA) && (periodA > 1040) && (periodA < 1055) && (tonA!=0))
    {
      angle = (float)tonA/periodA; // We calculat the new angle
    }
    else
    {
      angle = motorAlfaAngle; // We keep the last angle
    }
    break;
  case 1: // Beta motor
          // Some conditions to be met for the sensed value to be valid
    if ((tonB < periodB) && (periodB > 1040) && (periodB < 1055) && (tonB!=0))
    {
      angle = (float)tonB/periodB; // We calculate the new angle
    }
    else
    {
      angle = motorBetaAngle; // We keep the last angle
    }
    break;
  default:
    break;
  }
  return angle*360;
}

/* INTERRUPT HANDLER FOR ALFA ENCODER */
void alfaMotorEncoder_interriptHandler()
{
  if (digitalRead(encoderApin))
  { // If its high we start the timer and calculate the pwm frequency
    periodA = micros() - timer1;
    timer1  = micros();
  }
  if (!digitalRead(encoderApin))
  { // If its low we calculate elapsed time for ton
    tonA = micros() - timer1;
  }
}

/* INTERRUPT HANDLER FOR BETA ENCODER */
void betaMotorEncoder_interriptHandler()
{
  if (digitalRead(encoderBpin))
  { // If its high we start the timer and calculate the pwm frequency
    periodB = micros() - timer2;
    timer2  = micros();
  }
  if (!digitalRead(encoderBpin))
  { // If its low we calculate elapsed time for ton
    tonB = micros() - timer2;
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