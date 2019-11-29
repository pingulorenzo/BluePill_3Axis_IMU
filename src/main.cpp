#include <Arduino.h>

// MPU-6050 Short Example Sketch
#include<Wire.h>

TwoWire WIRE2 (2, I2C_FAST_MODE);
#define Wire WIRE2

#define N_SAMPLES 200
#define LED_BLUE PC15
#define LED_RED PC14
#define SCALE_FACTOR 8192.0

const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int32_t AcXOff, AcYOff, AcZOff;
float FAcX, FAcY, FAcZ;

// Calibrate by reading, averaging and calculating offsets
void calibrateMPU(){
    // LED on to signal start of calibration
    digitalWrite(LED_BLUE, HIGH);

    // Cycle for N_SAMPLES times and accumulate
    for (int i = 0; i < N_SAMPLES; i++){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);

        Wire.requestFrom(MPU, 6);  // request a total of 6 registers
        AcXOff += Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
        AcYOff += Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZOff += Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        delay(50);
    }

    // Compute average
    AcXOff = AcXOff / N_SAMPLES;
    AcYOff = AcYOff / N_SAMPLES;
    AcZOff = AcZOff / N_SAMPLES - 8192;

    // LED off to signal end of calibration
    digitalWrite(LED_BLUE, LOW);
}

void setup(){
    // Start LED Pins
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    // Start I2C Communication
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(false);

    // Select Accelerometer mode (4g)
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);  // AccSel register
    Wire.write(0x08);  // set to 00001000 to AccSel = 1
    Wire.endTransmission(true);

    // Start Serial Port 1 (A9, A10)
    Serial1.begin(9600);

    // Start Calibration
    calibrateMPU();
}
void loop(){
    // Write starting register
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);

    // Request data from MPU
    Wire.requestFrom(MPU, 6);  // request 6 registers
    AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    // Subtract Offsets
    AcX -= ((int16_t)AcXOff);
    AcY -= ((int16_t)AcYOff);
    AcZ -= ((int16_t)AcZOff);

    // Divide by SCALE_FACTOR for the selected mode (8192 for 4g sensitivity)
    FAcX = AcX / SCALE_FACTOR;
    FAcY = AcY / SCALE_FACTOR;
    FAcZ = AcZ / SCALE_FACTOR;

    // Cast to int16 the values*100 to send on Serial Port
    Serial1.write(int16_t(FAcX*100));
    Serial1.write(int16_t(FAcY*100));
    Serial1.write(int16_t(FAcZ*100));
    
    // DEBUG SECTION
    // Serial1.print("Accelerometer: ");
    // Serial1.print("X = "); Serial1.print(int16_t(FAcX*100));
    // Serial1.print(" | Y = "); Serial1.print(int16_t(FAcY*100));
    // Serial1.print(" | Z = "); Serial1.println(int16_t(FAcZ*100));
    // Serial1.print("OffX = "); Serial1.print(AcXOff);
    // Serial1.print(" | OffY = "); Serial1.print(AcYOff);
    // Serial1.print(" | OffZ = "); Serial1.println(AcZOff);
    
    // Delay 100 ms
    delay(100);
}