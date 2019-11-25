#include <Arduino.h>

// MPU-6050 Short Example Sketch
#include<Wire.h>

TwoWire WIRE2 (2, I2C_FAST_MODE);
#define Wire WIRE2

#define TRIES 500

const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int32_t AcXOff, AcYOff, AcZOff;

void calibrateMPU(){
    for (int i = 0; i < TRIES; i++){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);

        Wire.requestFrom(MPU, 6);  // request a total of 6 registers
        AcXOff += Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
        AcYOff += Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZOff += Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        delay(50);
    }

    AcXOff = AcXOff / TRIES;
    AcYOff = AcYOff / TRIES;
    AcZOff = AcZOff / TRIES - 8192;
}

void setup(){
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(false);
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);  // AccSel register
    Wire.write(0x08);  // set to 00001000 to AccSel = 1
    Wire.endTransmission(true);
    Serial1.begin(9600);
    calibrateMPU();
}
void loop(){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);

    Wire.requestFrom(MPU, 6);  // request a total of 6 registers
    AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    AcX -= ((int16_t)AcXOff);
    AcY -= ((int16_t)AcYOff);
    AcZ -= ((int16_t)AcZOff);

    float FAcX, FAcY, FAcZ;

    FAcX = AcX / 8192.0;
    FAcY = AcY / 8192.0;
    FAcZ = AcZ / 8192.0;

    Serial1.print("Accelerometer: ");
    Serial1.print("X = "); Serial1.print(FAcX);
    Serial1.print(" | Y = "); Serial1.print(FAcY);
    Serial1.print(" | Z = "); Serial1.println(FAcZ);
    // Serial1.print("OffX = "); Serial1.print(AcXOff);
    // Serial1.print(" | OffY = "); Serial1.print(AcYOff);
    // Serial1.print(" | OffZ = "); Serial1.println(AcZOff);
    delay(333);
}