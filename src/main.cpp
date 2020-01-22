#include <Arduino.h>

#include <Wire.h>

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
    digitalWrite(LED_RED, HIGH);

    // Cycle for N_SAMPLES times and accumulate
    for (int i = 0; i < N_SAMPLES; i++){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);

        Wire.requestFrom(MPU, 6);  // request a total of 6 registers
        AcXOff += Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
        AcYOff += Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZOff += Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        delay(50);
    }

    // Compute average
    AcXOff = AcXOff / N_SAMPLES;
    AcYOff = AcYOff / N_SAMPLES;
    AcZOff = AcZOff / N_SAMPLES - 8192;

    // LED off to signal end of calibration
    digitalWrite(LED_RED, LOW);
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

    // Cast to uint16 the values*100 to send on Serial Port
    uint16_t serial_ax, serial_ay, serial_az;
    serial_ax = (uint16_t)AcX;
    serial_ay = (uint16_t)AcY;
    serial_az = (uint16_t)AcZ;

    // Split High and Low
    uint8_t serial_hax, serial_lax, serial_hay, serial_lay, serial_haz, serial_laz;
    serial_hax = serial_ax >> 8;
    serial_lax = serial_ax & 0x00FF;
    serial_hay = serial_ay >> 8;
    serial_lay = serial_ay & 0x00FF;
    serial_haz = serial_az >> 8;
    serial_laz = serial_az & 0x00FF;

    // check for one byte and notify by turning on LED
    digitalWrite(LED_BLUE, HIGH);
    if (Serial1.available() > 0){
        Serial1.read();

        // Send on serial
        Serial1.write(serial_hax);
        Serial1.write(serial_lax);
        Serial1.write(serial_hay);
        Serial1.write(serial_lay);
        Serial1.write(serial_haz);
        Serial1.write(serial_laz);

        // Notify end of Serial Write by turning LED off
        digitalWrite(LED_BLUE, LOW);
    }
    
    // DEBUG SECTION
    // Serial1.println("DEBUG");
    
    // Delay 100 ms
    // delay(100);
}
