#include "MPU6050Handler.h"
#include <Wire.h>

// Define the interrupt pin
#define INTERRUPT_PIN 3

// Create the global instance
MPU6050Handler mpuHandler;
volatile bool mpuInterrupt = false;

// Interrupt service routine
void dmpDataReady() {
    mpuInterrupt = true;
}

MPU6050Handler::MPU6050Handler() : 
    mpu(0x69),
    dmpReady(false),
    mpuIntStatus(0),
    devStatus(0),
    packetSize(0),
    fifoCount(0),
    lastDataUpdate(0) {
    // Initialize arrays
    for (int i = 0; i < 3; i++) {
        ypr[i] = 0;
    }
    ax = ay = az = 0;
    gx = gy = gz = 0;
}

bool MPU6050Handler::initialize() {
    // Initialize I2C - use the default Wire for MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // Initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // Verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    
    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Set our DMP Ready flag
        Serial.println(F("DMP ready!"));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        // Initialize timers
        lastDataUpdate = millis();
        
        // Enable interrupt detection - attach interrupt pin
        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        // Enable DMP interrupt
        mpu.setIntEnabled(0x02); // Enable DMP interrupt
        
        Serial.println(F("Interrupt enabled"));
        
        return true;
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        
        return false;
    }
}

bool MPU6050Handler::readFifoData() {
    if (!dmpReady) return false;
    
    // Get interrupt status
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // Reset FIFO
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return false;
    }
    
    // Check for DMP data ready interrupt
    if (mpuIntStatus & 0x02) {
        // Wait for correct available data length
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }
        
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count in case there is more than one packet available
        fifoCount -= packetSize;
        
        return true;
    }
    
    return false;
}

void MPU6050Handler::processNewData() {
    // Get sensor data from the buffer
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Update timestamp for 100ms interval control
    lastDataUpdate = millis();
}

void MPU6050Handler::handleInterrupt() {
    // Check if interrupt flag is set
    if (!mpuInterrupt) return;
    mpuInterrupt = false; // Reset interrupt flag
    
    if (!dmpReady) return;
    
    // Get interrupt status and FIFO count
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow first
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // Reset FIFO on overflow
        mpu.resetFIFO();
        // Don't print overflow message to reduce spam
        return;
    }
    
    // Aggressively read FIFO to prevent overflow
    // Read all available packets, but only process the latest one
    bool dataAvailable = false;
    
    while (fifoCount >= packetSize) {
        // Read packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        // Store the latest packet data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        dataAvailable = true;
        
        // Update FIFO count for next iteration
        fifoCount = mpu.getFIFOCount();
        
        // Safety check to prevent infinite loop
        if (fifoCount > 1000) {
            mpu.resetFIFO();
            return;
        }
    }
    
    // Only update processed data every 100ms, but always clear FIFO
    if (dataAvailable) {
        unsigned long currentTime = millis();
        if (currentTime - lastDataUpdate >= DATA_UPDATE_INTERVAL) {
            lastDataUpdate = currentTime;
            // Data is already processed above, no additional processing needed
        }
    }
}

bool MPU6050Handler::shouldProcess() {
    unsigned long currentTime = millis();
    return (currentTime - lastDataUpdate >= DATA_UPDATE_INTERVAL);
}

void MPU6050Handler::update() {
    // Interrupt-based update - check and handle interrupt
    handleInterrupt();
}

float MPU6050Handler::getYaw() {
    return ypr[0] * 180 / M_PI; // Convert radians to degrees
}

float MPU6050Handler::getPitch() {
    return ypr[1] * 180 / M_PI; // Convert radians to degrees
}

float MPU6050Handler::getRoll() {
    return ypr[2] * 180 / M_PI; // Convert radians to degrees
}

float MPU6050Handler::getAccelX() {
    return ax * ACCEL_SCALE;
}

float MPU6050Handler::getAccelY() {
    return ay * ACCEL_SCALE;
}

float MPU6050Handler::getAccelZ() {
    return az * ACCEL_SCALE;
}

void MPU6050Handler::setInterruptFlag() {
    mpuInterrupt = true;
}

bool MPU6050Handler::isReady() {
    return dmpReady;
} 
