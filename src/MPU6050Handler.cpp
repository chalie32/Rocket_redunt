#include "MPU6050Handler.h"
#include "Config.h"
#include <Wire.h>

// Create the global instance
MPU6050Handler mpuHandler;
volatile bool mpuInterrupt = false;

// Interrupt service routine
void dmpDataReady() {
    mpuInterrupt = true;
}

MPU6050Handler::MPU6050Handler() : 
    mpu(0x68), // Try 0x68 first (more common when AD0 is grounded)
    dmpReady(false),
    mpuIntStatus(0),
    devStatus(0),
    packetSize(0),
    fifoCount(0),
    // Initialize Enhanced IMU EKF with appropriate noise parameters
    orientationEKF(0.001f, 0.0001f, 0.5f),
    // Simple orientation filters for backup (stable and proven)
    yawFilter(0.001f, 0.003f, 0.03f),
    pitchFilter(0.001f, 0.003f, 0.03f),
    rollFilter(0.001f, 0.003f, 0.03f),
    // Acceleration filters: higher process noise for faster response
    accelXFilter(0.01f, 0.1f, 0.1f),
    accelYFilter(0.01f, 0.1f, 0.1f),
    accelZFilter(0.01f, 0.1f, 0.1f),
    // Initialize filtered values
    filteredYaw(0),
    filteredPitch(0),
    filteredRoll(0),
    filteredAccelX(0),
    filteredAccelY(0),
    filteredAccelZ(0),
    rawGyroX(0), rawGyroY(0), rawGyroZ(0),
    rawAccelXms2(0), rawAccelYms2(0), rawAccelZms2(0),
    lastDataUpdate(0),
    interruptCount(0),
    packetCount(0),
    overflowCount(0) {
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
    Serial.print("MPU6050 I2C Address: 0x");
    Serial.println(0x68, HEX);
    
    mpu.initialize();

    // Verify connection
    Serial.println("Testing device connections...");
    bool connection = mpu.testConnection();
    if (connection) {
        Serial.println("MPU6050 connection successful");
    } else {
        Serial.println("MPU6050 connection failed");
        Serial.println("Please check:");
        Serial.println("  - VCC connected to 3.3V or 5V");
        Serial.println("  - GND connected to GND");
        Serial.println("  - SDA connected to pin 18 (Teensy 4.0)");
        Serial.println("  - SCL connected to pin 19 (Teensy 4.0)");
        Serial.println("  - AD0 pin grounded (for 0x68) or high (for 0x69)");
        return false;
    }
    
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
        pinMode(MPU_INTERRUPT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
        
        // Enable DMP interrupt
        mpu.setIntEnabled(0x02); // Enable DMP interrupt
        
        Serial.println(F("Interrupt enabled"));
        Serial.println(F("Using simple Kalman filters for stable orientation filtering"));
        
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
    
    // Apply Enhanced filtering
    applyEnhancedFiltering();
    
    // Update timestamp for 100ms interval control
    lastDataUpdate = millis();
}

void MPU6050Handler::applyEnhancedFiltering() {
    // Convert raw sensor data to proper units 
    rawGyroX = gx * GYRO_SCALE;  // Convert to rad/s
    rawGyroY = gy * GYRO_SCALE;
    rawGyroZ = gz * GYRO_SCALE;
    
    rawAccelXms2 = ax * ACCEL_SCALE;  // Convert to m/s²
    rawAccelYms2 = ay * ACCEL_SCALE;
    rawAccelZms2 = az * ACCEL_SCALE;
    
    // Debug output (every 50th packet to avoid spam)
    static int debugCounter = 0;
    if (++debugCounter >= 50) {
        debugCounter = 0;
        Serial.print(F("[MPU Debug] Raw values - YPR: "));
        Serial.print(ypr[0] * 180 / M_PI, 2);
        Serial.print(F("/"));
        Serial.print(ypr[1] * 180 / M_PI, 2);
        Serial.print(F("/"));
        Serial.print(ypr[2] * 180 / M_PI, 2);
        Serial.print(F(" Gyro: "));
        Serial.print(gx); Serial.print(F("/"));
        Serial.print(gy); Serial.print(F("/"));
        Serial.print(gz);
        Serial.print(F(" Accel: "));
        Serial.print(ax); Serial.print(F("/"));
        Serial.print(ay); Serial.print(F("/"));
        Serial.print(az);
        Serial.println();
    }
    
    // TEMPORARILY DISABLE Enhanced EKF due to stability issues
    // Use simple, proven Kalman filters on stable DMP orientation values
    // orientationEKF.update(rawGyroX, rawGyroY, rawGyroZ,
    //                      rawAccelXms2, rawAccelYms2, rawAccelZms2);
    // orientationEKF.getEulerAngles(filteredYaw, filteredPitch, filteredRoll);
    
    // Use simple filters on stable DMP orientation (degrees)
    float rawYawDeg = ypr[0] * 180 / M_PI;
    float rawPitchDeg = ypr[1] * 180 / M_PI;
    float rawRollDeg = ypr[2] * 180 / M_PI;
    
    filteredYaw = yawFilter.update(rawYawDeg);
    filteredPitch = pitchFilter.update(rawPitchDeg);
    filteredRoll = rollFilter.update(rawRollDeg);
    
    // Apply simple Kalman filters to acceleration data separately
    filteredAccelX = accelXFilter.update(rawAccelXms2);
    filteredAccelY = accelYFilter.update(rawAccelYms2);
    filteredAccelZ = accelZFilter.update(rawAccelZms2);
}

void MPU6050Handler::handleInterrupt() {
    // Check if interrupt flag is set
    if (!mpuInterrupt) return;
    mpuInterrupt = false; // Reset interrupt flag
    
    interruptCount++; // Debug counter
    
    if (!dmpReady) return;
    
    // Get interrupt status and FIFO count
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow first
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // Reset FIFO on overflow
        mpu.resetFIFO();
        overflowCount++; // Debug counter
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
        packetCount++; // Debug counter
        
        // Process the packet data (this will apply Enhanced EKF)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Apply Enhanced filtering to this packet
        applyEnhancedFiltering();
        
        dataAvailable = true;
        
        // Update FIFO count for next iteration
        fifoCount = mpu.getFIFOCount();
        
        // Safety check to prevent infinite loop
        if (fifoCount > 1000) {
            mpu.resetFIFO();
            overflowCount++; // Debug counter
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

// Getters for filtered EKF orientation values
float MPU6050Handler::getYaw() {
    return filteredYaw;
}

float MPU6050Handler::getPitch() {
    return filteredPitch;
}

float MPU6050Handler::getRoll() {
    return filteredRoll;
}

float MPU6050Handler::getAccelX() {
    return filteredAccelX;
}

float MPU6050Handler::getAccelY() {
    return filteredAccelY;
}

float MPU6050Handler::getAccelZ() {
    return filteredAccelZ;
}

// Getters for raw DMP values (unfiltered)
float MPU6050Handler::getRawYaw() {
    return ypr[0] * 180 / M_PI;
}

float MPU6050Handler::getRawPitch() {
    return ypr[1] * 180 / M_PI;
}

float MPU6050Handler::getRawRoll() {
    return ypr[2] * 180 / M_PI;
}

float MPU6050Handler::getRawAccelX() {
    return rawAccelXms2;
}

float MPU6050Handler::getRawAccelY() {
    return rawAccelYms2;
}

float MPU6050Handler::getRawAccelZ() {
    return rawAccelZms2;
}

// EKF-specific getters (return dummy values when using simple filters)
void MPU6050Handler::getQuaternion(float q[4]) {
    // Return dummy quaternion for identity (no rotation)
    q[0] = 1.0f; // w
    q[1] = 0.0f; // x
    q[2] = 0.0f; // y  
    q[3] = 0.0f; // z
    // orientationEKF.getQuaternion(q); // Disabled
}

void MPU6050Handler::getGyroBias(float& bias_x, float& bias_y, float& bias_z) {
    // Return zero bias when using simple filters
    bias_x = 0.0f;
    bias_y = 0.0f;
    bias_z = 0.0f;
    // orientationEKF.getGyroBias(bias_x, bias_y, bias_z); // Disabled
}

float MPU6050Handler::getFilterConfidence() {
    // Return dummy confidence value
    return 0.01f; // Low confidence to indicate simple filtering
    // return orientationEKF.getConfidence(); // Disabled
}

// Enhanced Kalman filter configuration methods
void MPU6050Handler::setOrientationEKFParams(float q_angle, float q_bias, float r_accel) {
    orientationEKF.setProcessNoise(q_angle, q_bias);
    orientationEKF.setMeasurementNoise(r_accel);
    Serial.print(F("EKF parameters updated - Q_angle: "));
    Serial.print(q_angle, 6);
    Serial.print(F(", Q_bias: "));
    Serial.print(q_bias, 6);
    Serial.print(F(", R_accel: "));
    Serial.println(r_accel, 3);
}

void MPU6050Handler::setAccelerationFilterParams(float q_angle, float q_velocity, float r_measure) {
    accelXFilter.setProcessNoise(q_angle, q_velocity);
    accelXFilter.setMeasurementNoise(r_measure);
    accelYFilter.setProcessNoise(q_angle, q_velocity);
    accelYFilter.setMeasurementNoise(r_measure);
    accelZFilter.setProcessNoise(q_angle, q_velocity);
    accelZFilter.setMeasurementNoise(r_measure);
}

void MPU6050Handler::resetFilters() {
    // orientationEKF.reset(); // Disabled Enhanced EKF
    yawFilter.reset();
    pitchFilter.reset();
    rollFilter.reset();
    accelXFilter.reset();
    accelYFilter.reset();
    accelZFilter.reset();
    Serial.println(F("All IMU filters reset (using simple orientation filters)"));
}

void MPU6050Handler::setInterruptFlag() {
    mpuInterrupt = true;
}

bool MPU6050Handler::isReady() {
    return dmpReady;
}

