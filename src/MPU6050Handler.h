#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"

class MPU6050Handler {
private:
    MPU6050 mpu;
    
    // MPU control/status vars
    bool dmpReady;        // set true if DMP init was successful
    uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
    uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;   // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorFloat gravity; // [x, y, z]            gravity vector
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    int16_t ax, ay, az;  // acceleration values
    int16_t gx, gy, gz;  // gyro values

    // Enhanced IMU Extended Kalman Filter for orientation
    IMUExtendedKalmanFilter orientationEKF;
    
    // Simple Kalman filters for orientation data (backup solution)
    KalmanFilter yawFilter;
    KalmanFilter pitchFilter;
    KalmanFilter rollFilter;
    
    // Simple Kalman filters for acceleration data only
    KalmanFilter accelXFilter;
    KalmanFilter accelYFilter;
    KalmanFilter accelZFilter;

    // Filtered values
    float filteredYaw;
    float filteredPitch;
    float filteredRoll;
    float filteredAccelX;
    float filteredAccelY;
    float filteredAccelZ;
    
    // Raw gyroscope values in rad/s (for EKF)
    float rawGyroX, rawGyroY, rawGyroZ;
    float rawAccelXms2, rawAccelYms2, rawAccelZms2; // Raw accel in m/s²

    // Data processing control - only process every 100ms
    unsigned long lastDataUpdate;
    static const unsigned long DATA_UPDATE_INTERVAL = 100; // Process filtered data every 100ms
    
    // Debug counters
    unsigned long interruptCount;
    unsigned long packetCount;
    unsigned long overflowCount;

    // Constants for conversion
    const float ACCEL_SCALE = 9.81f / 16384.0f; // Convert to m/s² (9.81 / 2^14)
    const float GYRO_SCALE = 0.0174533f / 131.0f; // Convert to rad/s (π/180 / 131)
    
    // Internal methods
    void processNewData();
    bool readFifoData();
    void applyEnhancedFiltering();

public:
    MPU6050Handler();
    
    bool initialize();
    void update();
    
    // Interrupt-based update method
    void handleInterrupt();
    
    // Check if new data should be processed (every 100ms)
    bool shouldProcess();
    
    // Getters for filtered EKF orientation values
    float getYaw();
    float getPitch();
    float getRoll();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    
    // Getters for raw DMP values (unfiltered)
    float getRawYaw();
    float getRawPitch();
    float getRawRoll();
    float getRawAccelX();
    float getRawAccelY();
    float getRawAccelZ();
    
    // EKF-specific getters
    void getQuaternion(float q[4]);
    void getGyroBias(float& bias_x, float& bias_y, float& bias_z);
    float getFilterConfidence();
    
    // Debug information getters
    unsigned long getInterruptCount() const { return interruptCount; }
    unsigned long getPacketCount() const { return packetCount; }
    unsigned long getOverflowCount() const { return overflowCount; }
    
    // Enhanced Kalman filter configuration
    void setOrientationEKFParams(float q_angle, float q_bias, float r_accel);
    void setAccelerationFilterParams(float q_angle, float q_velocity, float r_measure);
    void resetFilters();
    
    // Interrupt handling
    void setInterruptFlag();
    bool isReady();
};

// Extern declaration for the global instance
extern MPU6050Handler mpuHandler;
extern volatile bool mpuInterrupt;

#endif // MPU6050_HANDLER_H 