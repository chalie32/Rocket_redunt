#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

/**
 * Simple 1D Kalman Filter Implementation
 * 
 * This filter estimates a single state variable (position) and its derivative (velocity)
 * using noisy measurements. It's suitable for filtering sensor data like altitude,
 * temperature, etc.
 */
class KalmanFilter {
private:
    // State variables
    float x_est;      // Current state estimate (position)
    float x_vel;      // Current velocity estimate
    
    // Error covariance matrix (2x2)
    float P00, P01, P10, P11;
    
    // Process noise covariance
    float Q_angle;    // Process noise for angle/position
    float Q_velocity; // Process noise for velocity
    
    // Measurement noise covariance
    float R_measure;  // Measurement noise
    
    // Time tracking
    unsigned long last_time;
    bool initialized;

public:
    /**
     * Constructor
     * @param q_angle Process noise for the main state (position/angle)
     * @param q_velocity Process noise for velocity
     * @param r_measure Measurement noise
     */
    KalmanFilter(float q_angle = 0.001f, float q_velocity = 0.003f, float r_measure = 0.03f);
    
    /**
     * Initialize the filter with an initial measurement
     * @param initial_value Initial measurement
     */
    void initialize(float initial_value);
    
    /**
     * Update the filter with a new measurement
     * @param measurement New measurement value
     * @param dt Time step in seconds (if 0, will auto-calculate)
     * @return Filtered estimate
     */
    float update(float measurement, float dt = 0.0f);
    
    /**
     * Get the current state estimate
     * @return Current filtered value
     */
    float getState() const { return x_est; }
    
    /**
     * Get the current velocity estimate
     * @return Current velocity estimate
     */
    float getVelocity() const { return x_vel; }
    
    /**
     * Get the current estimation uncertainty
     * @return Position uncertainty (standard deviation)
     */
    float getUncertainty() const { return sqrt(P00); }
    
    /**
     * Reset the filter
     */
    void reset();
    
    /**
     * Set process noise parameters
     * @param q_angle Process noise for position/angle
     * @param q_velocity Process noise for velocity
     */
    void setProcessNoise(float q_angle, float q_velocity);
    
    /**
     * Set measurement noise parameter
     * @param r_measure Measurement noise
     */
    void setMeasurementNoise(float r_measure);
    
    /**
     * Check if filter is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized; }
};

/**
 * Enhanced IMU Extended Kalman Filter for Orientation Estimation
 * 
 * This EKF fuses gyroscope and accelerometer data to estimate orientation
 * using quaternions to avoid gimbal lock. It also estimates gyroscope bias.
 * 
 * State vector: [q0, q1, q2, q3, bias_x, bias_y, bias_z] (7 states)
 * - q0, q1, q2, q3: Unit quaternion representing orientation
 * - bias_x, bias_y, bias_z: Gyroscope bias estimates
 */
class IMUExtendedKalmanFilter {
private:
    // State vector [q0, q1, q2, q3, bias_x, bias_y, bias_z]
    float state[7];
    
    // Error covariance matrix (7x7)
    float P[7][7];
    
    // Process noise parameters
    float Q_angle;        // Process noise for quaternion
    float Q_bias;         // Process noise for bias
    
    // Measurement noise parameters
    float R_accel;        // Accelerometer measurement noise
    
    // Time tracking
    unsigned long last_time;
    bool initialized;
    
    // Gravity vector in earth frame
    static constexpr float gravity = 9.81f;
    
    // Utility functions
    void normalizeQuaternion();
    void quaternionToEuler(float& yaw, float& pitch, float& roll);
    void quaternionMultiply(float q1[4], float q2[4], float result[4]);
    void rotateVector(float q[4], float v[3], float result[3]);
    float vectorMagnitude(float v[3]);
    void normalizeVector(float v[3]);
    
    // EKF functions
    void predict(float gyro[3], float dt);
    void update(float accel[3]);
    void updateCovariance(float F[7][7], float dt);

public:
    /**
     * Constructor
     * @param q_angle Process noise for quaternion states
     * @param q_bias Process noise for bias states
     * @param r_accel Accelerometer measurement noise
     */
    IMUExtendedKalmanFilter(float q_angle = 0.001f, float q_bias = 0.0001f, float r_accel = 0.5f);
    
    /**
     * Initialize the filter with initial accelerometer reading
     * @param accel_x Initial accelerometer X (m/s²)
     * @param accel_y Initial accelerometer Y (m/s²)
     * @param accel_z Initial accelerometer Z (m/s²)
     */
    void initialize(float accel_x, float accel_y, float accel_z);
    
    /**
     * Update filter with new IMU measurements
     * @param gyro_x Gyroscope X (rad/s)
     * @param gyro_y Gyroscope Y (rad/s)
     * @param gyro_z Gyroscope Z (rad/s)
     * @param accel_x Accelerometer X (m/s²)
     * @param accel_y Accelerometer Y (m/s²)
     * @param accel_z Accelerometer Z (m/s²)
     * @param dt Time step in seconds (if 0, will auto-calculate)
     */
    void update(float gyro_x, float gyro_y, float gyro_z,
                float accel_x, float accel_y, float accel_z,
                float dt = 0.0f);
    
    /**
     * Get orientation as Euler angles
     * @param yaw Output yaw angle in degrees
     * @param pitch Output pitch angle in degrees
     * @param roll Output roll angle in degrees
     */
    void getEulerAngles(float& yaw, float& pitch, float& roll);
    
    /**
     * Get quaternion representation
     * @param q Output quaternion [w, x, y, z]
     */
    void getQuaternion(float q[4]);
    
    /**
     * Get estimated gyroscope bias
     * @param bias_x Output X bias (rad/s)
     * @param bias_y Output Y bias (rad/s)
     * @param bias_z Output Z bias (rad/s)
     */
    void getGyroBias(float& bias_x, float& bias_y, float& bias_z);
    
    /**
     * Reset the filter
     */
    void reset();
    
    /**
     * Set process noise parameters
     * @param q_angle Process noise for quaternion
     * @param q_bias Process noise for bias
     */
    void setProcessNoise(float q_angle, float q_bias);
    
    /**
     * Set measurement noise parameter
     * @param r_accel Accelerometer measurement noise
     */
    void setMeasurementNoise(float r_accel);
    
    /**
     * Check if filter is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized; }
    
    /**
     * Get filter confidence (trace of covariance matrix)
     * @return Filter confidence (lower is better)
     */
    float getConfidence();
};

#endif // KALMAN_FILTER_H 