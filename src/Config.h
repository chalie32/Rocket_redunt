#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// ROCKET FLIGHT COMPUTER CONFIGURATION
// =====================================================================

// =====================================================================
// TIMING CONFIGURATION
// =====================================================================
const unsigned long OUTPUT_INTERVAL = 100;         // Data output interval (ms)
const unsigned long SERIAL_BAUD_RATE = 115200;     // Serial communication baud rate

// =====================================================================
// COMMUNICATION CONFIGURATION
// =====================================================================
const int GPS_BAUD_RATE = 9600;                    // GPS serial baud rate
const long LORA_FREQUENCY = 915000000;             // LoRa frequency (Hz)
const int LORA_DEVICE_ID = 1;                      // LoRa device identifier

// =====================================================================
// PIN DEFINITIONS
// =====================================================================
const int VOLTAGE_PIN_0 = A0;                      // Battery voltage monitor 1
const int VOLTAGE_PIN_1 = A1;                      // Battery voltage monitor 2
const int RELAY_PIN = 7;                           // Recovery system relay pin
const int MPU_INTERRUPT_PIN = 3;                   // MPU6050 interrupt pin
const int BUZZER_PIN = 2;                          // Buzzer for audio notifications

// =====================================================================
// VOLTAGE MEASUREMENT CONFIGURATION
// =====================================================================
const float ADC_REFERENCE_VOLTAGE = 3.3;           // Arduino reference voltage (V)
const float ADC_RESOLUTION = 1023.0;               // 10-bit ADC resolution
const float VOLTAGE_DIVIDER_RATIO = (2.5 / 1.9) * 2.0;  // Hardware voltage divider ratio

// =====================================================================
// SENSOR CONFIGURATION
// =====================================================================

// BMP280 Configuration
const float DEFAULT_SEA_LEVEL_PRESSURE = 1013.25;  // Standard sea level pressure (hPa)

// MPU6050 Configuration
const int MPU6050_ADDRESS = 0x69;                  // MPU6050 I2C address
const unsigned long MPU_DATA_UPDATE_INTERVAL = 100; // MPU data processing interval (ms)

// GPS Configuration
const unsigned long GPS_LOCK_TIMEOUT = 60000;      // GPS lock timeout (ms)
const unsigned long GPS_STATUS_UPDATE_INTERVAL = 5000; // GPS status update interval (ms)

// =====================================================================
// KALMAN FILTER CONFIGURATION
// =====================================================================

// MPU6050 Enhanced IMU Extended Kalman Filter Parameters
// Enhanced orientation EKF - quaternion-based filter with bias estimation
const float MPU_ORIENTATION_EKF_Q_ANGLE = 0.001f;  // Process noise for quaternion states
const float MPU_ORIENTATION_EKF_Q_BIAS = 0.0001f;  // Process noise for gyroscope bias states
const float MPU_ORIENTATION_EKF_R_ACCEL = 0.5f;    // Accelerometer measurement noise

// Acceleration filters - simple 1D filters for acceleration data
const float MPU_ACCEL_Q_ANGLE = 0.01f;             // Process noise for acceleration
const float MPU_ACCEL_Q_VELOCITY = 0.1f;           // Process noise for acceleration rate
const float MPU_ACCEL_R_MEASURE = 0.1f;            // Measurement noise

// BMP280 Kalman Filter Parameters
// Altitude filter - moderate responsiveness for flight dynamics
const float BMP_ALTITUDE_Q_ANGLE = 0.01f;          // Process noise for altitude
const float BMP_ALTITUDE_Q_VELOCITY = 0.1f;        // Process noise for vertical velocity
const float BMP_ALTITUDE_R_MEASURE = 0.5f;         // Measurement noise

// Temperature filter - low noise as temperature changes slowly
const float BMP_TEMP_Q_ANGLE = 0.001f;             // Process noise for temperature
const float BMP_TEMP_Q_VELOCITY = 0.01f;           // Process noise for temperature rate
const float BMP_TEMP_R_MEASURE = 0.1f;             // Measurement noise

// =====================================================================
// FLIGHT CONTROLLER CONFIGURATION
// =====================================================================

// Launch Detection Thresholds
const float DEFAULT_LAUNCH_ACCEL_THRESHOLD = 2.0;  // g's of acceleration for launch detect
const float DEFAULT_LAUNCH_ALTITUDE_THRESHOLD = 10.0; // meters above ground

// Apogee Detection Thresholds
const float DEFAULT_MIN_APOGEE_ALTITUDE = 20.0;    // minimum altitude to consider apogee detection
const float DEFAULT_DESCENT_ALTITUDE_THRESHOLD = 2.0; // meters of altitude loss to confirm descent
const float DEFAULT_VERTICAL_VELOCITY_THRESHOLD = -1.0; // m/s negative velocity to help confirm descent

// Buffer Sizes
const int ALTITUDE_BUFFER_SIZE = 15;               // Number of altitude samples to average
const int ACCEL_BUFFER_SIZE = 10;                  // Number of acceleration samples to average

// Phase Duration Minimums
const unsigned long BOOST_MIN_DURATION = 500;      // minimum boost phase duration (ms)
const unsigned long COAST_MIN_DURATION = 500;      // minimum coast phase duration (ms)
const unsigned long DESCENT_CONFIRM_TIME = 200;    // ms of continuous descent to confirm

// =====================================================================
// LORA TRANSMISSION CONFIGURATION
// =====================================================================
const int LORA_SPREADING_FACTOR = 7;               // LoRa spreading factor (7-12)
const long LORA_SIGNAL_BANDWIDTH = 125000;         // LoRa signal bandwidth (Hz)
const int LORA_CODING_RATE = 5;                    // LoRa coding rate (5-8)
const int LORA_TX_POWER = 17;                      // LoRa transmission power (dBm)

#endif // CONFIG_H 