#include <Arduino.h>
#include "Config.h"
#include "MPU6050Handler.h"
#include "BMP280Handler.h"
#include "GPSHandler.h"
#include "LoRaHandler.h"
#include "FlightController.h"
#include "BuzzerHandler.h"
#include "KalmanFilter.h"

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================

// Global handler instances
FlightController flightController(RELAY_PIN);
BuzzerHandler buzzerHandler(BUZZER_PIN);

// Timing variables
unsigned long lastOutputTime = 0;

// State change tracking
FlightController::FlightState lastFlightState = FlightController::GROUND;

// Sensor data storage structure
struct SensorData {
    // IMU data
    float yaw, pitch, roll;
    float accX, accY, accZ;
    
    // Environmental data
    float temperature, altitude;
    
    // GPS data
    double latitude, longitude;
    uint8_t satellites;
    
    // Power monitoring
    float voltage0, voltage1;
} sensorData;

// =====================================================================
// FUNCTION DECLARATIONS
// =====================================================================

void initializeComponents();
void readSensorData();
void updateFlightController();
void outputSensorData();
void transmitLoRaData();
void handleStateChange(FlightController::FlightState newState);
float calculateVoltage(int analogPin);
void scanI2CDevices(); // I2C diagnostic function

// =====================================================================
// MAIN ARDUINO FUNCTIONS
// =====================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) { delay(10); }
    
    Serial.println(F("=== Rocket Flight Computer Initialization ==="));
    
    // Initialize all components
    initializeComponents();
    
    // Setup complete - Play Super Mario theme to celebrate!
    //Serial.println(F("🎵 Initialization Complete! Playing Super Mario Bros Theme..."));
    //buzzerHandler.playMarioTheme();
    
    // Setup complete
    Serial.println(F("\n=== Ready for Flight! ==="));
    Serial.println(F("Data format: State Yaw Pitch Roll VAcc VVel V0 V1 Temp Alt MaxAlt GPS"));
    Serial.println();
    
    lastOutputTime = millis();
}

void loop() {
    // Update all sensors
    mpuHandler.handleInterrupt();
    bmpHandler.update();
    gpsHandler.update();
    
    // Check LoRa messages
    if (loraHandler.isReady()) {
        loraHandler.checkForIncomingData();
    }
    
    // Simple diagnostic - check if interrupt flag ever gets set
    static unsigned long lastInterruptCheck = 0;
    static unsigned long lastInterruptCount = 0;
    if (millis() - lastInterruptCheck >= 2000) { // Every 2 seconds
        unsigned long currentInterruptCount = mpuHandler.getInterruptCount();
        if (currentInterruptCount == lastInterruptCount) {
            Serial.println(F("⚠️ WARNING: No MPU interrupts detected in last 2 seconds!"));
            Serial.print(F("   MPU Ready: ")); Serial.println(mpuHandler.isReady() ? "YES" : "NO");
            Serial.print(F("   Interrupt Pin ")); Serial.print(MPU_INTERRUPT_PIN); 
            Serial.print(F(": ")); Serial.println(digitalRead(MPU_INTERRUPT_PIN));
        }
        lastInterruptCount = currentInterruptCount;
        lastInterruptCheck = millis();
    }
    
    // Process and transmit data at regular intervals
    if (millis() - lastOutputTime >= OUTPUT_INTERVAL) {
        readSensorData();
        updateFlightController();
        outputSensorData();
        transmitLoRaData();
        
        lastOutputTime = millis();
    }
    
    // Keep MPU FIFO clear
    mpuHandler.handleInterrupt();
}

// =====================================================================
// INITIALIZATION FUNCTIONS
// =====================================================================

void initializeComponents() {
    bool allSensorsOK = true;
    
    // Initialize flight controller
    flightController.begin();
    
    // Initialize buzzer
    Serial.print(F("Buzzer: "));
    buzzerHandler.initialize();
    Serial.println(F("✓ Success"));
    
    // Scan I2C devices before MPU initialization
    Serial.println(F("\n=== I2C Device Scan ==="));
    scanI2CDevices();
    Serial.println();
    
    // Initialize MPU6050
    Serial.print(F("MPU6050: "));
    if (mpuHandler.initialize()) {
        Serial.println(F("✓ Success"));
        
        // Configure Enhanced IMU EKF with default parameters from Config.h
        mpuHandler.setOrientationEKFParams(MPU_ORIENTATION_EKF_Q_ANGLE, 
                                          MPU_ORIENTATION_EKF_Q_BIAS, 
                                          MPU_ORIENTATION_EKF_R_ACCEL);
        mpuHandler.setAccelerationFilterParams(MPU_ACCEL_Q_ANGLE, 
                                             MPU_ACCEL_Q_VELOCITY, 
                                             MPU_ACCEL_R_MEASURE);
        Serial.println(F("  Enhanced IMU EKF configured"));
    } else {
        Serial.println(F("✗ Failed"));
        allSensorsOK = false;
    }
    
    // Initialize BMP280
    Serial.print(F("BMP280: "));
    if (bmpHandler.initialize()) {
        Serial.println(F("✓ Success"));
        bmpHandler.setSeaLevelPressure(DEFAULT_SEA_LEVEL_PRESSURE);
        
        // Configure Kalman filters with default parameters from Config.h
        bmpHandler.setAltitudeFilterParams(BMP_ALTITUDE_Q_ANGLE, 
                                         BMP_ALTITUDE_Q_VELOCITY, 
                                         BMP_ALTITUDE_R_MEASURE);
        bmpHandler.setTemperatureFilterParams(BMP_TEMP_Q_ANGLE, 
                                            BMP_TEMP_Q_VELOCITY, 
                                            BMP_TEMP_R_MEASURE);
        Serial.println(F("  Kalman filters configured"));
    } else {
        Serial.println(F("✗ Failed"));
        allSensorsOK = false;
    }
    
    // Initialize GPS with lock
    Serial.print(F("GPS: "));
    if (gpsHandler.initializeWithLock(&Serial2, GPS_BAUD_RATE)) {
        Serial.println(F("✓ Success"));
    } else {
        Serial.println(F("✗ Failed"));
        allSensorsOK = false;
    }
    
    // Initialize LoRa
    Serial.print(F("LoRa: "));
    if (loraHandler.initialize(LORA_FREQUENCY, LORA_DEVICE_ID)) {
        Serial.println(F("✓ Success"));
        loraHandler.setTransmissionInterval(OUTPUT_INTERVAL);
    } else {
        Serial.println(F("✗ Failed"));
        allSensorsOK = false;
    }
    
    // Play error warning if any sensor failed
    if (!allSensorsOK) {
        Serial.println(F("⚠️ Some sensors failed! Playing error warning..."));
        buzzerHandler.playErrorWarning();
        delay(500); // Give time for error sound to complete
    }
}

// =====================================================================
// DATA PROCESSING FUNCTIONS
// =====================================================================

void readSensorData() {
    // Read IMU data
    sensorData.yaw = mpuHandler.getYaw();
    sensorData.pitch = mpuHandler.getPitch();
    sensorData.roll = mpuHandler.getRoll();
    sensorData.accX = mpuHandler.getAccelX();
    sensorData.accY = mpuHandler.getAccelY();
    sensorData.accZ = mpuHandler.getAccelZ();
    
    // Read environmental data
    sensorData.temperature = bmpHandler.isReady() ? bmpHandler.getTemperature() : 0.0;
    sensorData.altitude = bmpHandler.isReady() ? bmpHandler.getAltitude() : 0.0;
    
    // Read GPS data
    sensorData.latitude = gpsHandler.hasValidLocation() ? gpsHandler.getLatitude() : 0.0;
    sensorData.longitude = gpsHandler.hasValidLocation() ? gpsHandler.getLongitude() : 0.0;
    sensorData.satellites = gpsHandler.isReady() ? gpsHandler.getSatellites() : 0;
    
    // Read battery voltages
    sensorData.voltage0 = calculateVoltage(VOLTAGE_PIN_0);
    sensorData.voltage1 = calculateVoltage(VOLTAGE_PIN_1);
}

void updateFlightController() {
    // Update flight controller
    flightController.update(sensorData.altitude, sensorData.pitch, sensorData.roll, 
                          sensorData.accX, sensorData.accY, sensorData.accZ);
    
    // Check for state changes
    FlightController::FlightState newState = flightController.getCurrentState();
    if (newState != lastFlightState) {
        handleStateChange(newState);
        lastFlightState = newState;
    }
}

void transmitLoRaData() {
    if (loraHandler.isReady() && loraHandler.shouldTransmit()) {
        loraHandler.transmitSensorData(sensorData.yaw, sensorData.pitch, sensorData.roll,
                                     sensorData.accX, sensorData.accY, sensorData.accZ,
                                     sensorData.temperature, sensorData.altitude,
                                     sensorData.latitude, sensorData.longitude,
                                     sensorData.satellites,
                                     sensorData.voltage0, sensorData.voltage1);
    }
}

// =====================================================================
// OUTPUT FUNCTIONS
// =====================================================================

void outputSensorData() {
    // Flight state
    Serial.print(F("State: "));
    switch (flightController.getCurrentState()) {
        case FlightController::GROUND:  Serial.print(F("GROUND"));  break;
        case FlightController::BOOST:   Serial.print(F("BOOST"));   break;
        case FlightController::COAST:   Serial.print(F("COAST"));   break;
        case FlightController::APOGEE:  Serial.print(F("APOGEE"));  break;
        case FlightController::DESCENT: Serial.print(F("DESCENT")); break;
    }
    
    // Raw IMU data
    Serial.print(F(" | RAW - Y:"));   Serial.print(mpuHandler.getRawYaw(), 2);
    Serial.print(F(" P:"));   Serial.print(mpuHandler.getRawPitch(), 2);
    Serial.print(F(" R:"));   Serial.print(mpuHandler.getRawRoll(), 2);
    Serial.print(F(" AccX:"));   Serial.print(mpuHandler.getRawAccelX(), 3);
    Serial.print(F(" AccY:"));   Serial.print(mpuHandler.getRawAccelY(), 3);
    Serial.print(F(" AccZ:"));   Serial.print(mpuHandler.getRawAccelZ(), 3);
    
    // Filtered IMU data (EKF)
    Serial.print(F(" | EKF - Y:"));   Serial.print(sensorData.yaw, 2);
    Serial.print(F(" P:"));   Serial.print(sensorData.pitch, 2);
    Serial.print(F(" R:"));   Serial.print(sensorData.roll, 2);
    Serial.print(F(" AccX:"));   Serial.print(sensorData.accX, 3);
    Serial.print(F(" AccY:"));   Serial.print(sensorData.accY, 3);
    Serial.print(F(" AccZ:"));   Serial.print(sensorData.accZ, 3);
    
    // Enhanced EKF debug information (every 5th output to see bias evolution)
    static int outputCounter = 0;
    if (++outputCounter >= 5) {
        outputCounter = 0;
        float bias_x, bias_y, bias_z;
        mpuHandler.getGyroBias(bias_x, bias_y, bias_z);
        float confidence = mpuHandler.getFilterConfidence();
        
        // Check if EKF is initialized
        Serial.print(F(" | EKF Status - "));
        if (mpuHandler.isReady()) {
            Serial.print(F("Ready"));
        } else {
            Serial.print(F("NOT_READY"));
        }
        
        Serial.print(F(" Bias["));
        Serial.print(bias_x, 4);
        Serial.print(F(","));
        Serial.print(bias_y, 4);
        Serial.print(F(","));
        Serial.print(bias_z, 4);
        Serial.print(F("] Conf:"));
        Serial.print(confidence, 2);
        
        // Debug statistics
        Serial.print(F(" | Debug - Interrupts:"));
        Serial.print(mpuHandler.getInterruptCount());
        Serial.print(F(" Packets:"));
        Serial.print(mpuHandler.getPacketCount());
        Serial.print(F(" Overflows:"));
        Serial.print(mpuHandler.getOverflowCount());
    }
    
    // Flight dynamics
    Serial.print(F(" | Flight - VAcc:")); Serial.print(flightController.getVerticalAcceleration(), 2);
    Serial.print(F(" VVel:")); Serial.print(flightController.getVerticalVelocity(), 2);
    
    // Environmental data
    if (bmpHandler.isReady()) {
        Serial.print(F(" Alt:")); Serial.print(sensorData.altitude, 1);
        Serial.print(F("m Temp:")); Serial.print(sensorData.temperature, 1);
        Serial.print(F("°C"));
    }
    
    // Power monitoring
    Serial.print(F(" | Power - V0:")); Serial.print(sensorData.voltage0, 1);
    Serial.print(F("V V1:")); Serial.print(sensorData.voltage1, 1);
    Serial.print(F("V"));
    
    Serial.println();
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================

void handleStateChange(FlightController::FlightState newState) {
    Serial.print(F("🚀 STATE CHANGE: "));
    
    switch (newState) {
        case FlightController::GROUND:
            Serial.println(F("GROUND"));
            // Simple tone for ground state
            buzzerHandler.playTone(NOTE_C4, 200);
            break;
            
        case FlightController::BOOST:
            Serial.println(F("BOOST - Launch Detected! 🔥"));
            //buzzerHandler.playLaunchDetected();
            break;
            
        case FlightController::COAST:
            Serial.println(F("COAST - Motor Burnout 🌙"));
            //buzzerHandler.playTone(NOTE_G4, 100);
            delay(50);
            //buzzerHandler.playTone(NOTE_C5, 150);
            break;
            
        case FlightController::APOGEE:
            Serial.println(F("APOGEE - Maximum Altitude Reached! 🎯"));
            //buzzerHandler.playApogeeDetected();
            break;
            
        case FlightController::DESCENT:
            Serial.println(F("DESCENT - Landing! 🪂"));
            Serial.println(F("🎵 Playing Tetris Theme for Safe Landing!"));
            buzzerHandler.playTetris();
            break;
    }
}

float calculateVoltage(int analogPin) {
    int adcValue = analogRead(analogPin);
    return (adcValue * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_RATIO;
}

void scanI2CDevices() {
    // Implementation of I2C scanner function
    Serial.println(F("Scanning I2C devices..."));
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmission to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print(F("I2C device found at address 0x"));
            if (address < 16)
                Serial.print(F("0"));
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    if (nDevices == 0)
        Serial.println(F("No I2C devices found"));
    else
        Serial.print(F("Total I2C devices found: "));
    Serial.println(nDevices);
}

