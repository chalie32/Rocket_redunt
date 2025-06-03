#include <Arduino.h>
#include "Config.h"
#include "MPU6050Handler.h"
#include "BMP280Handler.h"
#include "GPSHandler.h"
#include "LoRaHandler.h"
#include "FlightController.h"

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================

// Global handler instances
FlightController flightController(RELAY_PIN);

// Timing variables
unsigned long lastOutputTime = 0;

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
float calculateVoltage(int analogPin);

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
    
    // Setup complete
    Serial.println(F("\n=== Initialization Complete ==="));
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
    // Initialize flight controller
    flightController.begin();
    
    // Initialize MPU6050
    Serial.print(F("MPU6050: "));
    if (mpuHandler.initialize()) {
        Serial.println(F("✓ Success"));
    } else {
        Serial.println(F("✗ Failed"));
    }
    
    // Initialize BMP280
    Serial.print(F("BMP280: "));
    if (bmpHandler.initialize()) {
        Serial.println(F("✓ Success"));
        bmpHandler.setSeaLevelPressure(DEFAULT_SEA_LEVEL_PRESSURE);
    } else {
        Serial.println(F("✗ Failed"));
    }
    
    // Initialize GPS with lock
    Serial.print(F("GPS: "));
    if (gpsHandler.initializeWithLock(&Serial2, GPS_BAUD_RATE)) {
        Serial.println(F("✓ Success"));
    } else {
        Serial.println(F("✗ Failed"));
    }
    
    // Initialize LoRa
    Serial.print(F("LoRa: "));
    if (loraHandler.initialize(LORA_FREQUENCY, LORA_DEVICE_ID)) {
        Serial.println(F("✓ Success"));
        loraHandler.setTransmissionInterval(OUTPUT_INTERVAL);
    } else {
        Serial.println(F("✗ Failed"));
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
    flightController.update(sensorData.altitude, sensorData.pitch, sensorData.roll, 
                          sensorData.accX, sensorData.accY, sensorData.accZ);
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
    
    // IMU and derived data
    Serial.print(F(" Yaw:")); Serial.print(sensorData.yaw, 2);
    Serial.print(F(" Pitch:")); Serial.print(sensorData.pitch, 2);
    Serial.print(F(" Roll:")); Serial.print(sensorData.roll, 2);
    Serial.print(F(" VAcc:")); Serial.print(flightController.getVerticalAcceleration(), 2);
    Serial.print(F(" VVel:")); Serial.print(flightController.getVerticalVelocity(), 2);
    
    // Battery voltages
    Serial.print(F(" V0:")); Serial.print(sensorData.voltage0, 3);
    Serial.print(F("V V1:")); Serial.print(sensorData.voltage1, 3);
    Serial.print('V');
    
    // Environmental data
    if (bmpHandler.isReady()) {
        Serial.print(F(" Temp:")); Serial.print(sensorData.temperature, 1);
        Serial.print(F("°C Alt:")); Serial.print(sensorData.altitude, 1);
        Serial.print(F("m Max:")); Serial.print(flightController.getMaxAltitude(), 1);
        Serial.print('m');
    }
    
    // GPS data
    if (gpsHandler.isReady() && gpsHandler.hasValidLocation()) {
        Serial.print(F(" GPS:")); 
        Serial.print(sensorData.latitude, 6);
        Serial.print(',');
        Serial.print(sensorData.longitude, 6);
        Serial.print(F(" Sats:")); 
        Serial.print(sensorData.satellites);
    }
    
    Serial.println();
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================

float calculateVoltage(int analogPin) {
    int adcValue = analogRead(analogPin);
    return (adcValue * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_RATIO;
}

