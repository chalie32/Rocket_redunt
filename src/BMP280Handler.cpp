#include "BMP280Handler.h"
#include <Wire.h>

// Create the global instance
BMP280Handler bmpHandler;

BMP280Handler::BMP280Handler() : 
    seaLevelPressure(1013.25), // Standard sea level pressure in hPa
    rawAltitude(0),
    temperature(0),
    // Initialize Kalman filters with appropriate noise parameters
    // Altitude filter: moderate process noise for good responsiveness
    altitudeFilter(0.01f, 0.1f, 0.5f),
    // Temperature filter: low process noise as temperature changes slowly
    temperatureFilter(0.001f, 0.01f, 0.1f),
    // Initialize filtered values
    filteredAltitude(0),
    filteredTemperature(0),
    initialized(false) {
}

bool BMP280Handler::initialize() {
    // Initialize Wire1 for Teensy 4.1 using pins 17 (SDA1) and 16 (SCL1)
    Wire1.begin();
    Wire1.setClock(400000); // Set I2C clock speed to 400kHz for faster communication
    
    Serial.println(F("Initializing BMP280 on Wire1 (pins 17/16)..."));
    
    // The Adafruit_BMP280 library has different begin() method signatures
    // We need to use the correct one for our version of the library
    
    // First, try to initialize with just Wire1
    bmp = Adafruit_BMP280(&Wire1);
    
    // Try different I2C addresses for BMP280
    // First try the alternate address (0x76)
    if (!bmp.begin(0x76)) {
        // If that fails, try the default address (0x77)
        if (!bmp.begin(0x77)) {
            Serial.println(F("Could not find a valid BMP280 sensor on Wire1, check wiring!"));
            Serial.println(F("Trying to scan I2C bus for devices..."));
            
            // Scan I2C bus to find connected devices
            byte error, address;
            int deviceCount = 0;
            
            Serial.println(F("Scanning Wire1..."));
            for (address = 1; address < 127; address++) {
                Wire1.beginTransmission(address);
                error = Wire1.endTransmission();
                
                if (error == 0) {
                    Serial.print(F("I2C device found at address 0x"));
                    if (address < 16) {
                        Serial.print("0");
                    }
                    Serial.print(address, HEX);
                    Serial.println();
                    deviceCount++;
                }
            }
            
            if (deviceCount == 0) {
                Serial.println(F("No I2C devices found on Wire1. Check connections."));
            } else {
                Serial.print(deviceCount);
                Serial.println(F(" device(s) found on Wire1 bus."));
            }
            
            return false;
        }
    }
    
    // Configure the BMP280 sensor with more conservative settings
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                    Adafruit_BMP280::SAMPLING_X8,     // Pressure oversampling
                    Adafruit_BMP280::FILTER_X4,       // Filtering
                    Adafruit_BMP280::STANDBY_MS_250); // Standby time
    
    // Wait a moment for the sensor to be ready
    delay(100);
    
    // Take initial readings with error checking
    temperature = bmp.readTemperature();
    
    // Check if temperature reading is valid (not NaN)
    if (isnan(temperature)) {
        Serial.println(F("Failed to read valid temperature from BMP280!"));
        return false;
    }
    
    // Read pressure first (needed for altitude calculation)
    float pressure = bmp.readPressure();
    if (pressure == 0 || isnan(pressure)) {
        Serial.println(F("Failed to read valid pressure from BMP280!"));
        return false;
    }
    
    // Calculate altitude
    rawAltitude = bmp.readAltitude(seaLevelPressure);
    
    // Initialize Kalman filters with first readings
    filteredTemperature = temperatureFilter.update(temperature);
    filteredAltitude = altitudeFilter.update(rawAltitude);
    
    // Print initial readings for debugging
    Serial.print(F("Initial BMP280 readings - Temperature: "));
    Serial.print(temperature);
    Serial.print(F("°C, Pressure: "));
    Serial.print(pressure / 100.0); // Convert Pa to hPa
    Serial.print(F("hPa, Altitude: "));
    Serial.print(rawAltitude);
    Serial.println(F("m"));
    
    Serial.println(F("BMP280 initialized successfully with Kalman filters!"));
    initialized = true;
    return true;
}

void BMP280Handler::update() {
    if (!initialized) {
        // Try to initialize again if it failed before
        if (!initialize()) {
            return;
        }
    }
    
    // Read temperature with error checking
    float newTemp = bmp.readTemperature();
    if (!isnan(newTemp)) {
        temperature = newTemp;
        // Apply Kalman filter to temperature
        filteredTemperature = temperatureFilter.update(temperature);
    } else {
        Serial.println(F("Warning: Invalid temperature reading from BMP280"));
    }
    
    // Read pressure with error checking
    float pressure = bmp.readPressure();
    if (pressure == 0 || isnan(pressure)) {
        Serial.println(F("Warning: Invalid pressure reading from BMP280"));
        return;
    }
    
    // Read raw altitude
    rawAltitude = bmp.readAltitude(seaLevelPressure);
    if (isnan(rawAltitude)) {
        Serial.println(F("Warning: Invalid altitude calculation from BMP280"));
        return;
    }
    
    // Apply Kalman filter to altitude
    filteredAltitude = altitudeFilter.update(rawAltitude);
}

void BMP280Handler::setSeaLevelPressure(float pressure) {
    seaLevelPressure = pressure;
    Serial.print(F("Sea level pressure set to: "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
}

// Return filtered values by default
float BMP280Handler::getTemperature() {
    return filteredTemperature;
}

float BMP280Handler::getAltitude() {
    return filteredAltitude;
}

// New methods to get raw (unfiltered) values
float BMP280Handler::getRawTemperature() {
    return temperature;
}

float BMP280Handler::getRawAltitude() {
    return rawAltitude;
}

// Kalman filter configuration methods
void BMP280Handler::setAltitudeFilterParams(float q_angle, float q_velocity, float r_measure) {
    altitudeFilter.setProcessNoise(q_angle, q_velocity);
    altitudeFilter.setMeasurementNoise(r_measure);
}

void BMP280Handler::setTemperatureFilterParams(float q_angle, float q_velocity, float r_measure) {
    temperatureFilter.setProcessNoise(q_angle, q_velocity);
    temperatureFilter.setMeasurementNoise(r_measure);
}

void BMP280Handler::resetFilters() {
    altitudeFilter.reset();
    temperatureFilter.reset();
}

bool BMP280Handler::isReady() {
    return initialized;
} 