#ifndef BMP280_HANDLER_H
#define BMP280_HANDLER_H

#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include "KalmanFilter.h"

class BMP280Handler {
private:
    Adafruit_BMP280 bmp;
    
    // Store the sea level pressure for altitude calculations
    float seaLevelPressure;
    
    // Raw sensor values
    float rawAltitude;
    float temperature;
    
    // Kalman filters for sensor data
    KalmanFilter altitudeFilter;
    KalmanFilter temperatureFilter;
    
    // Filtered values
    float filteredAltitude;
    float filteredTemperature;
    
    // Flag to track initialization status
    bool initialized;

public:
    BMP280Handler();
    
    // Initialize the BMP280 sensor
    bool initialize();
    
    // Update sensor readings
    void update();
    
    // Set the sea level pressure (in hPa) for more accurate altitude
    void setSeaLevelPressure(float pressure);
    
    // Get the current filtered temperature in Celsius
    float getTemperature();
    
    // Get the current filtered altitude in meters
    float getAltitude();
    
    // Get the current raw (unfiltered) temperature in Celsius
    float getRawTemperature();
    
    // Get the current raw (unfiltered) altitude in meters
    float getRawAltitude();
    
    // Kalman filter configuration
    void setAltitudeFilterParams(float q_angle, float q_velocity, float r_measure);
    void setTemperatureFilterParams(float q_angle, float q_velocity, float r_measure);
    void resetFilters();
    
    // Check if sensor is initialized
    bool isReady();
};

// Extern declaration for the global instance
extern BMP280Handler bmpHandler;

#endif // BMP280_HANDLER_H 