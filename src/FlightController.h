#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include "MPU6050Handler.h"
#include "BMP280Handler.h"

class FlightController {
public:
    // Flight states
    enum FlightState {
        GROUND,
        BOOST,
        COAST,
        APOGEE,
        DESCENT
    };

    FlightController(int relayPin);
    void begin();
    
    // Main update function
    void update(float currentAltitude, float pitch, float roll,
                float accX, float accY, float accZ);
    
    // Status getters
    FlightState getCurrentState() const { return currentState; }
    float getMaxAltitude() const { return maxAltitude; }
    float getVerticalVelocity() const { return verticalVelocity; }
    float getVerticalAcceleration() const { return lastVerticalAcc; }
    bool isApogeeDetected() const { return apogeeDetected; }
    bool isLaunchDetected() const { return launchDetected; }

    // Configuration getters/setters
    void setLaunchThresholds(float accel, float alt) {
        LAUNCH_ACCEL_THRESHOLD = accel;
        LAUNCH_ALTITUDE_THRESHOLD = alt;
    }
    
    void setApogeeThresholds(float minAlt, float descentAlt, float verticalVel) {
        MIN_APOGEE_ALTITUDE = minAlt;
        DESCENT_ALTITUDE_THRESHOLD = descentAlt;
        VERTICAL_VELOCITY_THRESHOLD = verticalVel;
    }

private:
    // Pin configuration
    const int RELAY_PIN;

    // Buffer management
    static const int ALTITUDE_BUFFER_SIZE = 15;
    static const int ACCEL_BUFFER_SIZE = 10;
    float altitudeBuffer[ALTITUDE_BUFFER_SIZE];
    float accelBuffer[ACCEL_BUFFER_SIZE];
    int altBufferIndex;
    int accelBufferIndex;

    // Flight state tracking
    FlightState currentState;
    unsigned long stateStartTime;
    unsigned long lastStateChange;
    bool apogeeDetected;
    bool launchDetected;

    // Sensor data tracking
    float maxAltitude;
    float lastAltitude;
    float lastAverageAltitude;
    float verticalVelocity;
    float lastVerticalAcc;
    unsigned long lastVelocityCalc;

    // Configurable thresholds
    float LAUNCH_ACCEL_THRESHOLD;      // g's of acceleration for launch detect
    float LAUNCH_ALTITUDE_THRESHOLD;    // meters above ground
    float MIN_APOGEE_ALTITUDE;         // minimum altitude to consider apogee detection
    float DESCENT_ALTITUDE_THRESHOLD;   // meters of altitude loss to confirm descent
    float VERTICAL_VELOCITY_THRESHOLD;  // m/s negative velocity to help confirm descent
    
    // Fixed thresholds
    static const unsigned long BOOST_MIN_DURATION = 500;    // minimum boost phase duration (ms)
    static const unsigned long COAST_MIN_DURATION = 500;    // minimum coast phase duration (ms)
    static const unsigned long DESCENT_CONFIRM_TIME = 200;  // ms of continuous descent to confirm

    // Internal helper functions
    float calculateBufferAverage(float* buffer, int size);
    float calculateVerticalAcceleration(float accX, float accY, float accZ, float pitch, float roll);
    void updateFlightState(float currentAltitude, float averageAltitude, float verticalAcc);
};

#endif // FLIGHT_CONTROLLER_H 