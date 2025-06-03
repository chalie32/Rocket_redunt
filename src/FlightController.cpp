#include "FlightController.h"

FlightController::FlightController(int relayPin) : 
    RELAY_PIN(relayPin),
    altBufferIndex(0),
    accelBufferIndex(0),
    currentState(GROUND),
    stateStartTime(0),
    lastStateChange(0),
    apogeeDetected(false),
    launchDetected(false),
    maxAltitude(0),
    lastAltitude(0),
    lastAverageAltitude(0),
    verticalVelocity(0),
    lastVerticalAcc(0),
    lastVelocityCalc(0) {
    
    // Set default thresholds
    LAUNCH_ACCEL_THRESHOLD = 2.0;     // g's
    LAUNCH_ALTITUDE_THRESHOLD = 10.0;  // meters
    MIN_APOGEE_ALTITUDE = 20.0;       // meters
    DESCENT_ALTITUDE_THRESHOLD = 2.0;  // meters
    VERTICAL_VELOCITY_THRESHOLD = -1.0; // m/s
}

void FlightController::begin() {
    // Initialize relay pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    // Initialize buffers
    for (int i = 0; i < ALTITUDE_BUFFER_SIZE; i++) {
        altitudeBuffer[i] = 0;
    }
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
        accelBuffer[i] = 0;
    }
}

float FlightController::calculateBufferAverage(float* buffer, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

float FlightController::calculateVerticalAcceleration(float accX, float accY, float accZ, float pitch, float roll) {
    // Convert pitch and roll to radians
    float pitchRad = pitch * PI / 180.0;
    float rollRad = roll * PI / 180.0;
    
    // Calculate vertical acceleration considering rocket orientation
    float verticalAcc = -accZ * cos(pitchRad) * cos(rollRad) +
                        accY * sin(rollRad) +
                        accX * sin(pitchRad) * cos(rollRad);
    
    return verticalAcc - 1.0; // Subtract 1g to get acceleration relative to free-fall
}

void FlightController::updateFlightState(float currentAltitude, float averageAltitude, float verticalAcc) {
    unsigned long currentTime = millis();
    bool stateChanged = false;
    FlightState newState = currentState;
    static unsigned long descentStartTime = 0;

    switch (currentState) {
        case GROUND:
            // Detect launch using both acceleration and altitude
            if (verticalAcc > LAUNCH_ACCEL_THRESHOLD && currentAltitude > LAUNCH_ALTITUDE_THRESHOLD) {
                newState = BOOST;
                launchDetected = true;
                Serial.println(F("Launch detected! Entering BOOST phase"));
                stateChanged = true;
            }
            break;

        case BOOST:
            // Transition to coast when vertical acceleration drops near zero
            if (currentTime - stateStartTime > BOOST_MIN_DURATION && 
                abs(verticalAcc) < 0.2) {
                newState = COAST;
                Serial.println(F("Entering COAST phase"));
                stateChanged = true;
            }
            break;

        case COAST:
            // Complex apogee detection using multiple criteria
            if (currentTime - stateStartTime > COAST_MIN_DURATION &&
                currentAltitude > MIN_APOGEE_ALTITUDE &&
                verticalVelocity < VERTICAL_VELOCITY_THRESHOLD &&
                averageAltitude < (maxAltitude - DESCENT_ALTITUDE_THRESHOLD)) {
                
                // Additional check for consistent descent
                if (currentTime - descentStartTime >= DESCENT_CONFIRM_TIME) {
                    newState = APOGEE;
                    apogeeDetected = true;
                    Serial.print(F("APOGEE DETECTED at altitude: "));
                    Serial.print(maxAltitude);
                    Serial.println(F(" meters"));
                    stateChanged = true;
                    
                    // Trigger relay
                    digitalWrite(RELAY_PIN, HIGH);
                }
            } else {
                // Reset descent timer if we're not descending
                if (verticalVelocity >= 0) {
                    descentStartTime = currentTime;
                }
            }
            break;

        case APOGEE:
            // Transition to descent after apogee
            if (currentAltitude < (maxAltitude - DESCENT_ALTITUDE_THRESHOLD * 2)) {
                newState = DESCENT;
                Serial.println(F("Entering DESCENT phase"));
                stateChanged = true;
            }
            break;

        case DESCENT:
            // No state change after descent begins
            break;
    }

    // Update state if changed
    if (stateChanged) {
        currentState = newState;
        lastStateChange = currentTime;
        stateStartTime = currentTime;
    }
}

void FlightController::update(float currentAltitude, float pitch, float roll,
                            float accX, float accY, float accZ) {
    unsigned long currentTime = millis();

    // Calculate vertical acceleration
    float verticalAcc = calculateVerticalAcceleration(accX, accY, accZ, pitch, roll);
    lastVerticalAcc = verticalAcc;
    
    // Update acceleration buffer
    accelBuffer[accelBufferIndex] = verticalAcc;
    accelBufferIndex = (accelBufferIndex + 1) % ACCEL_BUFFER_SIZE;
    float averageAccel = calculateBufferAverage(accelBuffer, ACCEL_BUFFER_SIZE);
    
    // Update altitude buffer
    altitudeBuffer[altBufferIndex] = currentAltitude;
    altBufferIndex = (altBufferIndex + 1) % ALTITUDE_BUFFER_SIZE;
    float averageAltitude = calculateBufferAverage(altitudeBuffer, ALTITUDE_BUFFER_SIZE);
    
    // Calculate vertical velocity (every 100ms)
    if (currentTime - lastVelocityCalc >= 100) {
        verticalVelocity = (currentAltitude - lastAltitude) / ((currentTime - lastVelocityCalc) / 1000.0);
        lastAltitude = currentAltitude;
        lastVelocityCalc = currentTime;
    }
    
    // Update max altitude if still ascending
    if (averageAltitude > maxAltitude) {
        maxAltitude = averageAltitude;
    }
    
    // Update flight state
    updateFlightState(currentAltitude, averageAltitude, averageAccel);
    
    lastAverageAltitude = averageAltitude;
} 