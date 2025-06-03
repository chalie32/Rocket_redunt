#include "GPSHandler.h"

// Create the global instance
GPSHandler gpsHandler;

GPSHandler::GPSHandler() : 
    gpsSerial(nullptr),
    latitude(0.0),
    longitude(0.0),
    altitude(0.0),
    lastUpdate(0),
    validLocation(false),
    initialized(false) {
}

bool GPSHandler::initialize(HardwareSerial* serial, unsigned long baudRate) {
    if (serial == nullptr) {
        Serial.println(F("Error: GPS serial port is null"));
        return false;
    }
    
    gpsSerial = serial;
    gpsSerial->begin(baudRate);
    
    Serial.print(F("GPS initialized on serial port at "));
    Serial.print(baudRate);
    Serial.println(F(" baud"));
    
    // Wait a moment for GPS to stabilize
    delay(1000);
    
    initialized = true;
    lastUpdate = millis();
    
    Serial.println(F("GPS handler ready - waiting for satellite lock..."));
    return true;
}

bool GPSHandler::initializeWithLock(HardwareSerial* serial, unsigned long baudRate, unsigned long timeout) {
    // First initialize the GPS
    if (!initialize(serial, baudRate)) {
        return false;
    }
    
    // Wait for GPS lock
    Serial.println(F("Waiting for GPS lock..."));
    Serial.print(F("Timeout set to: "));
    Serial.print(timeout / 1000);
    Serial.println(F(" seconds"));
    
    unsigned long startTime = millis();
    bool lockAcquired = false;
    unsigned long lastStatusUpdate = 0;
    
    while (millis() - startTime < timeout && !lockAcquired) {
        update();
        
        if (hasValidLocation()) {
            lockAcquired = true;
            Serial.print(F("Lock acquired: "));
            Serial.print(getLatitude(), 6);
            Serial.print(F(", "));
            Serial.print(getLongitude(), 6);
            Serial.print(F(" (Sats: "));
            Serial.print(getSatellites());
            Serial.println(F(")"));
        } else if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
            Serial.print(F("Waiting... Sats: "));
            Serial.println(getSatellites());
            lastStatusUpdate = millis();
        }
        delay(100);
    }
    
    if (!lockAcquired) {
        Serial.println(F("⚠ No GPS lock - continuing anyway"));
    }
    
    return initialized;
}

void GPSHandler::update() {
    if (!initialized || gpsSerial == nullptr) {
        return;
    }
    
    // Read available GPS data
    while (gpsSerial->available() > 0) {
        char c = gpsSerial->read();
        
        if (gps.encode(c)) {
            // New valid sentence received
            if (gps.location.isValid()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                validLocation = true;
                lastUpdate = millis();
                
                // Print location when first acquired or every 10 seconds
                static unsigned long lastPrint = 0;
                if (lastPrint == 0 || (millis() - lastPrint > 10000)) {
                    Serial.println(F("GPS lock acquired!"));
                    lastPrint = millis();
                }
            }
            
            if (gps.altitude.isValid()) {
                altitude = gps.altitude.meters();
            }
        }
    }
    
    // Check if location data is getting old (older than 5 seconds)
    if (validLocation && (millis() - lastUpdate > 5000)) {
        // Don't immediately invalidate, but note that data is getting stale
        if (millis() - lastUpdate > 10000) {
            validLocation = false;
            Serial.println(F("GPS signal lost"));
        }
    }
}

double GPSHandler::getLatitude() {
    return latitude;
}

double GPSHandler::getLongitude() {
    return longitude;
}

double GPSHandler::getGPSAltitude() {
    return altitude;
}

bool GPSHandler::hasValidLocation() {
    return validLocation && gps.location.isValid();
}

int GPSHandler::getSatellites() {
    if (gps.satellites.isValid()) {
        return gps.satellites.value();
    }
    return 0;
}

bool GPSHandler::isReady() {
    return initialized;
}

unsigned long GPSHandler::getLocationAge() {
    if (gps.location.isValid()) {
        return gps.location.age();
    }
    return 0xFFFFFFFF; // Return max value if no valid location
} 