#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <Arduino.h>
#include <TinyGPS++.h>

class GPSHandler {
private:
    TinyGPSPlus gps;
    HardwareSerial* gpsSerial;
    
    // GPS data
    double latitude;
    double longitude;
    double altitude;
    unsigned long lastUpdate;
    bool validLocation;
    
    // GPS status
    bool initialized;
    
    // Configuration
    static const unsigned long DEFAULT_GPS_TIMEOUT = 1000;  // 60 seconds default timeout
    static const unsigned long STATUS_UPDATE_INTERVAL = 5000; // Status update every 5 seconds

public:
    GPSHandler();
    
    // Initialize GPS with specified serial port
    bool initialize(HardwareSerial* serial, unsigned long baudRate = 9600);
    
    // Initialize and wait for GPS lock with timeout
    bool initializeWithLock(HardwareSerial* serial, unsigned long baudRate = 9600, unsigned long timeout = DEFAULT_GPS_TIMEOUT);
    
    // Update GPS readings
    void update();
    
    // Get current latitude in degrees
    double getLatitude();
    
    // Get current longitude in degrees
    double getLongitude();
    
    // Get GPS altitude in meters
    double getGPSAltitude();
    
    // Check if GPS has valid location data
    bool hasValidLocation();
    
    // Get number of satellites
    int getSatellites();
    
    // Check if GPS is initialized
    bool isReady();
    
    // Get location age (time since last valid reading)
    unsigned long getLocationAge();
};

// Extern declaration for the global instance
extern GPSHandler gpsHandler;

#endif // GPS_HANDLER_H 