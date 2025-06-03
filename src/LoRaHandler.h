#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#include <Arduino.h>
#include <LoRa.h>

// Data packet structure for efficient transmission
struct SensorPacket {
    uint32_t timestamp;     // 4 bytes - milliseconds since boot
    float yaw;             // 4 bytes
    float pitch;           // 4 bytes  
    float roll;            // 4 bytes
    float accX;            // 4 bytes
    float accY;            // 4 bytes
    float accZ;            // 4 bytes
    float temperature;     // 4 bytes
    float altitude;        // 4 bytes
    double latitude;       // 8 bytes
    double longitude;      // 8 bytes
    float voltage0;        // 4 bytes - A0 voltage reading
    float voltage1;        // 4 bytes - A1 voltage reading
    uint8_t satellites;    // 1 byte
    uint8_t deviceId;      // 1 byte - unique device identifier
    uint16_t packetCount;  // 2 bytes - packet counter
    uint8_t checksum;      // 1 byte - simple checksum
} __attribute__((packed)); // Total: 66 bytes

class LoRaHandler {
private:
    // LoRa configuration
    long frequency;
    int spreadingFactor;
    long signalBandwidth;
    int codingRate;
    int txPower;
    
    // Device configuration
    uint8_t deviceId;
    uint16_t packetCounter;
    
    // Timing
    unsigned long lastTransmission;
    unsigned long transmissionInterval;
    
    // Status
    bool initialized;
    bool receiving;
    
    // Internal methods
    uint8_t calculateChecksum(const SensorPacket* packet);
    bool validatePacket(const SensorPacket* packet);
    void printReceivedData(const SensorPacket* packet, int rssi, float snr);

public:
    LoRaHandler();
    
    // Initialize LoRa with default or custom settings
    bool initialize(long freq = 915000000, uint8_t devId = 1);
    bool initialize(long freq, int sf, long bw, int cr, int power, uint8_t devId);
    
    // Configure transmission settings
    void setTransmissionInterval(unsigned long intervalMs);
    void setDeviceId(uint8_t id);
    
    // Transmission
    bool transmitSensorData(float yaw, float pitch, float roll, 
                           float accX, float accY, float accZ,
                           float temp, float alt, 
                           double lat, double lon, uint8_t sats,
                           float vol0, float vol1);
    
    bool shouldTransmit();
    
    // Reception
    void startReceiving();
    void stopReceiving();
    bool checkForIncomingData();
    
    // Status
    bool isReady();
    uint16_t getPacketCount();
    int getLastRSSI();
    float getLastSNR();
    
    // Utility
    void printConfiguration();
};

// Extern declaration for the global instance
extern LoRaHandler loraHandler;

#endif // LORA_HANDLER_H 