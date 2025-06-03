#include "LoRaHandler.h"

// Create the global instance
LoRaHandler loraHandler;

LoRaHandler::LoRaHandler() :
    frequency(915000000),    // 915 MHz (US frequency)
    spreadingFactor(7),      // SF7 for faster transmission
    signalBandwidth(125000), // 125 kHz
    codingRate(5),           // 4/5 coding rate
    txPower(17),             // 17 dBm
    deviceId(1),
    packetCounter(0),
    lastTransmission(0),
    transmissionInterval(5000), // 5 seconds default
    initialized(false),
    receiving(false) {
}

bool LoRaHandler::initialize(long freq, uint8_t devId) {
    return initialize(freq, 7, 125000, 5, 17, devId);
}

bool LoRaHandler::initialize(long freq, int sf, long bw, int cr, int power, uint8_t devId) {
    frequency = freq;
    spreadingFactor = sf;
    signalBandwidth = bw;
    codingRate = cr;
    txPower = power;
    deviceId = devId;
    
    Serial.print(F("Initializing LoRa at "));
    Serial.print(frequency / 1000000.0, 1);
    Serial.println(F(" MHz..."));
    
    // Set LoRa pins for Teensy - Custom pin mapping
    // CS=10, RESET=5, DIO0=9
    LoRa.setPins(10, 5, 9);
    
    if (!LoRa.begin(frequency)) {
        Serial.println(F("LoRa initialization failed!"));
        return false;
    }
    
    // Configure LoRa parameters for efficiency
    LoRa.setSpreadingFactor(spreadingFactor);
    LoRa.setSignalBandwidth(signalBandwidth);
    LoRa.setCodingRate4(codingRate);
    LoRa.setTxPower(txPower);
    
    // Enable CRC for error detection
    LoRa.enableCrc();
    
    initialized = true;
    Serial.println(F("LoRa initialized successfully!"));
    printConfiguration();
    
    // Start in receiving mode
    startReceiving();
    
    return true;
}

void LoRaHandler::setTransmissionInterval(unsigned long intervalMs) {
    transmissionInterval = intervalMs;
    Serial.print(F("LoRa transmission interval set to "));
    Serial.print(intervalMs / 1000.0, 1);
    Serial.println(F(" seconds"));
}

void LoRaHandler::setDeviceId(uint8_t id) {
    deviceId = id;
    Serial.print(F("LoRa device ID set to "));
    Serial.println(id);
}

uint8_t LoRaHandler::calculateChecksum(const SensorPacket* packet) {
    uint8_t checksum = 0;
    const uint8_t* data = (const uint8_t*)packet;
    
    // Calculate checksum for all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(SensorPacket) - 1; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

bool LoRaHandler::validatePacket(const SensorPacket* packet) {
    uint8_t calculatedChecksum = calculateChecksum(packet);
    return (calculatedChecksum == packet->checksum);
}

bool LoRaHandler::transmitSensorData(float yaw, float pitch, float roll, 
                                   float accX, float accY, float accZ,
                                   float temp, float alt, 
                                   double lat, double lon, uint8_t sats,
                                   float vol0, float vol1) {
    if (!initialized) {
        return false;
    }
    
    // Create packet
    SensorPacket packet;
    packet.timestamp = millis();
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.accX = accX;
    packet.accY = accY;
    packet.accZ = accZ;
    packet.temperature = temp;
    packet.altitude = alt;
    packet.latitude = lat;
    packet.longitude = lon;
    packet.voltage0 = vol0;
    packet.voltage1 = vol1;
    packet.satellites = sats;
    packet.deviceId = deviceId;
    packet.packetCount = ++packetCounter;
    packet.checksum = calculateChecksum(&packet);
    
    // Stop receiving mode for transmission
    if (receiving) {
        LoRa.idle();
        receiving = false;
    }
    
    // Send packet
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&packet, sizeof(SensorPacket));
    bool success = LoRa.endPacket();
    
    if (success) {
        lastTransmission = millis();
        Serial.print(F("LoRa TX ["));
        Serial.print(deviceId);
        Serial.print(F("]: Packet #"));
        Serial.print(packetCounter);
        Serial.print(F(" ("));
        Serial.print(sizeof(SensorPacket));
        Serial.println(F(" bytes)"));
    } else {
        Serial.println(F("LoRa transmission failed!"));
    }
    
    // Return to receiving mode
    startReceiving();
    
    return success;
}

bool LoRaHandler::shouldTransmit() {
    return (millis() - lastTransmission >= transmissionInterval);
}

void LoRaHandler::startReceiving() {
    if (initialized && !receiving) {
        LoRa.receive();
        receiving = true;
    }
}

void LoRaHandler::stopReceiving() {
    if (initialized && receiving) {
        LoRa.idle();
        receiving = false;
    }
}

bool LoRaHandler::checkForIncomingData() {
    if (!initialized || !receiving) {
        return false;
    }
    
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) {
        return false; // No packet received
    }
    
    if (packetSize != sizeof(SensorPacket)) {
        Serial.print(F("LoRa RX: Invalid packet size ("));
        Serial.print(packetSize);
        Serial.print(F(" bytes, expected "));
        Serial.print(sizeof(SensorPacket));
        Serial.println(F(")"));
        return false;
    }
    
    // Read packet
    SensorPacket receivedPacket;
    LoRa.readBytes((uint8_t*)&receivedPacket, sizeof(SensorPacket));
    
    // Get signal quality
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    // Validate packet
    if (!validatePacket(&receivedPacket)) {
        Serial.println(F("LoRa RX: Checksum validation failed"));
        return false;
    }
    
    // Don't process our own packets
    if (receivedPacket.deviceId == deviceId) {
        return false;
    }
    
    // Print received data
    printReceivedData(&receivedPacket, rssi, snr);
    
    return true;
}

void LoRaHandler::printReceivedData(const SensorPacket* packet, int rssi, float snr) {
    Serial.print(F("LoRa RX ["));
    Serial.print(packet->deviceId);
    Serial.print(F("]: "));
    Serial.print(F("Y:"));
    Serial.print(packet->yaw, 1);
    Serial.print(F(" P:"));
    Serial.print(packet->pitch, 1);
    Serial.print(F(" R:"));
    Serial.print(packet->roll, 1);
    Serial.print(F(" T:"));
    Serial.print(packet->temperature, 1);
    Serial.print(F("°C A:"));
    Serial.print(packet->altitude, 1);
    Serial.print(F("m V0:"));
    Serial.print(packet->voltage0, 2);
    Serial.print(F("V V1:"));
    Serial.print(packet->voltage1, 2);
    Serial.print(F("V"));
    
    if (packet->satellites > 0) {
        Serial.print(F(" GPS:"));
        Serial.print(packet->latitude, 6);
        Serial.print(F(","));
        Serial.print(packet->longitude, 6);
        Serial.print(F("("));
        Serial.print(packet->satellites);
        Serial.print(F("sat)"));
    }
    
    Serial.print(F(" #"));
    Serial.print(packet->packetCount);
    Serial.print(F(" RSSI:"));
    Serial.print(rssi);
    Serial.print(F(" SNR:"));
    Serial.println(snr, 1);
}

bool LoRaHandler::isReady() {
    return initialized;
}

uint16_t LoRaHandler::getPacketCount() {
    return packetCounter;
}

int LoRaHandler::getLastRSSI() {
    return LoRa.packetRssi();
}

float LoRaHandler::getLastSNR() {
    return LoRa.packetSnr();
}

void LoRaHandler::printConfiguration() {
    Serial.println(F("=== LoRa Configuration ==="));
    Serial.print(F("Frequency: "));
    Serial.print(frequency / 1000000.0, 1);
    Serial.println(F(" MHz"));
    Serial.print(F("Spreading Factor: "));
    Serial.println(spreadingFactor);
    Serial.print(F("Bandwidth: "));
    Serial.print(signalBandwidth / 1000.0, 1);
    Serial.println(F(" kHz"));
    Serial.print(F("Coding Rate: 4/"));
    Serial.println(codingRate);
    Serial.print(F("TX Power: "));
    Serial.print(txPower);
    Serial.println(F(" dBm"));
    Serial.print(F("Device ID: "));
    Serial.println(deviceId);
    Serial.print(F("Packet Size: "));
    Serial.print(sizeof(SensorPacket));
    Serial.println(F(" bytes"));
    Serial.println(F("========================"));
} 