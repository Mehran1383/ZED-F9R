#ifndef DEVICE_H
#define DEVICE_H

#include <vector>

// UBX Sync Chars
#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

// UBX Class and ID for ACKs
#define UBX_CLASS_ACK 0x05
#define UBX_ID_ACK    0x01
#define UBX_ID_NAK    0x00

// Max length of messages 
#define ACK_MAX_LEN   10

// Return codes
#define ACK 1
#define NAK 0
#define ERROR_TIMEOUT -1 
#define ERROR_CHECKSUM -2
#define ERROR_CLOSED -3

#define PORT "/dev/ttyACM0" // Adjust as needed

class Device {
private:
    int fd;
    char port[256];

    void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB); 
public:
    Device(const char* port);
    ~Device(void);

    bool openSerialPort(void);
    void closeSerialPort(void); 
    bool sendUBXMessage(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload);
    bool readUBXMessage(const size_t headerSize, const size_t maxPayload, 
        uint8_t cls, uint8_t id, uint8_t* response);
    int waitForAck(uint8_t expectedCls, uint8_t expectedId);
};

#endif // DEVICE_H