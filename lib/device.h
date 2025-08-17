#ifndef DEVICE_H
#define DEVICE_H

#include <vector>

// Return codes
#define ACK 1
#define NAK 0
#define ERROR_TIMEOUT -1 
#define ERROR_CHECKSUM -2
#define SOCK_NOT_OPEN -3

class Device {
private:
    int fd;
    const char* port;

    void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB); 
public:
    Device(const char* port);
    ~Device(void);

    bool openSerialPort(void);
    void closeSerialPort(void); 
    ssize_t sendUBXMessage(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload);
    bool readUBXMessage(uint8_t cls, uint8_t id, uint8_t* response);
    int waitForAck(uint8_t expectedCls, uint8_t expectedId);
};

#endif // DEVICE_H