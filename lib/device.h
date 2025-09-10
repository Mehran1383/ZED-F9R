#ifndef DEVICE_H
#define DEVICE_H

#include <vector>
#include <string>

// Return codes
#define ACK 1
#define NAK 0
#define ERROR_TIMEOUT -1 
#define ERROR_CHECKSUM -2
#define SOCK_NOT_OPEN -3

class Device {
private:
    int fd;
    std::string port;

    void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB); 
public:
    Device(const std::string port = "/dev/ttyACM0");
    ~Device(void);

    bool openSerialPort(void);
    void closeSerialPort(void); 
    bool sendUBXMessage(const uint8_t cls, const uint8_t id, const std::vector<uint8_t>& payload);
    bool readUBXMessage(const uint8_t cls, const uint8_t id, std::vector<uint8_t>& response);
    int waitForAck(const uint8_t expectedCls, const uint8_t expectedId);
};

#endif // DEVICE_H