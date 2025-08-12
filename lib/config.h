#ifndef CONFIG_H
#define CONFIG_H

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
#define ERROR -1 

#define PORT "/dev/ttyACM0" // Adjust as needed

class Config {
private:
    int fd;
    void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB); 
public:
    Config();
    ~Config();

    bool openSerialPort(const char* device);
    void closeSerialPort(void); 
    bool sendUBXMessage(int fd, uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload);
    int waitForAck(int fd, uint8_t expectedCls, uint8_t expectedId);
};






#endif // CONFIG_H