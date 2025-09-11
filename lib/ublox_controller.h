#ifndef UBLOX_CONTROLLER_H
#define UBLOX_CONTROLLER_H

#include <vector>
#include <string>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// Return codes
#define ACK 1
#define NAK 0
#define ERROR_TIMEOUT -1 
#define ERROR_CHECKSUM -2
#define SOCK_NOT_OPEN -3

// UBX Sync Chars
#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

// UBX Class and ID for ACKs
#define UBX_CLS_ACK 0x05
#define UBX_ID_ACK    0x01
#define UBX_ID_NAK    0x00

// Length constants 
#define ACK_MAX_LEN 10
#define HEADER_SIZE 6
#define CHECKSUM_SIZE 2

const unsigned int maxWait = 1000000; // 1s 
const unsigned int minWait = 1000; // 1ms

class UbloxController {
private:
    int fd;
    std::string port;

    void calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB); 
public:
    UbloxController(const std::string port = "/dev/ttyACM0");
    ~UbloxController();

    bool openSerialPort();
    void closeSerialPort(); 
    bool sendUBXMessage(const uint8_t cls, const uint8_t id, const std::vector<uint8_t>& payload);
    bool readUBXMessage(const uint8_t cls, const uint8_t id, std::vector<uint8_t>& response);
    int waitForAck(const uint8_t expectedCls, const uint8_t expectedId);
    string getPort() { retunr port; } 
};

UbloxController::UbloxController(const std::string portName) : fd(-1), port(portName)
{
}

UbloxController::~UbloxController()
{
    closeSerialPort();
}

bool UbloxController::openSerialPort() 
{
    if (fd > 0)
        return 0;

    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) 
        return 0;
    
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);

    // Set required flags
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return 1;
}

void UbloxController::closeSerialPort()
{
    if (fd > 0) {
        close(fd);
        fd = -1;
    }
}

void UbloxController::calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB) 
{
    ckA = 0;
    ckB = 0;
    for (uint8_t byte : payload) {
        ckA += byte;
        ckB += ckA;
    }
}

bool UbloxController::sendUBXMessage(const uint8_t cls, const uint8_t id, const std::vector<uint8_t>& payload) 
{
    uint16_t length = payload.size();
    std::vector<uint8_t> message;

    if (fd < 0)
        return 0;

    message.push_back(UBX_SYNC_CHAR1);
    message.push_back(UBX_SYNC_CHAR2);
    message.push_back(cls);
    message.push_back(id);
    message.push_back(length & 0xFF);
    message.push_back((length >> 8) & 0xFF);
    message.insert(message.end(), payload.begin(), payload.end());

    uint8_t ckA, ckB;
    calculateChecksum(std::vector<uint8_t>(message.begin() + 2, message.end()), ckA, ckB);
    message.push_back(ckA);
    message.push_back(ckB);

    ssize_t written = write(fd, message.data(), message.size());
    return written == static_cast<ssize_t>(message.size());
}

bool UbloxController::readUBXMessage(const uint8_t cls, const uint8_t id, std::vector<uint8_t>& response) 
{
    std::vector<uint8_t> buffer;
    uint8_t ckA, ckB;
    int totalRead = 0;
    unsigned int wait = 0;
    bool result;

    if (fd < 0)
        return 0;

    buffer.resize(HEADER_SIZE);

    while(wait < maxWait) {
        usleep(minWait);
        int r = read(fd, buffer.data() + totalRead, 1);
        if (r > 0) {
            totalRead += r;

            if (totalRead >= HEADER_SIZE &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == cls && buffer[3] == id) {

                uint16_t len = buffer[4] | (buffer[5] << 8);
                uint16_t totalLen = HEADER_SIZE + len + CHECKSUM_SIZE;
                buffer->resize(totalLen);

                while (totalRead < totalLen) {
                    r = read(fd, buffer.data() + totalRead, 1);
                    if (r > 0)
                        totalRead += r;
                    else if (r < 0) 
                        return 0;  
                }

                calculateChecksum(std::vector<uint8_t>(buffer.begin() + 2, buffer.end() - CHECKSUM_SIZE), ckA, ckB);
                if (ckA == buffer[totalLen - 2] && ckB == buffer[totalLen - 1])
                    result = 1;
                else
                    result = 0;

                response = buffer;
                return result;
            }
        }
        wait += minWait;
    }
    return 0;
}

int UbloxController::waitForAck(const uint8_t expectedCls, const uint8_t expectedId) 
{
    uint8_t buffer[ACK_MAX_LEN];
    int totalRead = 0;
    unsigned int wait = 0;
    uint8_t ckA, ckB;

    if (fd < 0)
        return SOCK_NOT_OPEN;

    // A UBX-ACK-ACK is sent at least within one second
    usleep(maxWait);

    while (wait < maxWait) {
        usleep(minWait);
        int bytesRead = read(fd, buffer + totalRead, 1);
        if (bytesRead > 0) {
            totalRead += bytesRead;

            // Shift if too long
            if (totalRead > ACK_MAX_LEN) {
                memmove(buffer, buffer + 1, 9);
                totalRead--;
            }

            if (totalRead >= ACK_MAX_LEN &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == UBX_CLASS_ACK &&
                buffer[6] == expectedCls &&
                buffer[7] == expectedId) {

                if (buffer[3] == UBX_ID_ACK) {
                    calculateChecksum(std::vector<uint8_t>(buffer + 2, buffer + 7), ckA, ckB);
                    if (ckA == buffer[8] && ckB == buffer[9])
                        return ACK;
                    return ERROR_CHECKSUM;
                } else if (buffer[3] == UBX_ID_NAK) {
                    calculateChecksum(std::vector<uint8_t>(buffer + 2, buffer + 7), ckA, ckB);
                    if (ckA == buffer[8] && ckB == buffer[9])
                        return NAK;
                    return ERROR_CHECKSUM;
                }
            }
        }
        wait += minWait;
    }

    return ERROR_TIMEOUT;
}

#endif // UBLOX_CONTROLLER_H