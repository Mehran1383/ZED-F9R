#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#include "device.h"

// UBX Sync Chars
#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

// UBX Class and ID for ACKs
#define UBX_CLASS_ACK 0x05
#define UBX_ID_ACK    0x01
#define UBX_ID_NAK    0x00

// Length constants 
#define ACK_MAX_LEN 10
#define HEADER_SIZE 6
#define CHECKSUM_SIZE 2

const unsigned int maxWait = 1000000; // 1s 
const unsigned int minWait = 1000; // 1ms

Device::Device(std::string portName) : fd(-1), port(portName)
{
}

Device::~Device(void)
{
    closeSerialPort();
}

bool Device::openSerialPort(void) 
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

void Device::closeSerialPort(void)
{
    if (fd > 0) {
        close(fd);
        fd = -1;
    }
}

void Device::calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB) 
{
    ckA = 0;
    ckB = 0;
    for (uint8_t byte : payload) {
        ckA += byte;
        ckB += ckA;
    }
}

ssize_t Device::sendUBXMessage(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) 
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

bool Device::readUBXMessage(uint8_t cls, uint8_t id, uint8_t* response) 
{
    uint8_t buffer[HEADER_SIZE];
    uint8_t ckA, ckB;
    int totalRead = 0;
    unsigned int wait = 0;
    bool result;

    if (fd < 0)
        return 0;

    while(wait < maxWait) {
        usleep(minWait);
        int r = read(fd, buffer + totalRead, 1);
        if (r > 0) {
            totalRead += r;

            if (totalRead >= HEADER_SIZE &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == cls && buffer[3] == id) {

                uint16_t len = buffer[4] | (buffer[5] << 8);
                uint8_t* payload = new uint8_t[len + CHECKSUM_SIZE];
                totalRead = 0;

                while (totalRead < len + CHECKSUM_SIZE) {
                    r = read(fd, payload + totalRead, 1);
                    if (r > 0) {
                        totalRead += r;
                    } else if (r < 0) {
                        delete [] payload;
                        return 0;
                    }    
                }

                std::vector<uint8_t> message;
                for (int i = 2; i < HEADER_SIZE; i++)
                    message.push_back(buffer[i]);
                for (int i = 0; i < len + CHECKSUM_SIZE; i++)
                    message.push_back(payload[i]);

                calculateChecksum(message, ckA, ckB);
                if (ckA == payload[len] && ckB == payload[len + 1])
                    result = 1;
                else
                    result = 0;

                delete [] payload;
                return result;
            }
        }
        wait += minWait;
    }
    return 0;
}

int Device::waitForAck(uint8_t expectedCls, uint8_t expectedId) 
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

int main() 
{
    Device dev;
    if (dev.openSerialPort() != 1) 
        return 1;

    // // UBX-CFG-VALSET example payload (set CFG-RATE-MEAS = 100ms)
    // std::vector<uint8_t> payload = {
    //     0x00,                   // version
    //     0x00,                   // layer (0: RAM only)
    //     0x00, 0x00,             // reserved

    //     // KeyID = 0x30210001 (CFG-RATE-MEAS), Value = 100
    //     0x01, 0x00, 0x21, 0x30, // Key ID (LE)
    //     0x64, 0x00, 0x00, 0x00  // Value = 100 (LE)
    // };

    // std::cout << "Sending UBX-CFG-VALSET..." << std::endl;
    // if (dev.sendUBXMessage(0x06, 0x8A, payload) > 0) {
    //     int result = dev.waitForAck(0x06, 0x8A);
    //     switch (result)
    //     {
    //     case ERROR_CHECKSUM: 
    //         std::cerr << "⚠ CheckSum Error!" << std::endl;
    //         break;
    //     case ERROR_TIMEOUT:
    //         std::cerr << "⚠ No ACK/NAK received (timeout)" << std::endl;
    //         break;
    //     case NAK:
    //         std::cerr << "✘ NAK received!" << std::endl;
    //         break;
    //     case ACK:
    //         std::cout << "✔ ACK received!" << std::endl;
    //         break;
    //     default:
    //         std::cout << "⚠ Open Serial port first!" << std::endl;
    //     }
    // } else {
    //     std::cerr << "⚠ Failed to send UBX message." << std::endl;
    // }

    std::cout << "📤 Sending UBX-MON-VER poll..." << std::endl;
    if (!sendUBXMessage(fd, 0x0A, 0x04, {})) {
        std::cerr << "Failed to send UBX-MON-VER." << std::endl;
        return 1;
    }

    std::cout << "📥 Waiting for UBX-MON-VER response..." << std::endl;
    readUBXMonVer(fd);

    dev.closeSerialPort();
    return 0;
}
