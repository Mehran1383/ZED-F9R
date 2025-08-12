#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#include "device.h"

const unsigned int maxWait = 1000000; // 1s 
const unsigned int minWait = 100000; // 0.1s

Device::Device(const char* device)
{
    this->fd = -1;
    this->device = device;
}

Device::~Device(void)
{
    closeSerialPort();
}

bool Device::openSerialPort(void) 
{
    if (fd > 0)
        return 0;

    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
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

bool Device::sendUBXMessage(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) 
{
    if (fd < 0)
        return 0;

    uint16_t length = payload.size();
    std::vector<uint8_t> message;

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

uint8_t* Device::readUBXMessage(const size_t headerSize, const size_t maxPayload, 
    uint8_t cls, uint8_t id, uint8_t* response) 
{
    if (fd < 0)
        return 0;

    uint8_t buffer[headerSize + maxPayload + 2]; // header + payload + checksum
    int totalRead = 0;

    // Wait and read the message
    for (int attempts = 0; attempts < (maxWait / minWait); ++attempts) {
        usleep(minWait);
        int r = read(fd, buffer, 1);
        if (r > 0) {
            totalRead += r;

            if (totalRead >= headerSize &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == cls && buffer[3] == id) {
                uint16_t len = buffer[4] | (buffer[5] << 8);

                while (totalRead < headerSize + len + 2) {
                    r = read(fd, buffer + totalRead, 1);
                    if (r > 0) 
                        totalRead += r;
                }

                response = buffer + 6;
                return 1;
            }
        }
    }
    return 0;
}

int Device::waitForAck(uint8_t expectedCls, uint8_t expectedId) 
{
    if(fd < 0)
        return ERROR_CLOSED

    uint8_t buffer[10];
    int totalRead = 0;
    int wait = 0;
    uint8_t ckA = 0, ckB = 0;

    while (wait < maxWait) {
        usleep(minWait);
        int bytesRead = read(fd, &buffer[totalRead], 1);
        if (bytesRead > 0) {
            totalRead += bytesRead;

            // Shift if too long
            if (totalRead > ACK_MAX_LEN) {
                memmove(buffer, buffer + 1, 9);
                totalRead--;
            }

            // Look for sync
            if (totalRead >= 10 &&
                buffer[0] == UBX_SYNC_CHAR1 &&
                buffer[1] == UBX_SYNC_CHAR2 &&
                buffer[2] == UBX_CLASS_ACK) {
                if (buffer[3] == UBX_ID_ACK &&
                    buffer[6] == expectedCls &&
                    buffer[7] == expectedId) {
                    calculateChecksum(std::vector<uint8_t>(buffer + 2, buffer + 7), ckA, ckB);
                    if (ckA == buffer[8] && ckB == buffer[9])
                        return ACK;
                    return ERROR_CHECKSUM;
                }
                else if (buffer[3] == UBX_ID_NAK &&
                         buffer[6] == expectedCls &&
                         buffer[7] == expectedId) {
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
    Device dev(PORT);
    if (dev.openSerialPort() != 1) 
        return 1;

    // UBX-CFG-VALSET example payload (set CFG-RATE-MEAS = 100ms)
    std::vector<uint8_t> payload = {
        0x00,                   // version
        0x00,                   // layer (0: RAM only)
        0x00, 0x00,             // reserved

        // KeyID = 0x30210001 (CFG-RATE-MEAS), Value = 100
        0x01, 0x00, 0x21, 0x30, // Key ID (LE)
        0x64, 0x00, 0x00, 0x00  // Value = 100 (LE)
    };

    std::cout << "Sending UBX-CFG-VALSET..." << std::endl;
    if (dev.sendUBXMessage(0x06, 0x8A, payload)) {
        int result = dev.waitForAck(0x06, 0x8A);
        switch (result)
        {
        case -2: 
            std::cerr << "⚠ CheckSum Error!" << std::endl;
            break;
        case -1:
            std::cerr << "⚠ No ACK/NAK received (timeout)" << std::endl;
            break;
        case 0:
            std::cerr << "✘ NAK received!" << std::endl;
            break;
        case 1:
            std::cout << "✔ ACK received!" << std::endl;
            break;
        default:
            std::cout << "Open Serial port first!" << std::endl;
        }
    } else {
        std::cerr << "Failed to send UBX message." << std::endl;
    }

    dev.closeSerialPort();
    return 0;
}
