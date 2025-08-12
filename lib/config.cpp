#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#include "config.h"

const unsigned int maxWait = 5000000; // 5s 
const unsigned int minWait = 1000000; // 1s

bool Config::openSerialPort(const char* device) 
{
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

void Config::closeSerialPort(void)
{
    if (fd > 0) {
        close(fd);
        fd = -1;
    }
}

void Config::calculateChecksum(const std::vector<uint8_t>& payload, uint8_t& ckA, uint8_t& ckB) 
{
    ckA = 0;
    ckB = 0;
    for (uint8_t byte : payload) {
        ckA += byte;
        ckB += ckA;
    }
}

bool Config::sendUBXMessage(int fd, uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) 
{
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

int Config::waitForAck(int fd, uint8_t expectedCls, uint8_t expectedId) 
{
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
    Config conf;
    if (conf.openSerialPort(PORT) != 1) 
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
    if (conf.sendUBXMessage(fd, 0x06, 0x8A, payload)) {
        int result = conf.waitForAck(fd, 0x06, 0x8A);
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
        }
    } else {
        std::cerr << "Failed to send UBX message." << std::endl;
    }

    conf.closeSerialPort();
    return 0;
}
