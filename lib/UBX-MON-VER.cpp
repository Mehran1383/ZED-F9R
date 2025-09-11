#include <iostream>
#include <sstream>

#include "ublox_controller.h"

#define UBX_CLS_MON_VER 0x0A
#define UBX_ID_MON_VER 0x04

#define SW_BYTE_OFFSET 30
#define HW_BYTE_OFFSET 10
#define EXT_BYTE_OFFSET 30

using namespace std;

int main() 
{
    UbloxController controller;
    if (!controller.openSerialPort()) {
        cout << "Failed to open " << controller.getPort() << endl;
        return 1;
    }
        
    cout << "Sending UBX-MON-VER poll..." << endl;
    if (!controller.sendUBXMessage(UBX_CLS_MON_VER, UBX_ID_MON_VER, {})) {
        cerr << "Failed to send UBX-MON-VER." << endl;
        return 1;
    }

    vector<uint8_t> response;
    cout << "Waiting for UBX-MON-VER response..." << endl;
    if (!controller.readUBXMessage(UBX_CLS_MON_VER, UBX_ID_MON_VER, response)) {
        cerr << "Failed to receive UBX-MON-VER." << endl;
        return 1;
    }

    stringstream swVer;
    for (int i = 0; i < SW_BYTE_OFFSET; i++)        
        swVer << response[i + HEADER_SIZE];
    cout << "Software Version: " << swVer.str() << endl;

    stringstream hwVer;
    for (int i = 0; i < HW_BYTE_OFFSET; i++)        
        hwVer << response[i + HEADER_SIZE + SW_BYTE_OFFSET];
    cout << "Hardvare Version: " << hwVer.str() << endl;

    int extCount = (response.size() - HEADER_SIZE - SW_BYTE_OFFSET - HW_BYTE_OFFSET - CHECKSUM_SIZE) / EXT_BYTE_OFFSET;
    cout << "Extention(s): " << endl;
    for (int i = 0 ; i < extCount; i++) {
        stringstream ext;
        for (int j = 0; j < EXT_BYTE_OFFSET; j++)        
            ext << response[j + HEADER_SIZE + SW_BYTE_OFFSET + HW_BYTE_OFFSET + i * 30];
        cout << ext.str() << endl;
    }

    controller.closeSerialPort();
    return 0; 
}

