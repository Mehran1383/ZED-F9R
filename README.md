# U-blox ZED-F9 GNSS Module Controller Library

A lightweight C++ library for communicating with u-blox ZED-F9 series GNSS modules on Linux systems, designed for headless environments where u-center is not available.

## Features

- Serial communication with u-blox GNSS modules
- UBX protocol message formatting and parsing
- Checksum calculation and validation
- ACK/NAK response handling
- Extensible message support system
- Header-only implementation for easy integration

## Supported Messages

Currently implemented messages:
- **UBX-MON-VER** - Retrieves software and hardware version information

## Prerequisites

- Linux-based operating system
- C++ compatible compiler
- Serial port access to u-blox device (typically `/dev/ttyACM0`)
- Proper permissions to access serial devices

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd ublox-controller
```

2. Include the header in your project:
```cpp
#include "ublox_controller.h"
```

3. Compile your application with the necessary flags:
```bash
g++ your_application.cpp -o your_application
```

## Basic Usage

```cpp
#include "ublox_controller.h"

int main() {
    UbloxController controller("/dev/ttyACM0");
    
    if (!controller.openSerialPort()) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }
    
    // Send a message with empty payload (poll request)
    if (controller.sendUBXMessage(0x0A, 0x04, {})) {
        std::vector<uint8_t> response;
        if (controller.readUBXMessage(0x0A, 0x04, response)) {
            // Process response
        }
    }
    
    controller.closeSerialPort();
    return 0;
}
```

## Message Implementation Format

To add support for new UBX messages, create a new `.cpp` file following this format:

### File Naming Convention
Use the format: `UBX-{CLASS}-{ID}.cpp` (e.g., `UBX-CFG-PRT.cpp`)

### Implementation Template

```cpp
#include <iostream>
#include <sstream>
#include "ublox_controller.h"

// UBX message definitions
#define UBX_CLS_EXAMPLE 0xXX  // Replace with actual class
#define UBX_ID_EXAMPLE 0xXX   // Replace with actual ID

using namespace std;

int main() 
{
    UbloxController controller;
    if (!controller.openSerialPort()) {
        cout << "Failed to open serial port" << endl;
        return 1;
    }
    
    // Create payload if needed, or use empty vector for poll requests
    vector<uint8_t> payload = {
        // Add payload bytes here
    };
    
    cout << "Sending UBX message..." << endl;
    if (!controller.sendUBXMessage(UBX_CLS_EXAMPLE, UBX_ID_EXAMPLE, payload)) {
        cerr << "Failed to send message." << endl;
        return 1;
    }

    // Wait for ACK/NAK if required
    int ackResult = controller.waitForAck(UBX_CLS_EXAMPLE, UBX_ID_EXAMPLE);
    if (ackResult != ACK) {
        cerr << "Message not acknowledged" << endl;
        return 1;
    }

    // Read response if expected
    vector<uint8_t> response;
    if (controller.readUBXMessage(UBX_CLS_EXAMPLE, UBX_ID_EXAMPLE, response)) {
        // Parse response according to message specification
        cout << "Response received successfully" << endl;
        
        // Add your parsing logic here
    }

    controller.closeSerialPort();
    return 0; 
}
```

## Contributing New Messages

I welcome contributions to expand the library's message support! To add a new UBX message:

1. **Study the message specification** in the u-blox protocol documentation
2. **Create a new .cpp file** following the naming convention above
3. **Implement the message** using the provided template
4. **Test thoroughly** with your hardware
5. **Submit a pull request** with your implementation

### Required Information for New Messages

- UBX class and message ID
- Payload structure (if any)
- Response format (if any)
- Any special handling requirements

## Error Handling

The library provides several error codes:
- `ACK` (1): Message acknowledged
- `NAK` (0): Message not acknowledged
- `ERROR_TIMEOUT` (-1): Response timeout
- `ERROR_CHECKSUM` (-2): Checksum validation failed
- `SOCK_NOT_OPEN` (-3): Serial port not open

## Serial Configuration

The library configures the serial port with these settings:
- Baud rate: 38400 (default for ZED-F9 modules)
- 8 data bits, no parity, 1 stop bit
- No flow control
- Raw mode (non-canonical)

## Resources

- [u-blox ZED-F9P Protocol Specification](https://www.u-blox.com/en/product/zed-f9r-module)
- [u-center Software](https://www.u-blox.com/en/product/u-center) (for testing and validation)


## Support

For issues and questions:
1. Check the u-blox protocol documentation
2. Review existing message implementations
3. Create an issue in the GitHub repository

## Disclaimer

This library is not affiliated with or endorsed by u-blox. Always verify message formats and behaviors against the official u-blox protocol documentation.
