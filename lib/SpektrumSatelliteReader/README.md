# SpektrumSatelliteReader Library

This library provides an interface for reading data directly from Spektrum-compatible satellite receivers (DSM2/DSMX) on Arduino platforms.

## Features

- Compatible with DSM2 and DSMX protocols
- Supports both 10-bit and 11-bit resolution modes
- Automatic protocol detection
- Drop-in replacement for CPPMReader with similar API
- Maps channel values to standard 1000-2000μs range
- Built-in signal validation and timeout detection

## Hardware Connection

Connect your Spektrum satellite receiver to the Feather M4 CAN as follows:

| Satellite Receiver Wire | Feather M4 CAN Pin | Purpose |
|------------------------|-------------------|---------|
| Orange (3.3V)          | 3V                | Power   |
| Black (Ground)         | GND               | Ground  |
| Gray (Signal)          | Serial1 RX (Pin 0)| Data & Bind |

For automatic bind mode functionality, also connect:

| Component              | Feather M4 CAN Pin | Purpose |
|------------------------|-------------------|---------|
| Jumper/Switch          | A0 to GND         | Bind Mode Enable |

**Important Notes:**
- Spektrum satellite receivers operate at 3.3V - do NOT connect to 5V
- The signal wire carries serial data at 115200 baud and is also used for bind pulses
- You may need to disconnect the signal wire when uploading code via USB
- No separate bind wire needed - bind pulses are sent on the same pin as data

## Usage

```cpp
#include "SpektrumSatelliteReader.h"

// Create reader instance using Serial1
SpektrumSatelliteReader spektrumReader(Serial1, 8, 1000, 2000, 1500);

void setup() {
    Serial.begin(115200);
    
    // Initialize the Spektrum reader
    if (spektrumReader.begin()) {
        Serial.println("Spektrum satellite reader initialized");
    } else {
        Serial.println("Failed to initialize Spektrum reader");
    }
}

void loop() {
    // Update the reader (call this regularly)
    spektrumReader.update();
    
    // Check if receiving signal
    if (spektrumReader.isReceiving()) {
        // Read channel values (0-7 for 8 channels)
        for (int i = 0; i < 8; i++) {
            int channelValue = spektrumReader.getChannel(i);
            Serial.print("Ch");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(channelValue);
            Serial.print("us ");
        }
        Serial.println();
    } else {
        Serial.println("No signal from Spektrum receiver");
    }
    
    delay(100);
}
```

## API Reference

### Constructor
```cpp
SpektrumSatelliteReader(HardwareSerial& serial, int numChannels = 8, int minValue = 1000, int maxValue = 2000, int defaultValue = 1500)
```

### Methods
- `bool begin()` - Initialize the reader
- `bool beginBindMode()` - Initialize serial after bind pulses have been sent (internal use)
- `int getChannel(int channel)` - Get channel value in microseconds (1000-2000)
- `bool isReceiving()` - Check if valid signal is being received
- `unsigned long lastFrameTime()` - Get timestamp of last received frame
- `int getChannelCount()` - Get number of configured channels
- `bool update()` - Process incoming data (call regularly)
- `static float mapToRange(int channelValue, float minOutput, float maxOutput, int minPulse = 1000, int maxPulse = 2000)` - Map channel value to custom range

## Binding Your Receiver

### Automatic Bind Mode (Recommended)

The JumpropeM4 project includes automatic bind mode functionality:

1. Connect a jumper between pin A0 and GND on the Feather M4 CAN
2. Power on the system - it will **immediately** send bind pulses on Pin 0 at startup
3. The receiver will enter DSMX 22ms bind mode and the OLED will show "BIND MODE"
4. Put your transmitter into DSMX bind mode
5. Wait for the satellite receiver LED to go solid (binding complete)
6. Power cycle both transmitter and receiver
7. Remove the jumper from A0 and restart for normal operation

### Manual Bind Mode

Before first use, you can also manually bind your satellite receiver:

1. Hold down the bind button on the satellite receiver
2. Apply power to the receiver (while holding bind button)
3. The receiver LED should flash rapidly
4. Put your transmitter into bind mode (DSM2 or DSMX)
5. The receiver LED should go solid when binding is complete
6. Power cycle both transmitter and receiver to test

## Troubleshooting

- **No signal detected**: Check wiring, ensure receiver is bound to transmitter
- **Erratic readings**: Verify 3.3V power supply, check for interference
- **Upload issues**: Disconnect signal wire during code upload
- **Wrong channel mapping**: Some receivers may have different channel orders

## Protocol Details

This library implements the Spektrum Remote Receiver Interface specification:
- Frame rate: ~22ms (45Hz) or ~11ms (90Hz) depending on mode
- Frame size: 16 bytes
- Channels per frame: 7
- Resolution: 10-bit (1024 steps) or 11-bit (2048 steps)
- Baud rate: 115200

## Compatibility

Tested with:
- Spektrum satellite receivers
- Orange RX DSM2/DSMX satellite receivers
- Generic DSM2/DSMX compatible satellite receivers

Should work with any Spektrum-compatible satellite receiver that follows the standard protocol. 