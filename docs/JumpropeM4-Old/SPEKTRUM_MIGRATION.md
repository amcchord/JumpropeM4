# Migration from CPPM to Spektrum Satellite Receiver

This document describes the changes made to migrate the JumpropeM4 project from CPPM (Compound Pulse Position Modulation) input to Spektrum satellite receiver input.

## Overview

The project has been successfully migrated to use a Spektrum-compatible satellite receiver instead of CPPM input. This provides several advantages:

- **Digital Signal**: True digital serial communication instead of analog PWM signals
- **Zero Jitter**: No signal jitter from the protocol itself
- **More Channels**: Support for up to 12 channels (depending on transmitter)
- **Better Reliability**: More robust signal transmission
- **Wider Compatibility**: Works with DSM2 and DSMX protocols

## Hardware Changes

### Old CPPM Setup
- CPPM signal connected to Pin 9
- Single analog signal carrying all channel data

### New Spektrum Satellite Setup
- **Orange wire (3.3V)** → **3V pin** on Feather M4 CAN
- **Black wire (Ground)** → **GND pin** on Feather M4 CAN  
- **Gray wire (Signal)** → **Serial1 RX (Pin 0)** on Feather M4 CAN

### Optional: Automatic Bind Mode
- **Jumper wire** between **A0 and GND** to enable automatic bind mode on startup
- **No separate bind wire needed** - bind pulses are sent on the same pin as data (Pin 0)

**Important Notes:**
- Spektrum satellite receivers operate at **3.3V only** - do NOT connect to 5V
- The signal wire carries serial data at 115200 baud
- You may need to disconnect the signal wire when uploading code via USB

## Software Changes

### New Library
A new `SpektrumSatelliteReader` library has been created in `lib/SpektrumSatelliteReader/` that provides:

- Compatible API with the old `CPPMReader` for easy migration
- Support for both DSM2 and DSMX protocols
- Automatic detection of 10-bit and 11-bit resolution modes
- Built-in signal validation and timeout detection
- Channel values mapped to standard 1000-2000μs range

### Code Changes
The following changes were made to `src/main.cpp`:

1. **Include Statement**: Changed from `CPPMReader.h` to `SpektrumSatelliteReader.h`
2. **Pin Configuration**: Replaced `CPPM_PIN` with `SPEKTRUM_SERIAL` (Serial1)
3. **Reader Instance**: Changed from `CPPMReader cppmReader(...)` to `SpektrumSatelliteReader spektrumReader(...)`
4. **Channel Names**: Updated `CPPM_CHANNEL` to `SPEKTRUM_CHANNEL`
5. **Comments and Messages**: Updated all references from "CPPM" to "Spektrum"

### API Compatibility
The new `SpektrumSatelliteReader` provides the same interface as the old `CPPMReader`:

```cpp
// Initialization
SpektrumSatelliteReader spektrumReader(Serial1, 8, 1000, 2000, 1500);

// Usage (same as before)
spektrumReader.begin();
spektrumReader.update();
int channelValue = spektrumReader.getChannel(0);  // Get channel 1 (0-indexed)
bool receiving = spektrumReader.isReceiving();
```

## Channel Mapping

The channel mapping remains the same:

- **Channel 1 (index 0)**: Motor velocity control (1000us = -20 rad/s, 1500us = stop, 2000us = 20 rad/s)
- **Channel 2 (index 1)**: Current limit control (1000us = 1A, 2000us = 40A)
- **Channel 3 (index 2)**: Mode control (>1800us = position mode, <1800us = velocity mode)
- **Channel 4 (index 3)**: Position control (1000us = -4 rad, 2000us = +4 rad in position mode)
- **Channel 5 (index 4)**: Motor selection (>1800us = motor 1, <1300us = motor 2, 1300-1800us = both)
- **Channel 6 (index 5)**: Position zeroing (>1800us = set current position as zero)

## Binding Your Receiver

### Automatic Bind Mode (Recommended)

The system now includes automatic bind mode functionality:

1. **Connect a jumper** between pin A0 and GND on the Feather M4 CAN
2. **Power on the system** - it will **immediately** send bind pulses on Pin 0 at startup
3. The receiver enters DSMX 22ms bind mode and the **OLED display** will show "BIND MODE"
4. Put your **transmitter into DSMX bind mode**
5. Wait for the **satellite receiver LED to go solid** (binding complete)
6. **Power cycle** both transmitter and receiver
7. **Remove the jumper** from A0 and restart for normal operation

### Manual Bind Mode

Alternatively, you can manually bind your satellite receiver:

1. **Hold down the bind button** on the satellite receiver
2. **Apply power** to the receiver (while holding bind button)
3. The receiver LED should **flash rapidly**
4. Put your **transmitter into bind mode** (DSM2 or DSMX)
5. The receiver LED should go **solid** when binding is complete
6. **Power cycle** both transmitter and receiver to test

## Protocol Details

The implementation supports the Spektrum Remote Receiver Interface specification:

- **Frame Rate**: ~22ms (45Hz) or ~11ms (90Hz) depending on mode
- **Frame Size**: 16 bytes
- **Channels per Frame**: 7
- **Resolution**: 10-bit (1024 steps) or 11-bit (2048 steps)
- **Baud Rate**: 115200
- **Protocols**: DSM2 and DSMX compatible

## Troubleshooting

### No Signal Detected
- Check wiring connections
- Ensure receiver is bound to transmitter
- Verify 3.3V power supply
- Check that transmitter is on and in range

### Erratic Readings
- Verify stable 3.3V power supply
- Check for electromagnetic interference
- Ensure good connections

### Upload Issues
- Disconnect signal wire (gray) during code upload
- Reconnect after upload is complete

### Wrong Channel Mapping
- Some receivers may have different channel orders
- Check your transmitter's channel assignment
- Verify binding was successful

## Compatibility

This implementation has been tested with:
- Spektrum satellite receivers
- Orange RX DSM2/DSMX satellite receivers
- Generic DSM2/DSMX compatible satellite receivers

It should work with any Spektrum-compatible satellite receiver that follows the standard protocol.

## Files Modified

- `src/main.cpp` - Updated to use Spektrum satellite receiver
- `lib/SpektrumSatelliteReader/SpektrumSatelliteReader.h` - New library header
- `lib/SpektrumSatelliteReader/SpektrumSatelliteReader.cpp` - New library implementation
- `lib/SpektrumSatelliteReader/README.md` - Library documentation

## Files Preserved

The original CPPM library remains in `lib/CPPMReader/` for reference, but is no longer used by the main application. 