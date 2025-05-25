# JumpropeM4

## Overview
JumpropeM4 is a firmware for the Adafruit Feather M4 CAN board to control RS03 motors for use in a jumprope training device. It supports both velocity and position control modes using RC receiver input via CPPM protocol, and can control up to two motors on the same CAN bus.

## Features
- Dual motor control using CAN bus communication
- Motor selection via RC channel
- Two operating modes:
  - Velocity mode: Control motor speed from -20 rad/s to +20 rad/s
  - Position mode: Control motor position from -4 rad to +4 rad
- RC control via CPPM input (standard RC receiver)
- Mode switching via RC channel
- Position zeroing capability with zero position hold mode
- Status indication via onboard NeoPixel
- OLED display showing real-time motor feedback and system status
- Active motor feedback reporting
- Current limiting and motor protection
- Error detection and reporting
- Configurable parameters
- Multi-level logging system

## Hardware Requirements
- Adafruit Feather M4 CAN board
- One or two RS03 motors with CAN interface (IDs 127 and 126)
- RC receiver with CPPM output
- NeoPixel (integrated on Feather board)
- SSD1306 OLED display (128x64 pixels, I2C)
- Power supply appropriate for the motors

## Pin Connections
- Pin 8: NeoPixel data (integrated on Feather board)
- Pin 9: CPPM signal input from RC receiver
- CAN pins: Connected to RS03 motors via CAN bus
- I2C pins: Connected to SSD1306 OLED display

## RC Channel Mapping
- Channel 1 (index 0): Motor velocity control
  - 1000μs = -20 rad/s
  - 1500μs = Stop (with ±50μs dead zone)
  - 2000μs = +20 rad/s
- Channel 3 (index 2): Mode selection
  - >1800μs = Position mode
  - <1800μs = Velocity mode
- Channel 4 (index 3): Position control (in position mode)
  - 1000μs = -4 radians
  - 2000μs = +4 radians
- Channel 5 (index 4): Motor selection
  - >1800μs = Motor 1 only (ID 127)
  - <1300μs = Motor 2 only (ID 126)
  - 1300-1800μs = Both motors (Motor 2 runs in reverse)
- Channel 6 (index 5): Position zeroing
  - >1800μs = Set current position as zero

## LED Status Indicators
- Green: Ready and operational in velocity mode
- Cyan/Magenta: Position mode (intensity indicates position)
- Red/Green: Velocity mode (color and intensity indicate direction and speed)
- White (pulsing): Zero position hold mode
- Yellow: Error or no CPPM signal
- Purple: Initializing
- Red: Motors not initialized or fatal error

## OLED Display
The OLED display shows:
- Current operating mode (position or velocity)
- Active motor selection
- Real-time position, velocity, and torque feedback for both motors
- Error messages when needed

## Configuration
The firmware has several configurable parameters in `main.cpp`:
- Motor IDs and master ID
- Motor current limits
- Acceleration limits
- Velocity and position limits
- Control update rates
- Debug output level
- Display update rate

## Advanced Features
- Zero position hold mode: When setting position to zero, system holds at zero for 1 second
- Active motor feedback reporting: Motors continuously report their status via CAN
- Direct parameter querying: System queries motors directly for reliable feedback
- Multiple debug levels: ERROR, WARNING, INFO, DEBUG, and VERBOSE levels for troubleshooting

## Building and Flashing
This project uses PlatformIO for building and flashing. To build and upload:

1. Install PlatformIO
2. Clone this repository
3. Connect your Feather M4 CAN board via USB
4. Run `pio run -t upload`

## Troubleshooting
- If the NeoPixel shows red, check serial output for error messages
- If the NeoPixel shows yellow, check RC receiver connection
- Check the OLED display for status information and error messages
- Serial output at 115200 baud provides detailed status and error information
- If one motor is not responding, check its ID and CAN bus connections

## License
[Specify license information here]