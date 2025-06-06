# JumpropeM4

## Overview
JumpropeM4 is a firmware for the Adafruit Feather M4 CAN board to control RS03 motors for use in a jumprope training device. It supports multiple control modes including velocity control and various position control schemes using Spektrum satellite receiver input, and can control up to two motors on the same CAN bus.

## Features
- Dual RS03 motor control using CAN bus communication (1 Mbit/s)
- Six distinct operating modes with different control schemes
- Spektrum satellite receiver input (DSM2/DSMX protocol)
- Real-time motor feedback and status monitoring
- Individual and synchronized motor control options
- Emergency stop and error clearing functionality
- Mechanical zero position setting and calibration
- Status indication via onboard NeoPixel LED
- OLED display showing real-time system status and motor targets
- Active motor feedback reporting with CAN message processing
- Motor direction control (Motor 1 reversed by default)
- Current limiting and motor protection
- Multi-level logging system with detailed error reporting
- Non-blocking initialization with graceful degradation

## Hardware Requirements
- Adafruit Feather M4 CAN board
- Two RS03 motors with CAN interface (configurable IDs)
- Spektrum satellite receiver (DSM2/DSMX compatible)
- SSD1306 OLED display (128x64 pixels, I2C)
- NeoPixel LED (integrated on Feather board)
- Appropriate power supply for motors and controller
- CAN bus termination resistors

## Pin Connections
- **NeoPixel**: Integrated on Feather M4 board
- **Spektrum Receiver**: Connected to hardware serial (configurable)
- **CAN Bus**: Connected to RS03 motors via CAN H/L pins
- **I2C Display**: SDA/SCL pins for SSD1306 OLED
- **CAN Control Pins**: Standby and Boost Enable pins for CAN transceiver

## Control Modes

The system supports 6 distinct control modes selected via Channel 4:

| Mode | Name | Description | Primary Control | Secondary Control |
|------|------|-------------|-----------------|-------------------|
| 1 | Velocity Mode | Direct velocity control of both motors | Channel 5 (±20 rad/s) | Switch A (Zero) |
| 2 | Position Mode (Switch) | Binary position control | Switch B (0 or 3.8 rad) | Switch A (Zero) |
| 3 | Position Mode (Analog) | Analog position control around base | Channel 5 (±2π around 1.9 rad) | Switch A (Zero) |
| 4 | Position Mode (Fixed) | Fixed position targets | Automatic (M1=-4.4, M2=-0.6 rad) | Switch A (Zero) |
| 5 | Position Mode (Individual) | Independent motor control | Ch7 (M1 ±7 rad), Ch9 (M2 ±7 rad) | Switch A (Zero) |
| 6 | Emergency Stop | Stop motors and clear errors | Automatic stop and fault reset | N/A |

## Channel Mapping

### RC Channel Functions

| Channel | Function | Range/Values | Used In Modes | Description |
|---------|----------|--------------|---------------|-------------|
| 4 | Mode Selection | 1-6 | All | Selects operating mode |
| 5 | Velocity/Position | 1000-2000μs | 1, 3 | Velocity control (Mode 1) or position offset (Mode 3) |
| 6 | Switch States | Switch A/B | All | Switch A: Zero setting, Switch B: Binary position |
| 7 | Motor 1 Position | 1000-2000μs | 5 | Individual Motor 1 position control (±7 rad) |
| 8 | State Tracking | Low/Mid/High | All | Channel 8 state monitoring (display only) |
| 9 | Motor 2 Position | 1000-2000μs | 5 | Individual Motor 2 position control (±7 rad) |

### Switch Functions

| Switch | Function | Action | Available In |
|--------|----------|--------|--------------|
| Switch A | Mechanical Zero | Rising edge sets current position as zero | All modes |
| Switch B | Binary Position | OFF=0 rad, ON=3.8 rad | Mode 2 only |

## Detailed Mode Operation

### Mode 1: Velocity Control
- **Control**: Channel 5 controls velocity of both motors simultaneously
- **Range**: ±20 rad/s with dead zone around center
- **Direction**: Motor 1 reversed, Motor 2 normal
- **Use Case**: Direct speed control for synchronized movement

### Mode 2: Position Control (Switch B)
- **Control**: Switch B provides binary position control  
- **Positions**: Switch OFF = 0 radians, Switch ON = 3.8 radians
- **Motors**: Both motors move to same target position
- **Use Case**: Simple two-position operation

### Mode 3: Position Control (Channel 5)
- **Control**: Channel 5 provides analog position control
- **Base Position**: 1.9 radians
- **Range**: ±2π radians around base (approximately -4.38 to +8.18 rad)
- **Motors**: Both motors move to same target position
- **Use Case**: Continuous position control around a specific operating point

### Mode 4: Fixed Position Control
- **Control**: Automatic movement to predetermined positions
- **Targets**: Motor 1 = -4.4 radians, Motor 2 = -0.6 radians
- **Operation**: Motors automatically move to fixed positions
- **Use Case**: Preset position recall

### Mode 5: Individual Motor Control
- **Control**: Independent position control for each motor
- **Motor 1**: Channel 7 controls position (±7 radians)
- **Motor 2**: Channel 9 controls position (±7 radians)
- **Use Case**: Asymmetric movements and individual motor testing

### Mode 6: Emergency Stop
- **Function**: Immediately stops both motors and clears all faults
- **Operation**: Automatic every 2 seconds while in mode
- **Recovery**: Motors re-enabled when exiting mode
- **Use Case**: Emergency situations and error recovery

## Motor Configuration

| Parameter | Motor 1 | Motor 2 | Notes |
|-----------|---------|---------|-------|
| Direction | Reversed | Normal | Motor 1 commands are inverted |
| Position Mode | PP Control | PP Control | 25 rad/s speed, 200 rad/s² accel, 40A current |
| Velocity Mode | Direct | Direct | No additional limits applied |
| Zero Range | -π to π | -π to π | Zero flag set to 1 for both motors |

## LED Status Indicators

| Color | Status | Meaning |
|-------|--------|---------|
| 🟢 Green | Fully Operational | Motors ready + RC signal present |
| 🔵 Cyan | Motors Ready | Motors initialized but no RC signal |
| 🟡 Yellow | Limited Function | Initialization issues, check connections |
| 🔴 Red | Error/No Signal | RC signal lost or system errors |

## OLED Display Information

The 128x64 OLED display shows:
- **Line 1**: "Motor Control" header
- **Line 2**: Current mode (VEL(1), POS(2), POS(3), FIX(4), IND(5), STOP(6))
- **Line 3**: Motor 1 desired position/velocity with units
- **Line 4**: Motor 2 desired position/velocity with units  
- **Line 5**: Switch states (Button A: ON/OFF, Button B: ON/OFF)

## Serial Output

Serial communication at 115200 baud provides:
- Detailed initialization sequence
- Real-time control values and motor targets
- Frame statistics every 5 seconds
- Error messages and warnings
- Control scheme documentation on startup

## System Configuration

Key parameters defined in the firmware:
- **Motors**: Configurable CAN IDs for two RS03 motors
- **Control Limits**: 
  - Velocity: ±20 rad/s maximum
  - Position: Various ranges depending on mode
  - Individual control: ±7 rad range
- **Timing**:
  - Motor updates: Every 20ms  
  - Display updates: Every 150ms
  - Serial output: Every 100ms
  - CAN feedback: Continuous processing
- **Dead Zones**: ±50μs around channel center positions

## Building and Flashing

This project uses PlatformIO:

```bash
# Install dependencies
pio lib install

# Build firmware  
pio run

# Upload to board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Troubleshooting

### LED Status Diagnosis
- **Red LED**: Check RC receiver connection and power
- **Yellow LED**: Verify motor CAN bus connections and IDs
- **Cyan LED**: RC receiver not detected, motors functional
- **Green LED**: System fully operational

### Common Issues
1. **Motors not responding**: 
   - Verify CAN bus termination
   - Check motor power supply
   - Confirm motor CAN IDs match firmware configuration

2. **No RC control**:
   - Verify Spektrum receiver binding
   - Check receiver power and signal connections
   - Monitor serial output for frame statistics

3. **Position errors**:
   - Use Switch A to reset mechanical zero
   - Verify motors are properly calibrated
   - Check for mechanical obstructions

### Serial Debugging
Monitor serial output for:
- Frame statistics showing RC signal quality
- Motor initialization sequence results
- Real-time control values and feedback
- Error messages and warnings

## Safety Features
- Emergency stop mode (Mode 6) for immediate motor shutdown
- Graceful degradation when components fail to initialize
- Motor fault detection and clearing
- Dead zone implementation prevents accidental activation
- Non-blocking initialization prevents system lockup

## License
[Specify license information here]