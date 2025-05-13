# RCPulseReader

A lightweight Arduino library for reading RC servo pulses (PWM) from receiver channels. It uses interrupts for precise timing measurement and provides an easy-to-use API.

## Features

- Interrupt-based PWM pulse measurement for accurate timing
- Support for up to 8 channels
- Easy to integrate into any Arduino project
- Compatible with most microcontrollers including Adafruit Feather M4 CAN
- Utility function to map pulse widths to custom ranges
- Clean, modern C++ interface

## Installation

### Arduino IDE

1. Download the ZIP file of this repository
2. Open Arduino IDE
3. Go to Sketch > Include Library > Add .ZIP Library...
4. Select the downloaded ZIP file

### PlatformIO

Add the following to your `platformio.ini` file:

```ini
lib_deps = 
    https://github.com/yourusername/RCPulseReader.git
```

Or just copy the `RCPulseReader` directory to the `lib` folder in your project.

## Usage

```cpp
#include <Arduino.h>
#include "RCPulseReader.h"

// Create an instance of RCPulseReader
RCPulseReader rcReader(3);  // We want to read 3 channels

// Define pins - using pins that support interrupts
const int channelPins[3] = {5, 6, 9};

void setup() {
  Serial.begin(115200);
  
  // Initialize pins for pulse reading
  for (int i = 0; i < 3; i++) {
    rcReader.attach(i, channelPins[i]);
  }
}

void loop() {
  // Call update in the loop (for future compatibility)
  rcReader.update();
  
  // Check if new data is available on channel 0
  if (rcReader.available(0)) {
    // Read the pulse width
    unsigned long pulseWidth = rcReader.read(0);
    
    // Map to a -100 to 100 percent range
    float percent = rcReader.mapToRange(pulseWidth, -100.0, 100.0);
    
    Serial.print("Channel 0: ");
    Serial.print(pulseWidth);
    Serial.print(" us (");
    Serial.print(percent);
    Serial.println("%)");
  }
  
  delay(20);
}
```

## API Reference

### Constructor

```cpp
RCPulseReader(int maxChannels = 8)
```

Creates a new RCPulseReader instance with the specified maximum number of channels (up to 8).

### Methods

```cpp
bool attach(int channelIndex, int pin)
```

Attaches a pin to a channel for pulse reading. Returns `true` if successful, `false` otherwise.

```cpp
unsigned long read(int channelIndex)
```

Reads the pulse width from the specified channel and clears the new data flag. Returns 0 if no new data is available.

```cpp
bool available(int channelIndex)
```

Returns `true` if new data is available on the specified channel.

```cpp
unsigned long getPulseWidth(int channelIndex)
```

Returns the current pulse width without clearing the new data flag.

```cpp
int getMaxChannels()
```

Returns the maximum number of channels supported by this instance.

```cpp
static float mapToRange(unsigned long pulseWidth, float minOutput, float maxOutput, unsigned long minPulse = 1000, unsigned long maxPulse = 2000)
```

Maps a pulse width to a custom range. By default, it maps from 1000-2000 microseconds to the specified output range.

```cpp
void update()
```

Updates the internal state of the library. Should be called in the main loop.

## License

This library is released under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. 