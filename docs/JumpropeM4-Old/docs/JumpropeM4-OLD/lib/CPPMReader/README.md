# CPPMReader Library

A lightweight library for Arduino that decodes CPPM (Combined/Composite PPM) signals commonly used in RC receivers.

## Features

- Interrupt-driven for accurate timing
- Support for up to 16 channels
- Timeouts for detecting signal loss
- Configurable min/max/default values
- Utility functions for mapping channel values to custom ranges

## Hardware Connection

Connect your CPPM signal from your RC receiver to an interrupt-capable pin on your Arduino. Make sure to check which pins support interrupts on your specific board.

## Usage

```cpp
#include <CPPMReader.h>

// Define the pin connected to CPPM signal (must support interrupts)
#define CPPM_PIN 9

// Create a CPPM reader instance (pin, channels, min value, max value, default value)
CPPMReader cppm(CPPM_PIN, 8, 1000, 2000, 1500);

void setup() {
  Serial.begin(115200);
  
  // Initialize the CPPM reader
  if (!cppm.begin()) {
    Serial.println("Failed to initialize CPPM reader!");
    while (1);
  }
  
  Serial.println("CPPM Reader initialized");
}

void loop() {
  // Update and check if signal is valid
  if (cppm.update() && cppm.isReceiving()) {
    // Print all channel values
    for (int i = 0; i < cppm.getChannelCount(); i++) {
      Serial.print("CH");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(cppm.getChannel(i));
      Serial.print("\t");
    }
    Serial.println();
  } else {
    Serial.println("No signal");
  }
  
  delay(100);
}
```

## Signal Timing

CPPM signals consist of a series of pulses where:
- The time between falling edges represents channel values
- Typical channel values range from 1000μs to 2000μs
- A sync/frame gap is longer (typically >2.5ms)

## Notes

- The library supports only one instance due to interrupt limitations
- Use INPUT_PULLUP to ensure clean signal when receiver is disconnected
- CPPM uses falling edge detection for timing calculation 