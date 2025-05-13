#include "RCPulseReader.h"

// Initialize static members
RCPulseReader* RCPulseReader::_instance = nullptr;
void (*RCPulseReader::_interruptHandlers[8])() = {
    RCPulseReader::_handleInterrupt0,
    RCPulseReader::_handleInterrupt1,
    RCPulseReader::_handleInterrupt2,
    RCPulseReader::_handleInterrupt3,
    RCPulseReader::_handleInterrupt4,
    RCPulseReader::_handleInterrupt5,
    RCPulseReader::_handleInterrupt6,
    RCPulseReader::_handleInterrupt7
};

RCPulseReader::RCPulseReader(int maxChannels) {
    // Limit maxChannels to 8 for now
    _maxChannels = min(maxChannels, 8);
    
    // Allocate memory for arrays
    _pins = new int[_maxChannels];
    _pulseWidths = new unsigned long[_maxChannels];
    _risingEdgeTimes = new unsigned long[_maxChannels];
    _newData = new bool[_maxChannels];
    
    // Initialize arrays
    for (int i = 0; i < _maxChannels; i++) {
        _pins[i] = -1;  // No pin assigned
        _pulseWidths[i] = 0;
        _risingEdgeTimes[i] = 0;
        _newData[i] = false;
    }
    
    // Set this instance as the global instance
    // Note: This means only one instance of RCPulseReader can be used
    _instance = this;
}

RCPulseReader::~RCPulseReader() {
    // Detach all interrupts
    for (int i = 0; i < _maxChannels; i++) {
        if (_pins[i] != -1) {
            detachInterrupt(digitalPinToInterrupt(_pins[i]));
        }
    }
    
    // Free memory
    delete[] _pins;
    delete[] _pulseWidths;
    delete[] _risingEdgeTimes;
    delete[] _newData;
    
    // Clear the static instance if it's this one
    if (_instance == this) {
        _instance = nullptr;
    }
}

bool RCPulseReader::attach(int channelIndex, int pin) {
    // Check if channelIndex is valid
    if (channelIndex < 0 || channelIndex >= _maxChannels) {
        return false;
    }
    
    // Check if pin supports interrupts
    if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT) {
        return false;
    }
    
    // Store the pin
    _pins[channelIndex] = pin;
    
    // Set the pin as input
    pinMode(pin, INPUT);
    
    // Attach the interrupt
    attachInterrupt(digitalPinToInterrupt(pin), 
                    _interruptHandlers[channelIndex], 
                    CHANGE);
    
    return true;
}

unsigned long RCPulseReader::read(int channelIndex) {
    // Check if channelIndex is valid
    if (channelIndex < 0 || channelIndex >= _maxChannels) {
        return 0;
    }
    
    // Check if there's new data
    if (_newData[channelIndex]) {
        _newData[channelIndex] = false;
        return _pulseWidths[channelIndex];
    }
    
    return 0;
}

bool RCPulseReader::available(int channelIndex) {
    // Check if channelIndex is valid
    if (channelIndex < 0 || channelIndex >= _maxChannels) {
        return false;
    }
    
    return _newData[channelIndex];
}

unsigned long RCPulseReader::getPulseWidth(int channelIndex) {
    // Check if channelIndex is valid
    if (channelIndex < 0 || channelIndex >= _maxChannels) {
        return 0;
    }
    
    return _pulseWidths[channelIndex];
}

float RCPulseReader::mapToRange(unsigned long pulseWidth, 
                               float minOutput, float maxOutput,
                               unsigned long minPulse, unsigned long maxPulse) {
    // Map the pulse width to the desired range
    if (pulseWidth <= minPulse) {
        return minOutput;
    } else if (pulseWidth >= maxPulse) {
        return maxOutput;
    } else {
        return minOutput + (maxOutput - minOutput) * 
               (float)(pulseWidth - minPulse) / (maxPulse - minPulse);
    }
}

void RCPulseReader::update() {
    // Nothing needed here for interrupt-based implementation
    // This function is included for compatibility with non-interrupt based versions
}

void RCPulseReader::handleInterrupt(int channel) {
    // Check if channel is valid
    if (channel < 0 || channel >= _maxChannels || _pins[channel] == -1) {
        return;
    }
    
    // Get the pin state
    int pinState = digitalRead(_pins[channel]);
    
    if (pinState == HIGH) {
        // Rising edge
        _risingEdgeTimes[channel] = micros();
    } else {
        // Falling edge
        if (_risingEdgeTimes[channel] != 0) {
            // Calculate pulse width
            _pulseWidths[channel] = micros() - _risingEdgeTimes[channel];
            _newData[channel] = true;
        }
    }
} 