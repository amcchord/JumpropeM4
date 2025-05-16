#include "CPPMReader.h"

// Initialize static members
CPPMReader* CPPMReader::_instance = nullptr;

CPPMReader::CPPMReader(int pin, int numChannels, int minValue, int maxValue, int defaultValue) {
    _pin = pin;
    _numChannels = max(1, min(numChannels, 16)); // Limit to 1-16 channels
    _minValue = minValue;
    _maxValue = maxValue;
    _defaultValue = defaultValue;
    
    // Allocate memory for channel values
    _channelValues = new volatile int[_numChannels];
    
    // Initialize arrays
    for (int i = 0; i < _numChannels; i++) {
        _channelValues[i] = _defaultValue;
    }
    
    // Initialize other variables
    _receivingSignal = false;
    _lastFrameTime = 0;
    _lastPulseTime = 0;
    _currentChannel = 0;
    
    // Set this instance as the global instance
    // Note: This means only one instance of CPPMReader can be used
    _instance = this;
}

CPPMReader::~CPPMReader() {
    // Detach interrupt
    detachInterrupt(digitalPinToInterrupt(_pin));
    
    // Free memory
    delete[] _channelValues;
    
    // Clear the static instance if it's this one
    if (_instance == this) {
        _instance = nullptr;
    }
}

bool CPPMReader::begin() {
    // Check if pin supports interrupts
    if (digitalPinToInterrupt(_pin) == NOT_AN_INTERRUPT) {
        return false;
    }
    
    // Set the pin as input with pull-up
    pinMode(_pin, INPUT_PULLUP);
    
    // Attach the interrupt (CPPM uses falling edge detection)
    attachInterrupt(digitalPinToInterrupt(_pin), _handleInterrupt, FALLING);
    
    return true;
}

int CPPMReader::getChannel(int channel) const {
    // Check if channel is valid (0-based indexing)
    if (channel < 0 || channel >= _numChannels) {
        return _defaultValue;
    }
    
    return _channelValues[channel];
}

bool CPPMReader::isReceiving() const {
    return _receivingSignal;
}

unsigned long CPPMReader::lastFrameTime() const {
    return millis() - _lastFrameTime;
}

float CPPMReader::mapToRange(int channelValue, float minOutput, float maxOutput,
                           int minPulse, int maxPulse) {
    // Map the channel value to the desired range
    if (channelValue <= minPulse) {
        return minOutput;
    } else if (channelValue >= maxPulse) {
        return maxOutput;
    } else {
        return minOutput + (maxOutput - minOutput) * 
               (float)(channelValue - minPulse) / (maxPulse - minPulse);
    }
}

bool CPPMReader::update() {
    // Check if the signal is still valid
    if (millis() - _lastFrameTime > SIGNAL_TIMEOUT_MS) {
        _receivingSignal = false;
    }
    
    return _receivingSignal;
}

void CPPMReader::handleInterrupt() {
    unsigned long currentTime = micros();
    unsigned long pulseWidth = currentTime - _lastPulseTime;
    _lastPulseTime = currentTime;
    
    // Check if this is a sync pulse (longer than normal pulses)
    if (pulseWidth > SYNC_PULSE_MIN_US) {
        // End of frame reached
        _lastFrameTime = millis();
        _currentChannel = 0;
        _receivingSignal = true;
        return;
    }
    
    // Regular channel pulse - store the value if we're within bounds
    if (_currentChannel >= 0 && _currentChannel < _numChannels) {
        // Convert pulse gap to channel value (pulse width)
        // Cast pulseWidth to avoid signed/unsigned comparison warning
        int value = constrain((int)pulseWidth, _minValue, _maxValue);
        _channelValues[_currentChannel] = value;
    }
    
    // Move to the next channel
    _currentChannel++;
} 