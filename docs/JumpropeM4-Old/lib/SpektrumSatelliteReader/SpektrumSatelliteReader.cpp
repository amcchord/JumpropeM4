#include "SpektrumSatelliteReader.h"

SpektrumSatelliteReader::SpektrumSatelliteReader(HardwareSerial& serial, int numChannels, int minValue, int maxValue, int defaultValue)
    : _serial(serial), _numChannels(numChannels), _minValue(minValue), _maxValue(maxValue), _defaultValue(defaultValue) {
    
    _channelValues = new int[_numChannels];
    
    // Initialize all channels to default value
    for (int i = 0; i < _numChannels; i++) {
        _channelValues[i] = _defaultValue;
    }
    
    _receivingSignal = false;
    _lastFrameTime = 0;
    _frameIndex = 0;
    _frameStarted = false;
}

SpektrumSatelliteReader::~SpektrumSatelliteReader() {
    delete[] _channelValues;
}

bool SpektrumSatelliteReader::begin() {
    // Initialize serial communication at Spektrum baud rate
    _serial.begin(SPEKTRUM_BAUD_RATE);
    
    // Clear any existing data in the buffer
    while (_serial.available()) {
        _serial.read();
    }
    
    _frameIndex = 0;
    _frameStarted = false;
    _receivingSignal = false;
    _lastFrameTime = millis();
    
    return true;
}

bool SpektrumSatelliteReader::beginBindMode() {
    // Note: Bind pulses should be sent BEFORE calling this function
    // This function only initializes the serial communication after bind pulses
    
    // Initialize serial communication at Spektrum baud rate
    // This will configure the pin for UART RX functionality
    _serial.begin(SPEKTRUM_BAUD_RATE);
    
    // Clear any existing data in the buffer
    while (_serial.available()) {
        _serial.read();
    }
    
    // The receiver should now be in bind mode (LED flashing rapidly)
    
    _frameIndex = 0;
    _frameStarted = false;
    _receivingSignal = false;
    _lastFrameTime = millis();
    
    return true;
}

int SpektrumSatelliteReader::getChannel(int channel) const {
    if (channel < 0 || channel >= _numChannels) {
        return _defaultValue;
    }
    return _channelValues[channel];
}

bool SpektrumSatelliteReader::isReceiving() const {
    return _receivingSignal && (millis() - _lastFrameTime < SIGNAL_TIMEOUT_MS);
}

unsigned long SpektrumSatelliteReader::lastFrameTime() const {
    return _lastFrameTime;
}

float SpektrumSatelliteReader::mapToRange(int channelValue, float minOutput, float maxOutput, int minPulse, int maxPulse) {
    // Constrain input to valid range
    channelValue = constrain(channelValue, minPulse, maxPulse);
    
    // Map to output range
    return minOutput + (maxOutput - minOutput) * (channelValue - minPulse) / (maxPulse - minPulse);
}

bool SpektrumSatelliteReader::update() {
    
    // Process all available bytes
    while (_serial.available()) {
        uint8_t byte = _serial.read();
        
        if (!_frameStarted) {
            // Look for frame start
            if (_frameIndex == 0) {
                _frameBuffer[0] = byte;
                _frameIndex = 1;
            } else if (_frameIndex == 1) {
                _frameBuffer[1] = byte;
                
                // Check if this is a valid frame start
                if (isValidFrameStart(_frameBuffer[0], _frameBuffer[1])) {
                    _frameStarted = true;
                    _frameIndex = 2;
                } else {
                    // Not a valid start, shift and try again
                    _frameBuffer[0] = _frameBuffer[1];
                    _frameIndex = 1;
                }
            }
        } else {
            // We're in a frame, collect bytes
            _frameBuffer[_frameIndex] = byte;
            _frameIndex++;
            
            // Check if we have a complete frame
            if (_frameIndex >= SPEKTRUM_FRAME_SIZE) {
                processFrame();
                _frameStarted = false;
                _frameIndex = 0;
                _lastFrameTime = millis();
                _receivingSignal = true;
            }
        }
    }
    
    // Check for timeout
    if (millis() - _lastFrameTime > SIGNAL_TIMEOUT_MS) {
        _receivingSignal = false;
    }
    
    return isReceiving();
}

void SpektrumSatelliteReader::processFrame() {
    // Spektrum frame format:
    // Byte 0: Fade count
    // Byte 1: System data (contains resolution info)
    // Bytes 2-15: 7 channels, 2 bytes each
    
    // Determine if this is 10-bit or 11-bit data
    // Bit 7 of byte 1 indicates resolution: 0 = 10-bit, 1 = 11-bit
    bool is11Bit = (_frameBuffer[1] & 0x80) != 0;
    
    // Debug: Print frame data occasionally for troubleshooting
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 2000) { // Every 2 seconds
        Serial.print("Spektrum Frame: ");
        for (int i = 0; i < SPEKTRUM_FRAME_SIZE; i++) {
            if (_frameBuffer[i] < 0x10) Serial.print("0");
            Serial.print(_frameBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print(" | Mode: ");
        Serial.print(is11Bit ? "11-bit" : "10-bit");
        Serial.println();
        lastDebugPrint = millis();
    }
    
    // Process channel data (7 channels per frame)
    for (int i = 0; i < SPEKTRUM_CHANNELS_PER_FRAME && i < _numChannels; i++) {
        int byteIndex = 2 + (i * 2);
        
        // Combine two bytes into 16-bit value (big-endian)
        uint16_t rawValue = (_frameBuffer[byteIndex] << 8) | _frameBuffer[byteIndex + 1];
        
        // Extract channel number and value
        int channelNum;
        uint16_t channelValue;
        
        if (is11Bit) {
            // 11-bit mode: CCCC VVVV VVVV VVVV
            // Channel number in upper 4 bits, value in lower 11 bits
            channelNum = (rawValue >> 11) & 0x0F;
            channelValue = rawValue & 0x07FF;
        } else {
            // 10-bit mode: CCCC CVVV VVVV VVVV
            // Channel number in upper 4 bits, value in lower 10 bits
            channelNum = (rawValue >> 10) & 0x0F;
            channelValue = rawValue & 0x03FF;
        }
        
        // Store channel value if it's within our range
        if (channelNum < _numChannels) {
            int convertedValue = convertChannelValue(channelValue, is11Bit);
            
            // Validate the converted value is reasonable
            if (convertedValue >= _minValue && convertedValue <= _maxValue) {
                _channelValues[channelNum] = convertedValue;
            }
            // If value is out of range, keep the previous value (don't update)
        }
    }
}

int SpektrumSatelliteReader::convertChannelValue(uint16_t rawValue, bool is11Bit) {
    // Convert raw Spektrum value to microsecond pulse width (1000-2000us range)
    
    int minRaw, maxRaw;
    
    if (is11Bit) {
        // 11-bit mode: Spektrum uses ~342-1706 range for 1000-2000us
        // Full range is 0-2047, but actual servo range is smaller
        minRaw = 342;   // Corresponds to ~1000us
        maxRaw = 1706;  // Corresponds to ~2000us
    } else {
        // 10-bit mode: Spektrum uses ~171-853 range for 1000-2000us  
        // Full range is 0-1023, but actual servo range is smaller
        minRaw = 171;   // Corresponds to ~1000us
        maxRaw = 853;   // Corresponds to ~2000us
    }
    
    // Clamp raw value to expected range first
    rawValue = constrain(rawValue, minRaw, maxRaw);
    
    // Map raw value to 1000-2000us range
    int mappedValue = map(rawValue, minRaw, maxRaw, _minValue, _maxValue);
    
    // Constrain to valid range
    return constrain(mappedValue, _minValue, _maxValue);
}

bool SpektrumSatelliteReader::isValidFrameStart(uint8_t byte1, uint8_t byte2) {
    // Spektrum frame validation:
    // Byte 0: Fade count (can be any value, but typically 0x00-0xFF)
    // Byte 1: System data
    //   - Bit 7: Resolution (0=10-bit, 1=11-bit)
    //   - Bits 6-0: System ID and other flags
    
    // More strict validation to reduce false frame starts
    // The system data byte should have a reasonable pattern
    
    // Check if byte2 looks like valid system data
    // Common patterns: 0x01, 0x02, 0x12, 0x81, 0x82, 0x92, etc.
    // Reject obviously invalid patterns like 0xFF, 0x00 in certain contexts
    
    // For DSM2/DSMX, the system byte typically has specific patterns
    // Let's be more selective to avoid false frame detection
    if (byte2 == 0xFF || byte2 == 0x00) {
        return false;  // These are unlikely to be valid system bytes
    }
    
    // Accept frames where the system byte looks reasonable
    return true;
} 