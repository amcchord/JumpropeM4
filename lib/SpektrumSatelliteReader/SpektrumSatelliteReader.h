#ifndef SPEKTRUM_SATELLITE_READER_H
#define SPEKTRUM_SATELLITE_READER_H

#include <Arduino.h>

class SpektrumSatelliteReader {
public:
    // Constructor with serial port and optional parameters
    SpektrumSatelliteReader(HardwareSerial& serial, int numChannels = 8, int minValue = 1000, int maxValue = 2000, int defaultValue = 1500);
    
    // Destructor
    ~SpektrumSatelliteReader();
    
    // Initialize the Spektrum satellite reader
    bool begin();
    
    // Initialize the Spektrum satellite reader in bind mode (uses same pin as serial RX)
    bool beginBindMode();
    
    // Get channel value (in microseconds, mapped to 1000-2000 range like CPPM)
    int getChannel(int channel) const;
    
    // Returns true if signal has been detected since initialization
    bool isReceiving() const;
    
    // Returns time since the last frame was received (in milliseconds)
    unsigned long lastFrameTime() const;
    
    // Returns number of channels configured
    int getChannelCount() const { return _numChannels; }
    
    // Map channel value to a custom range (same as CPPM interface)
    static float mapToRange(int channelValue, float minOutput, float maxOutput,
                           int minPulse = 1000, int maxPulse = 2000);
    
    // Call this to update and process incoming data
    // Returns true if signal is valid
    bool update();

private:
    // Spektrum satellite protocol constants
    static const int SPEKTRUM_FRAME_SIZE = 16;  // 16 bytes per frame
    static const int SPEKTRUM_CHANNELS_PER_FRAME = 7;  // 7 channels per frame
    static const unsigned long SPEKTRUM_BAUD_RATE = 115200;
    static const unsigned long FRAME_TIMEOUT_MS = 100;  // Frame timeout
    static const unsigned long SIGNAL_TIMEOUT_MS = 500; // Signal timeout
    
    HardwareSerial& _serial;
    int _numChannels;
    int _minValue;
    int _maxValue;
    int _defaultValue;
    
    int* _channelValues;
    bool _receivingSignal;
    unsigned long _lastFrameTime;
    
    uint8_t _frameBuffer[SPEKTRUM_FRAME_SIZE];
    int _frameIndex;
    bool _frameStarted;
    
    // Process a complete frame
    void processFrame();
    
    // Convert raw Spektrum channel data to microsecond values
    int convertChannelValue(uint16_t rawValue, bool is11Bit);
    
    // Check if we have a valid frame start
    bool isValidFrameStart(uint8_t byte1, uint8_t byte2);
};

#endif // SPEKTRUM_SATELLITE_READER_H 