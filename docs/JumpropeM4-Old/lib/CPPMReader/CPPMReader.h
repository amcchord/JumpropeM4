#ifndef CPPM_READER_H
#define CPPM_READER_H

#include <Arduino.h>

class CPPMReader {
public:
    // Constructor with pin and optional parameters
    CPPMReader(int pin, int numChannels = 8, int minValue = 1000, int maxValue = 2000, int defaultValue = 1500);
    
    // Destructor
    ~CPPMReader();
    
    // Initialize the CPPM reader
    bool begin();
    
    // Get channel value (in microseconds)
    int getChannel(int channel) const;
    
    // Returns true if signal has been detected since initialization
    bool isReceiving() const;
    
    // Returns time since the last CPPM frame was received (in milliseconds)
    unsigned long lastFrameTime() const;
    
    // Returns number of channels configured
    int getChannelCount() const { return _numChannels; }
    
    // Map channel value (typically 1000-2000us) to a custom range
    static float mapToRange(int channelValue, float minOutput, float maxOutput,
                           int minPulse = 1000, int maxPulse = 2000);
    
    // Call this to check if the signal is still valid
    // Returns true if signal is valid
    bool update();

private:
    // Median filter size
    static const int FILTER_SIZE = 3;
    
    int _pin;
    int _numChannels;
    int _minValue;
    int _maxValue;
    int _defaultValue;
    
    volatile int* _channelValues;
    volatile int** _channelHistory;
    volatile int* _historyIndex;
    
    volatile bool _receivingSignal;
    volatile unsigned long _lastFrameTime;
    volatile unsigned long _lastPulseTime;
    volatile int _currentChannel;
    
    // Calculate median of 3 values
    int calculateMedian(const int values[FILTER_SIZE]) const;
    
    // Internal interrupt handler (non-static)
    void handleInterrupt();
    
    // Static instance for interrupt handling
    static CPPMReader* _instance;
    
    // Static interrupt handler
    static void _handleInterrupt() { if(_instance) _instance->handleInterrupt(); }
    
    // Timeout for considering signal lost (in milliseconds)
    static const unsigned long SIGNAL_TIMEOUT_MS = 500;
    
    // Sync pulse is usually longer than channel pulses
    static const unsigned long SYNC_PULSE_MIN_US = 2500;
};

#endif // CPPM_READER_H 