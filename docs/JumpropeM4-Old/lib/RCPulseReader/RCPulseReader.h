#ifndef RC_PULSE_READER_H
#define RC_PULSE_READER_H

#include <Arduino.h>

class RCPulseReader {
public:
    // Constructor
    RCPulseReader(int maxChannels = 8);
    
    // Destructor
    ~RCPulseReader();
    
    // Initialize a specific pin for pulse reading
    bool attach(int channelIndex, int pin);
    
    // Read current pulse width (in microseconds)
    // Returns 0 if no valid pulse has been detected
    unsigned long read(int channelIndex);
    
    // Returns true if new pulse data is available since last read
    bool available(int channelIndex);
    
    // Get the most recent pulse width (without clearing available flag)
    unsigned long getPulseWidth(int channelIndex);
    
    // Get maximum number of supported channels
    int getMaxChannels() const { return _maxChannels; }
    
    // Static utility functions
    // Map RC pulse value (typically 1000-2000us) to a custom range
    static float mapToRange(unsigned long pulseWidth, 
                           float minOutput, float maxOutput,
                           unsigned long minPulse = 1000, unsigned long maxPulse = 2000);
                           
    // Update function should be called in your main loop
    void update();
    
private:
    int _maxChannels;
    int* _pins;
    volatile unsigned long* _pulseWidths;
    volatile unsigned long* _risingEdgeTimes;
    volatile bool* _newData;
    
    // Internal interrupt handler (non-static)
    void handleInterrupt(int channel);
    
    // Static instance for interrupt handling
    static RCPulseReader* _instance;
    
    // Arrays to store interrupt handlers
    static void (*_interruptHandlers[8])();
    
    // Static interrupt handlers - limited to 8 channels for simplicity
    static void _handleInterrupt0() { if(_instance) _instance->handleInterrupt(0); }
    static void _handleInterrupt1() { if(_instance) _instance->handleInterrupt(1); }
    static void _handleInterrupt2() { if(_instance) _instance->handleInterrupt(2); }
    static void _handleInterrupt3() { if(_instance) _instance->handleInterrupt(3); }
    static void _handleInterrupt4() { if(_instance) _instance->handleInterrupt(4); }
    static void _handleInterrupt5() { if(_instance) _instance->handleInterrupt(5); }
    static void _handleInterrupt6() { if(_instance) _instance->handleInterrupt(6); }
    static void _handleInterrupt7() { if(_instance) _instance->handleInterrupt(7); }
};

#endif // RC_PULSE_READER_H 