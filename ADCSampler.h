#ifndef ADCSAMPLER_H
#define ADCSAMPLER_H

#include <Arduino.h>

#define NUM_CHANNELS 2
/* ch7:A0 ch6:A1 ch5:A2 ch4:A3 ch3:A4 ch2:A5 ch1:A6 ch0:A7 */
#define ADC_CHANNELS ADC_CHER_CH7 | ADC_CHER_CH6  
#define BUFFER_SIZE 100*NUM_CHANNELS
#define NUMBER_OF_BUFFERS 5  /// Make this 3 or greater

class ADCSampler {
  public:
    ADCSampler();
    void begin(unsigned int samplingRate);
    void end();
    void handleInterrupt();
    bool available();
    unsigned int getSamplingRate();
    uint16_t* getFilledBuffer(int *bufferLength);
    void readBufferDone();
  private:
    unsigned int sampleingRate;
    volatile bool dataReady;
    uint16_t adcBuffer[NUMBER_OF_BUFFERS][BUFFER_SIZE];
    unsigned int adcDMAIndex;        //!< This hold the index of the next DMA buffer
    unsigned int adcTransferIndex;   //!< This hold the last filled buffer

};

#endif /* ADCSAMPLER_H */

