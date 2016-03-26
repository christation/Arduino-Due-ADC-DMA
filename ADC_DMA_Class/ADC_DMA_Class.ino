#include <rtc_clock.h>
#include "ADCSampler.h"

ADCSampler sampler;
unsigned int samplingRate = 20;


// Create RTC_Clock
RTC_clock rtc(XTAL);

#include <Scheduler.h>
void setup() {
  rtc.init();
  rtc.set_time(0, 0, 0);
  rtc.set_date(1, 1, 2016);


  Serial.begin(115200);  // To print debugging messages.
  Serial.print("Start ... Sampling Rate is set to ");
  Serial.print(samplingRate);
  Serial.println("Hz");

  Serial.print("The DMA uses ");;
  Serial.print(NUMBER_OF_BUFFERS);
  Serial.print(" buffers with ");
  Serial.print(BUFFER_SIZE);
  Serial.println(" elements each");
  Serial.print("A serial stream will be send every ");
  double interval = (((double) BUFFER_SIZE) / samplingRate) / NUM_CHANNELS;
  Serial.print(interval);
  Serial.println(" seconds");

  Scheduler.startLoop(loopADC);
  sampler.begin(samplingRate);

  Serial.print("Current Time: ");
  Serial.println(rtc.unixtime());
}

void loop() {
  yield();
}



void loopADC() {

  if (sampler.available()) {
    Serial.print("Current Time: ");
    Serial.println(rtc.unixtime());
    int bufferLength = 0;
    uint16_t* cBuf = sampler.getFilledBuffer(&bufferLength);
    for (int i = 0; i < bufferLength; i=i+2)
    {
      Serial.print(cBuf[i]);
      Serial.print(",");
      Serial.println(cBuf[i+1]);
    }
    Serial.println("----");
    sampler.readBufferDone();
  }
  yield();
}

void ADC_Handler() {
  sampler.handleInterrupt();
}
