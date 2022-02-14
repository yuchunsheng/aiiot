//https://github.com/arijitaich/i2s_sampling/blob/main/src/main.cpp

#include <Arduino.h>
#include "I2SMEMSSampler.h"

#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_34
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_35
#define I2S_MIC_SERIAL_DATA GPIO_NUM_33

I2SSampler *i2sSampler = NULL;

// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s pins
i2s_pin_config_t i2sPins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

// send data to a remote address
void sendData(uint8_t *bytes, size_t count)
{
  Serial.write(bytes, count);
}


// Task to write samples to our server
void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some samples to save
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      sendData((uint8_t *)sampler->getCapturedAudioBuffer(), sampler->getBufferSizeInBytes());
    }
  }
}

void setup()
{
  Serial.begin(115200);
 
  // Direct i2s input from INMP441 or the SPH0645
  i2sSampler = new I2SMEMSSampler(i2sPins, false);

  // set up the i2s sample writer task
  TaskHandle_t i2sMemsWriterTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);

  // start sampling from i2s device
  i2sSampler->start(I2S_NUM_0, i2sMemsConfigBothChannels, 32768, i2sMemsWriterTaskHandle);
}

void loop()
{
  // nothing to do here - everything is taken care of by tasks
}
