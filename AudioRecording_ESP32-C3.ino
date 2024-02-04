// An example of recording audio onto a MicroSD card, using an INMP441 microphone and a XIAO ESP32-C3
//
// Since the values for i2s.dma_buf_count and i2s.dma_buf_len can be somewhat of a mystery, in this
// example they are calculated automatically to use the least amount of DMA memory given your preference for 
// latency and CPU interrupts.
//
// With a low latency setup, the DMA will interrupt the CPU more frequently, but use less DMA memory.
// With a high latency setup, the DMA will interrupt the CPU less frequently, but use more DMA memory.
//
// For example, this code includes test calculations in a fixed for-loop. With a very low latency setup, the
// loop takes 12225 ms to complete, compared to ony 12027 ms on a high latency setup. A very measurable difference.
//
// See this excellent video: https://youtu.be/ejyt-kWmys8?si=oVVuLGKX63CKpm2t
//
// Note, the I2S pins for the XIAO ESP32-C3 isn't well documented, had to find the info searching through forums
//
//  XIAO ESP32-C3 | I2S
//  ------------------------------------
//  DO            | WS / LRC / LCK
//  D1            | SCK / BCLK / BCK
//  D2            | SD / DIN / DATA
//
// Hardware:
//    XIAO ESP32-C3
//    INMP441 Microphone

// Reading audio data
#include <driver/i2s.h>

// Write to MicroSD card
#include "FS.h"
#include "SD.h"

// Define I2S pins specifically for Xiao ESP32-C3
#define I2S_WS  D0
#define I2S_SCK D1
#define I2S_SD  D2

// Recording times
const int TotalSecondsToRecord = 10;
const int AudioDurationPerFileWriteInMs = 500;  // Not enough RAM to buffer long audio durations, so need to write out smaller durations to file, e.g. every 500 ms

// Info about audio recording at 16KHz, 16-bit per sample, mono
const int SampleRate = 16000;
const i2s_bits_per_sample_t BitsPerSample = I2S_BITS_PER_SAMPLE_16BIT;
const i2s_channel_t NumChannels = I2S_CHANNEL_MONO;
const int BytesPerSecond = NumChannels * SampleRate * BitsPerSample / 8;

// Audio data size info
const int TotalAudioSizeInBytes = TotalSecondsToRecord * BytesPerSecond;
const int DMACopyBufferSizeInBytes = BytesPerSecond * AudioDurationPerFileWriteInMs / 1000; // Size of the DMA copy buffer that can hold audio data for the duration of AudioDurationPerFileWriteInMs

// Static audio buffers for each time we write data to a file, for a duration of AudioDurationPerFileWriteInMs
DMA_ATTR static uint8_t DMACopyBuffer[DMACopyBufferSizeInBytes];  // DMA buffers are copied into this buffer when calling i2s_read()
static uint8_t FileWriteBuffer[DMACopyBufferSizeInBytes];         // Audio data processed into this buffer and used to write to file

// Calc DMA buffer info to avoid buffer overflow, which will drop audio while copying out of DMA buffers
const int SPIFFSWriteSpeed = 20;  // KB/s Just testing on my own, I only get 20-32KB per second writing to a SPIFFS file :-(
const int SDCardWriteSpeed = 180; // KB/s Just testing on my own, I get 180-220KB per second writing to a SD Card
const int TimeToWriteDataToFileInMs = DMACopyBufferSizeInBytes / SDCardWriteSpeed;
const int ExtraProcessingTimeInMs = 5;  // Let's add a few extra ms of processing time to accomodate for copying, rescaling, printfs, etc.
const int EstimatedTimeToProcessAudioBufferInMs = TimeToWriteDataToFileInMs + ExtraProcessingTimeInMs;
const int TotalNumSamples = EstimatedTimeToProcessAudioBufferInMs * SampleRate / 1000;
const int TotalBytesRequiredForDMA = TotalNumSamples * NumChannels * BitsPerSample / 8;

// Derive buf_len and buf_count based on latency preference, i.e. choose high, medium or low. By default this example uses medium
const int NumSamplesPerBuffer_HighLatency = 1024;
const int NumSamplesPerBuffer_MediumLatency = 256;
const int NumSamplesPerBuffer_LowLatency = 64;
const int NumSamplesPerBuffer_VeryLowLatency = 8;
const int NumSamplesPerBuffer = min(NumSamplesPerBuffer_MediumLatency, 1024);                       // i2s buf_len (Must be less than 1024)
const int NumBuffers = min(max(int(float(TotalNumSamples / NumSamplesPerBuffer) + 1.0f), 2), 128);  // i2s buf_count (Must be between 2 and 128)

// File IO
File file;
const char fileName[] = "/Recording.wav";
const int wavHeaderSize = 44;

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n\n********* Audio Recording with ESP32-C3 *********\n\n");

  Serial.printf("BytesPerSecond: %d\n", BytesPerSecond);
  Serial.printf("TotalSecondsToRecord: %d\n", TotalSecondsToRecord);
  Serial.printf("TotalAudioSizeInBytes: %d\n", TotalAudioSizeInBytes);
  Serial.printf("AudioDurationPerFileWriteInMs: %d\n", AudioDurationPerFileWriteInMs);
  Serial.printf("DMACopyBufferSizeInBytes: %d\n", DMACopyBufferSizeInBytes);
  Serial.printf("EstimatedTimeToProcessAudioBufferInMs: %d ms\n", EstimatedTimeToProcessAudioBufferInMs);
  Serial.flush();
  Serial.printf("TotalNumSamples: %d\n", TotalNumSamples);
  Serial.printf("TotalBytesRequiredForDMA: %d\n", TotalBytesRequiredForDMA);
  Serial.printf("NumSamplesPerBuffer (buf_length): %d\n", NumSamplesPerBuffer);
  Serial.printf("NumBuffers (buf_count): %d\n", NumBuffers);
  Serial.flush();

  InitMicroSD();
  InitI2S();

  // Create async task to stream audio DMA buffers into our copy DMA buffer for processing on the CPU
  const int stackSize = 1024 * 3;
  xTaskCreate(AudioRecordingTask, "AudioRecordingTask", stackSize, NULL, 1, NULL);

  // Test to see just how much CPU work can get interfered with when the DMA interrupts the CPU
  unsigned long startTime = millis();
  SomeCalculationsToKeepTheCPUBusy();
  Serial.printf("CPU work done. Took %d ms to complete\n", millis() - startTime);
}

void loop()
{
}

void AudioRecordingTask(void* pArg)
{
  size_t numBytesRead;
  int numBytesWrittenToFile = 0;

  // Drain a bit of the DMA buffers before starting the recording
  i2s_read(I2S_NUM_0, (void*)DMACopyBuffer, DMACopyBufferSizeInBytes, &numBytesRead, portMAX_DELAY);
  
  Serial.printf("\nRecording %d seconds of audio ...\n", TotalSecondsToRecord);

  // Time the total duration
  unsigned long startTime = millis();

  while (numBytesWrittenToFile < TotalAudioSizeInBytes)
  {
    // Read data from DMA buffers into our copy buffer
    unsigned long startRead = millis();
    i2s_read(I2S_NUM_0, (void*)DMACopyBuffer, DMACopyBufferSizeInBytes, &numBytesRead, portMAX_DELAY);
    unsigned long stopRead = millis();

    // Scale volume
    i2s_adc_data_scale(FileWriteBuffer, (uint8_t*)DMACopyBuffer, DMACopyBufferSizeInBytes);
    
    // Write to file
    unsigned long startWrite = millis();
    file.write((const byte*)FileWriteBuffer, DMACopyBufferSizeInBytes);
    unsigned long stopWrite = millis();

    numBytesWrittenToFile += DMACopyBufferSizeInBytes;

    Serial.printf("    %u%% (i2s_read %d ms, file.write %d ms )\n", numBytesWrittenToFile * 100 / TotalAudioSizeInBytes, stopRead - startRead, stopWrite - startWrite);
  }

  Serial.printf("Done. Recorded %d seconds in %d ms\n", TotalSecondsToRecord, millis() - startTime);
  Serial.flush();

  file.close();
  SD.end();

  vTaskDelete(NULL);
}

void InitI2S()
{
  i2s_config_t i2s_config =
  {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SampleRate,
    .bits_per_sample = BitsPerSample,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = NumBuffers,
    .dma_buf_len = NumSamplesPerBuffer,
    .use_apll = true
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config =
  {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// Adjust volume
void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len)
{
  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2)
  {
    dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}

// Initialize MicroSD card
static bool InitMicroSD()
{
  Serial.println("\nInitialilzeMicroSD");

  while (!SD.begin(SS))
  {
      Serial.println("  ERROR: No MicroSD card found!");
      delay(500);
  }

  Serial.println("  MicroSD card found");

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("  ERROR: MicroSD card not usable!");
    SD.end();
    return false;
  }

  Serial.println("  MicroSD card ready");

  SD.remove(fileName);

  file = SD.open(fileName, FILE_WRITE);
  if (!file)
  {
    Serial.println("ERROR: File couldn't be opened!");
    return false;
  }

  // Write the WAV header
  byte header[wavHeaderSize];
  createWAVHeader(header, TotalAudioSizeInBytes);
  file.write(header, wavHeaderSize);

  return true;
}

void createWAVHeader(byte* header, int wavSize)
{
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + wavHeaderSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;  // linear PCM
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;  // linear PCM
  header[21] = 0x00;
  header[22] = 0x01;  // normal
  header[23] = 0x00;
  header[24] = 0x80;  // Sampling rate
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x02;  // 16-bit mono
  header[33] = 0x00;
  header[34] = 0x10;  // 16-bit
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
}

// Test to see just how much CPU work can get interfered with when the DMA interrupts the CPU
// For this, we can't just use delay(), we need to do some actual calculations, e.g. a sqrt()
void SomeCalculationsToKeepTheCPUBusy()
{
  float someCalculation = 100.0f;
  const unsigned long numIterations = 2000000;
  for (unsigned long i = 0; i < numIterations; i++)
  {
    // Some processing to keep the CPU busy
    someCalculation += sqrt(someCalculation * i) * sqrt(someCalculation * i);
  }
}