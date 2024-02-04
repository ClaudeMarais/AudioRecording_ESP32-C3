# AudioRecording_ESP32-C3
 An example of recording audio onto a MicroSD card, using an INMP441 microphone and a XIAO ESP32-C3

Since the values for i2s.dma_buf_count and i2s.dma_buf_len can be somewhat of a mystery, in this
example they are calculated automatically to use the least amount of DMA memory given your preference for 
latency and CPU interrupts.

With a low latency setup, the DMA will interrupt the CPU more frequently, but use less DMA memory.
With a high latency setup, the DMA will interrupt the CPU less frequently, but use more DMA memory.

For example, this code includes test calculations in a fixed for-loop. With a very low latency setup, the
loop takes 12225 ms to complete, compared to ony 12027 ms on a high latency setup. A very measurable difference.

See this excellent video: https://youtu.be/ejyt-kWmys8?si=oVVuLGKX63CKpm2t

Note, the I2S pins for the XIAO ESP32-C3 isn't well documented, had to find the info searching through forums

Specifically for XIAO ESP32-C3

DO = WS / LRC / LCK

D1 = SCK / BCLK / BCK

D2 = SD / DIN / DATA

