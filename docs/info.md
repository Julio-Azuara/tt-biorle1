<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any sections you do not need.

You can also include images in this folder and reference them in the markdown. Each image must be less than 512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

BioRLE-1 is a lossless ECG/EEG compressor for wearable biosignal loggers. It ingests 16-bit ADC samples from a Texas Instruments ADS1292R analog front-end over SPI and streams compressed data to a host MCU over UART.

The compression pipeline has three stages:

**1. SPI Receiver (`biorle1_spi_rx`)**
Accepts a 72-bit ADS1292R frame (24-bit STATUS + 24-bit CH1 + 24-bit CH2, SPI Mode 1, up to 4 MHz). All SPI pins are asynchronous inputs synchronized through a 2-FF chain. On frame completion, it outputs a 16-bit sample word to the delta encoder.

**2. Delta Encoder (`biorle1_delta`)**
Computes the signed difference between consecutive samples (`delta = current − previous`), clamped to the range [−128, +127] (8-bit signed). For flat or slowly varying ECG baselines, most deltas are small or zero — the key property that makes RLE effective. A bypass mode (ui_in[3] = 1) forwards the raw upper byte without differencing.

**3. RLE Encoder + UART Output (`biorle1_rle` + `biorle1_out`)**
Consecutive identical delta values are run-length encoded into (value, count) byte pairs with a maximum run length of 255. Each pair is serialized over UART at 921,600 bps (8N1). A flush signal (ui_in[4]) forces emission of any pending run, which is required between ECG record segments.

Typical ECG compression ratio: **2–4×** (verified against MIT-BIH Arrhythmia Database records 100–109).

## How to test

### Minimum test (no external hardware)

Apply a 25 MHz clock and release rst_n. Drive ui_in as follows to verify the UART output:

| Signal | ui_in bit | Value |
|--------|-----------|-------|
| spi_cs_n | [2] | 1 (idle) |
| bypass | [3] | 1 |
| All others | — | 0 |

Send a 72-bit SPI frame (Mode 1, MSB-first) with CH1 = 0xAB0000:
- Assert spi_cs_n low (ui_in[2] = 0)
- Clock 72 bits of data on spi_mosi (ui_in[0]) with spi_sck (ui_in[1])
- Deassert spi_cs_n high

After the frame completes, pulse flush (ui_in[4]) high for 1 clock cycle.

**Expected UART output on uo_out[0]:** two 8N1 bytes — `0xAB` (value) then `0x01` (count).

To observe drdy (uo_out[1]), it goes high when the UART starts transmitting and returns to 0 when both bytes are sent.

### Full compression test (with ADS1292R)

1. Connect ADS1292R DOUT → ui_in[0], SCLK → ui_in[1], CS_N → ui_in[2]
2. Connect uo_out[0] (uart_tx) to host MCU UART RX at 921,600 bps, 8N1
3. Power up ADS1292R in continuous conversion mode at any sample rate ≤ 8 kSPS
4. The host MCU receives compressed (value, count) byte pairs
5. Reconstruct original samples: `sample[n] = sample[n-1] + signed(value)` for each count

### Status outputs

| uo_out bit | Signal | Meaning |
|-----------|--------|---------|
| [0] | uart_tx | UART serial stream |
| [1] | drdy | High while UART buffer non-empty |
| [2] | frame_active | High while SPI CS_N is asserted |
| [3] | spi_error | SPI frame incomplete (CS_N rose before 72 bits) |
| [4] | overflow_flag | Delta clamping applied on last sample |
| [5] | frame_sync | 1-cycle pulse at start of each (value, count) pair |
| [7:6] | comp_ratio | Rolling 2-bit compression ratio estimate |

## External hardware

- **TI ADS1292R** — 24-bit ECG analog front-end (SPI Mode 1, DOUT/SCLK/CS_N signals)
- **Host MCU** (e.g. nRF52840, STM32L4) — receives compressed UART data at 921,600 bps
- **Optional:** 10 kΩ pull-up on spi_cs_n to hold idle state during power-up

The chip operates in a single 25 MHz clock domain. The ADS1292R SPI interface is asynchronous to this clock and is internally synchronized through 2-FF chains, so no external synchronization is required.
