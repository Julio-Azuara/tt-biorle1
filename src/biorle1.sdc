# =============================================================================
# BioRLE-1 — OpenLane SDC Constraints
# File   : biorle1.sdc
# Project: BioRLE-1 Lossless Biosignal Compressor ASIC
# Target : SkyWater 130nm via Tiny Tapeout (sky130_fd_sc_hd)
# Tool   : OpenSTA (invoked by OpenLane)
# Date   : 2026-06-14
# =============================================================================
#
# This file defines the two constraint categories required for a clean OpenLane
# static timing analysis (STA) run:
#
#   1. Primary clock definition — tells OpenSTA the frequency of the single
#      system clock driving all flip-flops in the design.
#
#   2. False-path declarations — exempts the three asynchronous ADS1292R SPI
#      input pins (ui_in[0:2]) from setup/hold timing analysis. Without these
#      declarations, OpenSTA attempts to close timing from those ports through
#      the first-stage flip-flops of the 2-FF synchronizer chains (mosi_meta,
#      sck_meta, cs_n_meta). That analysis is physically meaningless — those
#      FFs are the metastability boundary by design — and produces impossible
#      hold-time violations that block the entire OpenLane run.
#
# =============================================================================
# REFERENCE: architecture.md Section 7 (Timing Parameters)
#   System clock period  : 40.0 ns (25 MHz)
#   ADS1292R SPI clock   : ≤ 4 MHz  → 6.25 system cycles per SCK half-period
#   Synchronizer latency : 2 system cycles (80 ns), leaving 45 ns hold margin
#   Critical path        : 17-bit subtractor in biorle1_delta (~1.7 ns)
# =============================================================================

# -----------------------------------------------------------------------------
# 1. Primary Clock
# -----------------------------------------------------------------------------
# The Tiny Tapeout carrier board provides the system clock on the 'clk' port.
# Period = 40.0 ns (25 MHz). Waveform: rising edge at 0 ns, falling at 20 ns.
# All five modules (biorle1_spi_rx, biorle1_delta, biorle1_rle, biorle1_out,
# tt_um_biorle1) are driven from this single clock — single clock domain.
# -----------------------------------------------------------------------------
create_clock -name clk -period 40.0 -waveform {0 20} [get_ports clk]

# -----------------------------------------------------------------------------
# 2. Asynchronous SPI Input False Paths
# -----------------------------------------------------------------------------
# ui_in[0] = spi_mosi : ADS1292R DOUT serial data
# ui_in[1] = spi_sck  : ADS1292R SCLK, up to 4 MHz, independent clock source
# ui_in[2] = spi_cs_n : ADS1292R CS_N, frame boundary signal
#
# These three signals originate from the ADS1292R ECG analog front-end, which
# runs on its own internal oscillator (up to 4 MHz SPI rate). They are
# asynchronous to the 25 MHz system clock.
#
# Inside biorle1_spi_rx, each signal passes through an independent 2-flip-flop
# synchronizer chain before any combinational or sequential logic acts on it:
#   spi_mosi → mosi_meta (FF1) → mosi_sync (FF2) → shift logic
#   spi_sck  → sck_meta  (FF1) → sck_sync  (FF2) → edge detector
#   spi_cs_n → cs_n_meta (FF1) → cs_n_sync (FF2) → frame control
#
# The first-stage FFs (mosi_meta, sck_meta, cs_n_meta) are the intentional
# metastability capture points. STA must NOT propagate timing constraints
# from the input ports into or through these FFs.
# -----------------------------------------------------------------------------
set_false_path -from [get_ports {ui_in[0]}]   ;# spi_mosi — ADS1292R DOUT (async)
set_false_path -from [get_ports {ui_in[1]}]   ;# spi_sck  — ADS1292R SCLK (async)
set_false_path -from [get_ports {ui_in[2]}]   ;# spi_cs_n — ADS1292R CS_N (async)

# -----------------------------------------------------------------------------
# 3. Quasi-Static Control Input False Paths
# -----------------------------------------------------------------------------
# ui_in[3] = bypass     : High = raw-byte bypass mode (no delta compression)
# ui_in[4] = flush      : Pulse high one cycle to force-flush RLE buffer
# ui_in[6] = channel_sel: 0 = extract CH1, 1 = extract CH2
#
# These are driven by host MCU GPIOs (nRF52840, STM32L4). They are expected
# to be stable between ECG recording sessions — they are not cycle-accurate
# control signals. Their setup/hold timing relative to the system clock is
# not critical at the 250 SPS ECG sample rate (100,000 system cycles per
# sample). False-path declarations prevent OpenSTA from reporting spurious
# multi-cycle path violations on these slow-moving signals.
#
# ui_in[5] and ui_in[7] are unused (tied to _unused_ok in tt_um_biorle1.v).
# -----------------------------------------------------------------------------
set_false_path -from [get_ports {ui_in[3]}]   ;# bypass     — static during session
set_false_path -from [get_ports {ui_in[4]}]   ;# flush      — infrequent pulse
set_false_path -from [get_ports {ui_in[6]}]   ;# channel_sel — static during session

# =============================================================================
# End of biorle1.sdc
# =============================================================================
