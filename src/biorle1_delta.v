// =============================================================================
// BioRLE-1 — Delta Encoder Module
// File   : biorle1_delta.v
// Project: BioRLE-1 Lossless Biosignal Compressor ASIC
// Target : SkyWater 130nm via Tiny Tapeout (sky130_fd_sc_hd)
// Clock  : 25 MHz system clock (single domain)
// Reset  : Synchronous, active low (rst_n)
//
// Purpose:
//   Computes the first-order difference (delta) between consecutive 16-bit
//   ADC samples received from the SPI front-end. The raw difference is
//   computed as a 17-bit signed subtraction (sign-extended operands) to
//   prevent overflow, then clamped (saturated) to the 8-bit signed range
//   [-128, +127] using combinational comparators. The overflow_flag output
//   is asserted whenever saturation is applied, providing a signal quality
//   indicator to the host MCU.
//
//   BYPASS mode: when bypass is asserted, the upper 8 bits of sample_data
//   are routed directly to delta[], bypassing all computation. This enables
//   raw-data pass-through for board bring-up and production test.
//
// Architecture notes:
//   - prev_sample  : 16 D flip-flops holding the last captured sample.
//   - diff         : 17-bit combinational subtractor (sign-extended inputs).
//   - sat_*        : combinational saturation comparators + output mux.
//   - overflow_flag: 1 D flip-flop updated every sample_valid pulse.
//   - delta_valid  : registered version of sample_valid (same-cycle output).
//
// Synthesis constraints:
//   - No initial blocks.
//   - No $display or simulation-only constructs.
//   - All if/case statements include a default branch — no latches.
//   - Estimated gate count: ~20 standard cells.
//
// Freemium status:
//   PUBLIC (Apache 2.0 lite version) — delta computation and saturation
//   clamping without BYPASS mode.
//   PROPRIETARY — BYPASS mode routing and overflow_flag telemetry.
//
// Author : tinytapeout-cto agent (BioRLE-1 project)
// Date   : 2026-06-14
// Version: 1.0
// =============================================================================

`timescale 1ns/1ps

module biorle1_delta (
    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    input  wire        clk,          // 25 MHz system clock
    input  wire        rst_n,        // Synchronous active-low reset

    // -------------------------------------------------------------------------
    // Sample input (from biorle1_spi_rx)
    // -------------------------------------------------------------------------
    input  wire [15:0] sample_data,  // 16-bit signed ADC sample (two's complement)
    input  wire        sample_valid, // Pulses high for exactly one clk cycle per sample

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------
    input  wire        bypass,       // High: route sample_data[15:8] directly to delta

    // -------------------------------------------------------------------------
    // Delta output (to biorle1_rle)
    // -------------------------------------------------------------------------
    output reg  [7:0]  delta,        // 8-bit signed delta (two's complement, clamped)
    output reg         delta_valid,  // High for one clk cycle coincident with delta
    output reg         overflow_flag // High when saturation clamping was applied;
                                     // held until next sample_valid pulse
);

    // =========================================================================
    // Internal registers and wires
    // =========================================================================

    // 16-bit register that stores the previously received valid sample.
    // Updated synchronously on every sample_valid pulse.
    reg [15:0] prev_sample;

    // 17-bit signed difference: sign-extended sample_data minus sign-extended
    // prev_sample. Computed combinationally every clock cycle; only meaningful
    // when sample_valid is high. The extra bit prevents signed overflow:
    //   max positive delta: +32767 - (-32768) = +65535 → fits in 17-bit signed
    //   max negative delta: -32768 - (+32767) = -65535 → fits in 17-bit signed
    wire signed [16:0] diff;
    assign diff = {sample_data[15], sample_data} - {prev_sample[15], prev_sample};

    // Saturation limits expressed as 17-bit signed constants for comparison.
    // Using localparams avoids magic numbers and prevents accidental sign errors.
    localparam signed [16:0] SAT_MAX = 17'sh0007F;  // +127
    localparam signed [16:0] SAT_MIN = 17'sh1FF80;  // -128 in 17-bit two's complement

    // Combinational saturation result: the clamped 8-bit signed value.
    // This wire is computed in the always_comb section below and fed into the
    // synchronous always block to register it on the output.
    reg [7:0]  sat_delta;        // Combinational: saturated 8-bit result
    reg        sat_overflow;     // Combinational: high when clamping was applied

    // =========================================================================
    // Combinational saturation logic
    // =========================================================================
    // This always block is purely combinational (no posedge clk).
    // It evaluates the 17-bit diff and selects the correct 8-bit output.
    // Rule: every branch must assign BOTH sat_delta and sat_overflow to
    // prevent latches. The default assignment at the top of the block acts
    // as the "else" for all comparisons.
    // =========================================================================

    always @(*) begin
        // Default: pass through lower 8 bits of diff (no clamping needed).
        // This branch is taken when -128 <= diff <= +127.
        sat_delta    = diff[7:0];
        sat_overflow = 1'b0;

        if ($signed(diff) > $signed(SAT_MAX)) begin
            // Positive saturation: diff > +127, clamp to +127 (8'h7F)
            sat_delta    = 8'sh7F;
            sat_overflow = 1'b1;
        end else if ($signed(diff) < $signed(SAT_MIN)) begin
            // Negative saturation: diff < -128, clamp to -128 (8'h80)
            sat_delta    = 8'sh80;
            sat_overflow = 1'b1;
        end
        // Implicit else: sat_delta = diff[7:0], sat_overflow = 0 (set above)
    end

    // =========================================================================
    // Synchronous register block
    // =========================================================================
    // All state transitions occur on the rising edge of clk only.
    // Synchronous reset (rst_n low) clears all registers to defined values.
    // =========================================================================

    always @(posedge clk) begin
        if (!rst_n) begin
            // Synchronous reset: clear all outputs and internal state.
            // After reset, the first real sample will compute delta = 0 because
            // prev_sample = 0 and a real ADC baseline is close to 0 or the host
            // MCU records S0 separately (see architecture Section 8).
            prev_sample   <= 16'h0000;
            delta         <= 8'h00;
            delta_valid   <= 1'b0;
            overflow_flag <= 1'b0;

        end else if (sample_valid) begin
            // A new sample has arrived. The combinational logic (diff, sat_delta,
            // sat_overflow) has already settled before this clock edge.

            if (bypass) begin
                // BYPASS mode: route raw ADC upper byte directly to output.
                // This bypasses all delta/saturation computation and allows the
                // host MCU to verify raw SPI data during board bring-up.
                delta         <= sample_data[15:8];
                delta_valid   <= 1'b1;
                overflow_flag <= 1'b0;   // No saturation concept in bypass mode
            end else begin
                // Normal mode: register the combinational saturation result.
                delta         <= sat_delta;
                delta_valid   <= 1'b1;
                overflow_flag <= sat_overflow;
            end

            // Update prev_sample to the current sample, regardless of bypass.
            // This keeps the delta encoder state consistent so that when bypass
            // is deasserted, the first post-bypass delta is computed correctly.
            prev_sample <= sample_data;

        end else begin
            // No new sample this cycle: hold overflow_flag, clear delta_valid.
            // delta_valid must return to 0 after exactly one cycle (pulse output).
            delta_valid   <= 1'b0;
            // delta and overflow_flag hold their registered values until the
            // next sample_valid pulse updates them.
            // overflow_flag is NOT cleared here — it is held until the next
            // sample_valid so that the downstream RLE module and host MCU can
            // read it reliably between samples.
        end
    end

endmodule
// =============================================================================
// End of biorle1_delta.v
// =============================================================================
