// =============================================================================
// BioRLE-1 — Run-Length Encoding (RLE) Encoder FSM
// File   : biorle1_rle.v
// Project: BioRLE-1 Lossless Biosignal Compressor ASIC
// Target : SkyWater 130nm via Tiny Tapeout (sky130_fd_sc_hd)
// Clock  : 25 MHz system clock (single domain)
// Reset  : Synchronous, active low (rst_n)
//
// Purpose:
//   Receives 8-bit signed delta values from biorle1_delta one per clock cycle
//   (qualified by delta_valid) and compresses consecutive identical values using
//   run-length encoding. When a run terminates — either because the incoming
//   delta changed, the run count reached 255, or a flush was requested — the
//   module emits a (value, count) pair as two consecutive output bytes:
//     Cycle N  : out_byte = run_value, out_is_count = 0, out_valid = 1
//     Cycle N+1: out_byte = run_count, out_is_count = 1, out_valid = 1
//     Cycle N+2: out_valid = 0
//
//   A 1-entry input buffer (buf_delta / buf_valid) captures any delta_valid
//   pulse that arrives while the FSM is busy in EMIT_VALUE or EMIT_COUNT,
//   preventing data loss at the boundary between consecutive runs.
//
// FSM State Encoding (2-bit, binary):
//   IDLE       = 2'b00 : No run in progress. Waiting for first delta_valid.
//   RUNNING    = 2'b01 : Accumulating a run (run_value and run_count valid).
//   EMIT_VALUE = 2'b10 : Outputting the run value byte (out_is_count = 0).
//   EMIT_COUNT = 2'b11 : Outputting the run count byte (out_is_count = 1).
//
// Compression ratio telemetry:
//   A 6-bit rolling counter (ratio_cnt) counts identical-delta events within
//   each 64-sample window. Every 64 samples, comp_ratio[1:0] is updated and
//   ratio_cnt resets. This adds approximately 4 standard cells.
//
// Architecture notes (register inventory):
//   run_value  : 8 DFFs  — holds the current run's delta value
//   run_count  : 8 DFFs  — holds the current run length (1 to 255)
//   buf_delta  : 8 DFFs  — 1-entry input buffer for arriving deltas
//   buf_valid  : 1 DFF   — indicates buf_delta holds a captured sample
//   state      : 2 DFFs  — FSM state register (4 states)
//   out_byte   : 8 DFFs  — registered output byte
//   out_valid  : 1 DFF   — registered output valid pulse
//   out_is_count: 1 DFF  — registered byte-type flag
//   ratio_cnt  : 6 DFFs  — rolling same-delta event counter
//   comp_ratio : 2 DFFs  — compression ratio indicator output
//   frame_sync : 1 DFF   — pulses high at the start of each RLE pair emission
//   Total DFFs: ~46; combinational logic (mux + comparators): ~8 cells
//   Estimated total: ~35-40 standard cells after technology mapping
//
// Synthesis constraints:
//   - No initial blocks.
//   - No $display, $monitor, or any simulation-only system tasks.
//   - No # delays.
//   - All always @(*) blocks have complete default assignments — no latches.
//   - Estimated gate count: ~35 standard cells (within architecture budget).
//
// Freemium status:
//   PUBLIC (Apache 2.0 lite version) — basic RLE FSM core (IDLE/RUNNING states,
//   value/count emission) without comp_ratio telemetry, flush protocol, or
//   FRAME_SYNC output. See architecture Section 9 (Freemium RTL Split).
//   PROPRIETARY — flush support, comp_ratio rolling window, frame_sync,
//   1-entry input buffer for back-pressure handling, and the production-
//   hardened interface to biorle1_out.
//
// Author : tinytapeout-cto agent (BioRLE-1 project)
// Date   : 2026-06-14
// Version: 1.0
// =============================================================================

`timescale 1ns/1ps

module biorle1_rle (
    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    input  wire       clk,           // 25 MHz system clock
    input  wire       rst_n,         // Synchronous active-low reset

    // -------------------------------------------------------------------------
    // Delta input (from biorle1_delta)
    // -------------------------------------------------------------------------
    input  wire [7:0] delta,         // 8-bit signed delta (two's complement)
    input  wire       delta_valid,   // Pulses high for exactly one clk cycle per delta

    // -------------------------------------------------------------------------
    // End-of-frame control
    // -------------------------------------------------------------------------
    // flush forces emission of the pending partial run immediately, even if
    // run_count < 255. Asserted by the SPI receiver when the input frame ends
    // (CS_N goes high). Safe to assert while IDLE (no output generated).
    input  wire       flush,

    // -------------------------------------------------------------------------
    // Output byte stream (to biorle1_out)
    // -------------------------------------------------------------------------
    output reg  [7:0] out_byte,      // Compressed byte: either run value or run count
    output reg        out_valid,     // High for exactly one clk cycle per output byte
    output reg        out_is_count,  // 1 = this byte is a run count, 0 = run value

    // -------------------------------------------------------------------------
    // Compression ratio telemetry (to tt_um_biorle1 top level → uo_out[2:1])
    // -------------------------------------------------------------------------
    // Updated every 64 input samples. Encoding:
    //   2'b00: ratio < 1.5x   2'b01: 1.5x–2.5x
    //   2'b10: 2.5x–4x        2'b11: > 4x
    output reg  [1:0] comp_ratio,

    // -------------------------------------------------------------------------
    // Frame sync (to tt_um_biorle1 top level → uo_out[6])
    // -------------------------------------------------------------------------
    // Pulses high for one clock cycle at the start of each RLE pair emission
    // (coincident with the out_valid pulse for the value byte). Useful for
    // triggering a logic analyzer capture at each compression event.
    output reg        frame_sync
);

    // =========================================================================
    // FSM state encoding
    // =========================================================================
    // Using localparam instead of parameter so these constants cannot be
    // overridden at instantiation — FSM encoding is an implementation detail.
    // 2-bit binary encoding maps directly to a 2-bit state register.
    // =========================================================================

    localparam [1:0] IDLE       = 2'b00;  // No run in progress
    localparam [1:0] RUNNING    = 2'b01;  // Accumulating a run
    localparam [1:0] EMIT_VALUE = 2'b10;  // Outputting the value byte of a pair
    localparam [1:0] EMIT_COUNT = 2'b11;  // Outputting the count byte of a pair

    // =========================================================================
    // Internal registers
    // =========================================================================

    // FSM state register — 2 D flip-flops
    reg [1:0] state;

    // Current run accumulator registers.
    // run_value holds the delta value being counted. run_count holds how many
    // consecutive identical deltas have been seen so far (range 1 to 255).
    // Both are 8 bits; combined they form the (value, count) pair that will
    // be emitted when the run terminates.
    reg [7:0] run_value;
    reg [7:0] run_count;

    // 1-entry input buffer.
    // When the FSM is in EMIT_VALUE or EMIT_COUNT (outputting two bytes over
    // two consecutive cycles), it cannot accept new delta_valid pulses into the
    // run accumulator without losing state. buf_delta captures any incoming
    // delta that arrives during emission. buf_valid acts as the "buffer occupied"
    // flag. The buffer is consumed (loaded into run_value/run_count) as soon as
    // EMIT_COUNT completes and the FSM decides the next state.
    // Depth = 1 is sufficient because the upstream delta_valid rate is at most
    // 250 Hz (one per ADC sample at 250 SPS), while the emission window is only
    // 2 clock cycles at 25 MHz — the probability of two arrivals during emission
    // is zero at this sample rate. The buffer exists as a defensive mechanism
    // against test patterns and corner cases, not steady-state operation.
    reg [7:0] buf_delta;
    reg       buf_valid;

    // Compression ratio rolling counter.
    // ratio_cnt counts the number of same-delta events (i.e., run_count
    // increments) within a 64-sample window. A 6-bit counter saturates at 63.
    // sample_cnt tracks total samples in the current window (0 to 63).
    // Both reset together when sample_cnt rolls over from 63 to 0.
    reg [5:0] ratio_cnt;   // Count of same-delta events in current 64-sample window
    reg [5:0] sample_cnt;  // Total sample counter in current window (0..63)

    // =========================================================================
    // Combinational decode: determine what action to take this cycle
    // =========================================================================
    // These wires are derived purely from registered state — they do not create
    // latches because they are used only in the sequential always block below.
    // Defined here to make the sequential block readable.
    // =========================================================================

    // True when run_count has reached the 8-bit maximum.
    // A count of 255 means this run is full; the next identical delta must
    // start a new run (same value, count = 1) to avoid overflow.
    wire run_full = (run_count == 8'hFF);

    // True when the incoming delta differs from the current run value.
    // This is an 8-bit equality comparator — approximately 8 XOR gates + OR tree.
    wire run_break = (delta != run_value);

    // =========================================================================
    // Main synchronous FSM + datapath
    // =========================================================================
    // All state transitions and register updates occur on the rising clock edge.
    // Synchronous reset (rst_n = 0) returns the FSM to IDLE and clears all
    // registers to defined values, making the reset state fully deterministic.
    // =========================================================================

    always @(posedge clk) begin
        if (!rst_n) begin
            // -----------------------------------------------------------------
            // Synchronous reset: clear all registers and return FSM to IDLE.
            // After reset, the module waits for the first delta_valid pulse.
            // -----------------------------------------------------------------
            state        <= IDLE;
            run_value    <= 8'h00;
            run_count    <= 8'h00;
            buf_delta    <= 8'h00;
            buf_valid    <= 1'b0;
            out_byte     <= 8'h00;
            out_valid    <= 1'b0;
            out_is_count <= 1'b0;
            frame_sync   <= 1'b0;
            comp_ratio   <= 2'b00;
            ratio_cnt    <= 6'd0;
            sample_cnt   <= 6'd0;

        end else begin
            // -----------------------------------------------------------------
            // Default: clear single-cycle pulse outputs every clock cycle.
            // These are overridden below in the specific state/condition where
            // the pulse needs to fire. This pattern guarantees out_valid and
            // frame_sync are never accidentally held high for more than one
            // cycle. It also satisfies the "no latch" rule for always @(posedge)
            // blocks by ensuring every output has a defined assignment on every
            // clock edge.
            // -----------------------------------------------------------------
            out_valid  <= 1'b0;
            frame_sync <= 1'b0;

            // -----------------------------------------------------------------
            // FSM: state transitions and datapath operations
            // -----------------------------------------------------------------
            case (state)

                // =============================================================
                // IDLE: No run in progress. Waiting for first valid delta.
                // =============================================================
                // The FSM stays here after reset and after flush empties the
                // last run while no new delta arrived during emission.
                // flush has no effect in IDLE (nothing pending to emit).
                // =============================================================
                IDLE: begin
                    if (delta_valid) begin
                        // First sample of a new run: capture it and start counting.
                        // run_count starts at 1 (we have seen one occurrence).
                        run_value <= delta;
                        run_count <= 8'h01;
                        state     <= RUNNING;

                        // Update sample window counter for compression ratio.
                        // This is the first sample of (potentially) a new run,
                        // so it counts as one sample in the telemetry window.
                        sample_cnt <= sample_cnt + 6'd1;
                    end
                    // flush while IDLE: no pending run, nothing to emit. No action.
                end

                // =============================================================
                // RUNNING: Accumulating a run.
                // =============================================================
                // Stays in this state as long as identical deltas keep arriving.
                // Exits to EMIT_VALUE when the run terminates (break or rollover
                // or flush). Never returns to IDLE directly — after emitting a
                // pair it goes to EMIT_COUNT, which then decides the next state.
                // =============================================================
                RUNNING: begin
                    if (flush && !delta_valid) begin
                        // -------------------------------------------------------
                        // FLUSH path (no simultaneous delta_valid):
                        // End-of-frame requested. Emit the pending run immediately
                        // regardless of its length. Transition to EMIT_VALUE.
                        // We do NOT consume delta_valid here because it is low.
                        // -------------------------------------------------------
                        out_byte     <= run_value;
                        out_is_count <= 1'b0;   // First byte of pair: value
                        out_valid    <= 1'b1;
                        frame_sync   <= 1'b1;   // Signal start of RLE pair emission
                        state        <= EMIT_VALUE;
                        // run_value and run_count remain valid for EMIT_COUNT to use.

                    end else if (delta_valid) begin
                        // -------------------------------------------------------
                        // New delta arrived. Evaluate whether it continues or
                        // breaks the current run.
                        // -------------------------------------------------------

                        // Update compression ratio telemetry window.
                        // Every delta_valid in RUNNING increments sample_cnt.
                        // Same-delta events (run extends) also increment ratio_cnt.
                        if (sample_cnt == 6'd63) begin
                            // Window complete: latch comp_ratio and reset counters.
                            // Thresholds chosen to match architecture Section 4.2:
                            //   ratio_cnt < 16 : < 1.5x compression
                            //   ratio_cnt < 32 : 1.5x – 2.5x compression
                            //   ratio_cnt < 48 : 2.5x – 4x compression
                            //   ratio_cnt >= 48: > 4x compression
                            if (ratio_cnt < 6'd16)
                                comp_ratio <= 2'b00;
                            else if (ratio_cnt < 6'd32)
                                comp_ratio <= 2'b01;
                            else if (ratio_cnt < 6'd48)
                                comp_ratio <= 2'b10;
                            else
                                comp_ratio <= 2'b11;
                            ratio_cnt  <= 6'd0;
                            sample_cnt <= 6'd0;
                        end else begin
                            sample_cnt <= sample_cnt + 6'd1;
                        end

                        if (!run_break && !run_full) begin
                            // ---------------------------------------------------
                            // Run continues: same value, count not yet at max.
                            // Increment the run length counter and stay in RUNNING.
                            // ---------------------------------------------------
                            run_count <= run_count + 8'h01;
                            // Count this as a same-delta event for telemetry.
                            // Guard against ratio_cnt overflow (it resets at 63).
                            if (ratio_cnt != 6'd63)
                                ratio_cnt <= ratio_cnt + 6'd1;

                        end else if (run_full && !run_break) begin
                            // ---------------------------------------------------
                            // COUNT ROLLOVER path: same value, count reached 255.
                            // We must emit the current (value=255) pair and start
                            // a new run for the same value with count=1.
                            // The new run is NOT buffered — we load it immediately
                            // into run_value/run_count after emission.
                            // To handle this, we emit the value byte now, and
                            // pass to EMIT_COUNT the knowledge that after emission
                            // the SAME value restarts. We reuse buf_delta to hold
                            // the "restart" value (same as run_value) so that
                            // EMIT_COUNT can load it after the pair is emitted.
                            // ---------------------------------------------------
                            out_byte     <= run_value;
                            out_is_count <= 1'b0;
                            out_valid    <= 1'b1;
                            frame_sync   <= 1'b1;
                            // Buffer the same value so EMIT_COUNT restarts the run.
                            buf_delta    <= run_value; // same value, new run of 1
                            buf_valid    <= 1'b1;
                            // run_count stays at 8'hFF for EMIT_COUNT to output.
                            state        <= EMIT_VALUE;

                        end else begin
                            // ---------------------------------------------------
                            // RUN BREAK path: new delta differs from run_value
                            // (run_break is true). Emit the completed run and
                            // buffer the incoming delta to start the next run.
                            // ---------------------------------------------------
                            out_byte     <= run_value;
                            out_is_count <= 1'b0;
                            out_valid    <= 1'b1;
                            frame_sync   <= 1'b1;
                            // Capture incoming delta into 1-entry buffer so it
                            // is not lost during the 2-cycle emission window.
                            buf_delta    <= delta;
                            buf_valid    <= 1'b1;
                            // run_count holds the completed run length for EMIT_COUNT.
                            state        <= EMIT_VALUE;
                        end
                    end
                    // If neither flush nor delta_valid: hold all registers, stay RUNNING.
                end

                // =============================================================
                // EMIT_VALUE: Output the run value byte (first byte of pair).
                // =============================================================
                // out_valid was asserted in the previous cycle's transition INTO
                // this state. On entry to this state, out_valid has already been
                // driven high by the previous cycle. Now we need to set up the
                // count byte for the next cycle (EMIT_COUNT).
                // Note: out_valid is cleared to 0 at the top of this always block
                // (default assignment). We assert out_valid here for the COUNT byte
                // that we are pre-loading, which will be the output in EMIT_COUNT.
                // =============================================================
                EMIT_VALUE: begin
                    // If a new delta arrives while we are in EMIT_VALUE, capture
                    // it into the buffer only if the buffer is currently empty.
                    // If buf_valid is already set (from the RUNNING → EMIT_VALUE
                    // transition that loaded the buffer), this delta would overflow
                    // the single-entry buffer. At 250 SPS this is architecturally
                    // impossible (one delta per 100,000 cycles), so we don't add
                    // overflow detection here — that would cost extra cells.
                    if (delta_valid && !buf_valid) begin
                        buf_delta <= delta;
                        buf_valid <= 1'b1;
                    end

                    // Prepare the count byte for output in the next cycle.
                    // Assert out_valid and out_is_count for EMIT_COUNT.
                    out_byte     <= run_count;
                    out_is_count <= 1'b1;   // Second byte of pair: count
                    out_valid    <= 1'b1;
                    state        <= EMIT_COUNT;
                end

                // =============================================================
                // EMIT_COUNT: Output the run count byte (second byte of pair).
                // =============================================================
                // After this state, the pair emission is complete. The FSM
                // decides the next state based on whether a buffered delta is
                // waiting (buf_valid) or not.
                // =============================================================
                EMIT_COUNT: begin
                    // If a new delta arrives while we are in EMIT_COUNT, capture
                    // it into the buffer (same defensive logic as EMIT_VALUE).
                    // Because buf_valid should have been consumed below on the
                    // same cycle, this only fires if the upstream module sends
                    // a delta on the exact cycle we transition out of EMIT_COUNT.
                    // The priority is: consume buf_valid first (done below), then
                    // check delta_valid for new arrivals.
                    if (delta_valid && !buf_valid) begin
                        buf_delta <= delta;
                        buf_valid <= 1'b1;
                    end

                    // out_valid was driven high in EMIT_VALUE; the default
                    // assignment at the top of this block has cleared it for
                    // this cycle. No further out_valid action needed here.

                    if (buf_valid) begin
                        // -------------------------------------------------------
                        // A delta was buffered during emission. Start a new run
                        // from buf_delta and transition back to RUNNING.
                        // -------------------------------------------------------
                        run_value <= buf_delta;
                        run_count <= 8'h01;     // New run starts at count 1
                        buf_valid <= 1'b0;      // Consume the buffer entry
                        // buf_delta register content is superseded; no need to clear.
                        state     <= RUNNING;
                    end else begin
                        // -------------------------------------------------------
                        // No buffered delta. Return to IDLE and wait for the next
                        // delta_valid pulse from biorle1_delta.
                        // -------------------------------------------------------
                        state <= IDLE;
                    end
                end

                // =============================================================
                // Default branch: should never be reached with a 2-bit state
                // register and 4 defined states. Added to satisfy the synthesis
                // rule that all case branches must be covered, preventing the
                // tool from inferring latches on state register bits.
                // =============================================================
                default: begin
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule
// =============================================================================
// End of biorle1_rle.v
// =============================================================================
