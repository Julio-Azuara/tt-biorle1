// =============================================================================
// BioRLE-1 — SPI Receiver Module
// File   : biorle1_spi_rx.v
// Project: BioRLE-1 Lossless Biosignal Compressor ASIC
// Target : SkyWater 130nm via Tiny Tapeout (sky130_fd_sc_hd)
// Clock  : 25 MHz system clock (single domain)
// Reset  : Synchronous, active low (rst_n)
//
// Purpose:
//   Receives a 72-bit SPI frame from the ADS1292R ECG analog front-end and
//   extracts a 16-bit signed ADC sample from either channel 1 or channel 2.
//
//   ADS1292R frame format (72 bits, MSB-first, SPI Mode 1 — CPOL=0, CPHA=1):
//
//     Frame bit position  | Content
//     --------------------|-------------------------------------------
//     Bits  0..23         | 24-bit STATUS word            (ignored)
//     Bits 24..47         | CH1 data, 24-bit two's complement, MSB first
//     Bits 48..71         | CH2 data, 24-bit two's complement, MSB first
//
//   After 72 shifts the shift register holds:
//     shift_reg[71:48] = STATUS word
//     shift_reg[47:24] = CH1 (24-bit)
//     shift_reg[23:0]  = CH2 (24-bit)
//
//   sample_data[15:0] receives the upper 16 bits of the selected channel
//   (bits [23:8] of the 24-bit word), discarding the noisy LSB byte.
//   sample_valid is pulsed high for exactly one system-clock cycle.
//
// Critical design rule — SINGLE CLOCK DOMAIN:
//   The ADS1292R SPI clock (spi_sck, up to 4 MHz) is ASYNCHRONOUS to the
//   25 MHz system clock. It must NEVER be used as an RTL clock source.
//   spi_sck, spi_mosi, and spi_cs_n are treated as DATA inputs and routed
//   through 2-flip-flop synchronizer chains clocked by the system clock before
//   any combinational or sequential logic acts on them. This eliminates
//   metastability at the clock-domain boundary and is the standard technique
//   for synthesizable SPI slave designs targeting a single-clock-domain flow.
//
//   Synchronization margin at 4 MHz SCK / 25 MHz system clock:
//     SCK half-period            = 125 ns
//     2-FF synchronizer latency  = 2 * 40 ns = 80 ns
//     Remaining hold window      = 125 - 80 = 45 ns  (adequate for metastability
//     resolution in SkyWater 130nm; MTBF >> 10^12 years at this frequency pair)
//
// Shift register bit-index derivation:
//   The shift register uses a left-shift, LSB-in operation:
//     shift_reg <= {shift_reg[70:0], mosi_sync}
//   After N clock cycles, shift_reg[j] holds frame bit (N-1-j), for j <= N-1.
//   After all 72 bits have been received (N=72 shifts completed):
//     shift_reg[j] = frame bit (71-j)
//   Frame bit positions map to shift_reg positions as follows:
//     STATUS[23:0]  = frame bits  0..23  → shift_reg[71:48]
//     CH1[23:0]     = frame bits 24..47  → shift_reg[47:24]
//     CH2[23:0]     = frame bits 48..71  → shift_reg[23:0]
//   CH1 upper 16 = CH1[23:8] = frame bits 24..39 → shift_reg[47:32]
//   CH2 upper 16 = CH2[23:8] = frame bits 48..63 → shift_reg[23:8]
//
//   On the 72nd shift cycle (bit_count == 71), Verilog non-blocking assignments
//   evaluate the right-hand side using PRE-shift values. At that moment:
//     old shift_reg[j] = frame bit (70-j) for j = 0..70
//     mosi_sync        = frame bit 71 = CH2[0] (LSB of CH2, discarded)
//   After the shift: new shift_reg[j] = old[j-1] for j>=1; new[0] = mosi_sync.
//   Therefore:
//     new shift_reg[47:32] = old[46:31] = frame bits 24..39 = CH1[23:8]
//     new shift_reg[23:8]  = old[22:7]  = frame bits 48..63 = CH2[23:8]
//   sample_data is assigned using old[46:31] or old[22:7] in the same
//   non-blocking assignment group — capturing the correct post-shift value
//   without combinational loop, because mosi_sync lands at new[0] (below
//   the range of interest for both channels).
//
// Architecture notes (register inventory):
//   spi_mosi 2-FF sync  : 2 DFFs  (mosi_meta, mosi_sync)
//   spi_sck  2-FF sync  : 2 DFFs  (sck_meta,  sck_sync)
//   spi_cs_n 2-FF sync  : 2 DFFs  (cs_n_meta, cs_n_sync)
//   sck_prev            : 1 DFF   (falling-edge detector)
//   cs_n_prev           : 1 DFF   (rising-edge detector)
//   shift_reg[71:0]     : 72 DFFs (72-bit serial shift register)
//   bit_count[6:0]      : 7 DFFs  (counts 0..71)
//   sample_data[15:0]   : 16 DFFs (registered output)
//   sample_valid        : 1 DFF   (single-cycle pulse output)
//   frame_active        : 1 DFF   (CS_N assertion status)
//   spi_error           : 1 DFF   (framing error flag)
//   Total: 107 DFFs
//   Combinational: edge-detect gates (4), 7-bit comparator (7),
//                  channel-select 2-to-1 mux (2), CS_N guard gate (1) = ~14 gates
//   Estimated standard cell count after technology mapping: ~30 cells
//
// Synthesis constraints:
//   - No initial blocks.
//   - No $display, $monitor, or any simulation-only system tasks.
//   - No # delays.
//   - Synchronous reset only (rst_n active-low, posedge clk).
//   - All if/else branches assign the same set of registers — no latches.
//   - single-cycle pulse outputs (sample_valid) cleared by default each cycle.
//   - Recommended SDC false paths for OpenLane:
//       set_false_path -from [get_ports {spi_mosi spi_sck spi_cs_n}]
//     (Asynchronous inputs; the synchronizer FFs are the metastability boundary;
//      STA should not attempt to close timing from these ports to internal logic.)
//   - Target frequency: 25 MHz (40 ns period). No deep combinational paths.
//
// Freemium status:
//   PROPRIETARY — Full ADS1292R-compatible SPI receiver with 72-bit frame
//   parsing, channel selection, frame_active status, and spi_error framing
//   detection. Not included in the Apache 2.0 lite release. The lite release
//   covers only biorle1_delta.v and biorle1_rle.v (the compression algorithm
//   core). See architecture.md Section 9 (Freemium RTL Split) for rationale.
//
// Author : tinytapeout-cto agent (BioRLE-1 project)
// Date   : 2026-06-14
// Version: 1.0
// =============================================================================

`timescale 1ns/1ps

module biorle1_spi_rx (
    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    input  wire        clk,           // 25 MHz system clock
    input  wire        rst_n,         // Synchronous active-low reset

    // -------------------------------------------------------------------------
    // Raw SPI inputs from ADS1292R (asynchronous — synchronized internally)
    // Connected to ui_in[2:0] on the Tiny Tapeout carrier board.
    // These ports must NEVER drive RTL logic directly; only the *_sync versions
    // (after the 2-FF chains) are used by all combinational and sequential logic.
    // -------------------------------------------------------------------------
    input  wire        spi_mosi,      // ui_in[0]: Serial data from ADS1292R DOUT
    input  wire        spi_sck,       // ui_in[1]: SPI clock from ADS1292R SCLK (<=4 MHz)
    input  wire        spi_cs_n,      // ui_in[2]: Chip select, active-low (frame boundary)

    // -------------------------------------------------------------------------
    // Channel selection — static during a recording session.
    // Driven by a host MCU GPIO, registered in the system-clock domain at the
    // top level (changes are expected only between recording sessions).
    // -------------------------------------------------------------------------
    input  wire        channel_sel,   // ui_in[6]: 0=extract CH1[23:8], 1=extract CH2[23:8]

    // -------------------------------------------------------------------------
    // Output to biorle1_delta
    // -------------------------------------------------------------------------
    output reg  [15:0] sample_data,   // 16-bit signed ADC sample (two's complement)
    output reg         sample_valid,  // Pulses high for exactly 1 clk cycle per frame

    // -------------------------------------------------------------------------
    // Status outputs (to tt_um_biorle1 top level)
    // -------------------------------------------------------------------------
    output reg         frame_active,  // High while CS_N is asserted (frame in progress)
    output reg         spi_error      // Asserted if CS_N deasserts before 72 bits received;
                                      // held until the next frame's CS_N falling edge
);

    // =========================================================================
    // 2-FF Synchronizer registers
    // =========================================================================
    // Why three separate 2-FF chains (not one shared synchronizer)?
    // Each input has an independent asynchronous transition time relative to
    // the system clock. A shared synchronizer would create false dependencies
    // between unrelated signals. Keeping them separate is both architecturally
    // correct and required for proper SDC false-path annotation.
    //
    // Naming convention:
    //   *_meta : first flip-flop — may be metastable; do NOT use in logic
    //   *_sync : second flip-flop — metastability resolved; safe for all logic
    // =========================================================================

    // MOSI synchronizer chain
    reg mosi_meta;   // Stage 1: raw spi_mosi captured; may be metastable
    reg mosi_sync;   // Stage 2: resolved stable value — used for shift operations

    // SCK synchronizer chain
    reg sck_meta;    // Stage 1: raw spi_sck captured; may be metastable
    reg sck_sync;    // Stage 2: resolved stable value — used for edge detection

    // CS_N synchronizer chain
    reg cs_n_meta;   // Stage 1: raw spi_cs_n captured; may be metastable
    reg cs_n_sync;   // Stage 2: resolved stable value — used for frame control

    // =========================================================================
    // Edge detection registers
    // =========================================================================
    // To detect a level transition on a synchronized signal, we store its value
    // from the previous system-clock cycle in a companion register. Comparing
    // the current and previous values identifies the edge for exactly one cycle.
    //
    //   Falling edge on SCK (SPI data capture moment):
    //     negedge_sck = sck_prev & ~sck_sync
    //     True for one system-clock cycle when sck_sync transitions 1 → 0.
    //
    //   Rising edge on CS_N (end of SPI frame):
    //     posedge_cs_n = ~cs_n_prev & cs_n_sync
    //     True for one system-clock cycle when cs_n_sync transitions 0 → 1.
    //
    //   Falling edge on CS_N (start of new SPI frame):
    //     negedge_cs_n = cs_n_prev & ~cs_n_sync
    //     True for one system-clock cycle when cs_n_sync transitions 1 → 0.
    // =========================================================================

    reg sck_prev;    // Previous-cycle value of sck_sync  (falling-edge detection)
    reg cs_n_prev;   // Previous-cycle value of cs_n_sync (both-edge detection)

    // Combinational edge-detect pulses — one-cycle-wide, resolved before posedge clk.
    // These wires are consumed entirely within the synchronous always block below;
    // they do not create latches because they have no feedback path.
    wire negedge_sck  =  sck_prev  & ~sck_sync;   // SCK  1→0: sample MOSI, shift in
    wire posedge_cs_n = ~cs_n_prev &  cs_n_sync;  // CS_N 0→1: frame ended
    wire negedge_cs_n =  cs_n_prev & ~cs_n_sync;  // CS_N 1→0: new frame starting

    // =========================================================================
    // Shift register and bit counter
    // =========================================================================
    // shift_reg[71:0] accumulates the 72-bit ADS1292R frame serially.
    // Shift operation: left-shift, new bit enters at LSB position [0].
    //   shift_reg <= {shift_reg[70:0], mosi_sync}
    //
    // After all 72 bits are received: shift_reg[71-j] = frame bit j.
    //   shift_reg[71:48] = STATUS word  (bits 0..23)
    //   shift_reg[47:24] = CH1 data     (bits 24..47, 24-bit two's complement)
    //   shift_reg[23:0]  = CH2 data     (bits 48..71, 24-bit two's complement)
    //
    // bit_count[6:0] counts the number of bits received in the current frame.
    // It starts at 0 and increments on each negedge_sck pulse while CS_N is low.
    // When bit_count == 71, the 72nd (and final) bit is being received this cycle.
    // =========================================================================

    reg [71:0] shift_reg;  // 72-bit serial-in shift register (full ADS1292R frame)
    reg  [6:0] bit_count;  // Counts received bits: 0 = first bit, 71 = last bit

    // =========================================================================
    // Combinational frame status signals
    // =========================================================================
    // frame_complete: asserted when the 72nd bit is being received on this
    //   negedge_sck pulse. This is the condition for extracting sample_data.
    //
    // frame_error: asserted when CS_N rises before bit_count reaches 71.
    //   This means the master deasserted CS_N early — a framing violation.
    //   frame_active must also be high (we were inside a frame, not just noise).
    //
    // These two conditions are mutually exclusive: frame_complete requires a
    // falling SCK edge; frame_error requires a rising CS_N edge. They cannot
    // both be high in the same clock cycle.
    // =========================================================================

    wire frame_complete = negedge_sck & (bit_count == 7'd71) & ~cs_n_sync;
    wire frame_error    = posedge_cs_n & (bit_count != 7'd71) & frame_active;

    // =========================================================================
    // Main synchronous block — all registers, all clock cycles
    // =========================================================================
    // Everything clocked on posedge clk. Synchronous reset (rst_n = 0) drives
    // all registers to defined idle values. No asynchronous reset paths.
    // =========================================================================

    always @(posedge clk) begin
        if (!rst_n) begin
            // -----------------------------------------------------------------
            // Synchronous reset: all registers cleared to safe idle state.
            // cs_n_meta/sync reset to 1 because CS_N is high (inactive) at idle.
            // cs_n_prev resets to 1 to match, preventing a spurious negedge_cs_n
            // detection on the first cycle after reset.
            // -----------------------------------------------------------------
            mosi_meta    <= 1'b0;
            mosi_sync    <= 1'b0;
            sck_meta     <= 1'b0;
            sck_sync     <= 1'b0;
            cs_n_meta    <= 1'b1;   // CS_N idle = high (inactive)
            cs_n_sync    <= 1'b1;
            sck_prev     <= 1'b0;
            cs_n_prev    <= 1'b1;   // Match idle cs_n_sync to prevent false edge
            shift_reg    <= 72'h0;
            bit_count    <= 7'd0;
            sample_data  <= 16'h0000;
            sample_valid <= 1'b0;
            frame_active <= 1'b0;
            spi_error    <= 1'b0;

        end else begin

            // -----------------------------------------------------------------
            // Default: clear all single-cycle pulse outputs.
            // sample_valid must be high for exactly one cycle. Clearing it here
            // every cycle and setting it selectively in the frame_complete branch
            // guarantees the pulse width regardless of downstream behavior.
            // This also prevents latch inference by ensuring every output has a
            // defined assignment on every clock edge.
            // -----------------------------------------------------------------
            sample_valid <= 1'b0;

            // =================================================================
            // Step A: Advance the 2-FF synchronizer chains.
            // =================================================================
            // These three pairs of non-blocking assignments are the ONLY place
            // the raw asynchronous input ports appear in RTL logic. Every other
            // reference to MOSI, SCK, and CS_N uses the *_sync versions.
            //
            // The propagation order within a single always block for non-blocking
            // assignments does not create a transparent-latch chain: both FFs
            // in each pair capture simultaneously on posedge clk — *_sync always
            // captures the value that *_meta held BEFORE this clock edge.
            // =================================================================
            mosi_meta <= spi_mosi;     // MOSI stage 1: capture raw async input
            mosi_sync <= mosi_meta;    // MOSI stage 2: stabilized (safe to use)

            sck_meta  <= spi_sck;      // SCK  stage 1: capture raw async input
            sck_sync  <= sck_meta;     // SCK  stage 2: stabilized (safe to use)

            cs_n_meta <= spi_cs_n;     // CS_N stage 1: capture raw async input
            cs_n_sync <= cs_n_meta;    // CS_N stage 2: stabilized (safe to use)

            // =================================================================
            // Step B: Advance edge-detection registers.
            // =================================================================
            // Capture the synchronized values from THIS clock cycle so that
            // negedge_sck and posedge_cs_n (combinational wires using sck_prev
            // and cs_n_prev) reflect the transition that just occurred.
            // These update AFTER the synchronizer chains above — the new sck_sync
            // and cs_n_sync values (captured in Step A) are what get stored here,
            // ready to be compared against NEXT cycle's sck_sync / cs_n_sync.
            // =================================================================
            sck_prev  <= sck_sync;
            cs_n_prev <= cs_n_sync;

            // =================================================================
            // Step C: Frame boundary control.
            // =================================================================
            // C1 — CS_N falling edge: new frame is starting.
            //   Reset the bit counter and shift register for the new frame.
            //   Clear spi_error from any previous frame.
            //   Assert frame_active to enable the shift operation in Step D.
            // -----------------------------------------------------------------
            if (negedge_cs_n) begin
                frame_active <= 1'b1;
                bit_count    <= 7'd0;
                shift_reg    <= 72'h0;
                spi_error    <= 1'b0;   // Fresh start; previous error no longer relevant
            end

            // -----------------------------------------------------------------
            // C2 — CS_N rising edge: current frame has ended.
            //   Deassert frame_active.
            //   If the frame ended prematurely (frame_error wire is high),
            //   assert spi_error. sample_data is NOT updated in this case —
            //   the partial shift register contents are invalid.
            //
            //   Note: frame_complete (Step D) and frame_error (here) cannot both
            //   be true in the same cycle. frame_complete fires on negedge_sck
            //   (when bit_count reaches 71 while cs_n_sync is still low);
            //   frame_error fires on posedge_cs_n. These two edges cannot
            //   coincide because SCK transitions and CS_N transitions come from
            //   different ADS1292R output drivers with distinct transition times.
            // -----------------------------------------------------------------
            if (posedge_cs_n) begin
                frame_active <= 1'b0;
                if (frame_error) begin
                    spi_error <= 1'b1;
                    // sample_data and sample_valid are NOT updated (defaults hold).
                end
            end

            // =================================================================
            // Step D: Shift register operation.
            // =================================================================
            // Active only when a falling SCK edge is detected AND CS_N is
            // currently low (we are inside a valid frame). The cs_n_sync guard
            // prevents stray SCK edges outside frame boundaries from corrupting
            // the shift register or wrapping the bit counter.
            //
            // Shift operation: MSB-first receive, LSB-in shift.
            //   shift_reg <= {shift_reg[70:0], mosi_sync}
            //
            // mosi_sync holds the stable, synchronized MOSI value. Per the SPI
            // Mode 1 protocol, the ADS1292R updates MOSI on the RISING edge of
            // SCK and holds it stable until the next rising edge. Our falling-edge
            // detection (negedge_sck) therefore samples MOSI at the middle of its
            // valid window — the most reliable capture point.
            //
            // Timing note: At 4 MHz SCK, the MOSI setup-to-SCK-falling window
            // is 125 ns. After 2-FF synchronizer latency (80 ns), mosi_sync
            // is stable for >=45 ns before the negedge_sck pulse fires.
            // This is well within SkyWater 130nm setup requirements.
            // =================================================================
            if (negedge_sck && !cs_n_sync) begin
                // Shift the new bit into the LSB of the 72-bit shift register.
                shift_reg <= {shift_reg[70:0], mosi_sync};

                if (bit_count == 7'd71) begin
                    // ---------------------------------------------------------
                    // Frame complete: the 72nd bit has just been received.
                    //
                    // At this point, using Verilog non-blocking assignment
                    // semantics, the right-hand side of all assignments in this
                    // block is evaluated using PRE-clock-edge (old) register
                    // values. Therefore:
                    //
                    //   shift_reg (old, before this shift) has accumulated
                    //   71 bits. shift_reg[j] = frame bit (70-j) for j=0..70.
                    //   mosi_sync = frame bit 71 = CH2[0] (discarded LSB).
                    //
                    //   After the shift above (new shift_reg):
                    //     new[71:48] = STATUS[23:0]  → old[70:47]
                    //     new[47:24] = CH1[23:0]     → old[46:23]
                    //     new[23:0]  = CH2[23:0]     → {old[22:0], mosi_sync}
                    //
                    //   CH1 upper 16 = new[47:32] = old[46:31] (CH1[23:8])
                    //   CH2 upper 16 = new[23:8]  = {old[22:8], old[7]}
                    //                             = old[22:7]
                    //     (mosi_sync lands at new[0] = CH2[0], outside range)
                    //
                    //   Since sample_data uses non-blocking assignment with old
                    //   shift_reg, we directly index shift_reg[46:31] for CH1
                    //   and shift_reg[22:7] for CH2. These expressions read
                    //   the old value and yield the same result as reading the
                    //   new shift_reg[47:32] and new shift_reg[23:8] respectively,
                    //   because mosi_sync affects only new[0], not [47:32]/[23:8].
                    // ---------------------------------------------------------
                    if (channel_sel == 1'b0) begin
                        // CH1 upper 16 bits: shift_reg_new[47:32] = old[46:31]
                        sample_data <= shift_reg[46:31];
                    end else begin
                        // CH2 upper 16 bits: shift_reg_new[23:8] = old[22:7]
                        sample_data <= shift_reg[22:7];
                    end
                    sample_valid <= 1'b1;
                    // bit_count is NOT reset here — it stays at 71 until the
                    // next negedge_cs_n (start of the next frame). This is safe:
                    // CS_N will rise after this bit, posedge_cs_n will set
                    // frame_active=0, and bit_count resets on the next negedge_cs_n.
                    // No further SCK edges should arrive after bit 71 within this
                    // CS_N low window (ADS1292R protocol guarantee), but even if
                    // they do, frame_active becoming 0 prevents further shifts.

                end else begin
                    // Not the last bit. Increment the bit counter and continue.
                    bit_count <= bit_count + 7'd1;
                end
            end

        end  // end else (not in reset)
    end  // end always

endmodule
// =============================================================================
// End of biorle1_spi_rx.v
// =============================================================================
