`timescale 1ns / 1ps

module f2026_top (
    input  wire       clk_50m,
    input  wire       fpga_reset_n,
    input  wire       hmi_spi_cs_n,
    input  wire       hmi_spi_sck,
    input  wire       hmi_spi_mosi,
    output wire       hmi_spi_miso,
    output wire       hmi_irq,
    output wire       ad_clk,
    input  wire [7:0] ad_data,
    input  wire       ad_otr,
    output wire       da_clk,
    output wire [7:0] da_data,
    output wire       uart_tx
);
    // Keep the last valid DDS state through brief tracker dropouts. Returning
    // immediately to DAC midpoint draws a horizontal line in oscilloscope XY
    // mode even when the input re-locks a few cycles later.
    localparam [18:0] LOCK_HOLD_TICKS = 19'd250000; // 5 ms at 50 MHz
    // Bench fit over 10..100 kHz. Because the DAC path uses mid-sine, the
    // measured output-minus-input phase moves opposite to this phase word.
    // 0xFE93E93F is -2 degrees in a 32-bit phase accumulator.
    localparam [31:0] TRACKING_PHASE_CALIBRATION = 32'hFE93_E93F;
    // Additional low-frequency fit from the 1 kHz and 5 kHz bench points:
    // -2.5 degrees + 0.2016 degrees/kHz, clamped to zero above 12.4 kHz.
    localparam [31:0] LOW_FREQUENCY_PHASE_BASE = 32'hFE38_E38E;
    localparam [31:0] LOW_FREQUENCY_INCREMENT_LIMIT = 32'd1065152;

    wire reset_n;
    reg ad_clk_reg = 1'b0;
    wire sample_ce = ~ad_clk_reg;

    wire input_edge;
    wire input_locked;
    wire [31:0] period_ticks;
    wire [31:0] edge_count;
    wire [7:0] sample_min;
    wire [7:0] sample_max;
    wire otr_seen;
    wire calibration_done;
    wire [31:0] calibration_ticks;

    wire [2:0] mode;
    wire [7:0] amplitude;
    wire output_enable;
    wire free_run;
    wire [39:0] phase_increment;
    wire [31:0] phase_offset;
    wire [7:0] dac_mid;
    wire [7:0] threshold_hysteresis;
    wire calibration_start;
    wire [7:0] effective_threshold_hysteresis;
    wire [31:0] phase_monitor;
    wire [31:0] tracked_phase_increment;
    wire tracked_increment_valid;
    wire [31:0] tracking_latency_phase;
    wire [31:0] low_frequency_phase_calibration;
    wire [39:0] effective_phase_increment;
    wire [31:0] effective_phase_offset;
    wire output_active;
    reg [18:0] lock_hold_counter = 19'd0;
    reg tracked_output_qualified = 1'b0;

    assign ad_clk = ad_clk_reg;
    assign da_clk = ~clk_50m;
    assign uart_tx = 1'b1;
    assign effective_threshold_hysteresis =
        (threshold_hysteresis < 8'd8) ? 8'd8 : threshold_hysteresis;

    reset_conditioner reset_inst (
        .clk(clk_50m),
        .external_reset_n(fpga_reset_n),
        .reset_n(reset_n)
    );

    always @(posedge clk_50m) begin
        if (!reset_n)
            ad_clk_reg <= 1'b0;
        else
            ad_clk_reg <= ~ad_clk_reg;
    end

    f2026_input_tracker tracker_inst (
        .clk(clk_50m),
        .reset_n(reset_n),
        .sample_ce(sample_ce),
        .ad_data(ad_data),
        .ad_otr(ad_otr),
        .threshold_hysteresis(effective_threshold_hysteresis),
        .calibration_start(calibration_start),
        .edge_pulse(input_edge),
        .locked(input_locked),
        .period_ticks(period_ticks),
        .edge_count(edge_count),
        .sample_min(sample_min),
        .sample_max(sample_max),
        .otr_seen(otr_seen),
        .calibration_done(calibration_done),
        .calibration_ticks(calibration_ticks)
    );

    f2026_phase_increment phase_increment_inst (
        .clk(clk_50m),
        .reset_n(reset_n),
        .period_ticks(period_ticks),
        .phase_increment(tracked_phase_increment),
        .valid(tracked_increment_valid)
    );

    // 53 = 32 + 16 + 4 + 1. Keep the fixed delay compensation in LUT carry
    // chains so the waveform multiplier remains the only DSP consumer.
    assign tracking_latency_phase =
        (tracked_phase_increment << 5) +
        (tracked_phase_increment << 4) +
        (tracked_phase_increment << 2) +
        tracked_phase_increment;
    // 28 = 16 + 8 + 4. Arithmetic intentionally wraps modulo 2^32,
    // matching the phase accumulator representation for negative angles.
    assign low_frequency_phase_calibration =
        (tracked_phase_increment <= LOW_FREQUENCY_INCREMENT_LIMIT)
            ? (LOW_FREQUENCY_PHASE_BASE +
               (tracked_phase_increment << 4) +
               (tracked_phase_increment << 3) +
               (tracked_phase_increment << 2))
            : 32'd0;
    assign effective_phase_increment = free_run
        ? phase_increment : {tracked_phase_increment, 8'd0};
    assign effective_phase_offset = free_run
        ? phase_offset
        : ((mode == 3'd3)
            ? (phase_offset + TRACKING_PHASE_CALIBRATION +
               low_frequency_phase_calibration +
               {tracking_latency_phase[30:0], 1'b0})
            : (phase_offset + TRACKING_PHASE_CALIBRATION +
               low_frequency_phase_calibration +
               tracking_latency_phase));
    assign output_active = output_enable && (mode != 3'd0) &&
        (free_run ? ((mode == 3'd5) || (phase_increment != 40'd0))
                  : tracked_output_qualified);

    always @(posedge clk_50m) begin
        if (!reset_n || free_run || !output_enable || (mode == 3'd0)) begin
            lock_hold_counter <= 19'd0;
            tracked_output_qualified <= 1'b0;
        end else if (input_locked && tracked_increment_valid &&
                     (tracked_phase_increment != 32'd0)) begin
            lock_hold_counter <= LOCK_HOLD_TICKS;
            tracked_output_qualified <= 1'b1;
        end else if (tracked_output_qualified && (lock_hold_counter != 19'd0)) begin
            lock_hold_counter <= lock_hold_counter - 1'b1;
        end else begin
            lock_hold_counter <= 19'd0;
            tracked_output_qualified <= 1'b0;
        end
    end

    f2026_spi_control control_inst (
        .clk(clk_50m),
        .reset_n(reset_n),
        .spi_cs_n(hmi_spi_cs_n),
        .spi_sck(hmi_spi_sck),
        .spi_mosi(hmi_spi_mosi),
        .spi_miso(hmi_spi_miso),
        .irq(hmi_irq),
        .input_locked(input_locked),
        .output_active(output_active),
        .otr_seen(otr_seen),
        .period_ticks(period_ticks),
        .edge_count(edge_count),
        .sample_min(sample_min),
        .sample_max(sample_max),
        .calibration_done(calibration_done),
        .calibration_ticks(calibration_ticks),
        .mode(mode),
        .amplitude(amplitude),
        .output_enable(output_enable),
        .free_run(free_run),
        .calibration_start(calibration_start),
        .phase_increment(phase_increment),
        .phase_offset(phase_offset),
        .dac_mid(dac_mid),
        .threshold_hysteresis(threshold_hysteresis)
    );

    f2026_waveform_core waveform_inst (
        .clk(clk_50m),
        .reset_n(reset_n),
        .enable(output_active),
        .free_run(free_run),
        .mode(mode),
        .amplitude(amplitude),
        .dac_mid(dac_mid),
        .phase_increment(effective_phase_increment),
        .phase_offset(effective_phase_offset),
        .input_edge(input_edge),
        .input_locked(input_locked),
        .dac_data(da_data),
        .phase_monitor(phase_monitor)
    );
endmodule
