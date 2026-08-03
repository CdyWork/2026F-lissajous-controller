`timescale 1ns / 1ps

module f2026_input_tracker #(
    parameter [31:0] CALIBRATION_PERIODS = 32'd50000
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        sample_ce,
    input  wire [7:0]  ad_data,
    input  wire        ad_otr,
    input  wire [7:0]  threshold_hysteresis,
    input  wire        calibration_start,
    output reg         edge_pulse = 1'b0,
    output reg         locked = 1'b0,
    output reg [31:0]  period_ticks = 32'd0,
    output reg [31:0]  edge_count = 32'd0,
    output reg [7:0]   sample_min = 8'h80,
    output reg [7:0]   sample_max = 8'h80,
    output reg         otr_seen = 1'b0,
    output reg         calibration_done = 1'b0,
    output reg [31:0]  calibration_ticks = 32'd0
);
    // A 100 kHz input is nominally 500 system clocks. ADC sampling and the
    // hysteresis crossing can move adjacent detected edges by one sample.
    localparam [31:0] MIN_PERIOD_TICKS = 32'd480;
    // Keep margin around the specified 1 kHz lower boundary for generator and
    // board-clock tolerance while still rejecting unrelated slow crossings.
    localparam [31:0] MAX_PERIOD_TICKS = 32'd55000;
    localparam [31:0] EDGE_TIMEOUT_TICKS = 32'd125000;
    // 50,000 periods of a 100 kHz reference last 0.5 s. One system-clock
    // count is then only 0.04 ppm, suitable for a quick bench calibration.

    reg [31:0] period_counter = 32'd0;
    reg [39:0] period_filter_q8 = 40'd0;
    reg [2:0] valid_edges = 3'd0;
    reg [31:0] jump_candidate_period = 32'd0;
    reg jump_candidate_valid = 1'b0;
    reg seen_edge = 1'b0;
    reg low_armed = 1'b0;
    reg [15:0] window_count = 16'd0;
    reg [7:0] window_min = 8'hFF;
    reg [7:0] window_max = 8'h00;
    reg calibration_start_last = 1'b0;
    reg calibration_wait_first_edge = 1'b0;
    reg calibration_running = 1'b0;
    reg [31:0] calibration_counter = 32'd0;
    reg [31:0] calibration_period_count = 32'd0;

    wire [8:0] threshold_low_wide = 9'd128 - {1'b0, threshold_hysteresis};
    wire [8:0] threshold_high_wide = 9'd128 + {1'b0, threshold_hysteresis};
    wire [7:0] threshold_low = threshold_low_wide[7:0];
    wire [7:0] threshold_high = threshold_high_wide[7:0];
    wire [39:0] measured_period_q8 = {period_counter, 8'd0};
    wire [39:0] filter_rise = period_filter_q8 +
        ((measured_period_q8 - period_filter_q8 + 40'd8) >> 4);
    wire [39:0] filter_fall = period_filter_q8 -
        ((period_filter_q8 - measured_period_q8 + 40'd8) >> 4);
    wire [39:0] filter_next = (measured_period_q8 >= period_filter_q8) ?
        filter_rise : filter_fall;
    wire [31:0] filtered_period_next =
        (filter_next + 40'd128) >> 8;
    wire [31:0] period_delta = (period_counter > period_ticks)
        ? (period_counter - period_ticks) : (period_ticks - period_counter);
    wire [31:0] jump_candidate_delta =
        (period_counter > jump_candidate_period)
            ? (period_counter - jump_candidate_period)
            : (jump_candidate_period - period_counter);
    wire period_matches_filter =
        period_delta <= ((period_ticks >> 4) + 8);
    wire period_matches_jump_candidate = jump_candidate_valid &&
        (jump_candidate_delta <= ((jump_candidate_period >> 4) + 8));

    always @(posedge clk) begin
        edge_pulse <= 1'b0;

        if (!reset_n) begin
            period_counter <= 32'd0;
            period_filter_q8 <= 40'd0;
            valid_edges <= 3'd0;
            jump_candidate_period <= 32'd0;
            jump_candidate_valid <= 1'b0;
            seen_edge <= 1'b0;
            low_armed <= 1'b0;
            locked <= 1'b0;
            period_ticks <= 32'd0;
            edge_count <= 32'd0;
            window_count <= 16'd0;
            window_min <= 8'hFF;
            window_max <= 8'h00;
            sample_min <= 8'h80;
            sample_max <= 8'h80;
            otr_seen <= 1'b0;
            calibration_start_last <= 1'b0;
            calibration_wait_first_edge <= 1'b0;
            calibration_running <= 1'b0;
            calibration_counter <= 32'd0;
            calibration_period_count <= 32'd0;
            calibration_done <= 1'b0;
            calibration_ticks <= 32'd0;
        end else begin
            calibration_start_last <= calibration_start;
            if (calibration_start && !calibration_start_last) begin
                calibration_wait_first_edge <= 1'b1;
                calibration_running <= 1'b0;
                calibration_counter <= 32'd0;
                calibration_period_count <= 32'd0;
                calibration_done <= 1'b0;
                calibration_ticks <= 32'd0;
            end else if (calibration_running) begin
                calibration_counter <= calibration_counter + 1'b1;
            end

            if (period_counter < EDGE_TIMEOUT_TICKS)
                period_counter <= period_counter + 1'b1;
            else begin
                locked <= 1'b0;
                valid_edges <= 3'd0;
                jump_candidate_valid <= 1'b0;
            end

            if (ad_otr)
                otr_seen <= 1'b1;

            if (sample_ce) begin
                if (ad_data < window_min)
                    window_min <= ad_data;
                if (ad_data > window_max)
                    window_max <= ad_data;

                if (window_count == 16'hFFFF) begin
                    sample_min <= (ad_data < window_min) ? ad_data : window_min;
                    sample_max <= (ad_data > window_max) ? ad_data : window_max;
                    window_count <= 16'd0;
                    window_min <= 8'hFF;
                    window_max <= 8'h00;
                end else begin
                    window_count <= window_count + 1'b1;
                end

                if (ad_data < threshold_low)
                    low_armed <= 1'b1;

                if (low_armed && (ad_data > threshold_high)) begin
                    low_armed <= 1'b0;
                    edge_pulse <= 1'b1;
                    edge_count <= edge_count + 1'b1;

                    if (calibration_wait_first_edge) begin
                        calibration_wait_first_edge <= 1'b0;
                        calibration_running <= 1'b1;
                        calibration_counter <= 32'd0;
                        calibration_period_count <= 32'd0;
                    end else if (calibration_running) begin
                        if (calibration_period_count >=
                            CALIBRATION_PERIODS - 1'b1) begin
                            calibration_running <= 1'b0;
                            calibration_done <= 1'b1;
                            // calibration_counter is read before its
                            // nonblocking increment on this terminal edge.
                            // Include that clock so the reported interval is
                            // the exact first-edge to last-edge tick count.
                            calibration_ticks <= calibration_counter + 1'b1;
                        end else begin
                            calibration_period_count <=
                                calibration_period_count + 1'b1;
                        end
                    end

                    if (!seen_edge) begin
                        seen_edge <= 1'b1;
                        valid_edges <= 3'd0;
                        jump_candidate_valid <= 1'b0;
                    end else if ((period_counter >= MIN_PERIOD_TICKS) &&
                                 (period_counter <= MAX_PERIOD_TICKS)) begin
                        if (period_ticks == 0) begin
                            period_ticks <= period_counter;
                            period_filter_q8 <= measured_period_q8;
                            valid_edges <= 3'd1;
                            jump_candidate_valid <= 1'b0;
                        end else if (period_matches_filter) begin
                            period_filter_q8 <= filter_next;
                            period_ticks <= filtered_period_next;
                            jump_candidate_valid <= 1'b0;

                            if (valid_edges < 3'd7)
                                valid_edges <= valid_edges + 1'b1;
                            if (valid_edges >= 3'd3)
                                locked <= 1'b1;
                        end else if (period_matches_jump_candidate) begin
                            // Two consecutive periods agree with each other
                            // but not the old filter: treat this as a genuine
                            // frequency step and acquire the new value quickly.
                            period_filter_q8 <= measured_period_q8;
                            period_ticks <= period_counter;
                            valid_edges <= 3'd2;
                            locked <= 1'b0;
                            jump_candidate_valid <= 1'b0;
                        end else begin
                            jump_candidate_period <= period_counter;
                            jump_candidate_valid <= 1'b1;
                            valid_edges <= 3'd0;
                            locked <= 1'b0;
                        end
                    end else begin
                        valid_edges <= 3'd0;
                        locked <= 1'b0;
                        jump_candidate_valid <= 1'b0;
                    end

                    period_counter <= 32'd0;
                end
            end
        end
    end
endmodule
