`timescale 1ns / 1ps

module f2026_waveform_core #(
    // 200 cyclic 2 ms frames = 400 ms for each visual sweep setting.
    parameter [7:0] PROBE_SWEEP_FRAMES_PER_STEP = 8'd200
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        enable,
    input  wire        free_run,
    input  wire [2:0]  mode,
    input  wire [7:0]  amplitude,
    input  wire [7:0]  dac_mid,
    input  wire [31:0] phase_increment,
    input  wire [31:0] phase_offset,
    input  wire        input_edge,
    input  wire        input_locked,
    output reg  [7:0]  dac_data = 8'h85,
    output reg  [31:0] phase_monitor = 32'd0
);
    localparam [2:0] MODE_IDLE     = 3'd0;
    localparam [2:0] MODE_DIAGONAL = 3'd1;
    localparam [2:0] MODE_CIRCLE   = 3'd2;
    localparam [2:0] MODE_DOUBLE   = 3'd3;
    localparam [2:0] MODE_PROBE    = 3'd4;
    localparam [2:0] MODE_PROBE_SWEEP = 3'd5;
    localparam [2:0] MODE_PROBE_TABLE = 3'd6;
    localparam [31:0] PROBE_SWEEP_FRAME_TICKS = 32'd100000;
    // The ADDA DAC transfer characteristic is inverted on this bench.
    // The DAC transfer is inverted: 8'hFF is the physical negative endpoint
    // and 8'h00 is the positive endpoint. Park at the negative endpoint.
    localparam [7:0] PROBE_START_CODE = 8'hFF;
    localparam [7:0] PROBE_END_CODE = 8'h00;
    localparam [7:0] PROBE_PARK_CODE = 8'hFF;

    reg [31:0] base_phase = 32'd0;
    reg signed [11:0] sine_pipeline = 12'sd0;
    reg signed [20:0] product_pipeline = 21'sd0;
    reg [31:0] probe_frame_counter = 32'd0;
    reg [32:0] probe_ramp_error = 33'd0;
    reg probe_active = 1'b0;
    reg [2:0] probe_sweep_index = 3'd0;
    reg [7:0] probe_sweep_frame_count = 8'd0;
    reg probe_sweep_running = 1'b0;
    reg probe_table_running = 1'b0;
    reg [5:0] probe_table_index = 6'd16;

    function [31:0] probe_sweep_ramp_ticks;
        input [2:0] index;
        begin
            case (index)
                3'd0: probe_sweep_ramp_ticks = 32'd500;   // 10 us
                3'd1: probe_sweep_ramp_ticks = 32'd1500;  // 30 us
                3'd2: probe_sweep_ramp_ticks = 32'd3500;  // 70 us
                3'd3: probe_sweep_ramp_ticks = 32'd7500;  // 150 us
                3'd4: probe_sweep_ramp_ticks = 32'd15000; // 300 us
                3'd5: probe_sweep_ramp_ticks = 32'd25000; // 500 us
                3'd6: probe_sweep_ramp_ticks = 32'd37500; // 750 us
                default: probe_sweep_ramp_ticks = 32'd50000; // 1000 us
            endcase
        end
    endfunction

    // Log-spaced 10 us..1000 us table plus two low-frequency fallbacks.
    // Index 32 uses a 2 ms ramp and index 33 a 6 ms ramp; both use 10 ms
    // frames. All other entries use 2 ms frames.
    function [31:0] probe_table_ramp_ticks;
        input [5:0] index;
        begin
            case (index)
                6'd0: probe_table_ramp_ticks = 32'd500;
                6'd1: probe_table_ramp_ticks = 32'd600;
                6'd2: probe_table_ramp_ticks = 32'd650;
                6'd3: probe_table_ramp_ticks = 32'd800;
                6'd4: probe_table_ramp_ticks = 32'd900;
                6'd5: probe_table_ramp_ticks = 32'd1050;
                6'd6: probe_table_ramp_ticks = 32'd1200;
                6'd7: probe_table_ramp_ticks = 32'd1400;
                6'd8: probe_table_ramp_ticks = 32'd1650;
                6'd9: probe_table_ramp_ticks = 32'd1900;
                6'd10: probe_table_ramp_ticks = 32'd2200;
                6'd11: probe_table_ramp_ticks = 32'd2550;
                6'd12: probe_table_ramp_ticks = 32'd2950;
                6'd13: probe_table_ramp_ticks = 32'd3450;
                6'd14: probe_table_ramp_ticks = 32'd4000;
                6'd15: probe_table_ramp_ticks = 32'd4650;
                6'd16: probe_table_ramp_ticks = 32'd5400;
                6'd17: probe_table_ramp_ticks = 32'd6250;
                6'd18: probe_table_ramp_ticks = 32'd7250;
                6'd19: probe_table_ramp_ticks = 32'd8400;
                6'd20: probe_table_ramp_ticks = 32'd9750;
                6'd21: probe_table_ramp_ticks = 32'd11300;
                6'd22: probe_table_ramp_ticks = 32'd13150;
                6'd23: probe_table_ramp_ticks = 32'd15250;
                6'd24: probe_table_ramp_ticks = 32'd17650;
                6'd25: probe_table_ramp_ticks = 32'd20500;
                6'd26: probe_table_ramp_ticks = 32'd23800;
                6'd27: probe_table_ramp_ticks = 32'd27600;
                6'd28: probe_table_ramp_ticks = 32'd32000;
                6'd29: probe_table_ramp_ticks = 32'd37150;
                6'd30: probe_table_ramp_ticks = 32'd43100;
                6'd31: probe_table_ramp_ticks = 32'd50000;
                6'd32: probe_table_ramp_ticks = 32'd100000; // 2 ms
                default: probe_table_ramp_ticks = 32'd300000; // 6 ms
            endcase
        end
    endfunction

    function [31:0] probe_table_frame_ticks;
        input [5:0] index;
        begin
            probe_table_frame_ticks = ((index == 6'd32) || (index == 6'd33)) ?
                32'd500000 : PROBE_SWEEP_FRAME_TICKS;
        end
    endfunction

    function signed [11:0] sine_from_phase;
        input [31:0] phase;
        reg [3:0] index;
        reg [11:0] magnitude;
        begin
            index = phase[29:26];
            case (index)
                4'd0:  magnitude = 12'd0;
                4'd1:  magnitude = 12'd201;
                4'd2:  magnitude = 12'd399;
                4'd3:  magnitude = 12'd594;
                4'd4:  magnitude = 12'd783;
                4'd5:  magnitude = 12'd963;
                4'd6:  magnitude = 12'd1137;
                4'd7:  magnitude = 12'd1299;
                4'd8:  magnitude = 12'd1448;
                4'd9:  magnitude = 12'd1582;
                4'd10: magnitude = 12'd1702;
                4'd11: magnitude = 12'd1805;
                4'd12: magnitude = 12'd1891;
                4'd13: magnitude = 12'd1959;
                4'd14: magnitude = 12'd2008;
                default: magnitude = 12'd2037;
            endcase

            case (phase[31:30])
                2'b00: sine_from_phase = $signed(magnitude);
                2'b01: sine_from_phase = $signed(
                    (index == 0) ? 12'd2047 : sine_from_phase_quarter(5'd16 - {1'b0, index}));
                2'b10: sine_from_phase = -$signed(magnitude);
                default: sine_from_phase = -$signed(
                    (index == 0) ? 12'd2047 : sine_from_phase_quarter(5'd16 - {1'b0, index}));
            endcase
        end
    endfunction

    function [11:0] sine_from_phase_quarter;
        input [4:0] index;
        begin
            case (index)
                5'd1:  sine_from_phase_quarter = 12'd201;
                5'd2:  sine_from_phase_quarter = 12'd399;
                5'd3:  sine_from_phase_quarter = 12'd594;
                5'd4:  sine_from_phase_quarter = 12'd783;
                5'd5:  sine_from_phase_quarter = 12'd963;
                5'd6:  sine_from_phase_quarter = 12'd1137;
                5'd7:  sine_from_phase_quarter = 12'd1299;
                5'd8:  sine_from_phase_quarter = 12'd1448;
                5'd9:  sine_from_phase_quarter = 12'd1582;
                5'd10: sine_from_phase_quarter = 12'd1702;
                5'd11: sine_from_phase_quarter = 12'd1805;
                5'd12: sine_from_phase_quarter = 12'd1891;
                5'd13: sine_from_phase_quarter = 12'd1959;
                5'd14: sine_from_phase_quarter = 12'd2008;
                5'd15: sine_from_phase_quarter = 12'd2037;
                5'd16: sine_from_phase_quarter = 12'd2047;
                default: sine_from_phase_quarter = 12'd0;
            endcase
        end
    endfunction

    wire [31:0] running_phase = base_phase + phase_increment;
    wire tracking_edge = input_edge && input_locked && !free_run;
    wire [31:0] base_phase_next = tracking_edge ? 32'd0 : running_phase;
    reg [31:0] selected_phase;

    always @* begin
        case (mode)
            MODE_CIRCLE:
                selected_phase = base_phase_next + 32'h4000_0000 + phase_offset;
            MODE_DOUBLE:
                selected_phase = {base_phase_next[30:0], 1'b0} + phase_offset;
            MODE_DIAGONAL:
                selected_phase = base_phase_next + phase_offset;
            default:
                selected_phase = phase_offset;
        endcase
    end

    wire signed [12:0] scaled_sample = product_pipeline >>> 11;
    wire signed [13:0] output_code =
        $signed({1'b0, dac_mid}) - scaled_sample;
    wire probe_sweep_mode = mode == MODE_PROBE_SWEEP;
    wire probe_table_mode = mode == MODE_PROBE_TABLE;
    wire probe_mode = (mode == MODE_PROBE) || probe_sweep_mode || probe_table_mode;
    // In manual probe mode phase_offset is the complete frame duration.
    wire [31:0] probe_frame_ticks = probe_sweep_mode
        ? PROBE_SWEEP_FRAME_TICKS :
        (probe_table_mode ? probe_table_frame_ticks(probe_table_index) : phase_offset);
    wire [31:0] probe_ramp_ticks = probe_sweep_mode
        ? probe_sweep_ramp_ticks(probe_sweep_index) :
        (probe_table_mode ? probe_table_ramp_ticks(probe_table_index) : phase_increment);
    wire probe_valid_ramp = (probe_ramp_ticks != 32'd0) &&
                            (probe_ramp_ticks < probe_frame_ticks);
    wire [8:0] probe_ramp_span =
        {1'b0, PROBE_START_CODE} - {1'b0, PROBE_END_CODE};
    wire [32:0] probe_next_error = probe_ramp_error + probe_ramp_span;

    always @(posedge clk) begin
        if (!reset_n) begin
            base_phase <= 32'd0;
            sine_pipeline <= 12'sd0;
            product_pipeline <= 21'sd0;
            phase_monitor <= 32'd0;
            dac_data <= PROBE_PARK_CODE;
            probe_frame_counter <= 32'd0;
            probe_ramp_error <= 33'd0;
            probe_active <= 1'b0;
            probe_sweep_index <= 3'd0;
            probe_sweep_frame_count <= 8'd0;
            probe_sweep_running <= 1'b0;
            probe_table_running <= 1'b0;
            probe_table_index <= 6'd16;
        end else begin
            if (!enable || (mode == MODE_IDLE)) begin
                base_phase <= 32'd0;
                sine_pipeline <= 12'sd0;
                product_pipeline <= 21'sd0;
                phase_monitor <= 32'd0;
                dac_data <= PROBE_PARK_CODE;
                probe_frame_counter <= 32'd0;
                probe_ramp_error <= 33'd0;
                probe_active <= 1'b0;
                probe_sweep_index <= 3'd0;
                probe_sweep_frame_count <= 8'd0;
                probe_sweep_running <= 1'b0;
                probe_table_running <= 1'b0;
                probe_table_index <= 6'd16;
            end else if (probe_mode) begin
                base_phase <= 32'd0;
                sine_pipeline <= 12'sd0;
                product_pipeline <= 21'sd0;
                phase_monitor <= probe_frame_counter;

                // Entering or leaving the autonomous table always begins a
                // fresh frame; otherwise the first sweep setting could start
                // midway through the preceding manual PROBE frame.
                if ((probe_sweep_mode != probe_sweep_running) ||
                    (probe_table_mode != probe_table_running)) begin
                    probe_frame_counter <= 32'd0;
                    probe_ramp_error <= 33'd0;
                    probe_active <= 1'b0;
                    probe_sweep_index <= 3'd0;
                    probe_sweep_frame_count <= 8'd0;
                    probe_sweep_running <= probe_sweep_mode;
                    probe_table_running <= probe_table_mode;
                    probe_table_index <= phase_increment[5:0];
                    dac_data <= PROBE_PARK_CODE;
                end else if (!probe_valid_ramp) begin
                    probe_frame_counter <= 32'd0;
                    probe_ramp_error <= 33'd0;
                    probe_active <= 1'b0;
                    dac_data <= PROBE_PARK_CODE;
                end else if (!probe_active) begin
                    probe_frame_counter <= 32'd0;
                    probe_ramp_error <= 33'd0;
                    probe_active <= 1'b1;
                    dac_data <= PROBE_START_CODE;
                end else if (probe_active) begin
                    if (probe_frame_counter >= probe_frame_ticks - 1'b1) begin
                        probe_frame_counter <= 32'd0;
                        probe_ramp_error <= 33'd0;
                        dac_data <= PROBE_START_CODE;
                        if (probe_sweep_mode) begin
                            if (probe_sweep_frame_count >=
                                PROBE_SWEEP_FRAMES_PER_STEP - 1'b1) begin
                                probe_sweep_frame_count <= 8'd0;
                                probe_sweep_index <= probe_sweep_index + 1'b1;
                            end else begin
                                probe_sweep_frame_count <=
                                    probe_sweep_frame_count + 1'b1;
                            end
                        end else begin
                            probe_sweep_index <= 3'd0;
                            probe_sweep_frame_count <= 8'd0;
                            if (probe_table_mode)
                                // The control register may update at any
                                // time; latch its index only at this frame
                                // boundary to preserve a complete sawtooth.
                                probe_table_index <= phase_increment[5:0];
                        end
                    end else begin
                        probe_frame_counter <= probe_frame_counter + 1'b1;
                        if (probe_frame_counter >= probe_ramp_ticks - 1'b1) begin
                            probe_ramp_error <= 33'd0;
                            dac_data <= PROBE_PARK_CODE;
                        end else if (probe_next_error >= {1'b0, probe_ramp_ticks}) begin
                            probe_ramp_error <=
                                probe_next_error - {1'b0, probe_ramp_ticks};
                            if (dac_data > PROBE_END_CODE)
                                dac_data <= dac_data - 1'b1;
                        end else begin
                            probe_ramp_error <= probe_next_error;
                        end
                    end
                end
            end else begin
                base_phase <= base_phase_next;
                phase_monitor <= selected_phase;
                sine_pipeline <= sine_from_phase(selected_phase);
                product_pipeline <= sine_pipeline * $signed({1'b0, amplitude});
                probe_frame_counter <= 32'd0;
                probe_ramp_error <= 33'd0;
                probe_active <= 1'b0;
                probe_sweep_index <= 3'd0;
                probe_sweep_frame_count <= 8'd0;
                probe_sweep_running <= 1'b0;
                probe_table_running <= 1'b0;
                probe_table_index <= 6'd16;
                if (output_code < 0)
                    dac_data <= 8'h00;
                else if (output_code > 14'sd255)
                    dac_data <= 8'hFF;
                else
                    dac_data <= output_code[7:0];
            end
        end
    end
endmodule
