`timescale 1ns / 1ps

module f2026_waveform_core (
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
    output reg  [7:0]  dac_data = 8'h80,
    output reg  [31:0] phase_monitor = 32'd0
);
    localparam [2:0] MODE_IDLE     = 3'd0;
    localparam [2:0] MODE_DIAGONAL = 3'd1;
    localparam [2:0] MODE_CIRCLE   = 3'd2;
    localparam [2:0] MODE_DOUBLE   = 3'd3;
    reg [31:0] base_phase = 32'd0;
    reg signed [11:0] sine_pipeline = 12'sd0;
    reg signed [20:0] product_pipeline = 21'sd0;

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

    always @(posedge clk) begin
        if (!reset_n) begin
            base_phase <= 32'd0;
            sine_pipeline <= 12'sd0;
            product_pipeline <= 21'sd0;
            phase_monitor <= 32'd0;
            dac_data <= 8'h80;
        end else begin
            if (!enable || (mode == MODE_IDLE)) begin
                base_phase <= 32'd0;
                sine_pipeline <= 12'sd0;
                product_pipeline <= 21'sd0;
                phase_monitor <= 32'd0;
                dac_data <= dac_mid;
            end else begin
                base_phase <= base_phase_next;
                phase_monitor <= selected_phase;
                sine_pipeline <= sine_from_phase(selected_phase);
                product_pipeline <= sine_pipeline * $signed({1'b0, amplitude});
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
