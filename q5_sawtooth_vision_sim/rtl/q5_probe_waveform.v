`timescale 1ns / 1ps

module q5_probe_waveform #(
    parameter integer FRAME_TICKS = 500000
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        enable,
    input  wire [31:0] ramp_ticks,
    input  wire [7:0]  bottom_code,
    input  wire [7:0]  top_code,
    input  wire [7:0]  park_code,
    output reg  [7:0]  dac_data = 8'h80,
    output reg         visible_ramp = 1'b0,
    output reg         frame_start = 1'b0
);
    reg [31:0] frame_counter = 32'd0;
    reg [32:0] ramp_error = 33'd0;
    reg active = 1'b0;

    wire [8:0] ramp_span = {1'b0, top_code} - {1'b0, bottom_code};
    wire [32:0] next_error = ramp_error + ramp_span;
    wire valid_ramp = (ramp_ticks != 0) &&
                      (ramp_ticks < FRAME_TICKS) &&
                      (top_code > bottom_code);

    always @(posedge clk) begin
        frame_start <= 1'b0;

        if (!reset_n) begin
            frame_counter <= 32'd0;
            ramp_error <= 33'd0;
            active <= 1'b0;
            dac_data <= park_code;
            visible_ramp <= 1'b0;
        end else if (!enable || !valid_ramp) begin
            frame_counter <= 32'd0;
            ramp_error <= 33'd0;
            active <= 1'b0;
            dac_data <= park_code;
            visible_ramp <= 1'b0;
        end else if (!active) begin
            active <= 1'b1;
            frame_counter <= 32'd0;
            ramp_error <= 33'd0;
            dac_data <= bottom_code;
            visible_ramp <= 1'b1;
            frame_start <= 1'b1;
        end else begin
            if (frame_counter == FRAME_TICKS - 1) begin
                frame_counter <= 32'd0;
                ramp_error <= 33'd0;
                dac_data <= bottom_code;
                visible_ramp <= 1'b1;
                frame_start <= 1'b1;
            end else begin
                frame_counter <= frame_counter + 1'b1;

                if (frame_counter < ramp_ticks - 1) begin
                    visible_ramp <= 1'b1;
                    if (next_error >= ramp_ticks) begin
                        ramp_error <= next_error - ramp_ticks;
                        if (dac_data < top_code)
                            dac_data <= dac_data + 1'b1;
                    end else begin
                        ramp_error <= next_error;
                    end
                end else begin
                    visible_ramp <= 1'b0;
                    ramp_error <= 33'd0;
                    dac_data <= park_code;
                end
            end
        end
    end
endmodule
