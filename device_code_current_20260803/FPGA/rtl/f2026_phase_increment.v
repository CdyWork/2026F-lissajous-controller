`timescale 1ns / 1ps

module f2026_phase_increment (
    input  wire        clk,
    input  wire        reset_n,
    input  wire [31:0] period_ticks,
    output reg  [31:0] phase_increment = 32'd0,
    output reg         valid = 1'b0
);
    reg [31:0] last_period = 32'd0;
    reg [31:0] divisor = 32'd1;
    reg [32:0] dividend = 33'd0;
    reg [32:0] remainder = 33'd0;
    reg [32:0] quotient = 33'd0;
    reg [5:0] bits_remaining = 6'd0;
    reg busy = 1'b0;

    wire [32:0] shifted_remainder = {remainder[31:0], dividend[32]};
    wire subtract_divisor = shifted_remainder >= {1'b0, divisor};
    wire [32:0] reduced_remainder = subtract_divisor
        ? (shifted_remainder - {1'b0, divisor})
        : shifted_remainder;
    wire [32:0] shifted_quotient = {quotient[31:0], subtract_divisor};

    always @(posedge clk) begin
        if (!reset_n) begin
            last_period <= 32'd0;
            divisor <= 32'd1;
            dividend <= 33'd0;
            remainder <= 33'd0;
            quotient <= 33'd0;
            bits_remaining <= 6'd0;
            busy <= 1'b0;
            phase_increment <= 32'd0;
            valid <= 1'b0;
        end else if (busy) begin
            dividend <= {dividend[31:0], 1'b0};
            remainder <= reduced_remainder;
            quotient <= shifted_quotient;

            if (bits_remaining == 6'd1) begin
                phase_increment <= shifted_quotient[31:0];
                valid <= 1'b1;
                busy <= 1'b0;
                bits_remaining <= 6'd0;
            end else begin
                bits_remaining <= bits_remaining - 1'b1;
            end
        end else if ((period_ticks != 32'd0) &&
                     (period_ticks != last_period)) begin
            last_period <= period_ticks;
            divisor <= period_ticks;
            // Rounded quotient: (2^32 + period/2) / period.
            dividend <= {1'b1, 32'd0} + {2'b00, period_ticks[31:1]};
            remainder <= 33'd0;
            quotient <= 33'd0;
            bits_remaining <= 6'd33;
            busy <= 1'b1;
        end
    end
endmodule
