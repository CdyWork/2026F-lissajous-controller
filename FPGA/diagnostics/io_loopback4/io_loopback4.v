`timescale 1ns / 1ps

module io_loopback4 (
    input  wire       clk_50m,
    input  wire [3:0] ad_data,
    output reg  [3:0] da_data = 4'b0000,
    output wire       uart_tx
);
    localparam integer SETTLE_CYCLES = 5_000_000;

    reg [22:0] settle_counter = 23'd0;
    reg [1:0] bit_index = 2'd0;
    reg test_high = 1'b0;
    reg [3:0] low_ok = 4'b0000;
    reg [3:0] high_ok = 4'b0000;
    reg [3:0] report_low = 4'b0000;
    reg [3:0] report_high = 4'b0000;
    reg [3:0] report_pass = 4'b0000;
    reg report_toggle = 1'b0;

    wire [3:0] bit_mask = 4'b0001 << bit_index;
    wire [3:0] high_with_current =
        (high_ok & ~bit_mask) | (ad_data[bit_index] ? bit_mask : 4'b0000);

    always @(posedge clk_50m) begin
        if (settle_counter == SETTLE_CYCLES - 1) begin
            settle_counter <= 23'd0;
            if (!test_high) begin
                if (!ad_data[bit_index])
                    low_ok <= low_ok | bit_mask;
                else
                    low_ok <= low_ok & ~bit_mask;
                da_data <= bit_mask;
                test_high <= 1'b1;
            end else begin
                high_ok <= high_with_current;
                da_data <= 4'b0000;
                test_high <= 1'b0;
                if (bit_index == 2'd3) begin
                    report_low <= low_ok;
                    report_high <= high_with_current;
                    report_pass <= low_ok & high_with_current;
                    report_toggle <= ~report_toggle;
                    low_ok <= 4'b0000;
                    high_ok <= 4'b0000;
                    bit_index <= 2'd0;
                end else begin
                    bit_index <= bit_index + 1'b1;
                end
            end
        end else begin
            settle_counter <= settle_counter + 1'b1;
        end
    end

    reg [7:0] uart_data = 8'h00;
    reg uart_start = 1'b0;
    wire uart_done;
    reg message_active = 1'b0;
    reg [5:0] message_index = 6'd0;
    reg report_seen = 1'b0;

    uart_tx_8n1 uart_inst (
        .clk(clk_50m),
        .start(uart_start),
        .data(uart_data),
        .tx(uart_tx),
        .done(uart_done)
    );

    function [7:0] hex_ascii;
        input [3:0] value;
        begin
            hex_ascii = (value < 10) ? (8'h30 + value) : (8'h41 + value - 10);
        end
    endfunction

    function [7:0] report_byte;
        input [5:0] index;
        begin
            case (index)
                6'd0:  report_byte = 8'h0D;
                6'd1:  report_byte = 8'h0A;
                6'd2:  report_byte = "I";
                6'd3:  report_byte = "O";
                6'd4:  report_byte = "4";
                6'd5:  report_byte = " ";
                6'd6:  report_byte = "P";
                6'd7:  report_byte = "A";
                6'd8:  report_byte = "S";
                6'd9:  report_byte = "S";
                6'd10: report_byte = "=";
                6'd11: report_byte = hex_ascii(report_pass);
                6'd12: report_byte = " ";
                6'd13: report_byte = "L";
                6'd14: report_byte = "O";
                6'd15: report_byte = "W";
                6'd16: report_byte = "=";
                6'd17: report_byte = hex_ascii(report_low);
                6'd18: report_byte = " ";
                6'd19: report_byte = "H";
                6'd20: report_byte = "I";
                6'd21: report_byte = "G";
                6'd22: report_byte = "H";
                6'd23: report_byte = "=";
                6'd24: report_byte = hex_ascii(report_high);
                6'd25: report_byte = 8'h0D;
                default: report_byte = 8'h0A;
            endcase
        end
    endfunction

    always @(posedge clk_50m) begin
        uart_start <= 1'b0;
        if (!message_active && (report_toggle != report_seen)) begin
            report_seen <= report_toggle;
            message_active <= 1'b1;
            message_index <= 6'd0;
            uart_data <= report_byte(6'd0);
            uart_start <= 1'b1;
        end else if (message_active && uart_done) begin
            if (message_index == 6'd26) begin
                message_active <= 1'b0;
            end else begin
                message_index <= message_index + 1'b1;
                uart_data <= report_byte(message_index + 1'b1);
                uart_start <= 1'b1;
            end
        end
    end
endmodule

module uart_tx_8n1 #(
    parameter integer CLOCK_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115200,
    parameter integer CLKS_PER_BIT = CLOCK_HZ / BAUD_RATE
) (
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx = 1'b1,
    output reg        done = 1'b0
);
    reg busy = 1'b0;
    reg [15:0] baud_counter = 16'd0;
    reg [3:0] bit_index = 4'd0;
    reg [9:0] frame = 10'h3FF;

    always @(posedge clk) begin
        done <= 1'b0;
        if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                frame <= {1'b1, data, 1'b0};
                tx <= 1'b0;
                busy <= 1'b1;
                baud_counter <= 16'd0;
                bit_index <= 4'd0;
            end
        end else if (baud_counter == CLKS_PER_BIT - 1) begin
            baud_counter <= 16'd0;
            if (bit_index == 4'd9) begin
                tx <= 1'b1;
                busy <= 1'b0;
                done <= 1'b1;
            end else begin
                bit_index <= bit_index + 1'b1;
                tx <= frame[bit_index + 1'b1];
            end
        end else begin
            baud_counter <= baud_counter + 1'b1;
        end
    end
endmodule
