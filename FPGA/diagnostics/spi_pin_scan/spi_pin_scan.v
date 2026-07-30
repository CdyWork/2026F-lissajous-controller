`timescale 1ns / 1ps

module spi_pin_scan (
    input  wire clk_50m,
    input  wire pin5,
    input  wire pin6,
    input  wire pin7,
    input  wire pin8,
    input  wire pin9,
    input  wire pin10,
    output wire uart_tx
);
    localparam integer CLOCK_HZ = 50_000_000;

    wire [5:0] pins = {pin10, pin9, pin8, pin7, pin6, pin5};
    reg [5:0] pin_meta = 6'd0;
    reg [5:0] pin_sync = 6'd0;
    reg [5:0] pin_previous = 6'd0;
    reg [31:0] edge_count [0:5];
    reg [31:0] previous_count [0:5];
    reg [15:0] report_count [0:5];
    reg [5:0] report_levels = 6'd0;
    reg [25:0] report_timer = 26'd0;
    reg report_toggle = 1'b0;
    integer i;

    initial begin
        for (i = 0; i < 6; i = i + 1) begin
            edge_count[i] = 32'd0;
            previous_count[i] = 32'd0;
            report_count[i] = 16'd0;
        end
    end

    always @(posedge clk_50m) begin
        pin_meta <= pins;
        pin_sync <= pin_meta;
        pin_previous <= pin_sync;

        for (i = 0; i < 6; i = i + 1) begin
            if (pin_sync[i] && !pin_previous[i])
                edge_count[i] <= edge_count[i] + 1'b1;
        end

        if (report_timer == CLOCK_HZ - 1) begin
            report_timer <= 26'd0;
            report_levels <= pin_sync;
            report_toggle <= ~report_toggle;
            for (i = 0; i < 6; i = i + 1) begin
                report_count[i] <= edge_count[i][15:0] - previous_count[i][15:0];
                previous_count[i] <= edge_count[i];
            end
        end else begin
            report_timer <= report_timer + 1'b1;
        end
    end

    reg [7:0] uart_data = 8'h00;
    reg uart_start = 1'b0;
    wire uart_busy;
    wire uart_done;
    reg message_active = 1'b0;
    reg [6:0] message_index = 7'd0;
    reg report_seen = 1'b0;

    uart_tx_8n1 #(
        .CLOCK_HZ(CLOCK_HZ),
        .BAUD_RATE(115200)
    ) uart_inst (
        .clk(clk_50m),
        .start(uart_start),
        .data(uart_data),
        .tx(uart_tx),
        .busy(uart_busy),
        .done(uart_done)
    );

    function [7:0] hex_ascii;
        input [3:0] value;
        begin
            hex_ascii = (value < 10) ? (8'h30 + value) : (8'h41 + value - 10);
        end
    endfunction

    function [7:0] report_byte;
        input [6:0] index;
        reg [6:0] offset;
        reg [6:0] group;
        reg [6:0] position;
        reg [15:0] value;
        begin
            if (index == 0)
                report_byte = 8'h0D;
            else if (index == 1)
                report_byte = 8'h0A;
            else if (index == 2)
                report_byte = "J";
            else if (index == 3)
                report_byte = " ";
            else if (index == 58)
                report_byte = 8'h0D;
            else if (index == 59)
                report_byte = 8'h0A;
            else begin
                offset = index - 7'd4;
                group = offset / 7'd9;
                position = offset % 7'd9;
                value = report_count[group];
                case (position)
                    4'd0: report_byte = (group == 5) ? "A" : ("5" + group);
                    4'd1: report_byte = "=";
                    4'd2: report_byte = hex_ascii(value[15:12]);
                    4'd3: report_byte = hex_ascii(value[11:8]);
                    4'd4: report_byte = hex_ascii(value[7:4]);
                    4'd5: report_byte = hex_ascii(value[3:0]);
                    4'd6: report_byte = "/";
                    4'd7: report_byte = report_levels[group] ? "1" : "0";
                    default: report_byte = " ";
                endcase
            end
        end
    endfunction

    always @(posedge clk_50m) begin
        uart_start <= 1'b0;
        if (!message_active && (report_toggle != report_seen)) begin
            report_seen <= report_toggle;
            message_active <= 1'b1;
            message_index <= 7'd0;
            uart_data <= report_byte(7'd0);
            uart_start <= 1'b1;
        end else if (message_active && uart_done) begin
            if (message_index == 7'd59) begin
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
    output reg        busy = 1'b0,
    output reg        done = 1'b0
);
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
                bit_index <= 4'd0;
            end else begin
                bit_index <= bit_index + 1'b1;
                tx <= frame[bit_index + 1'b1];
            end
        end else begin
            baud_counter <= baud_counter + 1'b1;
        end
    end
endmodule
