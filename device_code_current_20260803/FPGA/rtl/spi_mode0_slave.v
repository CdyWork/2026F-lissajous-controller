`timescale 1ns / 1ps

module spi_mode0_slave (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        spi_cs_n,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output reg         spi_miso,
    input  wire [7:0]  tx_byte,
    output reg         frame_start,
    output reg         frame_end,
    output reg         rx_byte_valid,
    output reg [7:0]   rx_byte,
    output reg [11:0]  byte_index
);
    reg cs_meta = 1'b1;
    reg cs_sync = 1'b1;
    reg cs_previous = 1'b1;
    reg sck_meta = 1'b0;
    reg sck_sync = 1'b0;
    reg sck_previous = 1'b0;
    reg mosi_meta = 1'b0;
    reg mosi_sync = 1'b0;
    reg active = 1'b0;
    reg [7:0] rx_shift = 8'h00;
    reg [7:0] tx_shift = 8'hFF;
    reg [2:0] rx_bit_index = 3'd0;
    reg [2:0] tx_bit_index = 3'd0;
    reg [7:0] idle_count = 8'hFF;

    localparam [7:0] IDLE_END_TICKS = 8'd64;

    wire sck_rising = sck_sync && !sck_previous;
    wire sck_falling = !sck_sync && sck_previous;
    wire sck_changed = sck_sync != sck_previous;

    always @(posedge clk) begin
        cs_meta <= spi_cs_n;
        cs_sync <= cs_meta;
        cs_previous <= cs_sync;
        sck_meta <= spi_sck;
        sck_sync <= sck_meta;
        sck_previous <= sck_sync;
        mosi_meta <= spi_mosi;
        mosi_sync <= mosi_meta;
        frame_start <= 1'b0;
        frame_end <= 1'b0;
        rx_byte_valid <= 1'b0;

        if (!reset_n) begin
            active <= 1'b0;
            spi_miso <= 1'b0;
            rx_shift <= 8'h00;
            tx_shift <= 8'hFF;
            rx_bit_index <= 3'd0;
            tx_bit_index <= 3'd0;
            byte_index <= 12'd0;
            rx_byte <= 8'h00;
            idle_count <= 8'hFF;
        end else if (!cs_sync && cs_previous) begin
            active <= 1'b1;
            frame_start <= 1'b1;
            rx_shift <= 8'h00;
            rx_bit_index <= 3'd0;
            tx_bit_index <= 3'd0;
            byte_index <= 12'd0;
            tx_shift <= tx_byte;
            spi_miso <= tx_byte[7];
            idle_count <= 8'd0;
        end else if (cs_sync && !cs_previous) begin
            active <= 1'b0;
            frame_end <= 1'b1;
            spi_miso <= 1'b0;
            idle_count <= 8'hFF;
        end else if (!active && sck_rising) begin
            // CS-less fallback: a clock burst after an idle gap starts a frame.
            active <= 1'b1;
            frame_start <= 1'b1;
            rx_shift <= {7'd0, mosi_sync};
            rx_bit_index <= 3'd1;
            tx_bit_index <= 3'd0;
            byte_index <= 12'd0;
            tx_shift <= tx_byte;
            spi_miso <= tx_byte[7];
            idle_count <= 8'd0;
        end else if (active) begin
            if (sck_changed)
                idle_count <= 8'd0;
            else if (idle_count < IDLE_END_TICKS)
                idle_count <= idle_count + 1'b1;

            if (!sck_changed && (idle_count == IDLE_END_TICKS - 1'b1)) begin
                active <= 1'b0;
                frame_end <= 1'b1;
                spi_miso <= 1'b0;
            end

            if (sck_rising) begin
                rx_shift <= {rx_shift[6:0], mosi_sync};
                if (rx_bit_index == 3'd7) begin
                    rx_byte <= {rx_shift[6:0], mosi_sync};
                    rx_byte_valid <= 1'b1;
                    rx_bit_index <= 3'd0;
                end else begin
                    rx_bit_index <= rx_bit_index + 1'b1;
                end
            end

            if (sck_falling) begin
                if (tx_bit_index == 3'd7) begin
                    tx_bit_index <= 3'd0;
                    byte_index <= byte_index + 1'b1;
                    tx_shift <= tx_byte;
                    spi_miso <= tx_byte[7];
                end else begin
                    tx_bit_index <= tx_bit_index + 1'b1;
                    case (tx_bit_index)
                        3'd0: spi_miso <= tx_shift[6];
                        3'd1: spi_miso <= tx_shift[5];
                        3'd2: spi_miso <= tx_shift[4];
                        3'd3: spi_miso <= tx_shift[3];
                        3'd4: spi_miso <= tx_shift[2];
                        3'd5: spi_miso <= tx_shift[1];
                        default: spi_miso <= tx_shift[0];
                    endcase
                end
            end
        end
    end
endmodule
