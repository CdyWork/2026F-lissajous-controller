`timescale 1ns / 1ps

module f2026_spi_control (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        spi_cs_n,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    output reg         irq = 1'b0,

    input  wire        input_locked,
    input  wire        output_active,
    input  wire        otr_seen,
    input  wire [31:0] period_ticks,
    input  wire [31:0] edge_count,
    input  wire [7:0]  sample_min,
    input  wire [7:0]  sample_max,
    input  wire        calibration_done,
    input  wire [31:0] calibration_ticks,

    output reg  [2:0]  mode = 3'd0,
    output reg  [7:0]  amplitude = 8'd51,
    output reg         output_enable = 1'b0,
    output reg         free_run = 1'b0,
    output reg         calibration_start = 1'b0,
    output reg  [39:0] phase_increment = 40'd0,
    output reg  [31:0] phase_offset = 32'd0,
    output reg  [7:0]  dac_mid = 8'h80,
    output reg  [7:0]  threshold_hysteresis = 8'd3
);
    localparam [7:0] CMD_READ_STATUS = 8'h01;
    localparam [7:0] CMD_SET_CONTROL = 8'h10;

    wire frame_start;
    wire frame_end;
    wire rx_byte_valid;
    wire [7:0] rx_byte;
    wire [11:0] byte_index;
    reg [7:0] tx_byte;
    reg [7:0] command = 8'h00;
    reg [4:0] received_payload = 5'd0;

    reg [2:0] staged_mode = 3'd0;
    reg [7:0] staged_amplitude = 8'd51;
    reg [7:0] staged_flags = 8'd0;
    reg [39:0] staged_phase_increment = 40'd0;
    reg [31:0] staged_phase_offset = 32'd0;
    reg [7:0] staged_dac_mid = 8'h80;
    reg [7:0] staged_hysteresis = 8'd3;
    reg previous_locked = 1'b0;
    reg previous_otr = 1'b0;

    spi_mode0_slave spi_phy (
        .clk(clk),
        .reset_n(reset_n),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .tx_byte(tx_byte),
        .frame_start(frame_start),
        .frame_end(frame_end),
        .rx_byte_valid(rx_byte_valid),
        .rx_byte(rx_byte),
        .byte_index(byte_index)
    );

    always @* begin
        tx_byte = 8'h00;
        if (command == CMD_READ_STATUS) begin
            case (byte_index)
                12'd0:  tx_byte = 8'hF6;
                12'd1:  tx_byte = 8'h04;
                12'd2:  tx_byte = {3'd0, calibration_done, free_run,
                                    output_active, otr_seen, input_locked};
                12'd3:  tx_byte = period_ticks[7:0];
                12'd4:  tx_byte = period_ticks[15:8];
                12'd5:  tx_byte = period_ticks[23:16];
                12'd6:  tx_byte = period_ticks[31:24];
                12'd7:  tx_byte = calibration_done ? calibration_ticks[7:0] : edge_count[7:0];
                12'd8:  tx_byte = calibration_done ? calibration_ticks[15:8] : edge_count[15:8];
                12'd9:  tx_byte = calibration_done ? calibration_ticks[23:16] : edge_count[23:16];
                12'd10: tx_byte = calibration_done ? calibration_ticks[31:24] : edge_count[31:24];
                12'd11: tx_byte = sample_min;
                12'd12: tx_byte = sample_max;
                12'd13: tx_byte = {5'd0, mode};
                12'd14: tx_byte = amplitude;
                default: tx_byte = 8'h00;
            endcase
        end
    end

    always @(posedge clk) begin
        if (!reset_n) begin
            command <= 8'h00;
            received_payload <= 5'd0;
            mode <= 3'd0;
            amplitude <= 8'd51;
            output_enable <= 1'b0;
            free_run <= 1'b0;
            calibration_start <= 1'b0;
            phase_increment <= 40'd0;
            phase_offset <= 32'd0;
            dac_mid <= 8'h80;
            threshold_hysteresis <= 8'd3;
            previous_locked <= 1'b0;
            previous_otr <= 1'b0;
            irq <= 1'b0;
        end else begin
            previous_locked <= input_locked;
            previous_otr <= otr_seen;
            if ((input_locked != previous_locked) || (otr_seen && !previous_otr))
                irq <= 1'b1;

            if (frame_start) begin
                command <= 8'h00;
                received_payload <= 5'd0;
                staged_mode <= mode;
                staged_amplitude <= amplitude;
                staged_flags <= {6'd0, free_run, output_enable};
                staged_phase_increment <= phase_increment;
                staged_phase_offset <= phase_offset;
                staged_dac_mid <= dac_mid;
                staged_hysteresis <= threshold_hysteresis;
            end

            if (rx_byte_valid) begin
                if (byte_index == 0) begin
                    command <= rx_byte;
                end else if (command == CMD_SET_CONTROL) begin
                    received_payload <= received_payload + 1'b1;
                    case (byte_index)
                        12'd1: staged_mode <= rx_byte[2:0];
                        12'd2: staged_amplitude <= rx_byte;
                        12'd3: staged_flags <= rx_byte;
                        12'd4: staged_phase_increment[7:0] <= rx_byte;
                        12'd5: staged_phase_increment[15:8] <= rx_byte;
                        12'd6: staged_phase_increment[23:16] <= rx_byte;
                        12'd7: staged_phase_increment[31:24] <= rx_byte;
                        12'd8: staged_phase_offset[7:0] <= rx_byte;
                        12'd9: staged_phase_offset[15:8] <= rx_byte;
                        12'd10: staged_phase_offset[23:16] <= rx_byte;
                        12'd11: staged_phase_offset[31:24] <= rx_byte;
                        12'd12: staged_dac_mid <= rx_byte;
                        12'd13: staged_hysteresis <= rx_byte;
                        12'd14: staged_phase_increment[39:32] <= rx_byte;
                        default: begin end
                    endcase
                end
            end

            if (frame_end) begin
                if ((command == CMD_SET_CONTROL) && (received_payload == 5'd15)) begin
                    mode <= staged_mode;
                    amplitude <= staged_amplitude;
                    output_enable <= staged_flags[0];
                    free_run <= staged_flags[1];
                    calibration_start <= staged_flags[2];
                    phase_increment <= staged_phase_increment;
                    dac_mid <= staged_dac_mid;
                    threshold_hysteresis <= staged_hysteresis;
                    phase_offset <= staged_phase_offset;
                end
                if (command == CMD_READ_STATUS)
                    irq <= 1'b0;
            end
        end
    end
endmodule
