`timescale 1ns / 1ps

module tb_f2026;
    reg clk_50m = 1'b0;
    reg fpga_reset_n = 1'b0;
    reg spi_cs_n = 1'b1;
    reg spi_sck = 1'b0;
    reg spi_mosi = 1'b0;
    wire spi_miso;
    wire hmi_irq;
    wire ad_clk;
    reg [7:0] ad_data = 8'h80;
    reg ad_otr = 1'b0;
    wire da_clk;
    wire [7:0] da_data;
    wire uart_tx;

    integer errors = 0;
    integer source_hz = 10000;
    reg source_enable = 1'b1;
    real source_phase = 0.0;
    real sample_value;
    reg [7:0] tx_frame [0:15];
    reg [7:0] rx_frame [0:15];
    integer index;

    always #10 clk_50m = ~clk_50m;

    always @(negedge ad_clk) begin
        if (fpga_reset_n && source_enable) begin
            source_phase = source_phase + (6.283185307179586 * source_hz / 25000000.0);
            if (source_phase >= 6.283185307179586)
                source_phase = source_phase - 6.283185307179586;
            sample_value = 128.0 + 50.0 * $sin(source_phase);
            ad_data <= $rtoi(sample_value);
        end else begin
            source_phase = 0.0;
            ad_data <= 8'h80;
        end
    end

    f2026_top dut (
        .clk_50m(clk_50m),
        .fpga_reset_n(fpga_reset_n),
        .hmi_spi_cs_n(spi_cs_n),
        .hmi_spi_sck(spi_sck),
        .hmi_spi_mosi(spi_mosi),
        .hmi_spi_miso(spi_miso),
        .hmi_irq(hmi_irq),
        .ad_clk(ad_clk),
        .ad_data(ad_data),
        .ad_otr(ad_otr),
        .da_clk(da_clk),
        .da_data(da_data),
        .uart_tx(uart_tx)
    );

    // Keep this regression short while preserving the production 400 ms
    // dwell in synthesis (the core's default parameter is 40 frames).
    defparam dut.waveform_inst.PROBE_SWEEP_FRAMES_PER_STEP = 8'd2;
    defparam dut.tracker_inst.CALIBRATION_PERIODS = 32'd16;

    task check;
        input condition;
        input [8*96-1:0] message;
        begin
            if (!condition) begin
                $display("FAIL: %0s", message);
                errors = errors + 1;
            end
        end
    endtask

    task spi_transfer_byte;
        input [7:0] transmit_byte;
        output [7:0] receive_byte;
        integer bit_number;
        begin
            receive_byte = 8'h00;
            for (bit_number = 7; bit_number >= 0; bit_number = bit_number - 1) begin
                spi_mosi = transmit_byte[bit_number];
                #100;
                spi_sck = 1'b1;
                #100;
                receive_byte[bit_number] = spi_miso;
                spi_sck = 1'b0;
            end
        end
    endtask

    task spi_frame;
        integer byte_number;
        begin
            spi_cs_n = 1'b0;
            #500;
            for (byte_number = 0; byte_number < 16; byte_number = byte_number + 1)
                spi_transfer_byte(tx_frame[byte_number], rx_frame[byte_number]);
            #200;
            spi_cs_n = 1'b1;
            #500;
        end
    endtask

    task spi_frame_csless;
        integer byte_number;
        begin
            spi_cs_n = 1'b1;
            #5000;
            for (byte_number = 0; byte_number < 16; byte_number = byte_number + 1)
                spi_transfer_byte(tx_frame[byte_number], rx_frame[byte_number]);
            #5000;
        end
    endtask

    task write_control;
        input [2:0] requested_mode;
        input [7:0] requested_amplitude;
        input [7:0] requested_flags;
        input [39:0] requested_increment;
        input [31:0] requested_offset;
        begin
            for (index = 0; index < 16; index = index + 1)
                tx_frame[index] = 8'h00;
            tx_frame[0] = 8'h10;
            tx_frame[1] = {5'd0, requested_mode};
            tx_frame[2] = requested_amplitude;
            tx_frame[3] = requested_flags;
            tx_frame[4] = requested_increment[7:0];
            tx_frame[5] = requested_increment[15:8];
            tx_frame[6] = requested_increment[23:16];
            tx_frame[7] = requested_increment[31:24];
            tx_frame[8] = requested_offset[7:0];
            tx_frame[9] = requested_offset[15:8];
            tx_frame[10] = requested_offset[23:16];
            tx_frame[11] = requested_offset[31:24];
            tx_frame[12] = 8'h80;
            tx_frame[13] = 8'd3;
            tx_frame[14] = requested_increment[39:32];
            spi_frame;
        end
    endtask

    task read_status;
        begin
            for (index = 0; index < 16; index = index + 1)
                tx_frame[index] = 8'h00;
            tx_frame[0] = 8'h01;
            spi_frame;
        end
    endtask

    task measure_span;
        input integer clock_count;
        output integer minimum_code;
        output integer maximum_code;
        integer count;
        begin
            minimum_code = 255;
            maximum_code = 0;
            for (count = 0; count < clock_count; count = count + 1) begin
                @(posedge clk_50m);
                if (da_data < minimum_code)
                    minimum_code = da_data;
                if (da_data > maximum_code)
                    maximum_code = da_data;
            end
        end
    endtask

    integer minimum_code;
    integer maximum_code;
    integer input_edges;
    integer output_edges;
    integer count;
    reg [7:0] previous_da;

    initial begin
        #200;
        fpga_reset_n = 1'b1;
        #650000;

        read_status;
        check(rx_frame[1] == 8'hF6, "SPI status signature");
        check(rx_frame[2] == 8'h04, "SPI protocol version");
        check(rx_frame[3][0] == 1'b1, "input tracker locked");
        check(({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} > 32'd4950) &&
              ({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} < 32'd5050),
              "10 kHz input period");

        for (index = 0; index < 16; index = index + 1)
            tx_frame[index] = 8'h00;
        tx_frame[0] = 8'h01;
        spi_frame_csless;
        check(rx_frame[1] == 8'hF6, "CS-less SPI status signature");

        // Tracking mode must ignore the MCU tuning word and derive it from ADC.
        write_control(3'd1, 8'd51, 8'h01, 32'd1, 32'd0);
        #100000;
        check((dut.tracked_phase_increment > 32'd858000) &&
              (dut.tracked_phase_increment < 32'd860000),
              "FPGA derives 10 kHz tuning word");
        read_status;
        check(rx_frame[3][2] == 1'b1, "tracked output becomes active");
        measure_span(6000, minimum_code, maximum_code);
        check((maximum_code - minimum_code) >= 98, "tracked diagonal amplitude span");
        check((maximum_code - minimum_code) <= 104, "tracked diagonal amplitude limit");
        input_edges = 0;
        output_edges = 0;
        previous_da = da_data;
        for (count = 0; count < 25000; count = count + 1) begin
            @(posedge clk_50m);
            if (dut.input_edge)
                input_edges = input_edges + 1;
            if ((previous_da >= 8'h80) && (da_data < 8'h80))
                output_edges = output_edges + 1;
            previous_da = da_data;
        end
        check((output_edges >= (input_edges - 1)) &&
              (output_edges <= (input_edges + 1)), "tracked diagonal frequency ratio");

        // A brief lock indication glitch must not force the DAC to midpoint;
        // that creates a visible horizontal line in oscilloscope XY mode.
        force dut.input_locked = 1'b0;
        #100000;
        check(dut.output_active == 1'b1, "short lock dropout uses DDS holdover");
        release dut.input_locked;
        #100000;
        check(dut.output_active == 1'b1, "holdover recovers without muting");

        // A frequency step must re-lock without another SPI control frame.
        source_hz = 20000;
        #3500000;
        read_status;
        check(rx_frame[3][0] == 1'b1, "frequency step autonomously re-locks");
        check(({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} > 32'd2450) &&
              ({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} < 32'd2550),
              "20 kHz stepped input period");
        input_edges = 0;
        output_edges = 0;
        previous_da = da_data;
        for (count = 0; count < 25000; count = count + 1) begin
            @(posedge clk_50m);
            if (dut.input_edge)
                input_edges = input_edges + 1;
            if ((previous_da >= 8'h80) && (da_data < 8'h80))
                output_edges = output_edges + 1;
            previous_da = da_data;
        end
        check((output_edges >= (input_edges - 1)) &&
              (output_edges <= (input_edges + 1)), "stepped diagonal frequency ratio");

        write_control(3'd2, 8'd51, 8'h01, 32'd1, 32'd0);
        #150000;
        measure_span(6000, minimum_code, maximum_code);
        check((maximum_code - minimum_code) >= 98, "circle amplitude span");
        check((maximum_code - minimum_code) <= 104, "circle amplitude limit");

        @(posedge dut.input_edge);
        #200;
        check(da_data < 8'd84, "circle is near positive Y peak at input rising zero");

        write_control(3'd3, 8'd51, 8'h01, 32'd1, 32'd0);
        #100000;
        input_edges = 0;
        output_edges = 0;
        previous_da = da_data;
        for (count = 0; count < 25000; count = count + 1) begin
            @(posedge clk_50m);
            if (dut.input_edge)
                input_edges = input_edges + 1;
            if ((previous_da >= 8'h80) && (da_data < 8'h80))
                output_edges = output_edges + 1;
            previous_da = da_data;
        end
        check((output_edges >= (input_edges * 2 - 1)) &&
              (output_edges <= (input_edges * 2 + 1)), "double-frequency ratio");

        source_enable = 1'b0;
        #9000000;
        read_status;
        check(rx_frame[3][0] == 1'b0, "missing input clears lock");
        check(rx_frame[3][2] == 1'b0, "missing input mutes tracked output");
        check(da_data == 8'hFF, "loss of lock parks DAC at the negative endpoint");
        source_enable = 1'b1;
        #600000;
        read_status;
        check(rx_frame[3][0] == 1'b1, "input return autonomously re-locks");
        check(rx_frame[3][2] == 1'b1, "input return autonomously unmutes");

        write_control(3'd2, 8'd13, 8'h01, 32'd1, 32'd0);
        #100000;
        measure_span(6000, minimum_code, maximum_code);
        check((maximum_code - minimum_code) >= 24, "1 Vpp calibrated-code span");
        check((maximum_code - minimum_code) <= 28, "1 Vpp span limit");

        write_control(3'd1, 8'd26, 8'h03, 40'd439804651, 32'd0);
        #100000;
        check(dut.phase_increment == 40'd439804651,
              "SPI transfers the complete 40-bit free-run tuning word");
        measure_span(12000, minimum_code, maximum_code);
        check((maximum_code - minimum_code) >= 48, "free-run Raspberry Pi path");

        // A Q5 PHASEQ command only changes phase_offset while free-run
        // frequency remains unchanged. Verify both the SPI register and the
        // physical DAC quadrants so a stale or ignored phase word cannot pass
        // this regression unnoticed. Increment 1 keeps the base phase nearly
        // stationary during the four SPI frames.
        write_control(3'd0, 8'd0, 8'h00, 32'd0, 32'd0);
        write_control(3'd1, 8'd51, 8'h03, 32'd1, 32'h0000_0000);
        #1000;
        check(dut.phase_offset == 32'h0000_0000,
              "free-run phase register accepts 0 degrees");
        check((da_data >= 8'd126) && (da_data <= 8'd130),
              "free-run 0 degree DAC is at midpoint");

        write_control(3'd0, 8'd0, 8'h00, 32'd0, 32'd0);
        write_control(3'd1, 8'd51, 8'h03, 32'd1, 32'h4000_0000);
        #1000;
        check(dut.phase_offset == 32'h4000_0000,
              "free-run phase register accepts 90 degrees");
        check((da_data >= 8'd75) && (da_data <= 8'd81),
              "free-run 90 degree DAC reaches positive peak");

        write_control(3'd0, 8'd0, 8'h00, 32'd0, 32'd0);
        write_control(3'd1, 8'd51, 8'h03, 32'd1, 32'h8000_0000);
        #1000;
        check(dut.phase_offset == 32'h8000_0000,
              "free-run phase register accepts 180 degrees");
        check((da_data >= 8'd126) && (da_data <= 8'd130),
              "free-run 180 degree DAC returns to midpoint");

        write_control(3'd0, 8'd0, 8'h00, 32'd0, 32'd0);
        write_control(3'd1, 8'd51, 8'h03, 32'd1, 32'hC000_0000);
        #1000;
        check(dut.phase_offset == 32'hC000_0000,
              "free-run phase register accepts 270 degrees");
        check((da_data >= 8'd175) && (da_data <= 8'd181),
              "free-run 270 degree DAC reaches negative peak");

        write_control(3'd0, 8'd0, 8'h00, 32'd0, 32'd0);
        #10000;
        check(da_data == 8'hFF, "idle DAC parks at the negative endpoint");

        fpga_reset_n = 1'b0;
        source_hz = 100000;
        source_phase = 0.0;
        #500;
        fpga_reset_n = 1'b1;
        #150000;
        read_status;
        check(rx_frame[3][0] == 1'b1, "100 kHz boundary locks");
        check(({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} >= 32'd495) &&
              ({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} <= 32'd505),
              "100 kHz boundary period");
        check(dut.low_frequency_phase_calibration == 32'd0,
              "high frequency bypasses low-frequency phase fit");

        // Shortened to 16 input periods by the testbench defparam. Production
        // uses 50,000 periods for a half-second reference calibration.
        write_control(3'd1, 8'd13, 8'h07, 40'd2199023256, 32'd0);
        #300000;
        read_status;
        check(rx_frame[3][4] == 1'b1, "reference calibration completes");
        check({rx_frame[11], rx_frame[10], rx_frame[9], rx_frame[8]} == 32'd8000,
              "reference calibration returns exact 16-period tick count");
        write_control(3'd1, 8'd13, 8'h03, 40'd2199023256, 32'd0);

        // A full-range frequency step must not wait for the old IIR state to
        // converge over dozens of slow input periods.
        source_hz = 1000;
        #8000000;
        read_status;
        check(rx_frame[3][0] == 1'b1, "100 kHz to 1 kHz fast re-lock");
        check(({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} >= 32'd49950) &&
              ({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} <= 32'd50050),
              "fast re-lock reaches 1 kHz period");
        check(($signed(dut.low_frequency_phase_calibration) < -32'sd27000000) &&
              ($signed(dut.low_frequency_phase_calibration) > -32'sd28000000),
              "1 kHz low-frequency phase fit word");

        ad_otr = 1'b1;
        #100;
        ad_otr = 1'b0;
        read_status;
        check(rx_frame[3][1] == 1'b1, "ADC OTR status is sticky");

        fpga_reset_n = 1'b0;
        source_hz = 1000;
        source_phase = 0.0;
        #500;
        fpga_reset_n = 1'b1;
        #6500000;
        read_status;
        check(rx_frame[3][0] == 1'b1, "1 kHz boundary locks");
        check(({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} >= 32'd49950) &&
              ({rx_frame[7], rx_frame[6], rx_frame[5], rx_frame[4]} <= 32'd50050),
              "1 kHz boundary period");
        check((rx_frame[12] < 8'd90) && (rx_frame[13] > 8'd166),
              "ADC statistics cover input span");

        // Q5 probe: a rising sawtooth repeats in a 10 ms frame.
        write_control(3'd4, 8'd0, 8'h03, 32'd2500, 32'd500000);
        measure_span(12000, minimum_code, maximum_code);
        check(minimum_code <= 1, "rising probe reaches the positive endpoint");
        check(maximum_code == 255, "rising probe begins at the negative endpoint");
        #9800000;
        check(dut.waveform_inst.probe_frame_counter < 32'd5000,
              "rising probe wraps on the 10 ms frame period");
        measure_span(12000, minimum_code, maximum_code);
        check(maximum_code == 255, "rising probe returns to the negative endpoint");

        // FPGA-owned Q5 sweep: each setting changes only after its full
        // cyclic frame, so a sawtooth cannot be cut in the middle.
        write_control(3'd5, 8'd0, 8'h03, 32'd500, 32'd500000);
        #1000;
        check(dut.waveform_inst.probe_sweep_index == 3'd0,
              "sweep starts at the 10 us setting");
        #20000000;
        check(dut.waveform_inst.probe_sweep_index == 3'd1,
              "sweep advances after the configured full-frame dwell");
        check(dut.waveform_inst.probe_frame_counter < 32'd1000,
              "sweep setting transition occurs on a frame boundary");
        #140000000;
        check(dut.waveform_inst.probe_sweep_index == 3'd0,
              "sweep wraps after all eight settings");
        check(da_data == 8'hFF || dut.waveform_inst.probe_frame_counter <
              dut.waveform_inst.probe_ramp_ticks,
              "sweep parks DAC at the negative endpoint outside the ramp");

        if (errors == 0) begin
            $display("MODELSIM_RESULT: PASS");
            $finish;
        end else begin
            $display("MODELSIM_RESULT: FAIL errors=%0d", errors);
            $fatal(1, "F2026 regression failed");
        end
    end
endmodule
