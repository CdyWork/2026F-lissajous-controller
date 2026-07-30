`timescale 1ns / 1ps

module tb_q5_probe_waveform;
    localparam integer FRAME_TICKS = 500000;
    localparam integer RAMP_TICKS = 50000;

    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg enable = 1'b0;
    wire [7:0] dac_data;
    wire visible_ramp;
    wire frame_start;

    integer cycle = 0;
    integer frame_cycle = 0;
    integer frame_count = 0;
    integer visible_count = 0;
    integer errors = 0;
    reg [7:0] previous_dac = 8'd0;
    reg previous_visible = 1'b0;

    always #10 clk = ~clk;

    q5_probe_waveform #(
        .FRAME_TICKS(FRAME_TICKS)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        .enable(enable),
        .ramp_ticks(RAMP_TICKS),
        .bottom_code(8'd77),
        .top_code(8'd179),
        .park_code(8'd255),
        .dac_data(dac_data),
        .visible_ramp(visible_ramp),
        .frame_start(frame_start)
    );

    task check;
        input condition;
        input [8*80-1:0] message;
        begin
            if (!condition) begin
                $display("CHECK FAILED: %0s", message);
                errors = errors + 1;
            end
        end
    endtask

    always @(posedge clk) begin
        #1;
        if (reset_n && enable) begin
            cycle = cycle + 1;

            if (frame_start) begin
                if (frame_count > 0) begin
                    check(frame_cycle == FRAME_TICKS,
                          "10 ms frame length is not 500000 clocks");
                    check(visible_count == RAMP_TICKS,
                          "visible ramp length is not 1 ms");
                end

                frame_count = frame_count + 1;
                frame_cycle = 0;
                visible_count = 0;
                previous_visible = 1'b0;

                if (frame_count == 4) begin
                    if (errors == 0)
                        $display("MODELSIM_RESULT: PASS");
                    else
                        $display("MODELSIM_RESULT: FAIL errors=%0d", errors);
                    $finish;
                end
            end

            if (visible_ramp) begin
                visible_count = visible_count + 1;
                if (previous_visible && (dac_data < previous_dac)) begin
                    $display("CHECK FAILED: ramp decreased at frame cycle %0d", frame_cycle);
                    errors = errors + 1;
                end
                check(dac_data >= 8'd77 && dac_data <= 8'd179,
                      "visible ramp code outside configured range");
            end else begin
                check(dac_data == 8'd255, "park interval is not outside-screen code");
            end

            previous_dac = dac_data;
            previous_visible = visible_ramp;
            frame_cycle = frame_cycle + 1;
        end
    end

    initial begin
        repeat (5) @(posedge clk);
        reset_n = 1'b1;
        enable = 1'b1;
    end
endmodule
