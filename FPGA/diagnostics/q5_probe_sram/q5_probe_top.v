`timescale 1ns / 1ps

module q5_probe_top (
    input  wire       clk_50m,
    output wire       da_clk,
    output wire [7:0] da_data
);
    localparam integer FRAME_TICKS = 500000; // 10 ms at 50 MHz
    localparam [31:0] RAMP_TICKS = 32'd50000; // 1 ms visible ramp

    reg [3:0] reset_pipe = 4'b0000;
    wire reset_n = reset_pipe[3];

    wire visible_ramp_unused;
    wire frame_start_unused;

    assign da_clk = ~clk_50m;

    always @(posedge clk_50m)
        reset_pipe <= {reset_pipe[2:0], 1'b1};

    q5_probe_waveform #(
        .FRAME_TICKS(FRAME_TICKS)
    ) waveform_inst (
        .clk(clk_50m),
        .reset_n(reset_n),
        .enable(1'b1),
        .ramp_ticks(RAMP_TICKS),
        .bottom_code(8'd77),
        .top_code(8'd179),
        .park_code(8'd255),
        .dac_data(da_data),
        .visible_ramp(visible_ramp_unused),
        .frame_start(frame_start_unused)
    );
endmodule
