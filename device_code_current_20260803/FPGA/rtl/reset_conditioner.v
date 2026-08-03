`timescale 1ns / 1ps

module reset_conditioner (
    input  wire clk,
    input  wire external_reset_n,
    output wire reset_n
);
    reg [7:0] power_reset_shift = 8'h00;
    reg reset_meta = 1'b0;
    reg reset_sync = 1'b0;

    assign reset_n = power_reset_shift[7] & reset_sync;

    always @(posedge clk) begin
        power_reset_shift <= {power_reset_shift[6:0], 1'b1};
        reset_meta <= external_reset_n;
        reset_sync <= reset_meta;
    end
endmodule
