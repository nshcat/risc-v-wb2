module leds #(
    parameter [31:0] BaseAddr = 32'h4000
)(
    input logic clk_in,
    input logic reset_in,

    // === WB secondary interface ===
    input logic wb_we,
    input logic wb_stb,
    input logic wb_cyc,
    input logic [3:0] wb_sel,
    input logic [31:0] wb_wdata,
    input logic [31:0] wb_addr,

    output logic wb_err,
    output logic wb_ack,
    output logic [31:0] wb_rdata,
    // ===

    output logic [3:0] leds_out
);

logic [3:0] led_state;

wire addr_valid = (wb_addr - BaseAddr) == 32'h0;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        led_state <= 4'h0;
    end
    else if (addr_valid & wb_we & wb_sel[0]) begin
        led_state <= wb_wdata[3:0];
    end
end

assign wb_rdata = { 28'h0, led_state };
assign wb_ack = wb_cyc & wb_stb;
assign wb_err = ~addr_valid & wb_cyc & wb_stb;

assign leds_out = led_state;

endmodule