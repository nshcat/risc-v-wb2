module systick #(
    parameter [31:0] BaseAddr = 32'h4020
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
    output logic [31:0] wb_rdata
    // ===
);

// ==== Configuration ====
// IMPORTANT: These have to be adjusted for actual clock speed
`ifdef VERILATOR
	localparam PRESCALER = 32'd9999;	// 100MhZ / 10000 = 10KhZ
	localparam COUNTER = 32'd9;      	// 10KhZ / 10 = 1KhZ
`elsif VIVADO_SIM
	localparam PRESCALER = 32'd1;
	localparam COUNTER = 32'd1;
`else
    localparam PRESCALER = 32'd999;      // 25MhZ / 1000 = 25KhZ
	localparam COUNTER = 32'd24;         // 25KhZ / 25 = 1KhZ
`endif
// ====

// ==== Port Registers ====
logic [31:0] tick_count;   // Address 0x4030 (read only)
// ====

// ==== Internal Registers ====
logic [31:0] prescaler_threshold;
logic [31:0] counter_threshold;

logic [31:0] prescaler_value;
logic [31:0] counter_value;
// ====


// ==== Timer Logic ====
always @(posedge clk_in) begin
    if(~reset_in) begin
        prescaler_threshold <= PRESCALER;
        counter_threshold <= COUNTER;
        prescaler_value <= 32'b0;
        counter_value <= 32'b0;
        tick_count <= 32'b0;
    end
    else begin
        prescaler_value <= prescaler_value + 32'b1;

        if (prescaler_value >= prescaler_threshold) begin
            prescaler_value <= 32'b0;
            counter_value <= counter_value + 32'b1;

            if (counter_value >= counter_threshold) begin
                counter_value <= 32'b0;
                tick_count <= tick_count + 32'b1;
            end
        end     
    end
end


// === Wishbone Interface
wire addr_valid = (wb_addr - BaseAddr) == 32'h0;

assign wb_rdata = tick_count;
assign wb_ack = wb_cyc & wb_stb;
assign wb_err = ~addr_valid & wb_cyc & wb_stb;


endmodule