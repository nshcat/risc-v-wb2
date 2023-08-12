module program_memory #(
    parameter [31:0] BaseAddr = 32'h0
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

function [31:0] endian_swap_word(input [31:0] word);
    endian_swap_word = {
        word[ 7: 0],
        word[15: 8],
        word[23:16],
        word[31:24]
    };
endfunction

// Error logic. We dont support writes or misaligned reads
wire err = (wb_we || wb_addr[1:0] != 2'h0);

// 12 kibibyte of program flash
(* ram_style = "block" *)
reg [31:0] memory [0:3071];

`ifdef VERILATOR
initial $readmemh("flash.txt", memory);
`elsif VIVADO_SIM
initial $readmemh("flash.txt", memory);
`else
initial $readmemh("./../../memory/flash.txt", memory);
`endif

// Reading
logic [31:0] rdata_raw;
wire [31:0] rdata = endian_swap_word(rdata_raw);

wire [31:0] absolute_addr = (wb_addr - BaseAddr);
wire [11:0] word_addr = absolute_addr[13:2];    // Word-based address

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        rdata_raw <= 32'h0;
    end
    else if (wb_stb) begin
        rdata_raw <= memory[word_addr];
    end
end

// Acknowledge signal
logic ack;

always_ff @(posedge(clk_in)) begin
    if (~reset_in) begin
        ack <= 1'h0;
    end
    else begin
        // We only ack if a transaction was initiated and no error was detected
        ack <= wb_stb & ~err;
    end
end

assign wb_ack = ack & wb_stb & ~err;
assign wb_rdata = rdata;
assign wb_err = err;

endmodule