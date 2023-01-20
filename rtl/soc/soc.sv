module soc(
    input logic clk_in,
    input logic reset_in,
    output logic led_out
);

// === WB wires ===
logic wb_soc_pm_stb;
logic wb_soc_pm_we;
logic wb_soc_pm_ack;
logic wb_soc_pm_cyc;
logic wb_soc_pm_err;
logic [3:0] wb_soc_pm_sel;
logic [31:0] wb_soc_pm_wdata;
logic [31:0] wb_soc_pm_rdata;
logic [31:0] wb_soc_pm_addr;
// ===


// === Module instances ===
program_memory pmem(
    .clk_in(clk_in),
    .reset_in(reset_in),

    .wb_we(wb_soc_pm_we),
    .wb_stb(wb_soc_pm_stb),
    .wb_cyc(wb_soc_pm_cyc),
    .wb_sel(wb_soc_pm_sel),
    .wb_wdata(wb_soc_pm_wdata),
    .wb_addr(wb_soc_pm_addr),
    .wb_rdata(wb_soc_pm_rdata),
    .wb_err(wb_soc_pm_err),
    .wb_ack(wb_soc_pm_ack)
);

core cpu_core(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .led_out(led_out),

    .wb_we(wb_soc_pm_we),
    .wb_stb(wb_soc_pm_stb),
    .wb_cyc(wb_soc_pm_cyc),
    .wb_sel(wb_soc_pm_sel),
    .wb_wdata(wb_soc_pm_wdata),
    .wb_addr(wb_soc_pm_addr),
    .wb_rdata(wb_soc_pm_rdata),
    .wb_err(wb_soc_pm_err),
    .wb_ack(wb_soc_pm_ack)
);
// ===

endmodule