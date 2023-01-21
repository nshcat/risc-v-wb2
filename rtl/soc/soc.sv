module soc(
    input logic clk_in,
    input logic reset_in,
    output logic [3:0] leds_out
);

// === WB wires ===

// = SOC <-> Interconnect
logic wb_soc_stb;
logic wb_soc_we;
logic wb_soc_ack;
logic wb_soc_cyc;
logic wb_soc_err;
logic [3:0] wb_soc_sel;
logic [31:0] wb_soc_wdata;
logic [31:0] wb_soc_rdata;
logic [31:0] wb_soc_addr;
// =

// = Interconnect <-> Program Memory
logic wb_pmem_stb;
logic wb_pmem_we;
logic wb_pmem_ack;
logic wb_pmem_cyc;
logic wb_pmem_err;
logic [3:0] wb_pmem_sel;
logic [31:0] wb_pmem_wdata;
logic [31:0] wb_pmem_rdata;
logic [31:0] wb_pmem_addr;
// =

// = Interconnect <-> Data Memory
logic wb_dmem_stb;
logic wb_dmem_we;
logic wb_dmem_ack;
logic wb_dmem_cyc;
logic wb_dmem_err;
logic [3:0] wb_dmem_sel;
logic [31:0] wb_dmem_wdata;
logic [31:0] wb_dmem_rdata;
logic [31:0] wb_dmem_addr;
// =

// = Interconnect <-> LEDs
logic wb_leds_stb;
logic wb_leds_we;
logic wb_leds_ack;
logic wb_leds_cyc;
logic wb_leds_err;
logic [3:0] wb_leds_sel;
logic [31:0] wb_leds_wdata;
logic [31:0] wb_leds_rdata;
logic [31:0] wb_leds_addr;
// =

// = Interconnect <-> Sys Tick
logic wb_syst_stb;
logic wb_syst_we;
logic wb_syst_ack;
logic wb_syst_cyc;
logic wb_syst_err;
logic [3:0] wb_syst_sel;
logic [31:0] wb_syst_wdata;
logic [31:0] wb_syst_rdata;
logic [31:0] wb_syst_addr;
// =

// ===


// === Module instances ===
program_memory pmem(
    .clk_in(clk_in),
    .reset_in(reset_in),

    .wb_we(wb_pmem_we),
    .wb_stb(wb_pmem_stb),
    .wb_cyc(wb_pmem_cyc),
    .wb_sel(wb_pmem_sel),
    .wb_wdata(wb_pmem_wdata),
    .wb_addr(wb_pmem_addr),
    .wb_rdata(wb_pmem_rdata),
    .wb_err(wb_pmem_err),
    .wb_ack(wb_pmem_ack)
);

data_memory dmem(
    .clk_in(clk_in),
    .reset_in(reset_in),

    .wb_we(wb_dmem_we),
    .wb_stb(wb_dmem_stb),
    .wb_cyc(wb_dmem_cyc),
    .wb_sel(wb_dmem_sel),
    .wb_wdata(wb_dmem_wdata),
    .wb_addr(wb_dmem_addr),
    .wb_rdata(wb_dmem_rdata),
    .wb_err(wb_dmem_err),
    .wb_ack(wb_dmem_ack)
);

systick syst(
    .clk_in(clk_in),
    .reset_in(reset_in),

    .wb_we(wb_syst_we),
    .wb_stb(wb_syst_stb),
    .wb_cyc(wb_syst_cyc),
    .wb_sel(wb_syst_sel),
    .wb_wdata(wb_syst_wdata),
    .wb_addr(wb_syst_addr),
    .wb_rdata(wb_syst_rdata),
    .wb_err(wb_syst_err),
    .wb_ack(wb_syst_ack)
);

leds led(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .leds_out(leds_out),

    .wb_we(wb_leds_we),
    .wb_stb(wb_leds_stb),
    .wb_cyc(wb_leds_cyc),
    .wb_sel(wb_leds_sel),
    .wb_wdata(wb_leds_wdata),
    .wb_addr(wb_leds_addr),
    .wb_rdata(wb_leds_rdata),
    .wb_err(wb_leds_err),
    .wb_ack(wb_leds_ack)
);

core cpu_core(
    .clk_in(clk_in),
    .reset_in(reset_in),

    .wb_we(wb_soc_we),
    .wb_stb(wb_soc_stb),
    .wb_cyc(wb_soc_cyc),
    .wb_sel(wb_soc_sel),
    .wb_wdata(wb_soc_wdata),
    .wb_addr(wb_soc_addr),
    .wb_rdata(wb_soc_rdata),
    .wb_err(wb_soc_err),
    .wb_ack(wb_soc_ack)
);


wb_interconnect #(
    .N(4),
    .AddrRanges({
        32'h0,      32'h2FFC,           // Program memory
        32'h3000,   32'h3FFC,           // Data memory
        32'h4000,   32'h4000,           // LEDs
        32'h4020,   32'h4020            // Systick
    })
) inter(
    // === To SoC
    .wb_we_in(wb_soc_we),
    .wb_stb_in(wb_soc_stb),
    .wb_cyc_in(wb_soc_cyc),
    .wb_sel_in(wb_soc_sel),
    .wb_wdata_in(wb_soc_wdata),
    .wb_addr_in(wb_soc_addr),
    .wb_rdata_in(wb_soc_rdata),
    .wb_err_in(wb_soc_err),
    .wb_ack_in(wb_soc_ack),

    // === To Peripherals 
    .wb_we_out('{wb_pmem_we, wb_dmem_we, wb_leds_we, wb_syst_we}),
    .wb_stb_out('{wb_pmem_stb, wb_dmem_stb, wb_leds_stb, wb_syst_stb}),
    .wb_cyc_out('{wb_pmem_cyc, wb_dmem_cyc, wb_leds_cyc, wb_syst_cyc}),
    .wb_sel_out('{wb_pmem_sel, wb_dmem_sel, wb_leds_sel, wb_syst_sel}),
    .wb_wdata_out('{wb_pmem_wdata, wb_dmem_wdata, wb_leds_wdata, wb_syst_wdata}),
    .wb_addr_out('{wb_pmem_addr, wb_dmem_addr, wb_leds_addr, wb_syst_addr}),
    .wb_rdata_out('{wb_pmem_rdata, wb_dmem_rdata, wb_leds_rdata, wb_syst_rdata}),
    .wb_err_out('{wb_pmem_err, wb_dmem_err, wb_leds_err, wb_syst_err}),
    .wb_ack_out('{wb_pmem_ack, wb_dmem_ack, wb_leds_ack, wb_syst_ack})
);
// ===

endmodule