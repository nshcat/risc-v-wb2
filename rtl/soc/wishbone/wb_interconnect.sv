// Combinational 1:N wishbone bus interconnect
module wb_interconnect #(
    parameter N = 2,
    parameter [(N*2*32)-1:0] AddrRanges
)(
    // === Wishbone Bus IN ===
    input logic wb_we_in,
    input logic wb_stb_in,
    input logic wb_cyc_in,
    input logic [3:0] wb_sel_in,
    input logic [31:0] wb_wdata_in,
    input logic [31:0] wb_addr_in,

    output logic wb_err_in,
    output logic wb_ack_in,
    output logic [31:0] wb_rdata_in,
    // ===

    // === Wishbone Bus OUT ===
    output logic wb_we_out [N-1:0],
    output logic wb_stb_out [N-1:0],
    output logic wb_cyc_out [N-1:0],
    output logic [3:0] wb_sel_out [N-1:0],
    output logic [31:0] wb_wdata_out [N-1:0],
    output logic [31:0] wb_addr_out [N-1:0],

    input logic wb_err_out [N-1:0],
    input logic wb_ack_out [N-1:0],
    input logic [31:0] wb_rdata_out [N-1:0]
    // ===
);

logic [31:0] addr_begin [N-1:0];
logic [31:0] addr_end [N-1:0];
genvar i;
for (i = 0; i < N; i++) begin
    assign addr_begin[i] = AddrRanges[((32*(i*2+1)+32)-1):((32*(i*2)+32))];
    assign addr_end[i] = AddrRanges[(32*(i*2+1)-1):(32*(i*2))];
end

logic select [N-1:0];
genvar j;
for (j = 0; j < N; j++) begin
    assign select[j] = (wb_addr_in >= addr_begin[j] && wb_addr_in <= addr_end[j]) & wb_cyc_in & wb_stb_in;
end

// Address is valid if it belongs to one of the slaves of this interconnect.
logic addr_valid;
always_comb begin
    addr_valid = 1'b0;
    for(int i = 0; i < N; i++)
        addr_valid |= select[i];
end

// Bus errors are either generated if the requested address doesnt correspond to any
// of the slaves connected to the interconnect, or if any of the slaves signalled an error.
logic bus_error;
always_comb begin
    // Error only makes sense when the bus is actually used
    bus_error = ~addr_valid & wb_cyc_in & wb_stb_in;

    for(int i = 0; i < N; i++)
        bus_error |= (select[i] & wb_err_out[i]);
end


// Read data coming from the slaves, and feedback control signals
logic [31:0] read_data;
logic ack;
always_comb begin
    read_data = 32'h0;
    ack = 1'h0;

    for(int i = 0; i < N; i++) begin
        if (select[i]) begin
            read_data = wb_rdata_out[i];
            ack = wb_ack_out[i];
            break;
        end
    end
end

// Connection of slave->master signals
assign wb_err_in = bus_error;
assign wb_ack_in = ack;
assign wb_rdata_in = read_data;

// Distribution of master->slave signals
genvar k;
for (k = 0; k < N; k++) begin
    assign wb_we_out[k] = select[k] ? wb_we_in : 1'h0;
    assign wb_stb_out[k] = select[k] ? wb_stb_in : 1'h0;
    assign wb_cyc_out[k] = select[k] ? wb_cyc_in : 1'h0;
    assign wb_wdata_out[k] = select[k] ? wb_wdata_in : 32'h0;
    assign wb_sel_out[k] = select[k] ? wb_sel_in : 4'h0;
    assign wb_addr_out[k] = select[k] ? wb_addr_in : 32'h0;
end


endmodule