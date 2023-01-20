`include "wb_master.vh"

// The address and write value/mask (if needed) are latched in, so only have to be valid on the rising edge
// on which the command is issued
module wb_master(
    input logic clk_in,
    input logic reset_in,

    // Interface to core
    input wb_command_t cmd_in,
    output logic busy_out,
    output logic err_out,

    input logic [31:0] addr_in,
    output logic [31:0] rdata_out,
    input logic [31:0] wdata_in,
    input logic [3:0] wmask_in,

    // === WB primary interface ===
    output logic wb_we,
    output logic wb_stb,
    output logic wb_cyc,
    output logic [3:0] wb_sel,
    output logic [31:0] wb_wdata,
    output logic [31:0] wb_addr,

    input logic wb_err,
    input logic wb_ack,
    input logic [31:0] wb_rdata
    // ===
);

typedef enum logic [1:0]
{  
    STATE_WAIT_FOR_CMD = 2'b00,
    STATE_WAIT_ACK     = 2'b01
} state_t;

logic [31:0] write_data;
logic [3:0] write_mask;
logic [31:0] read_data;
logic [31:0] address;
logic busy, error;
state_t state;

// TODO maybe have these be determined combinationally?
// Or can you directly <= to interface outputs?
// Or all WB signals combinationally derived from current state?
logic we, stb, cyc;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        read_data <= 32'h0;
        busy <= 1'b0;
        state <= STATE_WAIT_FOR_CMD;
        address <= 32'h0;
        write_mask <= 4'h0;
        write_data <= 32'h0;
        we <= 1'h0;
        stb <= 1'h0;
        cyc <= 1'h0;
        error <= 1'h0;
    end
    else begin
        case (state)  
            STATE_WAIT_ACK: begin
                // Check for errors
                if (wb_err) begin
                    // Set error indicator. Will be cleared upon next command.
                    error <= 1'b1;

                    // Transaction cancelled
                    stb <= 1'b0;
                    cyc <= 1'b0;
                    we <= 1'b0;
                    state <= STATE_WAIT_FOR_CMD;
                    busy <= 1'b0;
                end
                // Check for slaves ack
                else if (wb_ack) begin
                    if (~we) begin
                        // This was a load, retrieve read data from bus
                        read_data <= wb_rdata;
                    end

                    // Transaction is complete, deassert control lines
                    stb <= 1'b0;
                    cyc <= 1'b0;
                    we <= 1'b0;
                    state <= STATE_WAIT_FOR_CMD;
                    busy <= 1'b0;
                end
            end
            // STATE_WAIT_FOR_CMD
            default: begin
                // Check if core provided command
                if (cmd_in != WISHBONE_CMD_NONE) begin
                    // We are busy now
                    busy <= 1'b1;

                    // Latch task data
                    address <= addr_in;
                    write_data <= wdata_in;
                    write_mask <= wmask_in;

                    // Clear error indicator
                    error <= 1'h0;

                    // Start transaction
                    we <= (cmd_in == WISHBONE_CMD_LOAD) ? 1'b0 : 1'b1;
                    stb <= 1'b1;
                    cyc <= 1'b1;

                    // Wait for slave to ack transaction
                    state <= STATE_WAIT_ACK;
                end
            end
        endcase
    end
end

assign rdata_out = read_data;
assign busy_out = busy;
assign err_out = error;

// Bus assignments
assign wb_addr = address;
assign wb_wdata = write_data;
assign wb_sel = we ? write_mask : 4'b1111; // Always read full words
assign wb_we = we;
assign wb_stb = stb;
assign wb_cyc = cyc;


endmodule