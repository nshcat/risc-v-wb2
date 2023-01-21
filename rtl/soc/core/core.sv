`include "wb_master.vh"

module core(
    input logic clk_in,
    input logic reset_in,

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

// ==== Macros ====
`define IN_STATE(state) cpu_state == state

// ==== Constants ====
typedef enum logic[4:0]
{
    CPU_STATE_FETCH         = 5'b00001,
    CPU_STATE_WAIT_FETCH    = 5'b00010,
    CPU_STATE_EXECUTE       = 5'b00100,
    CPU_STATE_WAIT_MEM      = 5'b01000,
    CPU_STATE_DO_TRAP       = 5'b10000
} cpu_state_t;

typedef enum logic[6:0]
{
    OPCODE_ARITH_R      = 7'b0110011,   // rd <- rs1 OP rs2
    OPCODE_ARITH_I      = 7'b0010011,   // rd <- rs1 OP Iimm
    OPCODE_BRANCH       = 7'b1100011,   // if(rs1 OP rs2) PC<-PC+Bimm
    OPCODE_JAL          = 7'b1101111,   // rd <- PC+4; PC<-PC+Jimm
    OPCODE_JALR         = 7'b1100111,   // rd <- PC+4; PC<-rs1+Iimm
    OPCODE_LOAD         = 7'b0000011,   // rd <- mem[rs1+Iimm]
    OPCODE_STORE        = 7'b0100011,   // mem[rs1+Simm] <- rs2
    OPCODE_LUI          = 7'b0110111,   // rd <- Uimm
    OPCODE_AUIPC        = 7'b0010111,   // rd <- PC + Uimm
    OPCODE_SYSTEM       = 7'b1110011,   // rd <- CSR <- rs1/uimm5

    OPCODE_INVALID      = 7'b1111111    // Invalid opcode   
} opcode_t;

// Vivado doesnt support casting raw values to their enum representations, so we do have to do
// the casting manually here
function opcode_t castToOpcode(logic [6:0] value);
    case(value)
        7'b0110011: castToOpcode = OPCODE_ARITH_R;
        7'b0010011: castToOpcode = OPCODE_ARITH_I;
        7'b1100011: castToOpcode = OPCODE_BRANCH;
        7'b1101111: castToOpcode = OPCODE_JAL;
        7'b1100111: castToOpcode = OPCODE_JALR;
        7'b0000011: castToOpcode = OPCODE_LOAD;
        7'b0100011: castToOpcode = OPCODE_STORE;
        7'b0110111: castToOpcode = OPCODE_LUI;
        7'b0010111: castToOpcode = OPCODE_AUIPC;  
        7'b1110011: castToOpcode = OPCODE_SYSTEM;
        default: castToOpcode = OPCODE_INVALID;  
    endcase
endfunction

// Arithmetic and Branch interpretation of FUNCT3 instruction component
typedef enum logic [2:0]
{
    FUNCT3_ARITH_ADDSUB     = 3'b000,
    FUNCT3_ARITH_SHIFTL     = 3'b001,
    FUNCT3_ARITH_SHIFTR     = 3'b101,
    FUNCT3_ARITH_SLT        = 3'b010,
    FUNCT3_ARITH_SLTU       = 3'b011,
    FUNCT3_ARITH_XOR        = 3'b100,
    FUNCT3_ARITH_OR         = 3'b110,
    FUNCT3_ARITH_AND        = 3'b111
} funct3_arith_t;

function funct3_arith_t castToFunct3Arith(logic [2:0] value);
    case(value)
        3'b000: castToFunct3Arith = FUNCT3_ARITH_ADDSUB;
        3'b001: castToFunct3Arith = FUNCT3_ARITH_SHIFTL;
        3'b101: castToFunct3Arith = FUNCT3_ARITH_SHIFTR;
        3'b010: castToFunct3Arith = FUNCT3_ARITH_SLT;
        3'b011: castToFunct3Arith = FUNCT3_ARITH_SLTU;
        3'b100: castToFunct3Arith = FUNCT3_ARITH_XOR;
        3'b110: castToFunct3Arith = FUNCT3_ARITH_OR;
        default: castToFunct3Arith = FUNCT3_ARITH_AND;
    endcase
endfunction

typedef enum logic [2:0]
{
    FUNCT3_BRANCH_BEQ       = 3'b000,
    FUNCT3_BRANCH_BNE       = 3'b001,
    FUNCT3_BRANCH_BLT       = 3'b100,
    FUNCT3_BRANCH_BGE       = 3'b101,
    FUNCT3_BRANCH_BLTU      = 3'b110,
    FUNCT3_BRANCH_BGEU      = 3'b111
} funct3_branch_t;

function funct3_branch_t castToFunct3Branch(logic [2:0] value);
    case(value)
        3'b000: castToFunct3Branch = FUNCT3_BRANCH_BEQ;
        3'b001: castToFunct3Branch = FUNCT3_BRANCH_BNE;
        3'b100: castToFunct3Branch = FUNCT3_BRANCH_BLT;
        3'b101: castToFunct3Branch = FUNCT3_BRANCH_BGE;
        3'b110: castToFunct3Branch = FUNCT3_BRANCH_BLTU;
        default: castToFunct3Branch = FUNCT3_BRANCH_BGEU;  
    endcase
endfunction

typedef enum logic [2:0]
{
    FUNCT3_MEM_BYTE         = 3'b000,
    FUNCT3_MEM_HALF_WORD    = 3'b001,
    FUNCT3_MEM_WORD         = 3'b010
} funct3_mem_t;

function funct3_mem_t castToFunct3Mem(logic [1:0] value);
    case(value)
        2'b00: castToFunct3Mem = FUNCT3_MEM_BYTE;
        2'b01: castToFunct3Mem = FUNCT3_MEM_HALF_WORD;
        default: castToFunct3Mem = FUNCT3_MEM_WORD;  
    endcase
endfunction

typedef enum logic [1:0]
{
    FUNCT3_CSR_READ_WRITE       = 2'b01,
    FUNCT3_CSR_READ_SET         = 2'b10,
    FUNCT3_CSR_READ_CLEAR       = 2'b11
} funct3_csr_t;

function funct3_csr_t castToFunct3Csr(logic [1:0] value);
    case(value)
        2'b01: castToFunct3Csr = FUNCT3_CSR_READ_WRITE;
        2'b10: castToFunct3Csr = FUNCT3_CSR_READ_SET;
        default: castToFunct3Csr = FUNCT3_CSR_READ_CLEAR;  
    endcase
endfunction

// Funct7 field constants
typedef enum logic [6:0]
{
    FUNCT7_DEFAULT          = 7'b0,
    FUNCT7_ALT              = 7'b0100000        // Alternative operation, SUB instead of ADD, arithmetic shift
} funct7_t;

function funct7_t castToFunct7(logic [6:0] value);
    case(value)
        7'b0100000: castToFunct7 = FUNCT7_ALT;
        default: castToFunct7 = FUNCT7_DEFAULT;
    endcase
endfunction

// ==== Forward declarations ====
cpu_state_t cpu_state;
logic [31:0] pc;
logic [31:0] writeback_value;
logic [31:0] load_result; // The result of the load operation, after sign extending etc
logic bus_busy;

// ==== Instruction decoding ====

// The currently latched instruction
logic [31:0] instruction;

wire [4:0] instr_rs1 = instruction[19:15];
wire [4:0] instr_rs2 = instruction[24:20];
wire [4:0] instr_rd = instruction[11:7];
wire [4:0] instr_shamt = instruction[24:20];
wire [2:0] instr_func3 = instruction[14:12];
wire instr_mem_signed = ~instr_func3[2];
wire instr_csr_use_imm = instr_func3[2];
wire [11:0] instr_csr_addr = instruction[31:20];

// For some reason, direct continuous assignment doesnt work with enums and function calls..
opcode_t instr_opcode;
assign instr_opcode = castToOpcode(instruction[6:0]);

funct7_t instr_func7;
assign instr_func7 = castToFunct7(instruction[31:25]);

funct3_branch_t instr_func3_branch;
assign instr_func3_branch = castToFunct3Branch(instr_func3);

funct3_arith_t instr_func3_arith;
assign instr_func3_arith = castToFunct3Arith(instr_func3);

funct3_mem_t instr_func3_mem;
assign instr_func3_mem = castToFunct3Mem(instr_func3[1:0]);

funct3_csr_t instr_func3_csr;
assign instr_func3_csr = castToFunct3Csr(instr_func3[1:0]);

wire is_arith_reg = (instr_opcode == OPCODE_ARITH_R);
wire is_arith_imm = (instr_opcode == OPCODE_ARITH_I);
wire is_arith = (is_arith_reg | is_arith_imm);
wire is_branch = (instr_opcode == OPCODE_BRANCH);
wire is_jal = (instr_opcode == OPCODE_JAL);
wire is_jalr = (instr_opcode == OPCODE_JALR);
wire is_csr = (instr_opcode == OPCODE_SYSTEM) && (instr_func3 != 3'b000);
wire is_iret = (instr_opcode == OPCODE_SYSTEM) && (instr_func3 == 3'b000);
wire is_mret = (is_iret) && (instr_func7 == 7'b0011000);
wire is_lui = (instr_opcode == OPCODE_LUI);
wire is_auipc = (instr_opcode == OPCODE_AUIPC);
wire is_load = (instr_opcode == OPCODE_LOAD);
wire is_store = (instr_opcode == OPCODE_STORE);

// IMM extraction
wire imm_sign = instruction[31];
wire [31:0] imm_I = { (imm_sign ? 21'h1FFFFF : 21'h0), instruction[30:20] };
wire [31:0] imm_S = { (imm_sign ? 21'h1FFFFF : 21'h0), instruction[30:25], instruction[11:8], instruction[7] };
wire [31:0] imm_B = { (imm_sign ? 20'hFFFFF : 20'h0), instruction[7], instruction[30:25], instruction[11:8], 1'b0 };
wire [31:0] imm_U = { instruction[31:12], 12'h0 };
wire [31:0] imm_J = { (imm_sign ? 12'hFFF : 12'h0), instruction[19:12], instruction[20], instruction[30:25], instruction[24:21], 1'b0 };
wire [31:0] imm_CSR = { 27'h0, instruction[19:15] };

// ==== Register file ====
logic [31:0] registers [31:0];

// Latched copies of source register contents
logic [31:0] rs1_data;
logic [31:0] rs2_data;

// Write back control signal
logic do_writeback;

always_comb begin
    do_writeback = 1'b0;

    // Stores, branches, and iret dont do writeback
    if (~(is_branch | is_store | is_iret)) begin
        // Only do write back when reading is finished
        if (is_load & `IN_STATE(CPU_STATE_WAIT_MEM) & ~bus_busy) begin
            do_writeback = 1'b1;
        end

        if (~is_load & `IN_STATE(CPU_STATE_EXECUTE)) begin
            do_writeback = 1'b1;
        end
    end
end

// Register writeback and reset
always_ff @(posedge clk_in) begin
    if (do_writeback) begin
        if (instr_rd != 5'h0) begin
            registers[instr_rd] <= writeback_value;
        end
    end
end

// ==== ALU operations ====

// ArithR and ArithI both use the full ALU result: rd <- rs1 OP (rs2 | imm_I)
// JALR just uses the add result: PC <- rs1 + Iimm
// Branches use the subtraction to evaluate the branch condition
// Loads use imm_I as well, so we can reuse the adder: rd <- mem[rs1+imm_I]

// First input is always reg[rs1]
wire [31:0] arith_in1 = rs1_data;

// For branches and R type arithmetic instructions, the second ALU input is reg[rs2], for ALUimm, Load or JALR its I type IMM value
// We use this for the branch predicate evaluation as well, and JALR.
wire [31:0] arith_in2 = (is_arith_reg | is_branch) ? rs2_data : imm_I;

// Adder used for arithmetic operations, memory
wire [31:0] arith_add = arith_in1 + arith_in2;

// Subtraction used for both branch condition checking, and as ALU result for sub/subi
wire arith_C;
wire [31:0] arith_sub;
assign {arith_C, arith_sub} = arith_in1 - arith_in2;
wire arith_Z = (arith_sub == 32'h0) ? 1'b1 : 1'b0;
wire arith_V = (arith_in1[31] & !arith_in2[31] & !arith_sub[31]) | (!arith_in1[31] & arith_in2[31] & arith_sub[31]);
wire arith_N = arith_sub[31];
wire arith_S = arith_N ^ arith_V;

// Branch conditions, retrieved from subtraction result
wire cond_beq = arith_Z;
wire cond_bne = ~arith_Z;
wire cond_blt = arith_S;
wire cond_bge = ~arith_S;
wire cond_bltu = arith_C;
wire cond_bgeu = ~arith_C;

// ALU output. This is only used if isArith is true.
logic [31:0] arith_result;
wire [31:0] arith_right_shift = (arith_in1 >> (is_arith_imm ? { 27'h0, instr_shamt } : arith_in2));

// ALU output calculation
always_comb begin
    arith_result = 32'h0;
    
    // FUNCT3 field of instruction describes which operation to perform
    case (instr_func3_arith)
        FUNCT3_ARITH_ADDSUB: begin
            // For register arithmetic instruction, funct7 decides whether its addition or subtraction
            // For immediate arithmetic instructions, no subtraction instruction is provided
            if (is_arith_reg && (instr_func7 == FUNCT7_ALT)) begin
                arith_result = arith_sub;
            end
            else begin
                arith_result = arith_add;
            end
        end
        FUNCT3_ARITH_AND: begin
            arith_result = arith_in1 & arith_in2;
        end
        FUNCT3_ARITH_OR: begin
            arith_result = arith_in1 | arith_in2;
        end
        FUNCT3_ARITH_XOR: begin
            arith_result = arith_in1 ^ arith_in2;
        end
        FUNCT3_ARITH_SLT: begin
            arith_result = ($signed(arith_in1) < $signed(arith_in2)) ? 32'h1 : 32'h0;
        end
        FUNCT3_ARITH_SLTU: begin
            arith_result = ($unsigned(arith_in1) < $unsigned(arith_in2)) ? 32'h1 : 32'h0;
        end
        FUNCT3_ARITH_SHIFTL: begin
            // For imm arithmetic left shifts, we need to use the shamt field in the instruction
            arith_result = arith_in1 << (is_arith_imm ? { 27'h0, instr_shamt } : arith_in2);
        end
        /* FUNCT3_ARITH_SHIFTR */
        default: begin
            // FUNCT7_ALT implies arithmetic right shift
            if (instr_func7 == FUNCT7_ALT) begin
                // As for left shifts, imm arithmetic right shift uses shamt field of instruction
                arith_result = { (arith_in1[31] ? 1'b1 : 1'b0), arith_right_shift[30:0] };
            end
            else begin
                arith_result = arith_right_shift;
            end
        end
    endcase
end

// ==== Writeback value ====
always_comb begin
    writeback_value = 32'h0;

    case (1'b1)
        (is_arith_imm | is_arith_reg): begin
            writeback_value = arith_result;
        end
        (is_lui): begin
            writeback_value = imm_U;
        end
        (is_auipc): begin
            writeback_value = pc + imm_U;
        end
        (is_jal | is_jalr): begin
            writeback_value = pc + 32'h4;
        end
        (is_load): begin
            writeback_value = load_result;
        end
        (is_csr): begin
            writeback_value = 32'h0;//csrs[instr_csr_addr];
        end
        default: begin
            writeback_value = 32'h0;
        end
    endcase
end

// ==== Next PC logic and branching ====
logic branch_taken;

always_comb begin
    branch_taken = 1'b0;
    
    case (instr_func3_branch)
        FUNCT3_BRANCH_BEQ: branch_taken = cond_beq;
        FUNCT3_BRANCH_BGE: branch_taken = cond_bge;
        FUNCT3_BRANCH_BGEU: branch_taken = cond_bgeu;
        FUNCT3_BRANCH_BLT: branch_taken = cond_blt;
        FUNCT3_BRANCH_BLTU: branch_taken = cond_bltu;
        FUNCT3_BRANCH_BNE: branch_taken = cond_bne;
        default: branch_taken = 1'b0;
    endcase
end

logic [31:0] pc_next;

always_comb begin
    pc_next = pc + 32'h4;

    case (1'b1)
        (is_jal): begin
            pc_next = pc + imm_J;
        end
        (is_jalr): begin
            pc_next = rs1_data + imm_I;
        end
        (is_branch): begin
            if (branch_taken) begin
                pc_next = pc + imm_B;
            end
        end
        /*(is_iret): begin
            pc_next = csrs[CSR_MEPC];
        end*/
        default: begin
            pc_next = pc + 32'h4;
        end
    endcase
end

// ==== WB Interface ====
logic [31:0] bus_rdata;
logic [31:0] bus_wdata;
logic [3:0] bus_wmask;
logic [31:0] bus_word_addr;
logic [31:0] bus_addr;
wb_command_t bus_command;

assign bus_word_addr = { bus_addr[31:2], 2'h0 };
logic bus_error;

wb_master bus(
    .clk_in(clk_in),
    .reset_in(reset_in),

    .cmd_in(bus_command),
    .busy_out(bus_busy),
    .rdata_out(bus_rdata),
    .wdata_in(bus_wdata),
    .wmask_in(bus_wmask),
    .addr_in(bus_word_addr),
    .err_out(bus_error),

    .wb_we(wb_we),
    .wb_stb(wb_stb),
    .wb_cyc(wb_cyc),
    .wb_sel(wb_sel),
    .wb_wdata(wb_wdata),
    .wb_addr(wb_addr),
    .wb_rdata(wb_rdata),
    .wb_err(wb_err),
    .wb_ack(wb_ack)
);

// For stores, regs[rs2] is written to memory
// It has to be valid on state transition from EXECUTE to WAIT_MEM, so we keep the assignment valid for both states.
wire [31:0] store_data = (is_store & (`IN_STATE(CPU_STATE_EXECUTE) | `IN_STATE(CPU_STATE_WAIT_MEM)))
    ? rs2_data
    : 32'h0;

// Address inside word
wire [1:0] addr_byte_offset = bus_addr[1:0];
logic bus_align_error;

// Write mask and write data
always_comb begin
    bus_wdata = 32'h0;
    bus_wmask = 4'h0;
    bus_align_error = 1'b0;

    case (instr_func3_mem)
        FUNCT3_MEM_BYTE: begin
            // No misalignment possible
            bus_align_error = 1'b0;

            // Little endian "emulation", because bus is big endian
            case (addr_byte_offset)
                2'b00: begin
                    bus_wmask = 4'b0001; // Addressing lowest order byte
                    bus_wdata = { 24'h0, store_data[7:0] };
                end
                2'b01: begin
                    bus_wmask = 4'b0010;
                    bus_wdata = { 16'h0, store_data[7:0], 8'h0 };
                end
                2'b10: begin
                    bus_wmask = 4'b0100;
                    bus_wdata = { 8'h0, store_data[7:0], 16'h0 };
                end
                2'b11: begin
                    bus_wmask = 4'b1000; // Addressing highest order byte
                    bus_wdata = { store_data[7:0], 24'h0 };
                end
            endcase
        end
        FUNCT3_MEM_HALF_WORD: begin
            bus_align_error = (addr_byte_offset == 2'b01 || addr_byte_offset == 2'b11);

            if (addr_byte_offset == 2'b00) begin
                // Accessing lower half word
                bus_wmask = 4'b0011;
                bus_wdata = { 16'h0, store_data[15:0] };
            end
            else if (addr_byte_offset == 2'b10) begin
                // Accessing upper half word
                bus_wmask = 4'b1100;
                bus_wdata = { store_data[15:0], 16'h0 };
            end
        end
        /*FUNCT3_MEM_WORD*/
        default: begin // Word access
            bus_wmask = 4'b1111;
            bus_wdata = store_data;
            bus_align_error = (addr_byte_offset != 2'h0) ? 1'b1 : 1'b0;
        end
    endcase
end

wire pc_align_error = (pc[1:0] != 2'b00);

// Bus command and address handling
always_comb begin
    bus_command = WISHBONE_CMD_NONE;
    bus_addr = 32'h0;

    if (`IN_STATE(CPU_STATE_FETCH)) begin
        // When transitioning from FETCH -> WAITFETCH, we want to signal the WB master to start
        // a load
        bus_addr = pc;

        // Dont try to load if PC is misaligned
        if (~pc_align_error) begin
            bus_command = WISHBONE_CMD_LOAD;
        end
    end
    else if (`IN_STATE(CPU_STATE_EXECUTE) || `IN_STATE(CPU_STATE_WAIT_MEM)) begin      
        if (is_load) begin
            bus_addr = rs1_data + imm_I;
        end
        else if (is_store) begin
            bus_addr = rs1_data + imm_S;
        end

        // When transitioning from EXECUTE -> WAITMEM, we want to either do a load or a store depending on the 
        // instruction
        if (`IN_STATE(CPU_STATE_EXECUTE)) begin
            // Only perform memory access if address is aligned correctly

            if (~bus_align_error) begin
                if (is_load) begin
                    bus_command = WISHBONE_CMD_LOAD;
                end
                else if (is_store) begin
                    bus_command = WISHBONE_CMD_STORE;
                end
            end
        end
    end
end

always_comb begin
    load_result = 32'h0;
    
    case (instr_func3_mem)
        FUNCT3_MEM_BYTE: begin
            case (addr_byte_offset)
                2'b00: begin
                    // Addressing lowest order byte
                    load_result = { (instr_mem_signed & bus_rdata[7]) ? 24'hFFFFF : 24'h0, bus_rdata[7:0] };
                end
                2'b01: begin
                    load_result = { (instr_mem_signed & bus_rdata[15]) ? 24'hFFFFF : 24'h0, bus_rdata[15:8] };
                end
                2'b10: begin
                    load_result = { (instr_mem_signed & bus_rdata[22]) ? 24'hFFFFF : 24'h0, bus_rdata[23:16] };
                end
                2'b11: begin
                    // Addressing highest order byte
                    load_result = { (instr_mem_signed & bus_rdata[31]) ? 24'hFFFFF : 24'h0, bus_rdata[31:24] };
                end
            endcase
        end
        FUNCT3_MEM_HALF_WORD: begin
            if (addr_byte_offset == 2'b00) begin
                // Addressing lower half word
                load_result = { (instr_mem_signed & bus_rdata[15]) ? 16'hFFFF : 16'h0, bus_rdata[15:0] };
            end
            else if (addr_byte_offset == 2'b10) begin
                // Addressing upper half word
                load_result = { (instr_mem_signed & bus_rdata[31]) ? 16'hFFFF : 16'h0, bus_rdata[31:16] };
            end
        end
        /*FUNCT3_MEM_WORD*/
        default: begin // Word access
            load_result = bus_rdata;
        end
    endcase
end

// ==== State machine and program counter ====
always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        pc <= 32'h0;
        cpu_state <= CPU_STATE_FETCH;

        // TODO: Remove unneeded resets to save logic!
        instruction <= 32'h0;
        rs1_data <= 32'h0;
        rs2_data <= 32'h0;
    end
    else begin
        case (cpu_state)        
            CPU_STATE_WAIT_FETCH: begin
                // Wait for bus to report completed transaction
                if (~bus_busy) begin
                    instruction <= bus_rdata;
                    rs1_data <= registers[bus_rdata[19:15]];
                    rs2_data <= registers[bus_rdata[24:20]];
                    cpu_state <= CPU_STATE_EXECUTE;
                end
            end
            CPU_STATE_EXECUTE: begin
                // Handling of load and stores
                // For load/stores, transition from EXECUTE -> WAIT_MEM will initiate a WB bus transaction
                if (is_load | is_store) begin
                    cpu_state <= CPU_STATE_WAIT_MEM;
                end
                else begin
                    cpu_state <= CPU_STATE_FETCH;
                    pc <= pc_next;
                end
            end
            CPU_STATE_WAIT_MEM: begin
                // Wait for bus operation to complete
                if (~bus_busy) begin
                    cpu_state <= CPU_STATE_FETCH;
                    pc <= pc_next;
                end
            end
            // FETCH
            default: begin
                // The next transition will initiate a read from mem[pc].
                // We have to wait for it to finish in the next state.
                cpu_state <= CPU_STATE_WAIT_FETCH;
            end
        endcase
    end
end


endmodule