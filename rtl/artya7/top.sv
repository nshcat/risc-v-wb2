// Topmost module for Arty A7 board
module top(
    input logic sys_clk_in,

    input logic reset_btn_in,   // Gets asserted when pushed

    output logic led_out
);

// Clock generation
logic slow_clk;
logic locked;

pll clockgen(
    .fast_clk_in(sys_clk_in),
    .slow_clk_out(slow_clk),
    .locked_out(locked)
);

// Reset signal generation
logic reset_sync;
synchronizer reset_synchronizer(
    .clk_in(slow_clk),
    .signal_in(reset_btn_in),
    .signal_out(reset_sync)
);

logic reset_debounced;
logic reset_falling;
logic reset_rising; // This is asserted for one clock cycle when button is pressed
debouncer reset_debouncer(
    .clk_in(slow_clk),
    .signal_in(reset_sync),
    .signal_out(reset_debounced),
    .is_falling_out(reset_falling),
    .is_rising_out(reset_rising)
);

// The main reset signal
// XXX This might not be enough, and the reason the gateware doesnt work on the FPGA?
// Maybe a counter with reset deasserted until 5 clock cycles have happened?
logic reset = ~reset_rising & locked;

// SoC instance
soc cpu_soc(
    .clk_in(slow_clk),
    .reset_in(reset),
    .led_out(led_out)
);

endmodule