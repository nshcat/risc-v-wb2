module testbench;
timeunit 1ns;
timeprecision 100ps;

logic clk, reset;
logic led;

parameter PERIOD = 10;

initial begin
    reset = 1'b0;
    clk = 1'b0;
    #5 clk = 1'b1;
    #5 reset = 1'b1;
end

always #PERIOD clk=~clk;

soc dut(
    .clk_in(clk),
    .reset_in(reset),
    .led_out(led)
);

endmodule