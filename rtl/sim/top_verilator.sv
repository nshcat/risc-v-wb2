module top_verilator(
    input logic clk_i,
    input logic reset_i
);

core cpucore(
    .clk_in(clk_i),
    .reset_in(reset_i)
);

logic [31:0] counter;
always_ff @(posedge clk_i) begin
    if (~reset_i) begin
        counter <= 32'h0;
	end
    else begin
		counter <= counter + 32'h1;
    end
end

endmodule