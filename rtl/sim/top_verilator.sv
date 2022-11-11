module top_verilator(
    input logic clk_i,
    input logic reset_i
);

logic [31:0] counter;
always_ff @(posedge clk_i) begin
    if (~reset_i) begin
        counter <= 32'h0;
        test_irq <= 1'b0;
	end
    else begin
        if (counter == 32'h60) begin
            test_irq <= 1'b1;
        end
        else begin
            counter <= counter + 32'h1;
        end
    end
end

endmodule