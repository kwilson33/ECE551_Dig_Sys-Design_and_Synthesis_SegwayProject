module rst_synch(RST_n, clk, rst_n);

	//RST_n = raw input for push
	//rst_n = synchronized output which will form global reset. needs to be deasserted (set to 1) on negedge of clock
	input RST_n, clk;
	output  rst_n;
	reg reg1, rst_n;
	
	// make a reset synchronizer that takes in a
	// raw push button signal and creates a signal
	// that is deasserted at neg edge of clk
	always @(negedge clk, negedge RST_n) begin
		if (!RST_n) begin
			reg1 <= 0;
			rst_n <= 0;
		end
		else begin
		//want our reset to deassert on neg edge of clock to avoid big issues
			reg1 <= 1'b1; //double flop for metastability
			rst_n <= reg1;
		end
	end
	
endmodule