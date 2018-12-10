module PWM11 (PWM_sig, clk, rst_n, duty);
// Kevin Wilson
// 10/05/18

	/* 11 bit PWM => (50Mhz / 2^11) = 24.4 kHz
	   a PWF can not get both to 100% and to 0%. For our case, can only get to 0x7FF, which is on for 2047
	   out of 2048 clocks. When testing, make sure to keep duty for atleast 2048 clocks.
	*/ 
	
	input clk, rst_n; 		// clock is 50 Mhz, active low asynch reset
	input [10:0] duty; 		// result from balance controller telling how fast to run each motor
	output logic PWM_sig; 	// output of FF to H-bridge chip controlling the DC motor
	
	logic [10:0] count;	    // 11 bit value coming out of the counter 
	wire cnt_eq_0;			// if high, set
	wire cnt_gtEQ_duty;		// if high, reset
	
	
	
	// check what count is and assign flags
	assign cnt_eq_0 = (count == 11'h0000) ? 1:0;
	assign cnt_gtEQ_duty = (count >= duty) ? 1:0;
	
	//11 bit counter
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			count <= 11'h000;
		else 
			count <= count + 1;
	end
	
	//SR flop (set, reset)
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) 				// asynch reset
			PWM_sig <= 1'b0;
		else if (cnt_gtEQ_duty) 	// synch reset if count is greater than duty
			PWM_sig <= 1'b0;
		else if (cnt_eq_0)
			PWM_sig <= 1'b1;
		// else, retain current value of PWM_sig
	end

endmodule
		
	