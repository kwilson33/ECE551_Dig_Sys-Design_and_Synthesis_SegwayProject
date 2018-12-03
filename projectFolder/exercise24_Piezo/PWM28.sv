module PWM28(clk,rst_n,duty,period,PWM_sig);
	input logic clk,rst_n;
	input logic [28:0]duty,period;

	output logic PWM_sig;
	
	logic [28:0]cnt; //28 bit counter
	logic S,R;	 //set,reset for SR flip-flop

	//28 bit-counter
	always_ff @(posedge clk,negedge rst_n) begin
	  if (!rst_n) cnt <= 28'b0;
	  else if (cnt == period) cnt <= 28'b0;
	  else cnt <= cnt + 1;
	end

	//we assert PWM_sig when starting the count
	assign S = (cnt == 0);
  	//we want to reset if we've passed the duty cycle
	assign R = (cnt >= duty);

	//SR flip flop for PWM_sig
	always_ff @(posedge clk,negedge rst_n) begin
	  if (!rst_n) PWM_sig <= 0;
	  else if (R) PWM_sig <= 0;
	  else if (S) PWM_sig <= 1;
  end

endmodule
