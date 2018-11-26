module inert_sm();
	output logic wrt vld;
	output logic [15:0]cmd;
	output logic C_P_H, C_P_L, C_AZ_H, C_AZ_L;

	input clk, rst_n;
	input done, INT;

	typedef enum logic [3:0] {INIT1, INIT2, INIT3, INIT4,
				WAIT, READ1, READ2, READ3, READ4} state_t;
	state_t state, nxt_state;
	
  ////// state, output determined ////////////
	always_comb begin
		cmd = 0;
		wrt = 0;
		C_P_H = 0;
		C_P_L = 0;
		C_AZ_H = 0;
		C_AZ_L = 0;
		nxt_state = INIT1;
		case (state)
		  INIT1: begin
			cmd = 16'h0D02;
			if (&timer) begin
				nxt_state = INIT2;
				wrt = 1;
			end else begin
				nxt_state = INIT1;
			end
		  end
		  INIT2: begin
			cmd = 16'h1053;
			if (done) begin
				nxt_state = INIT3;
				wrt = 1;
			end else begin
				nxt_state = INIT2;
			end
		  end
		  INIT3: begin
			cmd = 16'h1150;
			if (done) begin
				nxt_state = INIT4;
				wrt = 1;
			end else begin
				nxt_state = INIT3;
			end
		  end
		  INIT4: begin
			cmd = 16'h1460;
			if (done) begin
				nxt_state = WAIT;
				wrt = 1;
			end else begin
				nxt_state = INIT4;
			end
		  end
		  WAIT: begin
			if (done && INT_ff2 == 1) begin
				nxt_state = READ1;
				wrt = 1;
			end else begin
				nxt_state = READ1;
			end
		  end
		  READ1: begin
			cmd = 16'hA200;
			if (done) begin
				nxt_state = READ2;
				wrt = 1;
				C_P_L = 1;
			end else begin
				nxt_state = READ1;
			end
		  end
		  READ2: begin
			cmd = 16'hA300;
			if (done) begin
				nxt_state = READ3;
				wrt = 1;
				C_P_H = 1;
			end else begin
				nxt_state = READ2;
			end
		  end
		  READ3: begin
			cmd = 16'hAC00;
			if (done) begin
				nxt_state = READ4;
				wrt = 1;
				C_AZ_L = 1;
			end else begin
				nxt_state = READ3;
			end
		  end
		  READ4: begin
			cmd = 16'hAD00;
			if (done) begin
				nxt_state = WAIT;
				wrt = 1;
				C_AZ_H = 1;
			end else begin
				nxt_state = READ4;
			end
		  end
		endcase
	end
  ////////////////////////////////////////////
	
  ////// state transition ////////////////////
	always_ff @(posedge clk, negedge rst_n)
		if (!rst_n)
			state <= INIT1;
		else state <= nxt_state;
  ////////////////////////////////////////////

	logic INT_ff1,INT_ff2;
  ////// doublt flop INT, metastability //////
	always_ff @(posedge clk, negedge rst_n)
		if (!rst_n) begin
			INT_ff1 <= 0;
			INT_ff2 <= 0;
		end else begin
			INT_ff1 <= INT;
			INT_ff2 <= INT_ff1;
		end 
  ////////////////////////////////////////////




endmodule
