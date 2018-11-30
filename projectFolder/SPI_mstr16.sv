// Kevin Wilson 11/4/18
module SPI_mstr16(done, rd_data, SS_n, SCLK, MOSI, MISO, wrt, clk, rst_n, cmd);

	// Serial Peripheral Interconnect, simple Master Slave serial interface
	// 4 wires for full duplex...
	// MOSI : Master Out Slave In (we are its master and we drive it)
	// MISO: Master In Slave Out (intertial sensor drives this back to us)
	// SCLK: serial clock, normally high
	// SS_n: Active low slave select
	

	// MOSI shifted on SCLK fall
	// MISO sampled on SCLK rise

	// Essentially a 16 bit shift register that can parallel load data we want to transmit
	// And then shift it out (MSB first) at the same time it receives data from slave in the LSB

	// The bit coming from the slave (MISO) is sampled on rise of SCLK and put into shift reg on fall of  SCLK

	// Inputs
	input clk, rst_n; 					// 50 MHz clock, active low reset
	input wrt;										// high for 1 clock period, initates SPI transaction
	input MISO;										// Master in Slave Out. Sampled on SCLK rise
	input [15:0] cmd; 								// data being sent to intertial sensor

	// Outputs
	output logic [15:0] rd_data;					// Data from SPI slave, use [7:0] for intertial sensor, [11:0] for A/D
	output logic SS_n; 								// active low slave select
	output logic SCLK;						   		// 1/32 of 50MHz clock, comes from MSB of a 5 bit counter running off clk
	output logic MOSI,  done;						//	MOSI signal we're driving, and done signal asserted when SPI transaction is complete.					 

	
	// Additional signals
	logic rst_cnt;									// rst_cnt is high when in IDLE state
	logic init;										// init is used to initialize an SPI transaction.
	logic set_done;									// asserted when going back to IDLE state. Used for SS_n and done signals.

	reg [4:0] sclk_div;								// 5 bit counter that will determine what SCLK is
	logic smpl; 				 					// asserted when sclk_div is 01111, 
	logic shft; 									// asserted when sclk_div is 11111
 
	reg MISO_smpl;									// sample MISO on SCLK rise

	reg [15:0] shft_reg; 							// The heart of SPI. MISO_smpl shifted at the LSB on SCLK rise. MOSI shifted OUT on SCLK fall
	
	reg [4:0] shft_cnt; 							// counts to see if we've shifteed enough, count up to 16
	



	
	assign SCLK = sclk_div[4]; 						// SCLK is 1/32 of 50Mhz clock, high when sclk_div = 10000
	assign rd_data = shft_reg;						// at end of transaction, rd_data will contain shft_reg;
	assign MOSI = shft_reg[15]; 					// MOSI is the MSB of the shift register. 
	


	// Both flops are synchronously reset by rst_cnt, which is high when in IDLE state. One flop is for sclk_div counter to see when we're ready to shift/sample
	// and the other flop is for shft_cnt counter to see when we've shifted enough.

	// sclk_div counter logic
	always @(posedge clk) begin
		if (rst_cnt) begin
			// synch reset to this value for creation of 'front porch'
			// which is the little bit of time before SCLK goes low when SS_n first goes low
			sclk_div <= 5'b10111;				 
		end
		
		else begin
			sclk_div <= sclk_div  + 1;
		end
	end

	//counter to see if we're done shifting
	always @(posedge clk) begin
		if (rst_cnt)  shft_cnt <= 0;				
		else if (shft) shft_cnt <= shft_cnt  + 1;
	end
	
	

	
	// sample MISO on SCLK rise
	always @(posedge clk) begin
		if (smpl) begin
			MISO_smpl <= MISO;
		end 
	end
	


	// shft_reg logic
	always @(posedge clk) begin
		casex ({wrt, shft}) 
			2'b1x: shft_reg <= cmd; 						// if wrt is high, initiate new transaction and load in data
			2'b01: shft_reg <= {shft_reg[14:0], MISO_smpl}; // shift in MISO at LSB. At end of transaction, shft_reg will be data in shft_reg
			default : shft_reg <= shft_reg; 
		endcase
	end
	
	
	
	
			
	/*
		active low preset flop for done signal. SR Flop

		assert done if shifted 16 times. Can't test by seeing if shft_cnt == 16 BECAUSE we're transitioning back to IDLE the same time we are shifting final time, 
		in which we are resetting count to 0, which has priority to the increment. Instead, use a set_done signal that is asserted when transitioning back to IDLE.
	*/
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) done <= 0;										
		else if (init) done <= 0;							// done stays high until next wrt is asserted, then init goes high for one clock
		else if (set_done) done <= 1; 							
	end
	

	/*
		active low preset flop for SS_n signal. SR Flop

		assert SS_n if shifted 16 times. Can't test by seeing if shft_cnt == 16 BECAUSE we're transitioning back to IDLE the same time we are shifting final time, 
		in which we are resetting count to 0, which has priority to the increment. Instead, use a set_done signal that is asserted when transitioning back to IDLE.
	*/
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) SS_n <= 1;										
		else if (init) SS_n <= 0;							// SS_n is high until next wrt, when init goes high for one clock
		else if (set_done) SS_n <= 1; 							
	end


	// ##################################################### State Machine ###############################################################
	typedef enum reg[1:0] {IDLE, SAMPLE_WAIT, SHIFT_WAIT, FINAL} state_t;
	state_t state, nxt_state;
	
	//sequential next state logic
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) state <= IDLE;
		else state <= nxt_state;
	end
	
	//combinational transition logic
	always_comb begin
		nxt_state = IDLE;
		rst_cnt = 0; 
		set_done = 0;										// asserted when going back to IDLE.						
		init = 0;
		smpl = 0;
		shft = 0;
	
		case (state)
			IDLE: begin
				rst_cnt = 1; 	// high only when in IDLE
				if (wrt) begin
					init = 1;								// wrt is an external signal, can stay high forever, so make init signal high when ready to transmit, then go low. Fixes bugs with wrt kept high
					nxt_state = SAMPLE_WAIT;
				end
			end
				

			SAMPLE_WAIT: begin 								// sample and wait until ready to shift

				if (sclk_div == 5'b01111) begin 			// check value of sclk_div
		
					smpl = 1;								// enable sample of MISO when sclk_div will be 10000 on next clock
								
					if (shft_cnt == 5'hF) begin 			// check if shft_cnt is 15, if so, sample final time in FINAL state		
						nxt_state = FINAL;															
					end

					else nxt_state = SHIFT_WAIT;			// if not, sample in the SHIFT_WAIT state 
					
				end 

				else nxt_state = SAMPLE_WAIT;				// if sclk_div not ready, stay in current state
				
			end
			

			SHIFT_WAIT: begin								// shift and wait until ready to sample	

				if(sclk_div == 5'b11111) begin				// on the next clock edge, sclk_div will be 0
					shft = 1;										// which means SCLK will fall, so assert a shift
					nxt_state = SAMPLE_WAIT;
				end
				 else begin
					nxt_state = SHIFT_WAIT;
				end
			end 
				
			
			FINAL: begin

				if (sclk_div == 5'b11111) begin 			// 'back porch': Wait for last time to shift. SCLK won't fall after 15 shifts, 
															// so have to wait until sclk_div increments to tell us to shift
					nxt_state = IDLE;
					rst_cnt = 1;							// reset count so sclk_div is 10111 and has a 1 in the MSB, keeping SCLK high
					set_done = 1;							// assert when we're going back to IDLE and shifting final time. 
															// Have to use because shft_cnt will be set to 0 so can't check if equal to 16
					shft = 1;
				end
				else begin
					nxt_state = FINAL;
				end
			end
				
		default: nxt_state = IDLE;
	
		endcase
	
	end

endmodule
