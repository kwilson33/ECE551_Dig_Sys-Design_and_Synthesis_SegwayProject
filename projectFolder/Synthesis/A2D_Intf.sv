//Kevin Wilson 11/9/18
//A2D interface
module A2D_Intf(clk,rst_n, nxt, lft_ld, rght_ld, batt, SS_n_A2D, SCLK_A2D, MOSI_A2D, MISO_A2D);

	// We use 3 channels of the AD converter for our segway
	// Channel 0 -> lft_ld
	// Channel 4 -> rght_ld
	// Channel 5 -> battery voltage
	
	localparam CHAN0 = 3'b000;
	localparam CHAN4 = 3'b100;
	localparam CHAN5 = 3'b101;
	
	// Clock and Reset
	input clk, rst_n; 				// 50 Mhz clock & active low asynch reset
	
	// signals for AD converter
	output logic [11:0] lft_ld;    	// result of last conversion of channel 0
	output logic [11:0] rght_ld;    // result of last conversion of channel 4
	output logic [11:0] batt;    	// result of last conversion of channel 5
	input nxt;						// initiates A2D conversion on next measured
	
	
	// signals for SPI interface that are inputs and outputs to AD converter
	input logic MISO_A2D; 				// MISO: Master In Slave Out (intertial sensor drives this back to us)
	output logic SS_n_A2D;				// active low slave select
	output logic SCLK_A2D;				// 1/32 of 50MHz clock, comes from MSB of a 5 bit counter running off clk
	output logic MOSI_A2D;				// MOSI : Master Out Slave In (we are its master and we drive it)
	
	// internal signals for the SPI sim:/Segway_tb/#INITIAL#56
	logic [15:0] rd_data;			// Data from SPI slave, use [11:0] for A/D
	logic [15:0] cmd;				// data being sent to intertial sensor
	logic wrt;						// high for 1 clock period, initates SPI transaction
	logic done;						// asserted when SPI transaction is complete.			

	// additional signals needed
	logic en_0, en_4, en_5; 		// enable signals needed for the 3 holding registers
	logic [2:0] channel ;			// channel comes from the round robin counter
	logic [1:0] round_robin_cnt ;	// counter for round robin counter
	logic update;					// asserted when both transactions complete
	
	
	// continuous assign for the cmd and enable signals
	assign cmd = {2'b00, channel[2:0], 11'h000}; 		// the cmd is specified by what the channel is
									
	assign en_0 = (update) && (channel == 3'b000);		// enable the registers to receive rd_data depending on what channel is
	assign en_4 = (update) && (channel == 3'b100);
	assign en_5 = (update) && (channel == 3'b101);
	
	// Instantiate SPI for use in AD converter. Performs transactions nearly back to back
	// First transaction tell the AD what channel to convert
	// Second transaction we read the channel back
	SPI_mstr16 SPI_Interface(.clk(clk), .rst_n(rst_n), .rd_data(rd_data), .SS_n(SS_n_A2D), .SCLK(SCLK_A2D), 
							 .MOSI(MOSI_A2D), .MISO(MISO_A2D), .wrt(wrt), .cmd(cmd), .done(done));
							 
	
		
		// choose what channel is depending on round robin counter
		always_comb begin
			case (round_robin_cnt) 
				2'b00 : channel = CHAN0;
				2'b01 : channel = CHAN4;
				2'b10 : channel = CHAN5;
				default : channel = CHAN0;
			endcase
		end
		
		
		//Round robin counter, incremented when update is asserted
	 always_ff @(posedge clk, negedge rst_n) begin
		 if (!rst_n) round_robin_cnt <= 0;
		 else if(update) begin
		 	if(round_robin_cnt == 2'b10) round_robin_cnt <= 2'b00;
		   	else round_robin_cnt <= round_robin_cnt + 1;
		 end
		end
			
	
		
		// registers for the 3 A2D readings
		always_ff @(posedge clk) begin
			if (en_0) lft_ld <= rd_data[11:0];	
		end
		
		always_ff @(posedge clk) begin
			if (en_4) rght_ld <= rd_data[11:0];	
		end
		
		always_ff @(posedge clk) begin
			if (en_5) batt <= rd_data[11:0];	
		end
		
			
		
	
	
	
	//########################################State Machine ############################################
	typedef enum reg [1:0]{IDLE,SPI_1,WAIT,SPI_2} state_t;
	state_t state,nxt_state;

	// Sequential Flop logic
	always_ff @(posedge clk,negedge rst_n) begin
		if (!rst_n) state <= IDLE;
		else state <= nxt_state;
	end

	// Combinational Flop logic
	always_comb begin
		nxt_state = IDLE;
		wrt = 0;							// wrt is initially 0 until transaction starts
		update = 0;							// update is 0 until both transactions done

		case(state)
			IDLE: begin				 		// Stay in IDLE until told to start transaction
				if (nxt) begin
					wrt = 1;
					nxt_state = SPI_1;
				end
			end
			
			SPI_1: begin					// Wait for first transaction to finish
				if (done) nxt_state = WAIT;
				else nxt_state = SPI_1;
			end
			
			WAIT: begin						// Wait one clock cycle to do another one
				wrt = 1;
				nxt_state = SPI_2;
			end
			
			default: begin					// Wait until both transactions are finished to update 
											// and then update counter
				if (done) begin
					update = 1;
				end
				else nxt_state = SPI_2;
			end
		endcase
	end

endmodule
