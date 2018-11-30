//Kevin W, Shawn Z, Tyler M, Severin D
module inert_intf(clk, rst_n,  vld, ptch, SS_n, SCLK, MOSI, MISO, INT);
	
	input clk, rst_n; 							// 50MHz clock, active low reset
	input  INT;									// Interrupt signal from intertial sensor, informs new measurement ready to be read
	output logic vld;							// Asserted from SM. Consumed in intertial_integrator. Also used in balance_ctrl
	output [15:0] ptch;							// Primary output. Fusion corrected ptch from Segway -
	
	//SPI interface to inertial sensor
	input MISO;
	output SS_n, SCLK, MOSI; 
	
	//// internal signals ///
	logic [15:0] cmd;
	logic [15:0] rd_data;
	logic [7:0] ptch_rt, AZ;
	logic wrt, done;
	logic C_P_L, C_P_H, C_AZ_L, C_AZ_H; 		// determines when to store readings of pitchL, pitchH, AZL, and AZH respectively
	logic [7:0] pitchL, pitchH, AZL, AZH; 		// the 4 readings we want from the sensor
	logic INT_meta1, INT_meta2;					// double flop INT signal for metastability reasons
	logic [15:0] timer;


	// initialize spi module
	SPI_mstr16 spi(.wrt(wrt), .cmd(cmd), .MISO(MISO), .clk(clk), .rst_n(rst_n),
			.done(done), .rd_data(rd_data), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI));

	// init module for intertial integrator
	inertial_integrator intgr(.clk(clk),.rst_n(rst_n), .vld(vld),
			.ptch_rt({pitchH, pitchL}), .AZ({AZH, AZL}), .ptch(ptch));

	// continuous assign to determine values of 4 readings needed from sensor
	assign pitchL = (C_P_L)? rd_data [7:0] : 0;
	assign pitchH = (C_P_H)? rd_data  [7:0] : 0;
	assign AZL = (C_AZ_L)? rd_data  [7:0] : 0;
	assign AZH = (C_AZ_H)? rd_data  [7:0] : 0;

	
	// double flop INT, metastability
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) begin
			INT_meta1 <= 0;
			INT_meta2 <= 0;
		end 
		else begin
			INT_meta1 <= INT;
			INT_meta2 <= INT_meta1;
		end 
	end



	// 16-bit counter, to wait until sensor woken up 
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) timer <= 0;
		else timer <= timer + 1;
	end
	
	
	// ################################################################### STATE MACHINE #################################################
	
	// states needed for state machine
	typedef enum logic [3:0] {INIT1, INIT2, INIT3, INIT4,
							WAIT, READ1, READ2, READ3, READ4} state_t;
	state_t state, nxt_state;
	
	// sequential state transition logic 
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) state <= INIT1;
		else state <= nxt_state;
	end
  
	
	// Combinational state and output logic
	always_comb begin
		// default outputs to avoid latches
		nxt_state = INIT1;			
		cmd = 0;
		wrt = 0;
		C_P_H = 0;
		C_P_L = 0;
		C_AZ_H = 0;
		C_AZ_L = 0;
		vld = 0;
		
		case (state)
		  //We start with a round of data writes to some
          //registers to configure the inertial sensor to
		  //operate in the mode we wish.
		  INIT1: begin
		    //Enable interrupt upon data ready
			cmd = 16'h0D02;
			if (&timer) begin
				nxt_state = INIT2;
				wrt = 1;
			end 
			else nxt_state = INIT1;
		  end
		  
		  
		  INIT2: begin
		  	//Setup accel for 208Hz data rate, +/- 2g accel
			//range, 50Hz LPF
			cmd = 16'h1053;
			if (done) begin
				nxt_state = INIT3;
				wrt = 1;
			end 
			else nxt_state = INIT2;
		  end
		  
		  
		  INIT3: begin
		    //Setup gyro for 208Hz data rate, +/- 245 degrees/sec range
			cmd = 16'h1150;
			if (done) begin
				nxt_state = INIT4;
				wrt = 1;
			end 
			else nxt_state = INIT3;
		  end
		  
		  
		  INIT4: begin
		    //Turn rounding on for both accel and gyro
			cmd = 16'h1460;
			if (done) begin
				nxt_state = WAIT;
				wrt = 1;
			end 
			else nxt_state = INIT4;
		  end
		  
		  //Now we're at the point in which we've completed initializing
          //the sensor and we go into an infinite loop of reading gyro
          //and accel data.
		  WAIT: begin
			if (done && INT_meta2 == 1) begin
				nxt_state = READ1;
				wrt = 1;
			end 
			else nxt_state = WAIT;
		  end
		  
		  //pitch L state: pitch rate low
		  READ1: begin
		    //Read and store pitchL from gyro
			cmd = 16'hA200;
			if (done) begin
				nxt_state = READ2;
				wrt = 1;
				C_P_L = 1;						//storing pitchL reading
			end
			else nxt_state = READ1;
		  end
		  
		  
		  //pitch H state: pitch rate high
		  READ2: begin
		    //Read and store pitchH from gyro
			cmd = 16'hA300;
			if (done) begin
				nxt_state = READ3;
				wrt = 1;
				C_P_H = 1;
			end
			else nxt_state = READ2;
		  end
		  
		  //AZL state: acceleration in Z low byte
		  READ3: begin
		    //Read and store AZL from accel
			cmd = 16'hAC00;
			if (done) begin
				nxt_state = READ4;
				wrt = 1;
				C_AZ_L = 1;
			end 
			else nxt_state = READ3;
		  end
		  
		  //AZH state: acceleration in Z high byte
		  READ4: begin
		    //Read and store AZH from acccel and then
			//indicate to inertial integrator that valid
			//readings are ready.
			cmd = 16'hAD00;
			if (done) begin
				nxt_state = WAIT;
				wrt = 1;
				C_AZ_H = 1;
				vld = 1;
			end 
			else nxt_state = READ4;
		  end
		  
		  // default case to make sure we end up in beginning state if something goes wrong
		  default: nxt_state = INIT1;

		endcase
	end

endmodule