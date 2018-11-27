//Kevin W, Shawn Z, Tyler M, Severin D
module intert_intf(clk, rst_n,  vld, ptch, SS_n, SCLK, MOSI, MISO, INT);
	
	input clk, rst_n; 					// 50MHz clock, active low reset
	input  INT;							// Interrupt signal from intertial sensor, informs new measurement ready to be read
	
	output logic vld;					// Asserted from SM. Consumed in intertial_integrator. Also used in balance_ctrl
	output [15:0] ptch;					// Primary output. Fusion corrected ptch from Segway -
	
	
	//SPI interface to inertial sensor
	input MISO;
	output SS_n, SCLK, MOSI; 
	
	
	
	
	output logic C_P_L,C_P_H,C_AZ_L,C_AZ_H;//determines when to store
					 //readings of pitchL, pitchH,
					 //AZL, and AZH respectively

	logic [15:0]timer;
	logic INT_ff1,INT_ff2;

	//// Typical SM initialization ////
	typedef enum reg [3:0]{INIT1,INIT2,INIT3,INIT4,WAIT,pitchL,pitchH,AZL,AZH} state_t;
	state_t state,nxt_state;

	always_ff @(posedge clk,negedge rst_n)
		if (!rst_n)
			state <= INIT1;
		else
			state <= nxt_state;
	///////////////////////////////////

	always_comb begin
		nxt_state = INIT1;
		wrt = 0;		//defaulting values
		vld = 0;		//to avoid unintended
		cmd = 16'h0000;	//latches
		C_P_L = 0;
		C_P_H = 0;
		C_AZ_L = 0;
		C_AZ_H = 0;

	case (state)
		//We start with a round of data writes to some
		//registers to configure the inertial sensor to
		//operate in the mode we wish.
		INIT1: begin
		//Enable interrupt upon data ready
		cmd = 16'h0D02;
		//if timer or done is full, we write the command
		if (&timer) begin
		wrt = 1;
		nxt_state = INIT2;
		end
		end
		INIT2: begin
		//Setup accel for 208Hz data rate, +/- 2g accel
		//range, 50Hz LPF
		cmd = 16'h1053;
		if (done) begin
		wrt = 1;
		nxt_state = INIT3;
		end
		else
		nxt_state = INIT2;
		end
		INIT3: begin
		//Setup gyro for 208Hz data rate, +/- 245 degrees/sec range
		cmd = 16'h1150;
		if (done) begin
		wrt = 1;
		nxt_state = INIT4;
		end
		else
		nxt_state = INIT3;
		end
		INIT4: begin
		//Turn rounding on for both accel and gyro
		cmd = 16'h1460;
		if (done) begin
		wrt = 1;
		nxt_state = WAIT;
		end
		else
		nxt_state = INIT4;
		end
		//Now we're at the point in which we've completed initializing
		//the sensor and we go into an infinite loop of reading gyro
		//and accel data.
		WAIT:
		if (INT_ff2)
		nxt_state = pitchL;
		else
		nxt_state = WAIT;
		//pitch L state: pitch rate low
		pitchL: begin
		//Read and store pitchL from gyro
		cmd = 16'hA2xx;
		//once done, we store the reading obtained from
		//the inertial sensor
		if (done) begin
		C_P_L = 1; //storing pitchL reading
		nxt_state = pitchH;
		end
		else
		nxt_state = pitchL;
		end
		//pitch H state: pitch rate high
		pitchH: begin
		//Read and store pitchH from gyro
		cmd = 16'hA23xx;
		if (done) begin
		C_P_H = 1;
		nxt_state = AZL;
		end
		else
		nxt_state = pitchH;
		end
		//AZL state: acceleration in Z low byte
		AZL: begin
		//Read and store AZL from accel
		cmd = 16'hACxx;
		if (done) begin
		C_AZ_L = 1;
		nxt_state = AZH;
		end
		else
		nxt_state = AZL;
		end
		//AZH state: acceleration in Z high byte
		default: begin
		//Read and store AZH from acccel and then
		//indicate to inertial integrator that valid
		//readings are ready.
		cmd = 16'hADxx;
		if (done) begin
		C_AZ_H = 1;
		vld = 1;
		nxt_state = WAIT;
		end
		else
		nxt_state = AZH;
		end
	endcase
	end

	//Simple timer used for the first sensor initializiation.
	//Even though this is only used that one time, it's necessary
	//to have this because we can't simply use done on the first read.
	//The sensor has to wake up and get acquainted before we can start
	//running it off of the done input.
	always_ff @(posedge clk,negedge rst_n)
		if (!rst_n)
		timer <= 16'h0000;
		else
		timer <= timer + 1;

	//double-flopping the INT input to prevent meta-stability
	always_ff @(posedge clk,negedge rst_n)
		if (!rst_n)
		INT_ff2 <= 0;
		else begin
		INT_ff1 <= INT;
		INT_ff2 <= INT_ff1;
		end
	




endmodule	