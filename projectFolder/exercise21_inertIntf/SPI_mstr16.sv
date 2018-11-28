module SPI_mstr16 (wrt, cmd, MISO, clk, rst_n,
				   done, rd_data, SS_n, SCLK, MOSI);

input [15:0]cmd;		// cmd is data passed to slave
input wrt, MISO, clk, rst_n;	// clock, active_low reset, write signal

// MOSI, MISO are signals between master and slave devices
output logic [15:0]rd_data;				// rd_data is read back
output logic done, SS_n, SCLK, MOSI;	// SS_n select slave device, SCLK is a slow clock, done when shift is done

// rst_cnt to reset sclk counter, init to load data into [15:0]cmd,
// smpl signal when to sample MISO,
// set_done and clr_done signal when we need to set signal done or clear it 
logic rst_cnt, init, smpl, shft, MISO_smpl, set_done, clr_done;

logic SCLK_r, SCLK_f;	// signal when SLCK is about to rise or fall
logic [4:0]sclk_div;	// SLCK divider
logic [3:0]shft_cnt;	// shift counter, counting # of shifts we have done
logic shft_cnt_clr;		// clear shift counter
logic shft_15;			// asserted when 15 shifts have been done

typedef enum logic [1:0] {IDLE, FRNT, SHFT, BAK} state_t;	// define states of FSM
state_t state, nxt_state;

// shift register, heart of spi, to hold the 16-bit value
logic [15:0]shft_reg;
always_ff @(posedge clk)
	if (init)
		shft_reg <= cmd;
	else if (shft) 
		shft_reg <= {shft_reg[14:0], MISO_smpl}; // new smpl shif into least significant bit 
	else shft_reg <= shft_reg;
		
assign MOSI = shft_reg[15];		// most significant bit out to slave
assign rd_data = shft_reg; // data read in is also stored in shift register

// sample MISO at rising edge(smpl is asserted)
always_ff @(posedge clk)
	if (smpl)	// 
		MISO_smpl <= MISO;
	else MISO_smpl <= MISO_smpl;

// sclk counter, a slower clock for shift and sample
always_ff @(posedge clk)
	if (rst_cnt)
		sclk_div <= 5'b10111;	// front porch
	else sclk_div <= sclk_div + 1;	// counting up

assign SCLK = sclk_div[4];	// 32 unit clock, 5 bit num, neg for high, pos for low
assign SCLK_r = (sclk_div == 5'b01111);	// when sclk is about to rise
assign SCLK_f = (sclk_div == 5'b11111); // when sclk is about to fall

// 4 bit counter for 16 shift
always_ff @(posedge clk)
	if (shft_cnt_clr)
		shft_cnt <= 4'h00;
	else if (SCLK_f)
		shft_cnt <= shft_cnt + 1;
	else shft_cnt <= shft_cnt;

assign shft_15 = (shft_cnt == 4'hf);	// 15 shifts have been done

// state transition
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		state <= IDLE;
	else state <= nxt_state;

// FSM transition conditions
always_comb begin
	//default values
	rst_cnt = 0;	// reset 
	init = 0;
	shft_cnt_clr = 0;	// dont clear so far
	shft = 0;
	smpl = 0;
	clr_done = 0;
	set_done = 0;
	
	case (state)
	  IDLE: begin
		rst_cnt = 1;
		if (wrt) begin
			nxt_state = FRNT;	// enter front porch, load data and clear done
			init = 1;
			clr_done = 1;
		end
	  end
	  FRNT: begin
		if (!SCLK) begin
			nxt_state = SHFT;	// when SLCK falls, ready to shift or sample
			shft_cnt_clr = 1;	// clear shift counter
		end
	  end
	  SHFT: begin
		if (shft_15 & SCLK_f) begin	// if 15 shifts have been done
			nxt_state = IDLE;		// and next shift is to happen in a clock, do last shift
			shft = 1;				// and return back to IDLE (stop SCLK)
			rst_cnt = 1;
			set_done = 1;
		end else if (SCLK_r)		// sample at rising edge of SCLK
			smpl = 1;
		else if (SCLK_f)			// shift at falling edge of SCLK
			shft = 1;
	  end
	  default: begin
		rst_cnt = 1;
		if (wrt) begin
			nxt_state = FRNT;	// enter front porch, load data and clear done
			init = 1;
			clr_done = 1;
		end
	  end
	endcase
end

// SS_n is controlled by a pair of done signals
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		SS_n <= 1;
	else if (clr_done)
		SS_n <= 0;
	else if (set_done)
		SS_n <= 1;
	else SS_n <= SS_n;

// done is controlled by a pair of done signals
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		done <= 0;
	else if (clr_done)
		done <= 0;
	else if (set_done)
		done <= 1;
		
endmodule
