module SPI_ADC128S(clk,rst_n,SS_n,SCLK,MISO,MOSI,A2D_data,cmd,rdy);
  //////////////////////////////////////////////////|
  // Model of a SPI Slave similar to what might be ||
  // found in a National Semi Conductor ADC128S    ||
  // 12-bit A2D converter.  NOTE: this model       ||
  // returns 0xABCD for the first read, and        ||
  // whatever you wrote to it last for subsequent  ||
  // reads.                                        ||
  ///////////////////////////////////////////////////

  input clk,rst_n;			// clock and active low asynch reset
  input SS_n;				// active low slave select
  input SCLK;				// Serial clock
  input MOSI;				// serial data in from master
  input [15:0] A2D_data;	// Data A2D is sending
  
  output MISO;				// serial data out to master
  output reg rdy;			// asserted when a transaction completes
  output [15:0] cmd;		// command to the A2D
  
  typedef enum reg[1:0] {IDLE,SKIP_1st,WAIT_SSn} state_t;
  ///////////////////////////////////////////////
  // Registers needed in design declared next //
  /////////////////////////////////////////////
  state_t state,nstate;
  reg [15:0] shft_reg_tx;	// SPI shift register for transmitted data
  reg [15:0] shft_reg_rx;	// SPI shift register for received data
  reg SCLK_ff1,SCLK_ff2;	// used for falling edge detection of SCLK
  
  /////////////////////////////////////////////
  // SM outputs declared as type logic next //
  ///////////////////////////////////////////
  logic ld_shft_reg, shift_tx, shift_rx, clr_rdy, set_rdy;
  
  wire SCLK_fall,SCLK_rise;
  
  //// Implement falling edge detection of SCLK ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  begin
	    SCLK_ff1 <= 1'b1;
	    SCLK_ff2 <= 1'b1;
	  end
	else
	  begin
	    SCLK_ff1 <= SCLK;
		SCLK_ff2 <= SCLK_ff1;
	  end  
	  
  /////////////////////////////////////////////////////
  // If SCLK_ff2 is still high, but SCLK_ff1 is low //
  // then a negative edge of SCLK has occurred.    //
  //////////////////////////////////////////////////
  assign SCLK_fall = ~SCLK_ff1 & SCLK_ff2;
  assign SCLK_rise = SCLK_ff1 & ~SCLK_ff2;
  
  //// Implement rdy as a set/reset flop ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  rdy <= 1'b0;
	else if (clr_rdy)
	  rdy <= 1'b0;
	else if (set_rdy)
	  rdy <= 1'b1;

  //// Infer main SPI shift register ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  shft_reg_tx <= 16'h0000;
	else if (shift_tx)
	  shft_reg_tx <= {shft_reg_tx[14:0],1'b0};
	else if (clr_rdy)				// will transmit what we received next time
	  shft_reg_tx <= A2D_data;

  //// Infer main SPI shift register ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  shft_reg_rx <= 16'h0000;
	else if (shift_rx)
	  shft_reg_rx <= {shft_reg_rx[14:0],MOSI};	

  
  //// Infer state register next ////
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
	  state <= IDLE;
	else
	  state <= nstate;

  //////////////////////////////////////
  // Implement state tranisiton logic //
  /////////////////////////////////////
  always_comb
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
	  ld_shft_reg = 0;
      shift_tx = 0;
	  shift_rx = 0;
      clr_rdy = 0;
	  set_rdy = 0;
      nstate = IDLE;	  

      case (state)
        IDLE : begin
          if (!SS_n) begin
		    clr_rdy = 1;
            nstate = SKIP_1st;
          end
        end
		SKIP_1st : begin		
		  //// Skip the first SCLK fall ////
		  shift_rx = SCLK_rise;
		  if (SCLK_fall)
		    nstate = WAIT_SSn;
		  else
		    nstate = SKIP_1st;
		end
		WAIT_SSn : begin
		  /////////////////////////////////////
		  // shift on falling edges of SCLK //
		  // and wait for rise of SS_n     //
		  //////////////////////////////////
		  shift_tx = SCLK_fall;
		  shift_rx = SCLK_rise;
		  if (SS_n) begin
		    nstate = IDLE;
			set_rdy = 1;
		  end else
		    nstate = WAIT_SSn;
		end
      endcase
    end

  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n) ? 1'bz : shft_reg_tx[15];

  assign cmd = shft_reg_rx[15:0];
endmodule  
  