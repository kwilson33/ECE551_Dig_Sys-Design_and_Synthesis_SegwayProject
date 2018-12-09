module ADC128S(clk,rst_n, lft_ld, rght_ld, batt, SS_n,SCLK,MISO,MOSI);
  //////////////////////////////////////////////////|
  // Model of a National Semi Conductor ADC128S    ||
  // 12-bit A2D converter.  NOTE: this model       ||
  // is used in the "Segway" project and gives a   ||
  // warning for any channels other than 0,4,5.    ||
  // The first two readings will return 0xC00, the ||
  // next two will return 0xBF0, next two after    ||
  //  that return 0xBE...                          ||
  ///////////////////////////////////////////////////

  input clk,rst_n;		// clock and active low asynch reset
  input SS_n;			// active low slave select
  input SCLK;			// Serial clock
  input MOSI;			// serial data in from master
  input [11:0] lft_ld, rght_ld, batt;
  
  output MISO;			// serial data out to master
  
  wire [15:0] A2D_data,cmd;
  wire rdy_rise;
	
  typedef enum reg {FIRST,SECOND} state_t;
  
  state_t state,nxt_state;
  
  ///////////////////////////////////////////////
  // Registers needed in design declared next //
  /////////////////////////////////////////////
  reg rdy_ff;				// used for edge detection on rdy
  reg [2:0] channel;		// pointer to last channel specified for A2D conversion to be performed on.
  wire [11:0] value;
  
  /////////////////////////////////////////////
  // SM outputs declared as type logic next //
  ///////////////////////////////////////////
  logic update_ch,dec_value;

  ////////////////////////////////
  // Instantiate SPI interface //
  //////////////////////////////
  SPI_ADC128S iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),
                 .MOSI(MOSI),.A2D_data(A2D_data),.cmd(cmd),.rdy(rdy));

	  
  //// channel pointer ////	  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  channel <= 3'b000;
	else if (update_ch) begin
	  channel <= cmd[13:11];
	  if ((channel!=3'b000) && (channel!=3'b100) && (channel!=3'b101))
	    $display("WARNING: Only channels 0,4,5 of A2D valid for this version of ADC128S\n");
	end
	

  assign value = channel[2]? (channel[0]? batt : rght_ld) : lft_ld;


  //// Infer state register next ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  state <= FIRST;
	else
	  state <= nxt_state;
	  
  //// positive edge detection on rdy ////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  rdy_ff <= 1'b0;
	else
	  rdy_ff <= rdy;
  assign rdy_rise = rdy & ~rdy_ff;
  

  //////////////////////////////////////
  // Implement state tranisiton logic //
  /////////////////////////////////////
  always_comb
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
      update_ch = 0;
      nxt_state = FIRST;	  

      case (state)
        FIRST : begin
          if (rdy_rise) begin
		    update_ch = 1;
            nxt_state = SECOND;
          end
        end
		default : begin		
		  if (rdy_rise) begin
			nxt_state = FIRST;
		  end else
		    nxt_state = SECOND;
		end
      endcase
    end

  assign A2D_data = {4'b0000,value};

endmodule  
  
