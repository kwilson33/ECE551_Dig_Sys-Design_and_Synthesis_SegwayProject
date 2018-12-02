module SegwayModel(clk,RST_n,SS_n,SCLK,MISO,MOSI,INT,PWM_rev_rght,PWM_frwrd_rght,
                   PWM_rev_lft,PWM_frwrd_lft,rider_lean);
  //////////////////////////////////////////////////
  // Model of physics of the Segway, including a //
  // model of the inertial sensor.              //
  ///////////////////////////////////////////////
  // This code is a terrible hack.  Remember  //
  // do as I say, not as I do.  I am the     //
  // instructor and I judge you.  Please    //
  // don't judge me.                       //
  //////////////////////////////////////////

  input clk;				// same 50MHz clock you give to Segway.v
  input RST_n;				// unsynchronized raw reset input
  input SS_n;				// active low slave select to inertial sensor
  input SCLK;				// Serial clock
  input MOSI;				// serial data in from master
  input PWM_frwrd_rght,PWM_rev_rght;
  input PWM_frwrd_lft,PWM_rev_lft;
  input signed [15:0] rider_lean;		// represents rider's forward/backward 
										// Don't exceed 0x1FFF postitive or 0xE000 negative

  
  output MISO;				// serial data out to master
  output reg INT;			// interrupt output, goes high when inertial sensor has data
  
  localparam MIN_DUTY = 15'h03D4;		// defines zone of duties that give little torque
  localparam TORQUE_GAIN = 4'hE;		// gain is (14/2)X what it is in the dead zone
  localparam FRICTION_TORQUE = 10'h380;	// torque in wheels needed to overcome friction (start rolling)
  localparam PTCH_RT_OFFSET = 16'h03C2;		// offset of this inertial sensor for pitch rate
  localparam AZ_OFFSET = 16'hFE80;
  
  typedef enum reg[1:0] {IDLE,HALF1,HALF2} state_t;
  ////////////////////////////////////////////////////////////////////
  // Registers needed in modeling of inertial sensor declared next //
  //////////////////////////////////////////////////////////////////
  state_t state,nstate;
  reg [15:0] shft_reg_tx;	// SPI shift register for transmitted data (falling edge)
  reg [15:0] shft_reg_rx;	// SPI shift register for received data (rising edge)
  reg [3:0] bit_cnt;		// Needed to know when to interpret R/Wn and address for tx_data
  reg write_reg;			// Used as sentinel to mark that command is write, so write is 
                            // completed at end of transaction
  reg POR_n;				// Power On Reset active low
  //////// Need array to hold all possible registers of iNEMO ////////
  reg [7:0]registers[0:127];
  reg internal_clk;				// 12.5MHz (represents internal clock of inertial sensor)
  reg [10:0] update_period;		// around 6100Hz (which is much faster than actual rate)
  reg clr_INT;
  
  //////////////////////////////////////////////////
  // Registers needed to capture PWM duty cycles //
  ////////////////////////////////////////////////
  reg PWM_frwrd_rght_ff1, PWM_rev_rght_ff1;			// Used for edge detection of PWM signals
  reg PWM_frwrd_lft_ff1, PWM_rev_lft_ff1;			// Used for edge detection of PWM signals
  reg [10:0] lft_rev_cnt,lft_frwrd_cnt;				// Used to measure PWM duty cycle
  reg [10:0] rght_rev_cnt,rght_frwrd_cnt;			// Used to measure PWM duty cycle
  reg signed [11:0] lft_duty;						// signed representation of drive to left motor
  reg signed [11:0] rght_duty;						// signed representation of drive to right motor
  reg rst_n, rst_synch;								// used to synchronize reset
  reg [11:0] time_since_rise_lft;					// used to detect zero duty cycle
  reg [11:0] time_since_rise_rght;					// used to detect zero duty cycle
  reg [11:0] time_since_calc;

  //////////////////////////////////////////////////////
  // Registers needed for modeling physics of Segway //
  ////////////////////////////////////////////////////
  reg signed [15:0] az;									// computed based on PWM inputs over time (angle of platform)
  reg signed [15:0] torque_lft,torque_rght;				// torque applied to motor, computed from PWM duty
  reg any_are_one_ff1;									// used to detect last_fall of any PWM signal
  reg calc_physics;										// delayed version of last_fall
  reg signed [15:0] net_torque;							// torque of rider lean minus motor torque
  reg signed [15:0] omega_lft,omega_rght;				// angular velocities of wheels
  reg signed [19:0] theta_lft,theta_rght;				// amount wheels have rotated since start
  reg signed [15:0] omega_platform;						// angular velocity of platform
  reg signed [15:0] theta_platform;						// angular position of platform
  
  /////////////////////////////////////////////
  // SM outputs declared as type logic next //
  ///////////////////////////////////////////
  logic ld_tx_reg, shft_tx, init;
  logic set_write,clr_write;
  logic [7:0] tx_data;
  
  wire NEMO_setup;		// once registers setup it will start the measurement cycle of inertial sensor
  wire lft_rise,rght_rise					;		// +edge detect on PWM signals
  wire lft_fall, rght_fall;							// -edge detection on PWM signals
  wire any_are_one, last_fall;						// used to detect last_fall which is when calcs are done
  wire signed [15:0] ptch_rate;
  wire signed [19:0] temp20;
	
  //////////////////////////////////////////////
  // Next is modeling of the inertial sensor //
  ////////////////////////////////////////////  
  
  //// Infer main SPI shift register ////
  always_ff @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  shft_reg_tx <= 16'h0000;
	else if (init)
	  shft_reg_tx <= 16'h0000;
	else if (ld_tx_reg)						// occurs at beginning and middle of 16-bit transaction
	  shft_reg_tx <= {tx_data,8'h00};
	else if (shft_tx)
	  shft_reg_tx <= {shft_reg_tx[14:0],1'b0};

  //// Infer main SPI shift register ////
  always_ff @(posedge SCLK, negedge POR_n)
    if (!POR_n)
	  shft_reg_rx <= 16'h0000;
	else if (!SS_n)
	  shft_reg_rx <= {shft_reg_rx[14:0],MOSI};
	  
  always_ff @(negedge SCLK)
    if (init)
	  bit_cnt <= 4'b0000;
	else if (shft_tx)
	  bit_cnt <= bit_cnt + 1;
	  
  always_ff @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  write_reg <= 1'b0;
	else if (set_write)
	  write_reg <= 1'b1;
	else if (write_reg)		// can only be high for one SCLK period
	  write_reg <= 1'b0;
	 
  ///////////////////////////////////////////////////
  // At end of SPI transaction, if it was a write //
  // the register being written is updated       //
  ////////////////////////////////////////////////
  always_ff @(posedge SS_n)
    if (write_reg)
      registers[shft_reg_rx[14:8]] <= shft_reg_rx[7:0];
	
  //////////////////////////////////////////////////
  // model update_period for ODR of inert sensor //
  ////////////////////////////////////////////////
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  update_period <= 11'h0000;
	else if (NEMO_setup)
	  update_period <= update_period + 1;
	
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  INT <= 1'b0;
	else if (clr_INT)
	  INT <= 1'b0;
	else if (&update_period)
	  INT <= 1'b1;
	
  //// Infer state register next ////
  always @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  state <= IDLE;
	else
	  state <= nstate;

  ///////////////////////////////////////
  // Implement state transition logic //
  /////////////////////////////////////
  always_comb
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
	  ld_tx_reg = 0;
      shft_tx = 0;
      init = 0;
	  tx_data = 16'h0000;
	  set_write = 0;
      nstate = IDLE;	  

      case (state)
        IDLE : begin
          if (!SS_n) begin
		    init = 1;
            nstate = HALF1;
          end
        end
		HALF1 : begin
		  shft_tx = 1;
		  if (bit_cnt==4'b0111) begin
		    ld_tx_reg = 1;
			tx_data = response(shft_reg_rx[7:0]);		// response if function of first 8-bits received
		    nstate = HALF2;
	      end else
		    nstate = HALF1;
		end
		HALF2 : begin
		  shft_tx = 1;		
		  if (bit_cnt==4'b1110) begin
		    set_write = ~shft_reg_rx[14];				// if it is a write set the write sentinel
		    nstate = IDLE;
		  end else
		    nstate = HALF2;
		end
      endcase
    end
	
  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n) ? 1'bz : shft_reg_tx[15];

  //////////////////////////////////////////
  // Next is capture of PWM pulse widths //
  ////////////////////////////////////////
  assign any_are_one = PWM_frwrd_lft | PWM_rev_lft | PWM_frwrd_rght | PWM_rev_rght;
  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  PWM_frwrd_lft_ff1 <= 1'b0;
	  PWM_rev_lft_ff1 <= 1'b0;
	  PWM_frwrd_rght_ff1 <= 1'b0;
	  PWM_rev_rght_ff1 <= 1'b0;
	  any_are_one_ff1 <= 1'b0;
	end else begin
	  PWM_frwrd_lft_ff1 <= PWM_frwrd_lft;
	  PWM_rev_lft_ff1 <= PWM_rev_lft;
	  PWM_frwrd_rght_ff1 <= PWM_frwrd_rght;
	  PWM_rev_rght_ff1 <= PWM_rev_rght;
	  any_are_one_ff1 <= any_are_one;
	end
	
  assign lft_rise = (PWM_frwrd_lft & ~PWM_frwrd_lft_ff1) | (PWM_rev_lft & ~PWM_rev_lft_ff1);
  assign rght_rise = (PWM_frwrd_rght & ~PWM_frwrd_rght_ff1) | (PWM_rev_rght & ~PWM_rev_rght_ff1);
	
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  lft_frwrd_cnt <= 11'h000;
	  lft_rev_cnt <= 11'h000;
	end else if (lft_rise) begin
	  lft_frwrd_cnt <= 11'h001;
	  lft_rev_cnt <= 11'h001;
	end else begin
	  lft_frwrd_cnt <= (PWM_frwrd_lft) ? lft_frwrd_cnt + 1 : lft_frwrd_cnt;
	  lft_rev_cnt <= (PWM_rev_lft) ? lft_rev_cnt + 1 : lft_rev_cnt;
	end
	
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  rght_frwrd_cnt <= 11'h000;
	  rght_rev_cnt <= 11'h000;
	end else if (rght_rise) begin
	  rght_frwrd_cnt <= 11'h001;
	  rght_rev_cnt <= 11'h001;
	end else begin
	  rght_frwrd_cnt <= (PWM_frwrd_rght) ? rght_frwrd_cnt + 1 : rght_frwrd_cnt;
	  rght_rev_cnt <= (PWM_rev_rght) ? rght_rev_cnt + 1 : rght_rev_cnt;
	end
	
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  time_since_rise_lft <= 1'b0;
	else if (lft_rise)
	  time_since_rise_lft <= 1'b0;
	else
	  time_since_rise_lft <= time_since_rise_lft + 1;
	  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  time_since_rise_rght <= 1'b0;
	else if (lft_rise)
	  time_since_rise_rght <= 1'b0;
	else
	  time_since_rise_rght <= time_since_rise_lft + 1;	

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  time_since_calc <= 1'b0;
	else if (calc_physics || time_since_calc[11])
	  time_since_calc <= 1'b0;
	else
	  time_since_calc <= time_since_calc + 1;	  
	  
  assign lft_fall = (~PWM_frwrd_lft & PWM_frwrd_lft_ff1) | (~PWM_rev_lft & PWM_rev_lft_ff1);
  assign rght_fall = (~PWM_frwrd_rght & PWM_frwrd_rght_ff1) | (~PWM_rev_rght & PWM_rev_rght_ff1);
  assign last_fall = ~any_are_one & any_are_one_ff1;
  
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  lft_duty <= 12'h000;
	else if (time_since_rise_lft[11])
	  lft_duty <= 1'b0;
	else if (lft_fall)
	  lft_duty <= {1'b0,lft_frwrd_cnt} -  {1'b0,lft_rev_cnt};

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  rght_duty <= 12'h000;
	else if (time_since_rise_rght[11])
	  rght_duty <= 1'b0;
	else if (rght_fall)
	  rght_duty <= {1'b0,rght_frwrd_cnt} - {1'b0,rght_rev_cnt};	 

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      calc_physics <= 1'b0;
    else if (time_since_calc[11])
	  calc_physics <= 1'b1;
	else
      calc_physics <= last_fall;
	  
 
  /////////////////////////////////////////
  // Next is modeling physics of Segway //
  ///////////////////////////////////////
  always @(posedge calc_physics) begin
    torque_lft = torque(lft_duty);
	torque_rght = torque(rght_duty);
	omega_lft = omega(omega_lft,torque_lft);		// angular velocity is integral of torque
	omega_rght = omega(omega_rght,torque_rght);		// angular velocity is integral of torque
	theta_lft = theta(theta_lft,omega_lft);			// wheel theta is integral of wheel omega
	theta_rght = theta(theta_rght,omega_rght);		// wheel theta is integral of wheel omega
	net_torque = rider_lean - torque_lft - torque_rght;		// net torque on segway
	omega_platform = omega_plat(omega_platform,net_torque);	// anglular velocity of platform is integral of net_torque
	theta_platform = theta_plat(theta_platform,omega_platform); // angle of platform is integral of angular velocity
	az = {{3{theta_platform[15]}},theta_platform[15:3]} + AZ_OFFSET;	// calculate AZ from angular position
  end
	
  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n) ? 1'bz : shft_reg_tx[15];

  initial begin
    POR_n = 0;
	clr_INT = 0;
	internal_clk = 0;
	omega_lft = 16'h0000;
	omega_rght = 16'h0000;
	theta_lft = 20'h00000;
	theta_rght = 20'h00000;
	omega_platform = 16'h0000;
	theta_platform = 16'h0000;
	az = 16'h0000;
	#10;
	POR_n = 1;
  end
  
  always
    #40 internal_clk = ~internal_clk;	// generate 12.5MHz internal clock
  
  //// Following 2 lines are a discovered fudge factor to get ////
  //// ptch_acc matching ptch_gyro better ///
  assign temp20 = omega_platform*$signed(5'd12);
  assign ptch_rate = temp20[19:4] + PTCH_RT_OFFSET;
  
  function [7:0] response (input [7:0] in_byte);
    if (in_byte[7])	begin		// if it is a read respond with requested register
	  case (in_byte[6:0])
	    7'h22 : begin response = ptch_rate[7:0]; clr_INT=1; end
		7'h23 : begin response = ptch_rate[15:8]; clr_INT=0; end
	    7'h2C : response = az[7:0];
		7'h2D : response = az[15:8];
	    default : response = registers[in_byte[6:0]];	// case it is just a generic register
	  endcase
	end else					// it is a write
	  response = 8'hA5;			// respond with 0xA5
  endfunction
  
  assign NEMO_setup = ((registers[7'h0d]===8'h02) && (registers[7'h11]===8'h50)) ? 1'b1 : 1'b0;
  

  
  //////////////////////////////////////////////////////
  // functions used in "physics" computations follow //
  ////////////////////////////////////////////////////
  function signed [15:0] torque (input [11:0] duty);
    reg [11:0] duty_abs;
	reg [15:0] torque_abs;

    duty_abs = (duty[11]) ? -duty : duty;
	
	if (duty_abs>MIN_DUTY)
	  torque_abs = TORQUE_GAIN*(duty_abs-MIN_DUTY) + MIN_DUTY;
	else
	  torque_abs = duty_abs;
	  //torque_abs = {duty_abs[10:0],1'b0};		// gain in dead zone is 2.

	torque = (duty[11])? -torque_abs[15:1] : torque_abs[15:1];
  endfunction
  
  function signed [15:0] omega (input signed [15:0] omega1,torque);
    reg [15:0] torque_abs;
	reg [15:0] omega_abs;

	reg [15:0] friction;
	
	friction = (|omega1[15:10]) ? {{10{omega1[15]}},omega1[15:10]} :
	           (|omega1[9:0]) ? 16'h0001 : 16'h0000;
	
	if ((torque_abs<FRICTION_TORQUE) && (omega_abs<16'h0030))
	  omega = 16'h0000;
	else
	  omega = omega1 + {{6{torque[15]}},torque[15:6]} - friction;
  endfunction
  
  function signed [19:0] theta (input signed [19:0] theta1, input signed [15:0] omega);

	theta = theta1 + {{11{omega[15]}},omega[15:7]};
	
  endfunction
  
  function signed [15:0] omega_plat (input signed [15:0] omega1,torque);
	
	reg [15:0] friction;
	
	friction = (|omega1[15:10]) ? {{10{omega1[15]}},omega1[15:10]} :
	           (|omega1[9:0]) ? 16'h0001 : 16'h0000;
			   
	omega_plat = omega1 - {{3{torque[15]}},torque[15:3]} - friction;
	  
  endfunction
  
  function signed [15:0] theta_plat (input signed [15:0] theta1,omega);
	
	theta_plat = theta1 - {{6{omega[15]}},omega[15:6]};
	  
  endfunction
  
  function [15:0] satSum16 (input [15:0] in1,in2);
    reg [15:0] simp_sum;

    simp_sum = in1 + in2;
    if (in1[15] && in2[15] && !simp_sum[15])
	satSum16 = 16'h8000;
    else if (!in1[15] && !in2[15] && simp_sum[15])
	satSum16 = 16'h7FFF;
    else
	satSum16 = simp_sum;
  endfunction
  
  ////////////////////////////////////////////////
  // synchronize raw RST_n input to form RST_n //
  //////////////////////////////////////////////
  always_ff @(negedge clk, negedge RST_n)
    if (!RST_n) begin
	  rst_synch <= 1'b0;
	  rst_n <= 1'b0;
	end else begin
	  rst_synch <= 1'b1;
	  rst_n <= rst_synch;
	end
  
endmodule  
  
