 //Kevin Wilson and Shaoheng Zhou
module balance_cntrl(clk,rst_n,vld,ptch,ld_cell_diff,lft_spd,lft_rev,
                     rght_spd,rght_rev,rider_off, en_steer, pwr_up, ovr_spd);
					 
	
  parameter fast_sim = 0; 								// defaulted to 0. Speeds up integral term by 16x if enabled
  
  //use these params for saturating  values
  localparam most_neg10b = $signed(10'h200); 			// most negative value in 10 bits
  localparam most_pos10b = $signed(10'h1FF); 			// most positive value in 10 bits signed
  localparam most_neg7b  =  $signed(7'h40);				// most negative value in 7 bits
  localparam most_pos7b  =  $signed(7'h3F);				// most positive value in 7 bits  signed
  localparam most_pos11b_unsigned = 11'h7FF; 			// most pos value in 11 bits unsigned
  
  localparam warningSpeed =11'h600;				// value to compare against to see if speed is too fast, 1536 in decimal. If saturated, speed will be 2047, which will trigger warning for too fast
 
  
  input clk,rst_n;
  input vld;									// tells when a new valid inertial reading ready
  input signed [15:0] ptch;						// actual pitch measured
  input signed [11:0] ld_cell_diff;				// lft_ld - rght_ld from steer_en block
  input rider_off;								// High when weight on load cells indicates no rider
  input en_steer;
  input pwr_up;									// comes from Auth_blk.sv. Enables device. lft and rght speed should be 0 if pwr_up is high
  
  output [10:0] lft_spd;						// 11-bit unsigned speed at which to run left motor
  output lft_rev;								// direction to run left motor (1==>reverse)
  output [10:0] rght_spd;						// 11-bit unsigned speed at which to run right motor
  output rght_rev;								// direction to run right motor (1==>reverse)
  output ovr_spd;								// asserted if lft rght speed are greater than 1536. Goes to piezo interface to warn rider.
  
  
  ////////////////////////////////////
  // Define needed registers below //
  //////////////////////////////////
  reg signed [17:0] integrator;
  reg [9:0] ptch_err_sat_Stored1; //do we have to make these signed?
  reg [9:0] prev_ptch_err ;
  
  ///////////////////////////////////////////
  // Define needed internal signals below //
  /////////////////////////////////////////
  
  
  //General signals
  wire signed [9:0]  ptch_err_sat;
  wire signed [17:0] ptch_err_sat_SignExt;
  
  //Signals needed for getting torque
  wire signed [15:0] PID_cntrl;
  wire signed [15:0] ld_cell_diff_div8_SignExt; 
  wire signed [15:0] lft_torque;
  wire signed [15:0] steerEN_lft_torque;
  wire signed [15:0] rght_torque;
  wire signed [15:0] steerEN_rght_torque;
  
  //Signals needed for shaping torque to get duty
  wire signed [15:0] lft_torque_SignMult;
  wire signed [15:0] rght_torque_SignMult;
  wire signed [15:0] lft_torque_plusMinus_MinDuty;
  wire signed [15:0] rght_torque_plusMinus_MinDuty;
  wire signed [15:0] lft_shaped;
  wire signed [15:0] rght_shaped;
  wire [15:0] abs_lft_torque;
  wire [15:0] abs_rght_torque;
  wire [15:0] abs_lft_shaped;
  wire [15:0] abs_rght_shaped;

  
	
  //P of PID
  wire signed [14:0] ptch_P_term;
  wire signed [15:0] ptch_P_term_SignExt;
  
  //I of PID
  wire signed [11:0] ptch_I_term;
  wire signed [15:0] ptch_I_term_SignExt;
  wire signed [17:0] nxt_integrator ; 
  wire neg_overflow;
  wire pos_overflow;
  wire overflow;
  
  //D of PID
  wire signed [12:0] ptch_D_term; //7 bits * 6 bits from D_COEFF = 13 bits
  wire signed [15:0] ptch_D_term_SignExt;
  wire signed [9:0] ptch_D_diff; //current - prev
  wire signed [6:0] ptch_D_diff_sat;
 
 

  /////////////////////////////////////////////
  // local params for increased flexibility //
  ///////////////////////////////////////////
  localparam P_COEFF = 5'h0E;
  localparam D_COEFF = 6'h14;			// D coefficient in PID control = +20 
    
  localparam LOW_TORQUE_BAND = 8'h46;	// LOW_TORQUE_BAND = 5*P_COEFF
  localparam GAIN_MULTIPLIER = 6'h0F;	// GAIN_MULTIPLIER = 1 + (MIN_DUTY/LOW_TORQUE_BAND)
  localparam MIN_DUTY = 15'h03D4;		// minimum duty cycle (stiffen motor and get it ready)
  

  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //################################################################################Ptch Err & Proportional Math##########################################################################################
  //######################################################################################################################################################################################################
  
  //Saturate ptch
  assign ptch_err_sat = $signed((ptch[15]) ? //check if pos or neg
						(&ptch[15:9] ? ptch[9:0] 	 : most_neg10b) : //if too neg sat to 0x200
						(|ptch[15:9] ? most_pos10b   : ptch[9:0])); //if too pos sat to 0x1FF
  
  //sign extended
  assign ptch_err_sat_SignExt = $signed({{8{ptch_err_sat[9]}}, ptch_err_sat});
  
  //signed multiplication, results in 14 bits
  assign ptch_P_term = ($signed(P_COEFF)) * (ptch_err_sat); //15 bits
  assign ptch_P_term_SignExt = $signed({ptch_P_term[14], ptch_P_term});
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  
  
  
  
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //#########################################################################################Integrator Math##############################################################################################
  //######################################################################################################################################################################################################
  
  //nxt_integrator is the current integator + the sign extended ptch_err
  assign nxt_integrator = integrator + ptch_err_sat_SignExt;

  //overflow logic
  assign neg_overflow = (integrator[17] & ptch_err_sat_SignExt[17])   ?   (~nxt_integrator[17]): 1'b0; //check if both MSBs are 1, if they are and nxt_integrator MSB is a 0, then overflow
  assign pos_overflow = (!integrator[17] & !ptch_err_sat_SignExt[17]) ?   nxt_integrator[17]   : 1'b0; //check if both MSBS are 0, if they are and nxt_integ MSB is 1, then overflow
  assign overflow = (neg_overflow | pos_overflow);

  /* only have to use upper 10 bits to get the I term for PID math
     ensure integral term is reset when ~pwr_up to prevent from winding up 
     if device is powered up  but not authorized to move
  */ 
  assign ptch_I_term = $signed(pwr_up ? (integrator[17:6]) : (12'd0)); //12 bits
  assign ptch_I_term_SignExt = $signed({{4{ptch_I_term[11]}}, ptch_I_term}); 
 
  //Flop that gets integrator, will clear if rider is off or reset is asserted
  always @(posedge clk, negedge rst_n) begin
	if(!rst_n) 
		integrator <= 18'h00000;
	else if (rider_off) 
		integrator <= 18'h00000;
	else if ((!overflow) & vld) //FREEZE and keep current value of integrator if not true
		integrator <= nxt_integrator;
  end
  
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  
  
  
  
  
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //#########################################################################################Derivative Math##############################################################################################
  //######################################################################################################################################################################################################
  
  //Logic to get prev_ptch_err term from two
  //readings ago. Basically double flop
  //the current ptch_err_sat term if valid
  //reading is ready
  always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		ptch_err_sat_Stored1 <= 10'd0;
		prev_ptch_err <= 10'd0;
	end
	else if (vld) begin
		ptch_err_sat_Stored1 <= $signed(ptch_err_sat);
		prev_ptch_err <= ptch_err_sat_Stored1;
	end
  end
  
  //ptch_D_diff = current ptch_err - sample from 2 readings ago
  //saturate to 7 bits
  //scale to a 13 bit number using D_COEFF
  assign ptch_D_diff = $signed(ptch_err_sat - prev_ptch_err);
  
  assign ptch_D_diff_sat = $signed((ptch_D_diff[9]) ? //check if pos or neg
						   (&ptch_D_diff[9:6] ? ptch_D_diff[6:0] : most_neg7b) : //check bits 10 to 7 for zeros, if any exist, saturate to 0x40
						   (|ptch_D_diff[9:6] ? most_pos7b : ptch_D_diff[6:0])); //check bits 10 to 7 for ones, if any exist, saturate to 0x3F
						   
							
  assign ptch_D_term = ($signed(D_COEFF)) * ptch_D_diff_sat; //13 bits
  assign ptch_D_term_SignExt = $signed({{3{ptch_D_term[12]}}, ptch_D_term}); 
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  
  
  
  
  
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //################################################################################Putting it All Together###############################################################################################
  //######################################################################################################################################################################################################
	
	assign ld_cell_diff_div8_SignExt = $signed({{7{ld_cell_diff[11]}}, ld_cell_diff[11:3]}); //getting rid of 3 LSBs, same as dividing by 8
	//either add sped up version of integrator or normal sign extended version based on fast_sim parameter
	assign PID_cntrl = $signed(ptch_P_term_SignExt + ptch_D_term_SignExt + (fast_sim ? ptch_I_term[17:2] : ptch_I_term_SignExt)); //16 bits
	
	//if steering is enabled, get left and right torque by subtracting from/ adding to the PID_cntrl
	//if not, just use the PID cntrl
	assign steerEN_lft_torque = PID_cntrl - ld_cell_diff_div8_SignExt;
	assign steerEN_rght_torque = PID_cntrl + ld_cell_diff_div8_SignExt;
	assign lft_torque = (en_steer) ? steerEN_lft_torque : PID_cntrl;
	assign rght_torque = (en_steer) ? steerEN_rght_torque : PID_cntrl;

  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  
  
  
  
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //################################################################################Shaping Desired Torque to form Duty###################################################################################
  //######################################################################################################################################################################################################
  
  //Left Torque///////
  assign lft_torque_plusMinus_MinDuty = $signed((lft_torque[15]) ? (lft_torque - MIN_DUTY) : (lft_torque + MIN_DUTY)); //add or subtract MIN_DUTY depending on if torque is + or - 
  assign lft_torque_SignMult = ($signed(GAIN_MULTIPLIER)) * lft_torque;
  
  assign abs_lft_torque = $signed(lft_torque[15] ? -lft_torque : lft_torque); //absolute value, if negative take 2s complement 
  
  //lft_shaped torque decided by if lft torque is greater or equal to LOW_TORQUE_BAND
  assign lft_shaped = (abs_lft_torque >= LOW_TORQUE_BAND) ? lft_torque_plusMinus_MinDuty : lft_torque_SignMult;
  assign lft_rev = lft_shaped[15]; //reverse left if 1
  
  //take abs value of lft_shaped and unsigned saturate to 11 bits
  assign abs_lft_shaped = $signed(lft_shaped[15] ? -lft_shaped : lft_shaped);
  //if pwr_up is not asserted, lft_speed is 0
  assign lft_spd = pwr_up ? ((|abs_lft_shaped[15:11]) ? most_pos11b_unsigned : abs_lft_shaped[10:0]) : 11'h000; //check bits 16 to 12 to see if any ones present, if there are, saturate to 0x7FF
  
  
  
  //Right Torque///////
  assign rght_torque_plusMinus_MinDuty = $signed((rght_torque[15]) ? (rght_torque - MIN_DUTY) : (rght_torque + MIN_DUTY)); //add or subtract MIN_DUTY depending on if torque is + or - 
  assign rght_torque_SignMult = ($signed(GAIN_MULTIPLIER)) * rght_torque;
  
  assign abs_rght_torque = $signed(rght_torque[15] ? -rght_torque : rght_torque); //absolute value, if negative take 2s complement 
  
  //rght_shaped torque decided by if rght torque is greater or equal to LOW_TORQUE_BAND
  assign rght_shaped = (abs_rght_torque >= LOW_TORQUE_BAND) ? rght_torque_plusMinus_MinDuty : rght_torque_SignMult;
  assign rght_rev = rght_shaped[15]; //reverse right if 1
  
  //take abs value of rght_shaped and unsigned saturate to 11 bits
  assign abs_rght_shaped = $signed(rght_shaped[15] ? -rght_shaped : rght_shaped);
  // if pwr_up is not asserted, rght_speed is 0
  assign rght_spd = pwr_up ? ((|abs_rght_shaped[15:11]) ? most_pos11b_unsigned : abs_rght_shaped[10:0]) : 11'h000; //check bits 16 to 12 to see if any ones present, if there are, saturate to 0x7FF
  
  
  
  
  //See if either left or right speed are too fast
  assign ovr_spd = ((lft_spd > warningSpeed) || ( rght_spd > warningSpeed));
  
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  //######################################################################################################################################################################################################
  
endmodule 