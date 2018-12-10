module balance_cntrl_dbg_tb();

reg clk, rst_n;
reg vld_sel,vld_tggle;
reg en_steer;
reg rider_off;
reg [15:0] ptch;
reg [11:0] ld_cell_diff;
reg [10:0] ans;
reg pwr_up;
wire ovr_spd;


///////////////////////////////////////
// Declare wires for outputs of DUT //
/////////////////////////////////////
wire [10:0] lft_spd,rght_spd;
wire lft_rev, rght_rev;
wire vld;

  localparam P_COEFF = 5'h0E;
  localparam D_COEFF = 6'h14;				// D coefficient in PID control = +20 
    
  localparam LOW_TORQUE_BAND = 8'h46;	// LOW_TORQUE_BAND = 5*P_COEFF
  localparam GAIN_MULTIPLIER = 6'h0F;	// GAIN_MULTIPLIER = 1 + (MIN_DUTY/LOW_TORQUE_BAND)
  localparam MIN_DUTY = 15'h03D4;		// minimum duty cycle (stiffen motor and get it ready)
  

initial begin
  //// initialize all inputs to DUT ////
  clk = 0;
  pwr_up =1;
  rst_n = 0;
  ptch = 16'h0000;
  ld_cell_diff = 12'h000;
  vld_sel = 1;		// forces vld high all the time
  rider_off = 0;
  en_steer = 1;
  /// hold reset and deassert at negedge ///
  repeat(2) @(negedge clk);
  rst_n = 1;
  
  
   @(negedge clk);
  if ((lft_spd===11'h000) && (rght_spd===11'h000))
    $display("GOOD1: zero in should give zero out");
  else begin
    $display("ERROR: output should be zero.  ptch_err_sat is and has been zero");
	$stop();
  end

  ptch = 16'h0002;
   @(negedge clk);

  
  if ((lft_spd===(2*(GAIN_MULTIPLIER*(P_COEFF+D_COEFF)))) && (rght_spd===(2*(GAIN_MULTIPLIER*(P_COEFF+D_COEFF)))) && !lft_rev)
    $display("GOOD2: we are in low torque zone with no time to integrate and a D-term of 3x error");
  else begin
    $display("ERROR: Output should be error*(P_COEFF + D_COEFF)*GAIN_MULTIPLIER right now");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h001C   ptch_D_diff = 10'h002    ptch_D_term = 13'h0028   PID_cntrl = 16'h0044");
    $display("  lft_torque = 16'h0044   lft_shaped = 16'h03fc");	
	$stop();
  end
  
  
  ptch = 16'h0000;
  repeat(3) @(negedge clk);		// give time for D queue to clear

  ptch = 16'hfffe;		// reverse error  ptch = -3  
  @(negedge clk);
 
  if ((lft_spd===(2*(GAIN_MULTIPLIER*(P_COEFF+D_COEFF)))) && (rght_spd===(2*(GAIN_MULTIPLIER*(P_COEFF+D_COEFF)))) && lft_rev)
    $display("GOOD3: we are in low torque zone with no time to integrate and a D-term of -3x error");
  else begin
    $display("ERROR: Output should be error*(P_COEFF + D_COEFF)*GAIN_MULTIPLIER and error is negative");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h7fe4   ptch_D_diff = 10'h3fe    ptch_D_term = 13'h1fd8   PID_cntrl = 16'hffbc");
    $display("  lft_torque = 16'hffbc   lft_shaped = 16'hfc04");
	$stop();
  end 
 
  ptch = 16'h0000;
  repeat(3) @(negedge clk);		// give time for D queue to clear
  
  ptch = 6;						// puts us out of low torque zone
  @(negedge clk);

  if ((lft_spd===(6*(P_COEFF+D_COEFF)+MIN_DUTY)) && (rght_spd==lft_spd) && !lft_rev)
    $display("GOOD4: we are out of low torque zone so should be using MIN_DUTY");
  else begin
    $display("ERROR: are you using the high gain result (you should not be)?  what about MIN_DUTY?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h0054   integrator = 18'h00006   ptch_D_diff = 10'h006    ptch_D_term = 13'h0078");
    $display("  PID_cntrl = 16'h00cc   lft_torque = 16'h00cc   lft_shaped = 16'h04a0");
	$stop();
  end  
  
  ptch = 16'h0000;
  repeat(3) @(negedge clk);		// give time for D queue to clear
 
  ptch = -6;					// puts us out of low torque zone
  @(negedge clk);

  if ((lft_spd===(6*(P_COEFF+D_COEFF)+MIN_DUTY)) && (rght_spd==lft_spd) && lft_rev)
    $display("GOOD5: we are out of low torque zone so should be using MIN_DUTY");
  else begin
    $display("ERROR: how did you get this wrong but the previous one right?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h7fac   integrator = 18'h00000   ptch_D_diff = 10'h3fa    ptch_D_term = 13'h1f88");
    $display("  PID_cntrl = 16'hff34   lft_torque = 16'hff34   lft_shaped = 16'hfb60");
	$stop();
  end   
   
  ////////////////////////////////
  // Testing that a steering input gives diff drive
  // and kicks us out of low torque zone
  ////////////////////////////////
  ptch = 16'h0000;
  repeat(3) @(negedge clk);		// give time for D queue to clear
 
  ld_cell_diff = 150;			// significant steering input
  ptch = 16'h0006;
  @(negedge clk);
 
  if ((lft_spd===(6*(P_COEFF+D_COEFF)+MIN_DUTY-ld_cell_diff[11:3])) && 
       (rght_spd===(6*(P_COEFF+D_COEFF)+MIN_DUTY+ld_cell_diff[11:3])) && !lft_rev)
    $display("GOOD6: high steering input kicked us out of low torque zone and affected steering");
  else begin
    $display("ERROR: Check that you subtract ld_cell_diff from left");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h0054   integrator = 18'h00006   ptch_D_diff = 10'h006    ptch_D_term = 13'h0078");
    $display("  PID_cntrl = 16'h00cc   lft_torque = 16'h00ba   rght_torque = 16'h00de   lft_shaped = 16'h048e");
	$stop();
  end
  
 
  ///////////////////////////////////////////////////
  // Checking if error is consistent for > depth of
  // D queue then D queue is out of the picture.
  /////////////////////////////////////////////////
  ld_cell_diff = 12'h000;
  ptch = 16'h0002;
  repeat(2) @(negedge clk);		// let D term zero out since error consistent
  
  if ((lft_spd===(2*P_COEFF*GAIN_MULTIPLIER)) && (rght_spd==lft_spd) && !lft_rev)
    $display("GOOD7: D-term out of picture since error consistent");
  else begin
    $display("ERROR: Is there something wrong with your D-term queue?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h001C   integrator = 18'h0000a   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'h001c   lft_torque = 16'h001c   rght_torque = 16'h001c   lft_shaped = 16'h01a4");
	$stop();
  end
	   
	   
  ///////////////////////////////////////////////////
  // Let the integral term wind up while in the 
  // low torque region.
  /////////////////////////////////////////////////
  ptch = 16'h0002;
  repeat(63) @(negedge clk);		// let integral term wind up
  
  if ((lft_spd===(2*P_COEFF+2)*GAIN_MULTIPLIER) && (rght_spd==lft_spd) && !lft_rev)
    $display("GOOD8: Integral term should be adding 2 to total drive");
  else begin
    $display("ERROR: Is there something wrong your integral term?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h001C   integrator = 18'h00088   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'h001e   lft_torque = 16'h001e   rght_torque = 16'h001e   lft_shaped = 16'h01c2");
	$stop();
  end

  
  ///////////////////////////////////////////////////
  // Change error to negative (-9) so integral term starts
  // marching backwards.  Also get out of low torque region
  /////////////////////////////////////////////////
  ptch = 16'hFFF7;
  repeat(64) @(negedge clk);		// let integral term wind up
  
  ans = 9*P_COEFF+7+MIN_DUTY;
  if ((lft_spd===(9*P_COEFF+7+MIN_DUTY)) && (rght_spd==lft_spd) && lft_rev)
    $display("GOOD9: Integral term should be subtracting 6 from total drive");
  else begin
    $display("ERROR: Was expecting a lft_spd=%h",ans);
	$display("What things should be at this time:");
	$display("  Pterm = 15'h7f82   integrator = 18'h3fe48   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'hff7b   lft_torque = 16'hff7b   rght_torque = 16'hff7b   lft_shaped = 16'hfba7");
	$stop();
  end

		   
  ///////////////////////////////////////////////////
  // check that rider_off clears the integral
  /////////////////////////////////////////////////  
  ptch = 16'h0000;
  rider_off = 1;		// rider_off should clear integral
  @(negedge clk);		// integral should clear
  rider_off = 0;
  if ((lft_spd===(9*D_COEFF+MIN_DUTY)) && (rght_spd==lft_spd) && !lft_rev)
    $display("GOOD10: Integral term should be cleared");
  else begin
    $display("ERROR: Are you clearing the integral term on rider_off?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h0000   integrator = 18'h00000   ptch_D_diff = 10'h009    ptch_D_term = 13'h00b4");
    $display("  PID_cntrl = 16'h00b4   lft_torque = 16'h00b4   rght_torque = 16'h00b4   lft_shaped = 16'h0488");
	$stop();
  end
  
		   
  ///////////////////////////////////////////////////
  // up until this time we had vld=1 all the time
  // now check that they only integrate when vld=1
  /////////////////////////////////////////////////  
  ptch = 16'h0002;
  vld_sel = 0;		// vld will toggle every other cycle now.
  repeat(128) @(negedge clk);		// let integral term wind up
  
  if ((lft_spd===(2*P_COEFF+2)*GAIN_MULTIPLIER) && (rght_spd==lft_spd) && !lft_rev)
    $display("GOOD11: Integral term only integrating on vld.");
  else begin
    $display("ERROR: integral term should be qualified on vld, recirculate otherwise");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h001c   integrator = 18'h00080   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'h001e   lft_torque = 16'h001e   rght_torque = 16'h001e   lft_shaped = 16'h01c2");
	$stop();
  end
 
  
  ///////////////////////////////////////////////////
  // Now check that D queue is only updated on vld
  /////////////////////////////////////////////////  
  ptch = 16'h0000;
  vld_sel = 0;		// vld will toggle every other cycle now.
  rider_off = 1;	// need to clear integral term too
  repeat(4) @(negedge clk);		// clear D queue
  rider_off = 0;
  ptch = 16'h0002;
  repeat(2) @(negedge clk);
  
  if ((lft_spd===(2*(P_COEFF+D_COEFF)*GAIN_MULTIPLIER)) && (rght_spd===rght_spd) && !lft_rev)
    $display("GOOD12: D-term should have effect because there should be a D-diff");
  else begin
    $display("ERROR: D queue should only be updated on vld");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h001c   integrator = 18'h00002   ptch_D_diff = 10'h002    ptch_D_term = 13'h0028");
    $display("  PID_cntrl = 16'h0044   lft_torque = 16'h0044   rght_torque = 16'h0044   lft_shaped = 16'h03fc");
	$stop();
  end

		   
  /////////////////////////////////////////////////////
  // Now check that ptch_err_sat is saturated correct
  /////////////////////////////////////////////////////
  ptch = 16'h0000;
  vld_sel = 1;		// vld will be high always
  rider_off = 1;
  @(negedge clk);		// clearing integrator
  rider_off = 0;
  ptch = 16'h0300;		// larger than 0x1FF
  repeat(2) @(negedge clk);		// integrate it twice
  ptch = 16'h0000;
  repeat (2) @(negedge clk);	// clear D-diff
  
  if ((lft_spd===(15*GAIN_MULTIPLIER)) && (rght_spd==lft_spd) && !lft_rev)
    $display("GOOD13: ptch_err_sat seems saturated positive.");
  else begin
    $display("ERROR: ptch_err_sat saturation seems wrong??");
	$display("What things should be at this time:");
	$display("  ptch_err_sat = should be 10'h000 right now, but should have been 10'h1ff couple cycles prior");
	$stop();
  end
  
  
  ///////////////////////////////////////////////////
  // Now check integrator saturates positive
  /////////////////////////////////////////////////  
  ptch = 16'h01ff;	// large positive
  vld_sel = 1;		// vld will be high always
  repeat(257) @(negedge clk);		// integrator should be saturated positive now
  ptch = 16'h0000;
  repeat(3) @(negedge clk);			// D-term should be zero
  
  if ((lft_spd===11'h7FF) && (rght_spd===rght_spd) && !lft_rev)
    $display("GOOD14: Integrator should be wound up +");
  else begin
    $display("ERROR: are you saturating integrator, do you have 11-bit saturation of output working?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h0000   integrator = 18'h1ff00   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'h07fc   lft_torque = 16'h07fc   rght_torque = 16'h07fc   lft_shaped = 16'h0bd0");
	$stop();
  end  
 
		   
  /////////////////////////////////////////////////////
  // Now check that ptch_err_sat saturates negative correctly
  /////////////////////////////////////////////////////
  ptch = 16'h0000;
  vld_sel = 1;		// vld will be high always
  rider_off = 1;
  @(negedge clk);		// clearing integrator
  rider_off = 0;
  ptch = 16'hFD00;		// more negative than 0x200
  repeat(2) @(negedge clk);		// integrate it twice
  ptch = 16'h0000;
  repeat (2) @(negedge clk);	// clear D-diff
  
  if ((lft_spd===(16*GAIN_MULTIPLIER)) && (rght_spd==lft_spd) && lft_rev)
    $display("GOOD15: ptch_err_sat seems saturated negative.");
  else begin
    $display("ERROR: ptch_err_sat saturation seems wrong??");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h0000   integrator = 18'h3fc00   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'hfff0   lft_torque = 16'hfff0   rght_torque = 16'hfff0   lft_shaped = 16'hff10");
	$stop();
  end
  
		   
  ///////////////////////////////////////////////////
  // Now check integrator saturates negative
  /////////////////////////////////////////////////  
  ptch = 16'hFE01;	// large negative
  repeat(257) @(negedge clk);		// integrator should be saturated negative now
  ptch = 16'h0000;
  repeat(3) @(negedge clk);			// D-term should be zero
  
  if ((lft_spd===11'h7FF) && (rght_spd===rght_spd) && lft_rev)
    $display("GOOD16: Integrator should be wound up -");
  else begin
    $display("ERROR: is your overflow for integrator right?");
	$display("What things should be at this time:");
	$display("  Pterm = 15'h0000   integrator = 18'h200fe   ptch_D_diff = 10'h000    ptch_D_term = 13'h0000");
    $display("  PID_cntrl = 16'hf803   lft_torque = 16'hf803   rght_torque = 16'hf803   lft_shaped = 16'hf42f");
	$stop();
  end  
  
  $display("YAHOO!! all tests passed...run randoms next\n");
  $stop();
	 
end

/////////////////////////////////
// Make a toggling flop for producing
// a vld signal that is high every
// other time
///////////////////////////////
always @(posedge clk, negedge rst_n)
  if (!rst_n)
    vld_tggle <= 1'b0;
  else
    vld_tggle <= ~vld_tggle;
	
assign vld = (vld_sel) ? 1'b1 : vld_tggle;
	
always
  #5 clk = ~clk;

					
//////////////////////
// Instantiate DUT //
////////////////////


balance_cntrl iDUT(.clk(clk),.rst_n(rst_n),.vld(vld),.ptch(ptch),.ld_cell_diff(ld_cell_diff),
				   .lft_spd(lft_spd),.lft_rev(lft_rev),.rght_spd(rght_spd),.rght_rev(rght_rev),
				   .rider_off(rider_off),.en_steer(en_steer), .ovr_spd(ovr_spd), .pwr_up(pwr_up));
		 
endmodule