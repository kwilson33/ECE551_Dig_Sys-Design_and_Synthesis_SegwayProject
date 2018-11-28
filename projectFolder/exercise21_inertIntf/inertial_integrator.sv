//Kevin Wilson, Shaoheng Zhou
module inertial_integrator(clk,rst_n,vld,ptch_rt,AZ,ptch);


	localparam PTCH_RT_OFFSET = 16'h03C2;
	localparam AZ_OFFSET = 16'hFE80;
	localparam fudge_factor = $signed(327);									// used to calculate product of pitch acceleration
	
	//defining module input/output ports
	input clk,rst_n;
	input vld;																// vld: high for single clock cycle when new
	input signed [15:0] ptch_rt; 											// ptch_rt: raw pitch rate from intertial sensor
	input signed [15:0] AZ;													// AZ: acceleration in Z direction. Used to calculate ptch_acc
	
	output signed [15:0]ptch;												// ptch: fully compensated and "fused" pitch

	
	//internal signals
	logic signed [26:0]ptch_int;											// ptch_int: ptch_rt is summed into this
	logic signed [26:0]fusion_ptch_offset;
	logic signed [15:0]ptch_rt_comp,AZ_comp;
	logic signed [25:0]ptch_acc_product;
	logic signed [15:0]ptch_acc;
		
  
  /*
  ptch_acc is sign extended version of ptch_acc_product
  which is calculated from multiplying the AZ_comp ( subtracting a pre-determined offset from AZ)
  by the fudge_factor, allowing for some room of error.
  
  ptch is basically tan of (AZ/AY) where AY is assumed to be 1. These calculations find ptch
  read from the accelerator readings
  */
  assign AZ_comp = AZ - AZ_OFFSET;		
  assign ptch_acc_product = AZ_comp * fudge_factor; 						// 327 is fudge factor
  assign ptch_acc = {{3{ptch_acc_product[25]}},ptch_acc_product[25:13]};  	// pitch angle calculated from accel only
  
  
   /*
   Flop for accumulating ptch_int. Subtract ptch_rt_comp and add whatever fusion_ptch_offset was calculated to be
   if acceleration is more than ptch, then add 512 to ptch_int, else you subtract 		
  */
  assign ptch = ptch_int[26:11];											// divide ptch by 2^11. Compare the accelerator angle to it. This is the integrated angle. 
  assign fusion_ptch_offset = (ptch_acc > ptch) ? 512 : -512;				// find offset by seeing if accelator angle is greater than ptch
  assign ptch_rt_comp = ptch_rt - PTCH_RT_OFFSET;							// how much you change compared to last time
  always_ff @(posedge clk,negedge rst_n) begin
    if (!rst_n)
      ptch_int <= 0;
    else if (vld)
      ptch_int <= ptch_int - {{11{ptch_rt_comp[15]}},ptch_rt_comp} + fusion_ptch_offset;
  end

  

endmodule