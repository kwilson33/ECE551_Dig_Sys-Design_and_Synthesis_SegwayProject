//Kevin Wilson 10/13/18
module mtr_drv (PWM_rev_lft, PWM_frwrd_lft, PWM_rev_rght, PWM_frwrd_rght,
				clk, rst_n, lft_spd, lft_rev, rght_spd, rght_rev);
				
	
	/*
	outputs of the motor drive module. PWM_frwrd_lft will be 
	high often if driving forward left, for example, and PWM_rev_left will be low.
	*/
	output logic PWM_rev_lft, PWM_frwrd_lft, PWM_rev_rght, PWM_frwrd_rght;

	
	//50 MHz clock, active low asynch reset,
	input clk, rst_n;
	
	//if lft_rev is high, left motor is driven in reverse
	input logic lft_rev, rght_rev;
	
	input [10:0] rght_spd, lft_spd; //left and right motor duty cycles
	
	//PWM values for left and right
	wire lftMotor, rghtMotor;
	
	PWM11 lftPWM(.PWM_sig(lftMotor), .clk(clk), .rst_n(rst_n), .duty(lft_spd));
	/*
	if the PWM signal from lftPWM (lftMotor) is high then you want to be driving. 
	To figure out the direction do this: If PWM signal is high and lft_rev is also high, then
	you'd want to be reversing the segway. But if the PWM signal is high and lft_rev is low, that means you want
	to be driving forward. When the PWM signal itself is low, stay still.
	*/
	assign PWM_rev_lft = lftMotor & lft_rev;
	assign PWM_frwrd_lft = lftMotor & ~lft_rev;
	
	
	
	PWM11 rghtPWM(.PWM_sig(rghtMotor), .clk(clk), .rst_n(rst_n), .duty(rght_spd));
	/*
	Same logic as the left motor direction
	*/
	assign PWM_rev_rght = rghtMotor & rght_rev;
	assign PWM_frwrd_rght = rghtMotor & ~rght_rev;
	
	
endmodule