module piezo(piezo, piezo_n, batt_low, ovr_spd, norm_mode, clk, rst_n);

	localparam batt_low_thresh = 7'h800;
	
	//calculate count by (50 Mhz/ Desired Freq) 
	// piezo will respond to signals in 300 Hz to 7000 Hz
	localparam norm_mode_max = 50000; // to achieve 1k Hz for norm_mode
	localparam batt_low_max = 25000;  // to achieve 2k Hz for batt_low
	localparam ovr_spd_max = 12500;   // to achieve 4k Hz for ovr_spd_cnt
	

	input clk, rst_n;			// 50 Mhz clock
	input norm_mode;			// tone should occur at least once every 2 seconds, not obnoxious
	input batt_low;				// should be alarming
	input ovr_spd;				// comes from balance_ctrl. Should be able to occur same time as batt_low sound

	reg [15:0] norm_mode_cnt
	reg [14:0] batt_low_cnt
	reg [13:0] ovr_spd_cnt;
	reg norm_mode_out, batt_low_out, ovr_spd_out;
	
	always_ff @(posedge clk, negedge rst_n) begin
		if(rst_n) begin
			norm_mode_cnt <= 0;
		end else if(!norm_mode) begin
			norm_mode_cnt <= 0;
		end else begin 
			norm_mode_cnt <= norm_mode_cnt + 1;
		end	
	end
	
	
	
	


endmodule