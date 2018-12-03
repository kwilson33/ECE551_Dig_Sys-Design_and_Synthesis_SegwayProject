module inert_intf_tb();

	logic SS_n, SCLK, MOSI, vld;
	logic [15:0]ptch;
	logic clk, rst_n;
	logic INT, MISO;
	
	logic PWM_rev_rght,PWM_frwrd_rght, PWM_rev_lft,PWM_frwrd_lft;
	logic [13:0]rider_lean;
	
	inert_intf intert_interface(.SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .vld(vld),
					.ptch(ptch), .clk(clk), .rst_n(rst_n), .INT(INT), .MISO(MISO));
	
	SegwayModel segway_model(.clk(clk),.RST_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),
					.MISO(MISO),.MOSI(MOSI),.INT(INT),
					.PWM_rev_rght(PWM_rev_rght),.PWM_frwrd_rght(PWM_frwrd_rght),
                    .PWM_rev_lft(PWM_rev_lft),.PWM_frwrd_lft(PWM_frwrd_lft),
					.rider_lean(rider_lean));

	initial begin
	  // initial values, reset, clk starts at 0
		rst_n = 0;
		clk = 0;
		PWM_rev_rght = 0;
		PWM_frwrd_rght = 0;
		PWM_rev_lft = 0;
		PWM_frwrd_lft = 0;
		rider_lean = 14'h0100;
	  //// initial done //////////////////////////
	  
		repeat(2) @(posedge clk);
		@(posedge clk) rst_n = 1;
		
		//wait until NEMO setup goes high to deassert reset
		/*
		fork
			begin: timeout1
				repeat (800000) @(posedge clk); //if done hasn't been asserted after 1000 clocks something is wrong
				$display("ERROR!");
				$stop();
			end

			begin
				@(posedge segway_model.NEMO_setup)
				disable timeout1;
			end
		join
		*/
	
		repeat(800000) @(posedge clk);
		$display("%h", segway_model.registers[7'h0d]); 
		$stop;
	
	end
	
	
	always #5 clk = ~clk;


endmodule
