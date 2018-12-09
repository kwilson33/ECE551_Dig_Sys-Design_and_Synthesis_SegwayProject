//`include "tb_tasks.sv"	
task test1;
    $display("starting test 1");
    batt = BATT_THRESHOLD + 5;
    lft_ld = MIN_RIDER_WEIGHT + 6;
    rght_ld = MIN_RIDER_WEIGHT + 7;
    rider_lean = 0;
    SendCmd(8'h67);
    repeat(100000)@(posedge clk);
    @(negedge clk);
    rider_lean = 16'h7fff;
    repeat(1000000)@(posedge clk);
    @(negedge clk);
    rider_lean = 14'h0;
    repeat(1000000)@(posedge clk);
    $stop();
    
endtask


task test2;
	 $display("starting test 2");

    batt = BATT_THRESHOLD + 1;
    lft_ld = MIN_RIDER_WEIGHT;
    rght_ld = MIN_RIDER_WEIGHT;
    rider_lean = 0;
    
    SendCmd(8'h67);
    repeat(200)@(posedge clk);
    
    @(negedge clk);
    lft_ld = MIN_RIDER_WEIGHT - 1;
    rght_ld = MIN_RIDER_WEIGHT - 1;
    repeat(200)@(posedge clk);      // should back to IDLE
    
    @(negedge clk);
    lft_ld = MIN_RIDER_WEIGHT;
    rght_ld = MIN_RIDER_WEIGHT;
    rider_lean = 14'h0;
    SendCmd(8'h67);                 // PWR1
    repeat(300000000)@(posedge clk);  // now should be in PWR2
    
    @(negedge clk);
    rider_lean = 14'h1fff;
    SendCmd(8'h73);
    repeat(200)@(posedge clk);  // TODO, wait long enough to stop?
    
    @(negedge clk);
    rider_lean = 14'h1fff;
    SendCmd(8'h73);
    repeat(200)@(posedge clk);
  
    $stop();


endtask



task test3;
	// initial values, reset, clk starts at 0
		RST_n =  0;
		clk = 0;
		PWM_rev_rght = 0;
		PWM_frwrd_rght = 0;
		PWM_rev_lft = 0;
		PWM_frwrd_lft = 0;
		rider_lean = 14'h0100;
	  //// initial done //////////////////////////
	  
		repeat(2) @(posedge clk);
		@(posedge clk) RST_n = 1;
		
	
		repeat(800000) @(posedge clk);
		$stop;
endtask


