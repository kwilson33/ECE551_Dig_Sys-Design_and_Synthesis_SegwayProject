localparam g = 8'h67;
localparam s = 8'h73;

//test min weight edge case

task test1;

    
    batt_V = BATT_THRESHOLD + 1;
    ld_cell_lft = MIN_RIDER_WEIGHT + 1;
    ld_cell_rght = MIN_RIDER_WEIGHT + 1;
    rider_lean = 0;
    SendCmd(8'h67);
    repeat(100000)@(posedge clk);   // power up, wait for a short time
    @(negedge clk);
    rider_lean = 16'h1fff;          // test most positive lean
    repeat(1000000)@(posedge clk);
    @(negedge clk);
    rider_lean = 16'hE000;          // most negative lean
    repeat(1000000)@(posedge clk);
    $stop();
    
endtask

task test2;

    
    batt_V = BATT_THRESHOLD + 1;
    ld_cell_lft = MIN_RIDER_WEIGHT + 1;
    ld_cell_rght = MIN_RIDER_WEIGHT + 1;
    rider_lean = 0;
    SendCmd(8'h67);
    repeat(100000)@(posedge clk);    // power up, wait for a short time


	 // test left turn with a a lot of weight backwards
    ld_cell_lft = MIN_RIDER_WEIGHT + 1500;
    repeat(1000000)@(posedge clk);
    rider_lean = 16'h1fff;          // test most positive lean while turning left
    repeat(1000000)@(posedge clk);
    rider_lean = 0;
    repeat (1000)@(posedge clk);
    rider_lean = 16'hE000;          // most negative lean while turning left
    repeat(1000000)@(posedge clk);

   
   
    // test right turn
    ld_cell_lft = MIN_RIDER_WEIGHT + 1;
    //rider_lean = 0;
    @(posedge clk);
    @(negedge clk);
    ld_cell_rght = MIN_RIDER_WEIGHT + 1500;
    repeat(1000000)@(posedge clk);
    rider_lean = 16'h1fff;          // test most positive lean while turning right
    repeat(1000000)@(posedge clk);
    rider_lean = 0;
    repeat(1000)@(posedge clk); 
    rider_lean = 16'hE000;          // most negative lean while turning right
    repeat(1000000)@(posedge clk);
    
    $stop();

endtask


task test3;


    
    batt_V = BATT_THRESHOLD + 1;
    ld_cell_lft = MIN_RIDER_WEIGHT + 1;
    ld_cell_rght = MIN_RIDER_WEIGHT + 1;

    rider_lean = 0;
	SendCmd(g);
	repeat(1000000) @(posedge clk);
	
    
    // go to power 1 
    SendCmd(g);
    repeat(1000000)@(posedge clk);		// now should be in PWR1
    @(negedge clk);
    // add self checking 

    // steer critera not met, rider_off is asserted
    ld_cell_lft = 1;
    ld_cell_rght = 2;
    repeat(1000000)@(posedge clk);      // should back to OFF
    @(negedge clk);
    // add self checking 

    // go to PWR1 again
    ld_cell_lft = MIN_RIDER_WEIGHT + 1;
    ld_cell_rght = MIN_RIDER_WEIGHT + 1;
    SendCmd(g);       
    repeat(1000000)@(posedge clk);// now should be in PWR1, again
    
    SendCmd(s);
    repeat(100000)@(posedge clk);// now should be in PWR2

	SendCmd(g);
	repeat(100000)@(posedge clk);// now should be in PWR1

	SendCmd(s);
    repeat(100000)@(posedge clk);// now should be in PWR2

	ld_cell_lft = 1;
    ld_cell_rght = 2;
	repeat(1000000)@(posedge clk);// now should be in OFF

	
	@(posedge clk); @(negedge clk);
	batt_V = BATT_THRESHOLD - 1;
	repeat (100000) @(posedge clk);
    
    $stop();

endtask



