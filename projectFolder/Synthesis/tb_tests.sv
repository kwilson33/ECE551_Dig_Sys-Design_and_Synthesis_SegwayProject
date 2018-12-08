//`include "tb_tasks.sv"	
task test1;
    Initialize();
    batt = BATT_THRESHOLD + 1;
    lft_ld = MIN_RIDER_WEIGHT;
    rght_ld = MIN_RIDER_WEIGHT;
    rider_lean = 0;
    SendCmd(8'h67);
    repeat(100000)@(posedge clk);
    @(negedge clk);
    rider_lean = 14'h1fff;
    repeat(1000000)@(posedge clk);
    @(negedge clk);
    rider_lean = 14'h0;
    repeat(1000000)@(posedge clk);
    $stop();
    
endtask


task test2;


endtask



task test3;


endtask


