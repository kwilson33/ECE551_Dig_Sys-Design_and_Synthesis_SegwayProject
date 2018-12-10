
//Initialize all stimulus
task Initialize;
	clk = 0;
	RST_n = 0;
	cmd = 0;
	send_cmd = 0;
	rider_lean = 0;
	ld_cell_lft = 0; ld_cell_rght = 0; batt_V = 0;
	@(posedge clk);
	@(negedge clk);
	//deassert reset
	RST_n = 1;
endtask

//`include "tb_tasks.sv"
/*  SendCmd is going to send 'g' or 's' into Segway.v, mimicing real siganls from usr's bluetooth
   'g' go is 0x67
   's' stop is 0x73
*/
task SendCmd(input byte command);
	@(negedge clk);
	cmd = command;
	send_cmd = 1;
	@(posedge clk);
	send_cmd = 0;
endtask

//easy way to wait number of cycles
task repeatClock(input shortint numClocks);
	repeat(numClocks) @(posedge clk);
endtask
