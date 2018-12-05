module tb_tasks();
	//Initialize all stimulus
	task Initialize();
		clk = 0;
		RST_n = 0;
		cmd = 0;
		send_cmd = 0;
		rider_lean = 0;
		lft_ld = 0; rght_ld = 0; batt = 0;
		@(posedge clk);
		@(negedge clk);
		//deassert reset
		RST_n = 1;
	endtask: Initialize

	//task to send 'g'
	task SendCmd(input byte command);
		@(negedge clk);
		cmd = command;
		send_cmd = 1;
		@(posedge clk);
		send_cmd = 0;
	endtask: SendCmd

	//easy way to wait number of cycles
	task repeatClock(input shortint numClocks);
		repeat(numClocks) @(posedge clk);
	endtask repeatClock

endmodule