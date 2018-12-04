module piezo_tb();

  reg clk,rst_n;
  reg norm_mode,ovr_spd,batt_low;
  wire piezo,piezo_n;


  //// Instantiating DUT /////
  piezo	iDUT(.clk(clk), .rst_n(rst_n), .norm_mode(norm_mode), .ovr_spd(ovr_spd),
	     .batt_low(batt_low), .piezo(piezo), .piezo_n(piezo_n));


  initial begin
    clk = 0;
    rst_n = 0;
    norm_mode = 0;
    ovr_spd = 0;
    batt_low = 0;
    @(posedge clk);
    @(negedge clk);
    rst_n = 1;

    norm_mode = 1;
    repeat (68000000) @(posedge clk);
    norm_mode = 0;
    ovr_spd = 1;
    repeat (68000000) @(posedge clk);
    ovr_spd = 0;
    batt_low = 1;
    repeat (68000000) @(posedge clk);
    ovr_spd = 1;
    repeat (68000000) @(posedge clk);

    $stop();

  end

  always #2 clk = ~clk; 

endmodule