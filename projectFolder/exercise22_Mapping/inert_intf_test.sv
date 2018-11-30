module inert_intf_test(clk,RST_n,MISO,INT,LED,SS_n,SCLK,MOSI);

  input clk,RST_n;
  input MISO,INT;

  output logic [7:0]LED;
  output logic SS_n,SCLK,MOSI;

  //Internal signals
  logic rst_n;
  logic [15:0]ptch;

  //// Instantiating Signals ////
  rst_synch	RS(.RST_n(RST_n),.clk(clk),.rst_n(rst_n));
  inert_intf	Interface(.clk(clk),.rst_n(rst_n),.vld(),.ptch(ptch),
			  .SS_n(SS_n), .SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),.INT(INT));
  
  assign LED = ptch[8:1];

endmodule