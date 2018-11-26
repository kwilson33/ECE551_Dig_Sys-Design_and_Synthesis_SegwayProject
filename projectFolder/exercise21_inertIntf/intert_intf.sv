//Kevin W, Shawn Z, Tyler M, Severin D
module intert_intf(clk, rst_n,  vld, ptch, SS_n, SCLK, MOSI, MISO, INT);
	
	input clk, rst_n; 					// 50MHz clock, active low reset
	input  INT;							// Interrupt signal from intertial sensor, informs new measurement ready to be read
	
	output logic vld;					// Asserted from SM. Consumed in intertial_integrator. Also used in balance_ctrl
	output [15:0] ptch;					// Primary output. Fusion corrected ptch from Segway
	
	
	//SPI interface to inertial sensor
	input MISO;
	output SS_n, SCLK, MOSI;
	
	




endmodule	