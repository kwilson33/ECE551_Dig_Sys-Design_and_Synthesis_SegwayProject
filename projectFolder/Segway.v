module Segway(clk,RST_n,LED,INERT_SS_n,INERT_MOSI,
              INERT_SCLK,INERT_MISO,A2D_SS_n,A2D_MOSI,A2D_SCLK,
			  A2D_MISO,PWM_rev_rght,PWM_frwrd_rght,PWM_rev_lft,
			  PWM_frwrd_lft,piezo_n,piezo,INT,RX);
			  
  input clk,RST_n;
  input INERT_MISO;						// Serial in from inertial sensor
  input A2D_MISO;						// Serial in from A2D
  input INT;							// Interrupt from inertial indicating data ready
  input RX;								// UART input from BLE module

  
  output [7:0] LED;						// These are the 8 LEDs on the DE0, your choice what to do
  output A2D_SS_n, INERT_SS_n;			// Slave selects to A2D and inertial sensor
  output A2D_MOSI, INERT_MOSI;			// MOSI signals to A2D and inertial sensor
  output A2D_SCLK, INERT_SCLK;			// SCLK signals to A2D and inertial sensor
  output PWM_rev_rght, PWM_frwrd_rght;  // right motor speed controls
  output PWM_rev_lft, PWM_frwrd_lft;	// left motor speed controls
  output piezo_n,piezo;					// diff drive to piezo for sound
  
  ////////////////////////////////////////////////////////////////////////
  // fast_sim is asserted to speed up fullchip simulations.  Should be //
  // passed to both balance_cntrl and to steer_en.  Should be set to  //
  // 0 when we map to the DE0-Nano.                                  //
  ////////////////////////////////////////////////////////////////////
  localparam fast_sim = 1;	// asserted to speed up simulations. 
  
  ///////////////////////////////////////////////////////////
  ////// Internal interconnecting sigals defined here //////
  /////////////////////////////////////////////////////////
  wire rst_n;                           // internal global reset that goes to all units
  
  // You will need to declare a bunch more interanl signals to hook up everything
  
  ////////////////////////////////////
   
  
  ///////////////////////////////////////////////////////
  // How you arrange the hierarchy of the top level is up to you.
  //
  // You could make a level of hierarchy called digital core
  // as shown in the block diagram in the spec.
  //
  // Or you could just instantiate all the components of the Segway
  // flat.
  //
  // Just for reference all the needed blocks (in no particular order) would be:
  //   Auth_blk
  //   inert_intf
  //   balance_cntrl
  //   steer_en
  //   mtr_drv
  //   A2D_intf
  //   piezo
  //////////////////////////////////////////////////////
  

  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////  
  reset_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));
  
endmodule
