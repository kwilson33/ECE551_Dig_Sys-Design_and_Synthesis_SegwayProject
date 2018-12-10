//Kevin Wilson 11/5/18
module Auth_blk(clk, rst_n, rider_off, pwr_up, RX);

	input clk, rst_n;
	input logic RX, rider_off;
	output logic pwr_up;
	
	//internal signals in the Auth_blk
	logic [7:0] rx_data;
	logic clr_rx_rdy, rx_rdy;
	
	
	localparam go = 8'h67; 				// signal from Bluetooth telling to power up. Go is asserted if rx_data == 0x67 and rx_rdy is asserted
	localparam stop = 8'h73; 			// signal from Bluetooth telling to stop. Stop is asserted if rx_data == 0x73 and rx_rdy is asserted
	
	//instantiate UART receiver. This will send input to auth block and receive output back
	UART_rcv receiver(.clk(clk), .rst_n(rst_n), .RX(RX), .clr_rdy(clr_rx_rdy), .rdy(rx_rdy), .rx_data(rx_data));
	
	
	
	
	//#######################################################State machine#######################################################
	typedef enum reg [1:0] {OFF, PWR1, PWR2} state_t;
	state_t state, nxt_state;
	
	//Sequential next state logic
	always_ff @(posedge clk, negedge rst_n) begin
		if(!rst_n) state <= OFF;
		else state <= nxt_state;
	end
	
	//Combinational transition logic
	always_comb begin
		nxt_state = OFF;
		pwr_up = 0; clr_rx_rdy = 0; 									// default outputs. Powered off by default. Not clearing rx_rdy by default
		
		case (state)
		
			OFF: begin 										// if off and receives a 'g', then go to PWR1 state
				if ((rx_data == go) && (rx_rdy)) begin
					clr_rx_rdy = 1;								// clear rx_rdy signal because every time UART_rcv receives a byte, rx_rdy is asserted,
														// and need to reset so it can receive another bit and assert rx_rdy again
					nxt_state =  PWR1;																					
				end
			end
			
			PWR1: begin
				pwr_up = 1; 											// always power up in this state
				//if (rider_off) nxt_state = OFF;							// if no rider go to OFF state				
				if((rx_data == stop) && (rx_rdy)) begin			// received an 's' ...
						clr_rx_rdy = 1;	
						nxt_state = PWR2;						// someone still on segway, so transition to PWR2 state and wait for rider off signal								
				end
				else nxt_state = PWR1;								// if nothing happens, just stay powered on
			end
			
			
			default: begin
				pwr_up = 1;									// stay powered on in this state while waiting for rider off signal
				//Going back to PWR1 state and waiting for stop signal has priority over turning off Segway
				if ((rx_data == go) && (rx_rdy)) begin
					clr_rx_rdy = 1;
					nxt_state = PWR1;
				end
				//only power off once gotten stop signal AND rider_off is asserted
				else if (rider_off) nxt_state = OFF;
				else nxt_state = PWR2;
			end
		endcase
	end
	
	
endmodule
