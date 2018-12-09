module steer_en(clk,rst_n,lft_ld,rght_ld,ld_cell_diff,en_steer,rider_off);

  localparam MIN_RIDER_WEIGHT = 12'h200;	// min weight rider must exceed
  
  parameter fast_sim = 0; 					// defaulted to 0. When enabled only 
											// waits for bits [14:0] of timer to become full instead of [25:0]
  
  input clk;								// 50MHz clock
  input rst_n;								// Active low asynch reset
  input signed [11:0]lft_ld,rght_ld;

  output signed [11:0]ld_cell_diff;
  output logic en_steer;					// enables steering (goes to balance_cntrl)
  output logic rider_off;					// pulses high for one clock on transition back to initial state
  
  // Internal Signals
  
  // Timer signals
  logic [25:0] waitTimer;					// 26-bit timer (1.34 seconds) to meet balance criteria
  logic tmr_full;							// asserted when timer reaches 1.3 sec
  logic clr_tmr;							// clears the 1.3sec timer
  
  // Signals to check rider balance/weight
  logic [11:0] ld_cell_diff_abs;
  logic signed [11:0] ld_cell_sum;
  logic sum_gt_min;							// asserted when left and right load cells together exceed min rider weight
  logic diff_gt_fourth;						// asserted if load cell difference exceeds 1/4 sum (rider not situated)
  logic diff_gt_15_16;						// asserted if load cell difference is great (rider stepping off)
  

  
  // Produce sum and difference of left side of segway and right side
  assign ld_cell_diff = lft_ld - rght_ld;
  assign ld_cell_sum = lft_ld + rght_ld;

  // first check if rider exceeds MIN weight
  assign sum_gt_min = ld_cell_sum >=  MIN_RIDER_WEIGHT;
  // use abs value of weight next
  assign ld_cell_diff_abs = !ld_cell_diff[11] ? ld_cell_diff : -ld_cell_diff;
  // then check if rider is balanced
  assign diff_gt_fourth = ld_cell_diff_abs > {{2{ld_cell_sum[11]}},ld_cell_sum[11:2]};
  // stay enabled until rider is not balanced OR rider steps of (sum_gt_min is low)
  assign diff_gt_15_16 = ld_cell_diff_abs > (($signed(15))*({{4{ld_cell_sum[11]}},ld_cell_sum[11:4]}));  
  
  //controlling timer
  always_ff @(posedge clk,negedge rst_n) begin
    if (!rst_n) waitTimer <= 0;
	else if (clr_tmr) waitTimer <= 0;
	else waitTimer <= waitTimer + 1;
  end
 
  //check if 1.34 seconds (2^26 cycles) if fast_sim is not enabled. If fast_sim is enabled only wait 2^15 clock cycles
  assign tmr_full = fast_sim ? (&waitTimer[14:0]) : (&waitTimer);
  
  
  
  
  /////////////////////////// State Machine ///////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  // HEY BUDDY...you are a moron.  sum_gt_min would simply be ~sum_lt_min. Why
  // have both signals coming to this unit??  ANSWER: What if we had a rider
  // (a child) who's weigth was right at the threshold of MIN_RIDER_WEIGHT?
  // We would enable steering and then disable steering then enable it again,
  // ...  We would make that child crash(children are light and flexible and 
  // resilient so we don't care about them, but it might damage our Segway).
  // We can solve this issue by adding hysteresis.  So sum_gt_min is asserted
  // when the sum of the load cells exceeds MIN_RIDER_WEIGHT + HYSTERESIS and
  // sum_lt_min is asserted when the sum of the load cells is less than
  // MIN_RIDER_WEIGHT - HYSTERESIS.  Now we have noise rejection for a rider
  // who's wieght is right at the threshold.  This hysteresis trick is as old
  // as the hills, but very handy...remember it.
  //////////////////////////////////////////////////////////////////////////// 
  typedef enum reg[1:0] {IDLE, WAIT, STEER_EN} state_t; //create enumerated type
  state_t state, nxt_state;		//declare state and nxt_state signals
  
  //next  state sequential logic : state register
  always_ff @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		state <= IDLE;
	else
		state <= nxt_state;
  end
  
  //combinational state transition and output logic, sensitivity list contains all inputs to the FSM
  always_comb  begin
	nxt_state = IDLE; //default to reset
	clr_tmr = 0;	//default outputs
	en_steer = 0;	//to
	rider_off = 0; //avoid latches
	
	case (state)
	
		IDLE: begin
			if (sum_gt_min) begin               //if user exceeds min weight, start waiting
				nxt_state = WAIT;
				clr_tmr = 1;
			end
		end
		
		
		WAIT: begin
		
			if (~sum_gt_min) nxt_state = IDLE;  //weight doesnt pass the min weight, go back to idle state
			else if (diff_gt_fourth) begin
				nxt_state = WAIT;				// too much/little weight on either side
				clr_tmr = 1;					// reset counter and stay in state
			end
			else if ( tmr_full) begin
				nxt_state = STEER_EN;
				en_steer = 1;
			end
			else nxt_state = WAIT; 				// keep on countin ;)
		end
		
		
		default: begin
			
			if (~sum_gt_min) begin 				//rider knocked off the device or is not heavy enough
				rider_off = 1;
			end
			else if (diff_gt_15_16) begin 		// rider steps off
				nxt_state = WAIT;
				clr_tmr = 1;
			end
			else begin							// rider stays balanced, keep ridin ;)
				nxt_state = STEER_EN;
				en_steer = 1;
			end
		end
		
		
	endcase
	end
	
	
endmodule
