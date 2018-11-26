module steer_en_SM(clk,rst_n,tmr_full,sum_gt_min,sum_lt_min,diff_gt_eigth,
                   diff_gt_15_16,clr_tmr,en_steer,rider_off);
// Kevin Wilson , 10/13/18
  input clk;				// 50MHz clock
  input rst_n;				// Active low asynch reset
  input tmr_full;			// asserted when timer reaches 1.3 sec
  input sum_gt_min;			// asserted when left and right load cells together exceed min rider weight
  input sum_lt_min;			// asserted when left_and right load cells are less than min_rider_weight

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

  input diff_gt_eigth;		// asserted if load cell difference exceeds 1/8 sum (rider not situated)
  input diff_gt_15_16;		// asserted if load cell difference is great (rider stepping off)
  output logic clr_tmr;		// clears the 1.3sec timer
  output logic en_steer;	// enables steering (goes to balance_cntrl)
  output logic rider_off;	// pulses high for one clock on transition back to initial state
  
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
	
	//TODO
	//the signals sum_gt_min and sum_lt_min can be left alone after being added to sensitivity list
	case (state)
	
		IDLE: begin
			if (sum_gt_min) begin                     	//if user exceeds min weight, start waiting
				nxt_state = WAIT;
				clr_tmr = 1;
			end
		end
		
		
		WAIT: begin
		
			if (~sum_gt_min | sum_lt_min) nxt_state = IDLE;  //weight doesnt pass the min weight, go back to idle state
			else if (diff_gt_eigth) begin
				nxt_state = WAIT;				// too much/little weight on either side
				clr_tmr = 1;					// reset counter and stay in state
			end
			else if ( tmr_full) begin
				nxt_state = STEER_EN;
				en_steer = 1;
			end
			else nxt_state = WAIT; // keep on countin ;)
		end
		
		
		STEER_EN: begin
			
			if ((~sum_gt_min) | (sum_lt_min)) begin 			//rider knocked off the device or is not heavy enough
				nxt_state = IDLE;
				rider_off = 1;
			end
			else if (diff_gt_15_16) begin 				// rider steps off
				nxt_state = WAIT;
				clr_tmr = 1;
			end
			else begin						// rider stays balanced, keep ridin ;)
				nxt_state = STEER_EN;
				en_steer = 1;
			end
		end
		
		
		default: nxt_state = IDLE; //reset state 
		
	endcase
	end
	
	
endmodule
