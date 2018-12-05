module piezo(clk, rst_n, norm_mode, ovr_spd, batt_low, piezo, piezo_n);

	input clk,rst_n;
	input norm_mode,ovr_spd,batt_low;
	
	output piezo,piezo_n;

	reg [25:0]timer;
	// digital square waves to drive piezo buzzer
	wire norm_mode_wave,ovr_spd_wave,batt_low_wave;
	wire norm_tone_dur,warn_tone_dur;// controls the duration of buzzer
					 // pulses when in normal or danger modes

	always_ff @(posedge clk,negedge rst_n)
	  if (!rst_n) timer <= 0;
	  else timer <= timer + 1;

	assign norm_mode_wave = timer[16];	// bit 16 of timer gives ~750Hz wave
	assign ovr_spd_wave = timer[14];	// ~3kHz wave
	assign batt_low_wave = timer[15];	// ~1.5kHz wave

	assign norm_tone_dur = &timer[25:24];	// buzzer duration when in normal operation
						// mode (goes for the last .34 sec of 1.34 sec
						// period)
	assign warn_tone1_dur = timer[24];	// buzzer duration when in danger, i.e., when
						// over speed (goes ~1/3 second during first and
						// and third quarter intervals of 1.34 sec period)
	assign warn_tone2_dur = timer[23];	// second buzzer danger duration, occurs when 
						// both over speed and battery low are high 
						// (goes for ~1/6 second 1/3 second period)


	//// Assigning piezo and it's complement ////
	assign piezo = (ovr_spd & batt_low) ? (ovr_spd_wave & warn_tone2_dur):
		       ovr_spd	 ? (ovr_spd_wave & warn_tone1_dur)  :
		       batt_low	 ? (batt_low_wave & norm_tone_dur) :
		       norm_mode ? (norm_mode_wave & norm_tone_dur):
		       0;
	assign piezo_n = ~piezo;


endmodule