What is done
------------
* Auth_blck
* UART 
* SPI
* A2D Interface
* Inertial integrator
* Balance control
* Steer_en


What to do
------------
* piezo driver, not an in class exercise
	* warnings to people in vicinity (norm_mode, every 2 seconds)
	* warn rider too fast (ovr_spd,alarming)
	* battery low warnings (threshold is 0x800), able to occur same time as too fast warning
	* range = 300 Hz --> 7 kHz 
* synthesis
* flesh out Segway.v
* flesh out SegwayModel.sv
* optional Segway_tb.v	
