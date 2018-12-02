What is done
------------
* Auth_blck
* UART 
* SPI

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
* A2D Interface (DETAILS IN misc things to do)
	* need a way to specify left and right load cell readings (and perhaps battery too) and have those translate into something that is accessed by your A2D_Intf block (SPI bus).
	* modify ADC128S or augment SPI_ADC128S to create such a block that can be used as part of fullchip testing.
	* What needs to drive nxt into A2D_Intf to force it to perform round robin readings? Really just any
periodic signal that happens often enough but not too often. Hey, don’t we get vld readings from the
inertial sensor 200+ times a second.

* flesh out Segway.v
* flesh out SegwayModel.sv
* optional Segway_tb.v	
