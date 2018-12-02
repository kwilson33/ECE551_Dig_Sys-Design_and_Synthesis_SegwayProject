What is done
------------
* Module instantiations in Segway.v
* Balance control
* Steer_en


What to do
------------
* Inertial integrator/ Inertial Interface : 
	* Verify waveform
	* Should we use don't cares for the read values?
* piezo
* synthesis
* A2D Interface (DETAILS IN misc things to do)
	* need a way to specify left and right load cell readings (and perhaps battery too) and have those translate into something that is accessed by your A2D_Intf block (SPI bus).
	* modify ADC128S or augment SPI_ADC128S to create such a block that can be used as part of fullchip testing.
	* What needs to drive nxt into A2D_Intf to force it to perform round robin readings? Really just any
periodic signal that happens often enough but not too often. Hey, don’t we get vld readings from the
inertial sensor 200+ times a second.

* Complete Segway_tb.v	
