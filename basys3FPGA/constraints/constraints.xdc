# Clock signal
set_property PACKAGE_PIN W5 [get_ports clk_100MHz]							
	set_property IOSTANDARD LVCMOS33 [get_ports clk_100MHz]
	create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk_100MHz]
	
#7 segment display
set_property PACKAGE_PIN U7 	 [get_ports {seg[0]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[0]}]
set_property PACKAGE_PIN V5 	 [get_ports {seg[1]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[1]}]
set_property PACKAGE_PIN U5 	 [get_ports {seg[2]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[2]}]
set_property PACKAGE_PIN V8 	 [get_ports {seg[3]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[3]}]
set_property PACKAGE_PIN U8 	 [get_ports {seg[4]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[4]}]
set_property PACKAGE_PIN W6 	 [get_ports {seg[5]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[5]}]
set_property PACKAGE_PIN W7 	 [get_ports {seg[6]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {seg[6]}]

#anodes
set_property PACKAGE_PIN U4 	 [get_ports {anode[0]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {anode[0]}]
set_property PACKAGE_PIN U2 	 [get_ports {anode[1]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {anode[1]}]
set_property PACKAGE_PIN V4 	 [get_ports {anode[2]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {anode[2]}]
set_property PACKAGE_PIN W4 	 [get_ports {anode[3]}]					
set_property IOSTANDARD LVCMOS33 [get_ports {anode[3]}]


##Buttons
set_property PACKAGE_PIN R2 	 [get_ports enableDisplay]						
set_property IOSTANDARD LVCMOS33 [get_ports enableDisplay]

##INPUT UART
set_property PACKAGE_PIN H1 [get_ports i_RX_Serial]
set_property IOSTANDARD LVCMOS33 [get_ports i_RX_Serial]



##LEDS
set_property PACKAGE_PIN U16 [get_ports led0_idle]
set_property IOSTANDARD LVCMOS33 [get_ports led0_idle] 
set_property PACKAGE_PIN E19 [get_ports led1_startbit]
set_property IOSTANDARD LVCMOS33 [get_ports led1_startbit] 
set_property PACKAGE_PIN U19 [get_ports led2_databits]
set_property IOSTANDARD LVCMOS33 [get_ports led2_databits] 
set_property PACKAGE_PIN V19 [get_ports led3_stopbit]
set_property IOSTANDARD LVCMOS33 [get_ports led3_stopbit] 


