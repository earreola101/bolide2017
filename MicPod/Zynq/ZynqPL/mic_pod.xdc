set_property PACKAGE_PIN G11 [get_ports BUS0_TX]
set_property PACKAGE_PIN G12 [get_ports BUS0_RX]
set_property PACKAGE_PIN H14 [get_ports MUTE_BUTTON]
set_property PACKAGE_PIN K15 [get_ports BUS1_TX]
set_property PACKAGE_PIN J14 [get_ports BUS1_RX]
set_property PACKAGE_PIN M14 [get_ports WS_23]
set_property PACKAGE_PIN L13 [get_ports SCK_23]
set_property PACKAGE_PIN K11 [get_ports ADC_SD_23]
set_property PACKAGE_PIN L12 [get_ports PLL_WCLK]
set_property PACKAGE_PIN N11 [get_ports MCLK]
set_property PACKAGE_PIN P15 [get_ports PLL_SPI_SCLK]
set_property PACKAGE_PIN R15 [get_ports PLL_SPI_CS]
set_property PACKAGE_PIN P11 [get_ports PLL_SPI_MOSI]
set_property PACKAGE_PIN R12 [get_ports PLL_AUX]
set_property PACKAGE_PIN R13 [get_ports WS_01]
set_property PACKAGE_PIN P13 [get_ports SCK_01]
set_property PACKAGE_PIN P14 [get_ports ADC_SD_01]
set_property PACKAGE_PIN M9 [get_ports MEMS01_D]
set_property PACKAGE_PIN N9 [get_ports MEMS23_D]
set_property PACKAGE_PIN R7 [get_ports ADC_RSTN_01]
set_property PACKAGE_PIN R8 [get_ports ADC_RSTN_23]
set_property PACKAGE_PIN M10 [get_ports {LEDS[0]}]
set_property PACKAGE_PIN M11 [get_ports {LEDS[1]}]
set_property PACKAGE_PIN N7 [get_ports {LEDS[2]}]
set_property PACKAGE_PIN N8 [get_ports {LEDS[3]}]
set_property PACKAGE_PIN P8 [get_ports {LEDS[4]}]
set_property PACKAGE_PIN P9 [get_ports {LEDS[5]}]
set_property PACKAGE_PIN P10 [get_ports {LEDS[6]}]
set_property PACKAGE_PIN R10 [get_ports {LEDS[7]}]




set_property IOSTANDARD LVCMOS33 [get_ports BUS0_TX]
set_property IOSTANDARD LVCMOS33 [get_ports BUS0_RX]
set_property IOSTANDARD LVCMOS33 [get_ports MUTE_BUTTON]
set_property IOSTANDARD LVCMOS33 [get_ports BUS1_TX]
set_property IOSTANDARD LVCMOS33 [get_ports BUS1_RX]
set_property IOSTANDARD LVCMOS33 [get_ports WS_23]
set_property IOSTANDARD LVCMOS33 [get_ports SCK_23]
set_property IOSTANDARD LVCMOS33 [get_ports ADC_SD_23]
set_property IOSTANDARD LVCMOS33 [get_ports PLL_WCLK]
set_property IOSTANDARD LVCMOS33 [get_ports MCLK]
set_property IOSTANDARD LVCMOS33 [get_ports PLL_SPI_SCLK]
set_property IOSTANDARD LVCMOS33 [get_ports PLL_SPI_CS]
set_property IOSTANDARD LVCMOS33 [get_ports PLL_SPI_MOSI]
set_property IOSTANDARD LVCMOS33 [get_ports PLL_AUX]
set_property IOSTANDARD LVCMOS33 [get_ports WS_01]
set_property IOSTANDARD LVCMOS33 [get_ports SCK_01]
set_property IOSTANDARD LVCMOS33 [get_ports ADC_SD_01]
set_property IOSTANDARD LVCMOS33 [get_ports MEMS01_D]
set_property IOSTANDARD LVCMOS33 [get_ports MEMS23_D]
set_property IOSTANDARD LVCMOS33 [get_ports ADC_RSTN_01]
set_property IOSTANDARD LVCMOS33 [get_ports ADC_RSTN_23]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[7]}]



create_clock -period 61.035 -name MCLK [get_ports MCLK]

#create_clock -period 61.035 -name IOD_D0_P [get_ports IOD_D0_P]

# ----------------------------------------------------------------------------------------------------
# eof
# ----------------------------------------------------------------------------------------------------



set_false_path -from [get_clocks clk_fpga_0] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks clk_fpga_0] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_16_384M_to_20_48M/inst/mmcm_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks clk_fpga_0] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks clk_fpga_0] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT2]]
set_false_path -from [get_clocks clk_fpga_0] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_16_384M_to_20_48M/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks clk_fpga_0]
set_false_path -from [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT0]] -to [get_clocks clk_fpga_0]
set_false_path -from [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT0]] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_16_384M_to_20_48M/inst/mmcm_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_16_384M_to_20_48M/inst/mmcm_adv_inst/CLKOUT1]] -to [get_clocks clk_fpga_0]
set_false_path -from [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_16_384M_to_20_48M/inst/mmcm_adv_inst/CLKOUT1]] -to [get_clocks -of_objects [get_pins mic_pod_design_i/clock_gen/clk_phases/inst/plle2_adv_inst/CLKOUT0]]
