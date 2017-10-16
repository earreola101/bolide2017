# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "Component_Name" -parent ${Page_0}

  ipgui::add_param $IPINST -name "NUM_I2S_STREAM"

}

proc update_PARAM_VALUE.NUM_I2S_STREAM { PARAM_VALUE.NUM_I2S_STREAM } {
	# Procedure called to update NUM_I2S_STREAM when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_I2S_STREAM { PARAM_VALUE.NUM_I2S_STREAM } {
	# Procedure called to validate NUM_I2S_STREAM
	return true
}


proc update_MODELPARAM_VALUE.NUM_I2S_STREAM { MODELPARAM_VALUE.NUM_I2S_STREAM PARAM_VALUE.NUM_I2S_STREAM } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_I2S_STREAM}] ${MODELPARAM_VALUE.NUM_I2S_STREAM}
}

