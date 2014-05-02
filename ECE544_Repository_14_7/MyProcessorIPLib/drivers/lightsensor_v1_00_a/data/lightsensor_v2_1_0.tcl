##############################################################################
## Filename:          C:\Users\Caren\Dropbox\PSU\ECE544\Projects\ECE544_Repository_14_7\MyProcessorIPLib/drivers/lightsensor_v1_00_a/data/lightsensor_v2_1_0.tcl
## Description:       Microprocess Driver Command (tcl)
## Date:              Wed Apr 30 11:34:36 2014 (by Create and Import Peripheral Wizard)
##############################################################################

#uses "xillib.tcl"

proc generate {drv_handle} {
  xdefine_include_file $drv_handle "xparameters.h" "lightsensor" "NUM_INSTANCES" "DEVICE_ID" "C_BASEADDR" "C_HIGHADDR" 
}
