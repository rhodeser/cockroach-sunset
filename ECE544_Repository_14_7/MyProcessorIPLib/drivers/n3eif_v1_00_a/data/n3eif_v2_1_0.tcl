##############################################################################
## Filename:          C:\PSU_Projects\Nexys3_Development\Nexys3_Repository\MyProcessorIPLib/drivers/n3eif_v1_00_a/data/n3eif_v2_1_0.tcl
## Description:       Microprocess Driver Command (tcl)
## Date:              Thu Mar 14 06:09:59 2013 (by Create and Import Peripheral Wizard)
##############################################################################

#uses "xillib.tcl"

proc generate {drv_handle} {
  xdefine_include_file $drv_handle "xparameters.h" "n3eif" "NUM_INSTANCES" "DEVICE_ID" "C_BASEADDR" "C_HIGHADDR" 
}
