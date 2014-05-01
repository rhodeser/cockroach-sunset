/*****************************************************************************
* Filename:          C:\Users\Caren\Dropbox\PSU\ECE544\Projects\ECE544_Repository_14_7\MyProcessorIPLib/drivers/lightsensor_v1_00_a/src/lightsensor_selftest.c
* Version:           1.00.a
* Description:       Contains a diagnostic self-test function for the lightsensor driver
* Date:              Wed Apr 30 11:34:36 2014 (by Create and Import Peripheral Wizard)
*****************************************************************************/


/***************************** Include Files *******************************/

#include "lightsensor.h"

/************************** Constant Definitions ***************************/


/************************** Variable Definitions ****************************/


/************************** Function Definitions ***************************/

/**
 *
 * Run a self-test on the driver/device. Note this may be a destructive test if
 * resets of the device are performed.
 *
 * If the hardware system is not built correctly, this function may never
 * return to the caller.
 *
 * @param   baseaddr_p is the base address of the LIGHTSENSOR instance to be worked on.
 *
 * @return
 *
 *    - XST_SUCCESS   if all self-test code passed
 *    - XST_FAILURE   if any self-test code failed
 *
 * @note    Caching must be turned off for this function to work.
 * @note    Self test may fail if data memory and device are not on the same bus.
 *
 */
XStatus LIGHTSENSOR_SelfTest(u32 baseaddr_p)
{
  int     Index;
  Xuint32 baseaddr;
  Xuint8  Reg8Value;
  Xuint16 Reg16Value;
  Xuint32 Reg32Value;
  
  /*
   * Check and get the device address
   */
  /*
   * Base Address maybe 0. Up to developer to uncomment line below.
  XASSERT_NONVOID(baseaddr_p != XNULL);
   */
  baseaddr = (Xuint32) baseaddr_p;

  xil_printf("**********************************\n\r");
  xil_printf("* LightSensor Peripheral Self Test\n\r");
  xil_printf("**********************************\n\n\r");

  /*
   * Write to lightsensor spare registers and read back
   */
  xil_printf("User logic slave module test...\n\r");
  
  xil_printf("   - write 0x5A5A5A5A to SPAREREG1\n\r");
  LIGHTSENSOR_mWriteSPAREREG1(baseaddr, 0x5A5A5A5A);
  Reg32Value = LIGHTSENSOR_mReadSPAREREG1(baseaddr);
  xil_printf("   - read %d from SPAREREG1\n\r", Reg32Value);
  if ( Reg32Value != (Xuint32) 0x5A5A5A5A )
  {
    xil_printf("   - SPAREREG1 write/read failed\n\r");
    return XST_FAILURE;
  }
  xil_printf("   - write 0xA5A5A5A5 to SPAREREG2\n\r");
  LIGHTSENSOR_mWriteSPAREREG2(baseaddr, 0xA5A5A5A5);
  Reg32Value = LIGHTSENSOR_mReadSPAREREG2(baseaddr);
  xil_printf("   - read %d from SPAREREG2\n\r", Reg32Value);
  if ( Reg32Value != (Xuint32) 0xA5A5A5A5 )
  {
    xil_printf("   - SPAREREG2 write/read failed\n\r");
    return XST_FAILURE;
  }
  xil_printf("   - LightSensor I/O periferal write/read passed\n\n\r");

  return XST_SUCCESS;
}
