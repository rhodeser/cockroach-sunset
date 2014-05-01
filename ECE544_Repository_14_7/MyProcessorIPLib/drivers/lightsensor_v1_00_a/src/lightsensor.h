/*****************************************************************************
* Filename:          C:\Users\Caren\Dropbox\PSU\ECE544\Projects\ECE544_Repository_14_7\MyProcessorIPLib/drivers/lightsensor_v1_00_a/src/lightsensor.h
* Version:           1.00.a
* Description:       lightsensor Driver Header File
* Date:              Wed Apr 30 11:34:36 2014 (by Create and Import Peripheral Wizard)
*****************************************************************************/

#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

/***************************** Include Files *******************************/

#include "xbasic_types.h"
#include "xstatus.h"
#include "xil_io.h"

/************************** Constant Definitions ***************************/


/**
 * LightSensor Interface Register Offsets
 * -- CONTROL	: Enable the PWM detection (bit0)		(O)
 * -- STATUS	: Periferal controller status (bit0)	(I)
 * -- HIGH_TIME	: Detected High Time Count				(I)
 * -- PERIOD	: Detected Period Count					(I)
 * -- SPAREREG1	: Spare register **RESERVED**			(I)
 * -- SPAREREG2	: Spare register **RESERVED**			(I)
 */
#define LIGHTSENSOR_USER_SLV_SPACE_OFFSET (0x00000000)
#define LIGHTSENSOR_CONTROL_OFFSET (LIGHTSENSOR_USER_SLV_SPACE_OFFSET + 0x00000000)
#define LIGHTSENSOR_STATUS_OFFSET (LIGHTSENSOR_USER_SLV_SPACE_OFFSET + 0x00000004)
#define LIGHTSENSOR_HIGH_TIME_OFFSET (LIGHTSENSOR_USER_SLV_SPACE_OFFSET + 0x00000008)
#define LIGHTSENSOR_PERIOD_OFFSET (LIGHTSENSOR_USER_SLV_SPACE_OFFSET + 0x0000000C)
#define LIGHTSENSOR_SPAREREG1_OFFSET (LIGHTSENSOR_USER_SLV_SPACE_OFFSET + 0x00000010)
#define LIGHTSENSOR_SPAREREG2_OFFSET (LIGHTSENSOR_USER_SLV_SPACE_OFFSET + 0x00000014)

/**************************** Type Definitions *****************************/


/***************** Macros (Inline Functions) Definitions *******************/

/**
 *
 * Write a value to a LIGHTSENSOR register. A 32 bit write is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is written.
 *
 * @param   BaseAddress is the base address of the LIGHTSENSOR device.
 * @param   RegOffset is the register offset from the base to write to.
 * @param   Data is the data written to the register.
 *
 * @return  None.
 *
 * @note
 * C-style signature:
 * 	void LIGHTSENSOR_mWriteReg(Xuint32 BaseAddress, unsigned RegOffset, Xuint32 Data)
 *
 */
#define LIGHTSENSOR_mWriteReg(BaseAddress, RegOffset, Data) \
 	Xil_Out32((BaseAddress) + (RegOffset), (Xuint32)(Data))

/**
 *
 * Read a value from a LIGHTSENSOR register. A 32 bit read is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is read from the register. The most significant data
 * will be read as 0.
 *
 * @param   BaseAddress is the base address of the LIGHTSENSOR device.
 * @param   RegOffset is the register offset from the base to write to.
 *
 * @return  Data is the data from the register.
 *
 * @note
 * C-style signature:
 * 	Xuint32 LIGHTSENSOR_mReadReg(Xuint32 BaseAddress, unsigned RegOffset)
 *
 */
#define LIGHTSENSOR_mReadReg(BaseAddress, RegOffset) \
 	Xil_In32((BaseAddress) + (RegOffset))


/**
 *
 * Write/Read 32 bit value to/from LIGHTSENSOR user logic slave registers.
 *
 * @param   BaseAddress is the base address of the LIGHTSENSOR device.
 * @param   RegOffset is the offset from the slave register to write to or read from.
 * @param   Value is the data written to the register.
 *
 * @return  Data is the data from the user logic slave register.
 *
 * @note
 * C-style signature:
 * 	void LIGHTSENSOR_mWriteSlaveRegn(Xuint32 BaseAddress, unsigned RegOffset, Xuint32 Value)
 * 	Xuint32 LIGHTSENSOR_mReadSlaveRegn(Xuint32 BaseAddress, unsigned RegOffset)
 *
 */
#define LIGHTSENSOR_mWriteCONTROL(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (LIGHTSENSOR_CONTROL_OFFSET) , (Xuint32)(Value))
#define LIGHTSENSOR_mWriteSTATUS(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (LIGHTSENSOR_STATUS_OFFSET) , (Xuint32)(Value))
#define LIGHTSENSOR_mWriteHIGH_TIME(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (LIGHTSENSOR_HIGH_TIME_OFFSET) , (Xuint32)(Value))
#define LIGHTSENSOR_mWritePERIOD(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (LIGHTSENSOR_PERIOD_OFFSET) , (Xuint32)(Value))
#define LIGHTSENSOR_mWriteSPAREREG1(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (LIGHTSENSOR_SPAREREG1_OFFSET) , (Xuint32)(Value))
#define LIGHTSENSOR_mWriteSPAREREG2(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (LIGHTSENSOR_SPAREREG2_OFFSET) , (Xuint32)(Value))

#define LIGHTSENSOR_mReadCONTROL(BaseAddress) \
 	Xil_In32((BaseAddress) + (LIGHTSENSOR_CONTROL_OFFSET) )
#define LIGHTSENSOR_mReadSTATUS(BaseAddress) \
 	Xil_In32((BaseAddress) + (LIGHTSENSOR_STATUS_OFFSET) )
#define LIGHTSENSOR_mReadHIGH_TIME(BaseAddress) \
 	Xil_In32((BaseAddress) + (LIGHTSENSOR_HIGH_TIME_OFFSET) )
#define LIGHTSENSOR_mReadPERIOD(BaseAddress) \
 	Xil_In32((BaseAddress) + (LIGHTSENSOR_PERIOD_OFFSET) )
#define LIGHTSENSOR_mReadSPAREREG1(BaseAddress) \
 	Xil_In32((BaseAddress) + (LIGHTSENSOR_SPAREREG1_OFFSET) )
#define LIGHTSENSOR_mReadSPAREREG2(BaseAddress) \
 	Xil_In32((BaseAddress) + (LIGHTSENSOR_SPAREREG2_OFFSET) )

/************************** Function Prototypes ****************************/

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
XStatus LIGHTSENSOR_SelfTest(u32 baseaddr_p);

/**
 * LIGHTSENSOR_Init() - Initialize the lightsensor periferal driver
 * This function waits until the light sensor self test is done then it sets
 * the control enable bit to 0.
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 *
 * @return
 *    - XST_SUCCESS   if all self-test code passed
 *    - XST_FAILURE   if any self-test code failed
 *
 */
XStatus LIGHTSENSOR_Init(u32 BaseAddress);


/**
 * LIGHTSENSOR_Start() - Starts the PWM detection in the periferal
 * This function sets the Control Enable bit to 1
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 *
 * @return
 *    - XST_SUCCESS   if passed
 *    - XST_FAILURE   if failed
 *
 */
XStatus LIGHTSENSOR_Start(u32 BaseAddress);

/**
 * LIGHTSENSOR_Stop() - Stops the PWM detection in the periferal
 * This function sets the Control Enable bit to 0
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 *
 * @return
 *    - XST_SUCCESS   if passed
 *    - XST_FAILURE   if failed
 *
 */
XStatus LIGHTSENSOR_Stop(u32 BaseAddress);

/**
 * LIGHTSENSOR_Capture() - This function returns the period count by reading the PERIOD register
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 * @param slope
 * @param offset
 *
 * If slope and offset are set, return scaled count [0:4095]
 * Else return actual count
 *
 * @return
 * 		- period count
 *
 */
Xuint32 LIGHTSENSOR_Capture(u32 BaseAddress, Xuint32 slope, Xuint32 offset);

/**
 * LIGHTSENSOR_SetScaling() - Sets the slope and offset values for scaling.
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 * @param min count
 * @param max count
 * @param pointer to slope
 * @param pointer to offset
 *
 * @return
 *    - XST_SUCCESS   if passed
 *    - XST_FAILURE   if failed
 *
 */
XStatus LIGHTSENSOR_SetScaling(u32 BaseAddress, Xuint32 minCount, Xuint32 maxCount, Xuint32 *slope, Xuint32 *offset);

/**
 * LIGHTSENSOR_Count2Volts() - Converts the actual 'count' measurements to a scaled
 * voltage output.
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 * @param scaledCount scaled to the range [0:4095]
 *
 * @return
 *    - voltage
 *
 */
Xuint32 LIGHTSENSOR_Count2Volts(u32 BaseAddress, Xuint32 scaledCount);

#endif /** LIGHTSENSOR_H */
