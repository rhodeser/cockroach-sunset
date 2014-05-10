/*****************************************************************************
* Filename:          C:\Users\Caren\Dropbox\PSU\ECE544\Projects\ECE544_Repository_14_7\MyProcessorIPLib/drivers/lightsensor_v1_00_a/src/lightsensor.c
* Version:           1.00.a
* Description:       lightsensor Driver Source File
* Date:              Wed Apr 30 11:34:36 2014 (by Create and Import Peripheral Wizard)
*****************************************************************************/


/***************************** Include Files *******************************/

#include "lightsensor.h"

/************************** Global Variables ***************************/

static bool	IsReady = false;			// lightsensor peripheral has been successfully initialized

/************************** Function Definitions ***************************/

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
XStatus LIGHTSENSOR_Init(u32 BaseAddress)
{
		XStatus status;

		// Run the driver self test.
		if (!IsReady)
		{
				status = LIGHTSENSOR_SelfTest(BaseAddress);
				if (status != XST_SUCCESS)
				{
						return XST_FAILURE;
				}
		}
		IsReady = true;

		LIGHTSENSOR_mWriteCONTROL(BaseAddress,0);

		return XST_SUCCESS;

}

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
XStatus LIGHTSENSOR_Start(u32 BaseAddress)
{
		LIGHTSENSOR_mWriteCONTROL(BaseAddress, 0x00000001);

		// Read the status register to make sure it has been enabled
		//if ( LIGHTSENSOR_mReadSTATUS(BaseAddress) != (Xuint32) 0x00000001)
		//		return XST_FAILURE;

		return XST_SUCCESS;
}


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
XStatus LIGHTSENSOR_Stop(u32 BaseAddress)
{
		LIGHTSENSOR_mWriteCONTROL(BaseAddress, 0x00000000);

		// Read the status register to make sure it has been enabled
		//if ( LIGHTSENSOR_mReadSTATUS(BaseAddress) != (Xuint32) 0x00000000)
		//		return XST_FAILURE;

		return XST_SUCCESS;
}

/**
 * LIGHTSENSOR_Capture() - This function returns the period count by reading the PERIOD register
 * 
 * @param BaseAddress is the base address of the LIGHTSENSOR instance to be worked on
 * @param slope
 * @param offset
 * @param boolean is_scaled:
 * 			If is_scaled == true, return scaled count [0:4095]
 * 			Else return actual count
 *
 * @return
 * 		- period count
 *
 */
Xuint32 LIGHTSENSOR_Capture(u32 BaseAddress, double slope, Xuint32 offset, bool is_scaled, Xuint32 min)
{
		Xuint32 count;
		volatile Xuint32 period, freq;

		period = LIGHTSENSOR_mReadPERIOD(BaseAddress);
        //freq = 66666666 / period;

		if (is_scaled)	// already characterized
		{
				count = (Xuint32)(slope * (freq - min)+ 1);
				//if (count > 4095)
			    //		count = 4095;
		}
		else			// in characterize function
		{
				//count = (Xuint32)freq;
				count = period;
		}

		return count;

}

/**
 * LIGHTSENSOR_SetScaling() - Sets the slope and offset values for scaling.
 * 
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
XStatus LIGHTSENSOR_SetScaling(Xuint32 maxCount, Xuint32 minCount, double *slope, Xuint32 *offset)
{
		double diff;
		diff = maxCount - minCount;

		*slope = 4095.0/diff;
		*offset = minCount;

		return XST_SUCCESS;
}


/**
 * LIGHTSENSOR_Count2Volts() - Converts the actual 'count' measurements to a scaled
 * voltage output.
 * 
 * @param scaledCount scaled to the range [0:4095]
 *
 * @return
 *    - voltage
 *
 */
double LIGHTSENSOR_Count2Volts(Xuint32 scaledCount)
{
		double volts;
		volts = (3.3 / 4095.0) * (scaledCount);

		return volts;
}
