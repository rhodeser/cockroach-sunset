/*****************************************************************************
*
* Copyright Caren Zgheib, Portland State University, 2014
*
* Filename:          lightSensor.h
* Version:           1.00.a
* Description:       lightSensor Driver Header File (API)
* Date:              Tue 29-Apr-2014 (by Caren Zgheib)
*
* NOTE:  This driver assumes that a control system with the light-to-frequency
* converter is plugged into a Pmod header on the Nexys3 board. 
* There is no error checking done by the driver to verify this though.
*****************************************************************************/

#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

/***************************** Include Files *******************************/
#include "stdbool.h"

/***************************** Constant Definitions ************************/


/***************************** Function Prototypes *************************/

// APIs for basic functionality

/**
 * lightSenor_init() - Initialize the lightSensor peripheral driver.  This function waits until
 * the n3eif self test is done then it sets the rotary encoder mode and clears the
 * rotary encoder count.  The LCD display is cleared on exit
 *
 * @param	BaseAddress is the base address in memory mapped I/O space for the peripheral
 *
 * @return	XST_SUCCESS
 *
 */
XStatus N3EIF_init(u32 BaseAddress);