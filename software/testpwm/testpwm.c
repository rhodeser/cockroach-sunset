/*****
 * testpwm.c - Test program for Xilinx timer/ctr PWM library
 *
 * Copyright Roy Kravitz,  Portland State University 2013, 2014, 2015
 *
 * 
 * Author:	Roy Kravitz
 * Version:	2.1
 * Date:	29-March-2014
 *
 * Revision History
 * ================
 * 10-Apr-09	RK		Created the first version
 * 01-Jan-11	RK		Modified for ISE 12.x and later.
 *						No changes to this other than to the Copyright line
 * 27-Mar-13	RK		Modified for Nexys 3, N3EIF (Nexys3 Extended Peripheral Interface) and ISE 14.x
 * 29-Mar-14	RK		Minor edits.  No functional changes.
 *
 *
 * Description:
 * ============
 * This program tests the Xilinx timer/counter PWM library for ECE 544.   The hardware for PWM is done with
 * a Xilinx Timer/Counter module set in PWM mode.   The PWM library builds on the Timer/Counter drivers
 * provided by Xilinx and encapsulates common PWM functions.  The program also provides a working example
 * of how to use the xps_s3eif driver to control the buttons, switches, rotary encoder, and display.
 *
 * The test program uses the rotary encoder and switches to choose a PWM frequency and duty cycle.  The selected
 * frequency and duty cycle are displayed on line 1 of the LCD.   The program also illustrates the use of a Xilinx
 * fixed interval timer module to generate a periodic interrupt for handling time-based (maybe) and/or sampled inputs/outputs
 *
 * NOTE TO ECE 544 STUDENTS:  This program could serve as a template for your Project 1 but may need to be customized to your system
 * In particular the #defines that map the parameters may be specific to my xparameters.h and could be different for yours
 * 	 				 
 *****/

/********** Include Files ***********/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xparameters.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xgpio.h"
#include "mb_interface.h"
#include "n3eif.h"
#include "pwm_tmrctr.h"


/************************** Macros and Constants ******************************/

// Toggles proj1 code on and off
#define PROJ1                           1       

// Debug mode
#define DEBUG                           1

// PWM and pulse detect timer parameters
#define PWM_TIMER_DEVICE_ID		XPAR_TMRCTR_0_DEVICE_ID

// N3EIF parameters
#define N3EIF_DEVICE_ID			XPAR_N3EIF_0_DEVICE_ID
#define N3EIF_BASEADDR			XPAR_N3EIF_0_BASEADDR
#define N3EIF_HIGHADDR			XPAR_N3EIF_0_HIGHADDR

// GPIO parameters
#define GPIO_DEVICE_ID			XPAR_XPS_GPIO_0_DEVICE_ID
#define GPIO_INPUT_CHANNEL		1
#define GPIO_OUTPUT_CHANNEL		2									
		
// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_XPS_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_XPS_INTC_0_FIT_TIMER_0_INTERRUPT_INTR		

// Fixed Interval timer - 83.33 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	XPAR_PROC_BUS_0_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40			

// PWM selected frequencies in Hz
#define PWM_FREQ_10HZ			10
#define PWM_FREQ_100HZ			100
#define PWM_FREQ_1KHZ			1000
#define PWM_FREQ_10KHZ			10000
#define PWM_FREQ_50KHZ			50000
#define PWM_FREQ_100KHZ			100000
#define PWM_FREQ_150KHZ			150000
#define PWM_FREQ_200KHZ			200000


#define INITIAL_FREQUENCY		PWM_FREQ_1KHZ
#define INITIAL_DUTY_CYCLE		50
#define DUTY_CYCLE_CHANGE		5

#define	PWM_SIGNAL_MSK			0x01
#define CLKFIT_MSK				0x01
#define PWM_FREQ_MSK			0x03
#define PWM_DUTY_MSK			0xFF


// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/*****************************************************************************/	

/****************************** typedefs and structures **********************/
/*****************************************************************************/	

/************************** Global Variables *********************************/
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;						// Interrupt Controller instance
XTmrCtr	PWMTimerInst;						// PWM timer instance
XGpio	GPIOInst;							// GPIO instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so it's frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()
volatile unsigned int	clkfit;				// clock signal is bit[0] (rightmost) of gpio 0 output port									
volatile unsigned long	timestamp;			// timestamp since the program began

// The following variables are shared between the functions in the program
// such that they must be global
int						pwm_freq;			// PWM frequency 
int						pwm_duty;			// PWM duty cycle
bool					new_perduty;		// new period/duty cycle flag
				
/*---------------------------------------------------------------------------*/					
int						debugen = 0;		// debug level/flag
/*---------------------------------------------------------------------------*/
		
/*****************************************************************************/	
	

/************************** Function Prototypes ******************************/
XStatus			do_init(void);											// initialize system
void			delay_msecs(unsigned int msecs);						// busy-wait delay for "msecs" miliseconds
void			voltstostrng(Xfloat32 v, char* s);						// converts volts to a string
void			update_lcd(int freq, int dutyccyle, u32 linenum);		// update LCD display
				
void			FIT_Handler(void);										// fixed interval timer interrupt handler
/*****************************************************************************/


/*********************************************/
/*              Main Program                 */
/*********************************************/		
int main()
{
	XStatus 	Status;
	u32			btnsw, old_btnsw = 0x000000FF;
	int			rotcnt, old_rotcnt = 0x1000;	
	bool		done = false;
	
	// initialize devices and set up interrupts, etc.
 	Status = do_init();
 	if (Status != XST_SUCCESS)
 	{
 		LCD_setcursor(1,0);
 		LCD_wrstring("****** ERROR *******");
 		LCD_setcursor(2,0);
 		LCD_wrstring("INIT FAILED- EXITING");
 		exit(XST_FAILURE);
 	}
 	
	// initialize the global variables
	timestamp = 0;							
	pwm_freq = INITIAL_FREQUENCY;
	pwm_duty = INITIAL_DUTY_CYCLE;
	clkfit = 0;
	new_perduty = false;
    
	// start the PWM timer and kick of the processing by enabling the Microblaze interrupt
	PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);	
	PWM_Start(&PWMTimerInst);
    microblaze_enable_interrupts();
    
	// display the greeting   
    LCD_setcursor(1,0);
    LCD_wrstring(" PWM LIB TEST R0");
	LCD_setcursor(2,0);
	LCD_wrstring(" by Roy Kravitz ");
	NX3_writeleds(0x000000FF);
	delay_msecs(2000);
	NX3_writeleds(0x00000000);
		
   // write the static text to the display
    LCD_clrd();
    LCD_setcursor(1,0);
    LCD_wrstring("G|FR:    DCY:  %");
    LCD_setcursor(2,0);
    LCD_wrstring("Vavg:           ");
      
    // main loop
	do
	{ 
		// check rotary encoder pushbutton to see if it's time to quit
		NX3_readBtnSw(&btnsw);
		if (btnsw & msk_BTN_ROT)
		{
			done = true;
		}
		else
		{
			new_perduty = false;				
			if (btnsw != old_btnsw)
			{	 
				switch (btnsw & PWM_FREQ_MSK)
				{
					case 0x00:	pwm_freq = PWM_FREQ_10HZ;	break;
					case 0x01:	pwm_freq = PWM_FREQ_100HZ;	break;
					case 0x02:	pwm_freq = PWM_FREQ_1KHZ;	break;
					case 0x03:	pwm_freq = PWM_FREQ_10KHZ;	break;
                                        case 0x04: 	pwm_freq = PWM_FREQ_50KHZ;	break;
                                        case 0x05: 	pwm_freq = PWM_FREQ_100KHZ;	break;
                                        case 0x06: 	pwm_freq = PWM_FREQ_150KHZ;	break;
                                        case 0x07: 	pwm_freq = PWM_FREQ_200KHZ;	break;
				}
				old_btnsw = btnsw;
				new_perduty = true;
			}
		
			// read rotary count and handle duty cycle changes
			// limit duty cycle to 0% to 99%
			ROT_readRotcnt(&rotcnt);
			if (rotcnt != old_rotcnt)
			{
				pwm_duty = MAX(0, MIN(rotcnt, 99));
				old_rotcnt = rotcnt;
				new_perduty = true;
			}

			// update generated frequency and duty cycle	
			if (new_perduty)
			{
				u32 freq, dutycycle;
				float vavg;
				char	s[10];
			
				// set the new PWM parameters - PWM_SetParams stops the timer
				Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
				if (Status == XST_SUCCESS)
				{
					PWM_GetParams(&PWMTimerInst, &freq, &dutycycle);
					update_lcd(freq, dutycycle, 1);
					vavg = dutycycle * .01 * 3.3;
					voltstostrng(vavg, s);
					LCD_setcursor(2,5);
					LCD_wrstring(s); 
										
					PWM_Start(&PWMTimerInst);
				}
			}
		}			
	} while (!done);
	
	// wait until rotary encoder button is released		
	do
	{
		NX3_readBtnSw(&btnsw);
		delay_msecs(10);
	} while ((btnsw & msk_BTN_ROT) == 0x80);

	// and say goodbye
	LCD_clrd();
	LCD_wrstring("That's all folks");
	delay_msecs(2000);
	LCD_clrd();
	exit(XST_SUCCESS);
 }



/*********************************************/
/*            Support Functions              */
/*********************************************/
		
/*****
 * do_init() - initialize the system
 * 
 * This function is executed once at start-up and after resets.  It initializes
 * the peripherals and registers the interrupt handlers
 *****/
XStatus do_init(void)
{
	XStatus 	Status;				// status from Xilinx Lib calls	
	
	// initialize the S3E starter board interface
	// rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE 
 	N3EIF_init(N3EIF_BASEADDR);
 	ROT_init(DUTY_CYCLE_CHANGE, true);
	ROT_clear();
	
	
	// initialize the GPIO instance
	Status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit input port.  bit[7:1] = reserved, bit[0] = PWM output (for duty cycle calculation)
	// GPIO channel 2 is an 8-bit output port.  bit[7:1] = reserved, bit[0] = FIT clock
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xFE);
			
	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts
	Status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	// initialize the interrupt controller
	Status = XIntc_Initialize(&IntrptCtlrInst,INTC_DEVICE_ID);
    if (Status != XST_SUCCESS)
    {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    Status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
                           (XInterruptHandler)FIT_Handler,
                           (void *)0);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
 
	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
    Status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

	// enable the FIT interrupt
    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
		

/*****
 * delay_msecs() - delay execution for "n" msecs
 * 
 *		timing is approximate but we're not looking for precision here, just
 *		a uniform delay function.  The function uses the global "timestamp" which
 *		is incremented every msec by FIT_Handler().
 *
 * NOTE:  Assumes that this loop is running faster than the fit_interval ISR 
 *****/
void delay_msecs(unsigned int msecs)
{
	unsigned long target;

	if ( msecs == 0 )
	{
		return;
	}
	target = timestamp + msecs;
	while ( timestamp != target )
	{
		// spin until delay is over
	}
}

/*****
 * voltstostrng() - converts volts to a fixed format string
 * 
 * accepts an float voltage reading and turns it into a 5 character string
 * of the following format:
 *		(+/-)x.yy 
 * where (+/-) is the sign, x is the integer part of the voltage and yy is
 * the decimal part of the voltage.
 *	
 * NOTE:  Assumes that s points to an array of at least 6 bytes.	
 *****/
 void	voltstostrng(float v, char* s)
 {
	float	dpf, ipf;
	u32		dpi;	
	u32		ones, tenths, hundredths;

	 // form the fixed digits 
	 dpf = modff(v, &ipf);
	 dpi = dpf * 100;
	 ones = abs(ipf) + '0';
	 tenths = (dpi / 10) + '0';
	 hundredths = (dpi - ((tenths - '0') * 10)) + '0';
	 
	 // form the string and return
	 if (ipf == 0 && dpf == 0)
	 {
	 	*s++ = ' ';
	 }
	 else	
	 { 
	 	*s++ = ipf >= 0 ? '+' : '-';
	 }
	 *s++ = (char) ones;
	 *s++ = '.';
	 *s++ = (char) tenths;
	 *s++ = (char) hundredths;
	 *s   = 0;
	 return;
 };
 
 
/*****
 * update_lcd() - update the frequency/duty cycle LCD display
 * 
 * writes the frequency and duty cycle to the specified line.  Assumes the
 * static portion of the display is already written and the format of each
 * line of the display is the same.
 *****/
void update_lcd(int freq, int dutycycle, u32 linenum)
{
	LCD_setcursor(linenum, 5);
	LCD_wrstring("   ");
	LCD_setcursor(linenum, 5);
	if (freq < 1000) // display Hz if frequency < 1Khz
	{
		LCD_putnum(freq, 10);
	}
	else  // display frequency in KHz
	{
		LCD_putnum((freq / 1000), 10);
		LCD_wrstring("K");
	}
	LCD_setcursor(linenum, 13);
	LCD_wrstring("  %");
	LCD_setcursor(linenum, 13);
	LCD_putnum(dutycycle, 10);
}


	
/*********************************************/
/*            Interrupt Handlers             */
/*********************************************/

/*****
 * FIT_Handler() - Fixed interval timer interrupt handler 
 *  
 * updates the global "timestamp" every milisecond.  "timestamp" is used for the delay_msecs() function
 * and as a time stamp for data collection and reporting
 *
 * toggles the FIT clock which can be used as a visual indication that the interrupt handler is being called
 *
 * NOTE TO ECE 544 STUDENTS:  When you implemement your software solution for pulse width detection in Project 1
 * this would be a reasonable place to do that processing.
 *****/

void FIT_Handler(void)
{
		
	static	int			ts_interval = 0;			// interval counter for incrementing timestamp


	// toggle FIT clock
	clkfit ^= 0x01;
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, clkfit);	

	// update timestamp	
	ts_interval++;	
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		ts_interval = 1;
	
        
        }
        if (PROJ1)
        {
        // check inputs
        // See which switch is enabled
            // if !(switch3 & MSK_SW3) 
            // sw_pwdet();
            // if (switch3 & MSK_SW3)
            // hw_pwdet();
        // 
        // how to acquire freq?
        // get wheel value
        //
        // Since fixed interval, period * num of up counts = high
        // 
        // 
        }
}	

// 
//TODO not in interrupt:
//design hw verilog to do same stuff
//create inputs/outputs with peripheral wheel (done?)
//
//Generate PWM
    // Get input from wheel, use table to output duty cycle %. 
// Drivers - clear load timer to complete operation
//
// LCD display
    // done - TOp line shows PWM duty cycle and frequency selected
    // Bottom line shows detected but with PWDET logic
    //
    // counter inside newer 
    // 
    //
