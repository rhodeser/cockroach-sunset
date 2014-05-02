/*****
 * test_PmodCtlSys_r3.c - Test Program for Control System Circuit used in Project 2
 *
 * Copyright Roy Kravitz, 2013, 2014
 *
 * 
 * Author:			Roy Kravitz
 * Modified by:		Robin Marshall
 * Version:			3.0
 * Date:			24-Apr-2014
 *
 * Revision History
 * ================
 * 18-Apr-13	RK		Created the first version from test_PmodCtlSys_r1
 * 23-Apr-13	RK		Moved PmodCtlSys functions to their own driver and header files
 * 24-Apr-14    RM      Modified to remove references to PmodCtlSys.h calls and use student-made peripheral instead
 *
 * Description:
 * ============
 * This program implements a minimal test program for the Control System Pmod
 * that will be used in ECE 544 Project 2.  The program uses a Xilinx timer/counter module in PWM mode
 * and a light sensor with a custom designed peripheral + driver.
 *
 * NOTE TO ECE 544 STUDENT
 *		Look for "//ECE544 Students:" in this program to see where edits need to be made in order
 * 		for this test application to work. There may, however, be other edits you'd like to (or need to)
 *		make elsewhere as well.
 *
 *		sw[1:0] = 00:		Use rotary knob to dial in the PWM duty cycle.  Read/display the
 *							the light sensor value in volts.
 *
 * 		sw[1:0] = 01:		Performs a step function on the PWM duty cycle.  Monitor/Save the response
 *							of the system. Press and hold the rotary encoder pushbutton to start the test.
 *                          Release the button when the "Run" (rightmost) LED turns off to upload the 
 *                          data via serial port to a PC. The step function goes from 1% duty cycle to
 *                          99% duty cycle.  
 *
 *`		sw[1:0] = 10:		Performs a step function on the PWM duty cycle.  Monitor/Save the response
 *							of the system. Press and hold the rotary encoder pushbutton to start the test.
 *                          Release the button when the "Run" (rightmost) LED turns off to upload the 
 *                          data via serial port to a PC. The step function goes from 99% duty cycle to
 *                          1% duty cycle.
 *
 *		sw[1:0] = 11:		Characterizes the response to the system by stepping the PWM duty cycle from
 *							min (1%) to max (99%) after allowing the light sensor output to settle. Press and hold
 *							the rotary encoder pushbutton to start the test.  Release the button when the
 *							"Run" (rightmost) LED turns off to upload the data via serial port to a PC
 *
 *****/

/********** Include Files ***********/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "xparameters.h"
#include "xbasic_types.h"
#include "xtmrctr.h"
#include "xintc.h"
#include "xgpio.h"
#include "n3eif.h"
#include "pwm_tmrctr.h"
#include "mb_interface.h"

//ECE544 Students: 
//#include "YOUR_PERIPHERAL_DRIVER.h"

/************************** Macros and Constants ******************************/
// Microblaze Cache Parameters
#define	USE_ICACHE				XPAR_MICROBLAZE_0_USE_ICACHE
#define	USE_DCACHE				XPAR_MICROBLAZE_0_USE_DCACHE
#define USE_DCACHE_WRITEBACK	XPAR_MICROBLAZE_DCACHE_USE_WRITEBACK

// GPIO and N3EIF parameters
#define GPIO_DEVICE_ID			XPAR_XPS_GPIO_0_DEVICE_ID
#define GPIO_BASEADDR			XPAR_XPS_GPIO_0_BASEADDR
#define GPIO_HIGHADDR			XPAR_XPS_GPIO_0_HIGHADDR
#define GPIO_OUTPUT_CHANNEL		1

#define N3EIF_DEVICEID			XPAR_N3EIF_0_DEVICE_ID
#define N3EIF_BASEADDR			XPAR_N3EIF_0_BASEADDR
#define N3EIF_HIGHADDR			XPAR_N3EIF_0_HIGHADDR

// PWM timer parameters
// Set PWM frequency = 10KHz, duty cycle increments by 5%
#define PWM_TIMER_DEVICE_ID		XPAR_XPS_TIMER_0_DEVICE_ID
#define PWM_TIMER_BASEADDR		XPAR_XPS_TIMER_0_BASEADDR
#define PWM_TIMER_HIGHADDR		XPAR_XPS_TIMER_0_HIGHADDR
#define PWM_FREQUENCY			10000	
#define PWM_VIN					3.3	
#define DUTY_CYCLE_CHANGE		2

// Min and Max duty cycle for step and characterization tests
#define STEPDC_MIN				1
#define STEPDC_MAX				99					
		
// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_XPS_INTC_0_DEVICE_ID
#define INTC_BASEADDR			XPAR_XPS_INTC_0_BASEADDR
#define INTC_HIGHADDR			XPAR_XPS_INTC_0_HIGHADDR
#define TIMER_INTERRUPT_ID		XPAR_XPS_INTC_0_XPS_TIMER_0_INTERRUPT_INTR
#define FIT_INTERRUPT_ID		XPAR_XPS_INTC_0_FIT_TIMER_0_INTERRUPT_INTR 
				
// Fixed Interval timer - 66.67MHz input clock, 5KHz output clock
// 1 msec time interval for FIT interrupt handler
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	XPAR_PROC_BUS_0_FREQ_HZ
#define FIT_COUNT				13333
#define FIT_CLOCK_FREQ_HZ		(FIT_IN_CLOCK_FREQ_HZ / FIT_COUNT)
#define FIT_COUNT_1MSEC			(FIT_CLOCK_FREQ_HZ / 1000)	

// Light Sensor Peripheral parameters
// Add whatever constants you need to use your Light Sensor peripheral driver


// Settings for PID calculations
#define NUM_FRQ_SAMPLES			250	
#define MIN_DUTY                        1
#define MAX_DUTY                        99 
#define CONTROL_OFFSET                  200
#define CONTROL_GAIN                    10 
#define INTL_GAIN                       
#define DERIV_GAIN
#define PROP_GAIN
#define USE_INTEGRAL                    1
#define USE_DERIV                       1
#define OFFSET_INCREMENT
#define PROP_INIT_OFFSET                
#define INT_INIT_OFFSET                
#define DERIV_INIT_OFFSET                
		
// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/*****************************************************************************/	



/****************************** typedefs and structures **********************/
typedef enum {TEST_TRACKING = 0x0, TEST_STEPLOHI = 0x01, TEST_STEPHILO = 0x02, 
				TEST_CHARACTERIZE = 0x03, TEST_INVALID = 0xFF} Test_t;


typedef enum {PROPORTIONAL = 0x0, INTEGRAL = 0x01, DERIVATIVE = 0x02} Control_t;

/*****************************************************************************/	



/************************** Global Variables *********************************/
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;								// Interrupt Controller instance
XTmrCtr	PWMTimerInst;								// PWM timer instance
XGpio	GPIOInst;									// GPIO instance
XSpi	SPIInst;									// SPI controller (master) instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile unsigned long	timestamp;					// timestamp since the program began
volatile u32			gpio_port = 0;				// GPIO port register - maintained in program

// The following variables are shared between the functions in the program
// such that they must be global
u16						sample[NUM_FRQ_SAMPLES];	// sample array 	
int						smpl_idx;					// index into sample array
int						frq_smple_interval;			// approximate sample interval			

int						pwm_freq;					// PWM frequency 
int						pwm_duty;					// PWM duty cycle

// Light Sensor Peripheral parameters
// Add whatever global variables you need to use your Light Sensor peripheral driver
double  setpoint;
double  prop_offset, integral_offset, deriv_offset;
//enum for control selection
Control_t PID_current_sel;
				
/*---------------------------------------------------------------------------*/					
int						debugen = 0;				// debug level/flag
/*---------------------------------------------------------------------------*/
		
/*****************************************************************************/	

	

/************************** Function Prototypes ******************************/
XStatus 		DoTest_Track(void);											// Perform Tracking test
XStatus			DoTest_Step(int dc_start);									// Perform Step test
XStatus			DoTest_Characterize(void);									// Perform Characterization test
			
XStatus			do_init(void);												// initialize system
void			delay_msecs(u32 msecs);										// busy-wait delay for "msecs" milliseconds
void			voltstostrng(Xfloat32 v, char* s);							// converts volts to a string
void			update_lcd(int vin_dccnt, short vout_frqcnt);				// update LCD display

void			FIT_Handler(void);											// fixed interval timer interrupt handler

// Count to voltage conversion
double count_to_volts(u16 count);

void set_PID_vals();
void calc_bang();
void calc_PID();
double duty_to_volts();
//TODO remove or create scale count function
void scale_count(u16 min, u16 max);

/*****************************************************************************/


/*********************************************/
/*              Main Program                 */
/*********************************************/		
int main()
{
	XStatus 	Status;
	u32			btnsw = 0x00000000, old_btnsw = 0x000000FF;
	int			rotcnt, old_rotcnt = 0x1000;
	Test_t		test, next_test;

        prop_offset     = PROP_INIT_OFFSET;
        integral_offset = INT_INIT_OFFSET;
        deriv_offset    = DERIV_INIT_OFFSET;

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

	// initialize the variables
	timestamp = 0;							
	pwm_freq = PWM_FREQUENCY;
	pwm_duty = STEPDC_MIN;
	next_test = TEST_INVALID;

	// Enable the Microblaze caches and kick off the processing by enabling the Microblaze
	// interrupt.  This starts the FIT timer which updates the timestamp.
	if (USE_ICACHE == 1)
	{
		microblaze_invalidate_icache();
		microblaze_enable_icache();
	}
	if (USE_DCACHE == 1)
	{
		microblaze_invalidate_dcache();
		microblaze_enable_dcache();
	}
	microblaze_enable_interrupts();
	 	  	
 	// display the greeting   
    LCD_setcursor(1,0);
    LCD_wrstring("PWM Control system ");
	LCD_setcursor(2,0);
	LCD_wrstring("Erik R, Caren Z");
	NX3_writeleds(0xFF);
	//Run the LED characterization routine to establish sensor min's and max's
    DoTest_Characterize();
	NX3_writeleds(0x00);
       
    // main loop - there is no exit except by hardware reset
	while (1)
	{ 
		// read sw[1:0] to get the test to perform.
		NX3_readBtnSw(&btnsw);
		test = btnsw & (msk_SWITCH1 | msk_SWITCH0);
			
		if (test == TEST_TRACKING)  // Test 0 = Track PWM voltage
		{
			// write the static info to display if necessary
			if (test != next_test)
			{
				LCD_clrd();
				LCD_setcursor(1,0);
				LCD_wrstring("| TRK| Vi: sx.xx");
				LCD_setcursor(2,0);
				LCD_wrstring("Vo:sx.xx C:sxxxx");
			}
				 						
			// read rotary count and handle duty cycle changes
			// limit duty cycle to between STEPDC_MIN and STEPDC_MAX
			// PWM frequency does not change in this test
			ROT_readRotcnt(&rotcnt);
			if (rotcnt != old_rotcnt)
			{
				pwm_duty = MAX(STEPDC_MIN, MIN(rotcnt, STEPDC_MAX));
                                //TODO change function name
                                setpoint = count_to_volts(pwm_duty);
				old_rotcnt = rotcnt;
			}
			DoTest_Track();
			next_test = TEST_TRACKING;
		}   // Test 0 = Track PWM voltage
		else if ((test == TEST_STEPHILO) || (test == TEST_STEPLOHI))  // Test 1 & 2 - Step response 
		{
			Xfloat32	v;
			char		s[20];	
			
			// write the static info to the display if necessary
			if (test != next_test)
			{
				if (test == TEST_STEPHILO)
				{
					strcpy(s, "|HILO|Press RBtn");
				}
				else
				{
					strcpy(s, "|LOHI|Press RBtn");
				}
				
				LCD_clrd();
				LCD_setcursor(1,0);
				LCD_wrstring(s);
				LCD_setcursor(2,0);
				LCD_wrstring("LED OFF-Release ");
			}
			
			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the light detector samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released
			//
			// NOTE ON DETECTING BUTTON CHANGES:
			// btnsw ^ old_btnsw will set the bits for all of the buttons and switches that
			// have changed state since the last time the buttons and switches were read
			// msk_BTN_ROT & btnsw will test whether the rotary encoder was one of the
			// buttons that changed from 0 -> 1	
			if ((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw))  // do the step test and dump data  
			{										
				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been 
				// captured to let the user know he/she can release the button
				NX3_writeleds(0x01);
				if (test == TEST_STEPHILO)  // perform High->Low step test
				{				
					DoTest_Step(STEPDC_MAX);
				}
				else  // perform Low->High step
				{
					DoTest_Step(STEPDC_MIN);
				}
				NX3_writeleds(0x00);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout
				do
				{
					NX3_readBtnSw(&btnsw);
					delay_msecs(10);
				} while ((btnsw & msk_BTN_ROT) == 0x80);
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD
				NX3_writeleds(0x02);
				LCD_clrd();
				LCD_setcursor(1, 0);
				LCD_wrstring("Sending Data....");
				LCD_setcursor(2, 0);
				LCD_wrstring("S:    DATA:     ");

				// print the descriptive heading followed by the data
				if (test == TEST_STEPHILO)
				{
					xil_printf("\n\rHigh->Low Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
				}
				else
				{
					xil_printf("\n\rLow->High Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
				}
				
				// trigger the serial charter program)
				xil_printf("===STARTPLOT===\n");

				// start with the second sample.  The first sample is not representative of
				// the data.  This will pretty-up the graph a bit								
				for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++)
				{
					u16 count;
					
					count = sample[smpl_idx];
					
					//ECE544 Students:
                    //Convert from count to 'volts'
                    //v = count_to_volts(count);
					
					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);
					
					LCD_setcursor(2, 2);
					LCD_wrstring("   ");
					LCD_setcursor(2, 2);
					LCD_putnum(smpl_idx, 10);
					LCD_setcursor(2, 11);
					LCD_wrstring("     ");
					LCD_setcursor(2, 11);
					LCD_putnum(count, 10);
				}
				
				// stop the serial charter program				
				xil_printf("===ENDPLOT===\n");
				
				NX3_writeleds(0x00);
				old_btnsw = btnsw;								
				next_test = TEST_INVALID;			
			}  // do the step test and dump data
			else
			{
				next_test = test;
			}
		} // Test 1 & 2 - Step response
		else if (test == TEST_CHARACTERIZE)  // Test 3 - Characterize Response
		{
			if (test != next_test)
			{				
				LCD_clrd();
				LCD_setcursor(1,0);
				LCD_wrstring("|CHAR|Press RBtn");
				LCD_setcursor(2,0);
				LCD_wrstring("LED OFF-Release ");
			}
			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released
			if ((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw))  // do the step test and dump data  
			{
				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been
				// captured to let the user know he/she can release the button
				NX3_writeleds(0x01);			
				DoTest_Characterize();
				NX3_writeleds(0x00);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout
				do
				{
					NX3_readBtnSw(&btnsw);
					delay_msecs(10);
				} while ((btnsw & msk_BTN_ROT) == 0x80);
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD
				NX3_writeleds(0x02);
				LCD_clrd();
				LCD_setcursor(1, 0);
				LCD_wrstring("Sending Data....");
				LCD_setcursor(2, 0);
				LCD_wrstring("S:    DATA:     ");

				xil_printf("\n\rCharacterization Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);

				// trigger the serial charter program)
				xil_printf("===STARTPLOT===\n\r");
				
				for (smpl_idx = STEPDC_MIN; smpl_idx <= STEPDC_MAX; smpl_idx++)
				{
					u16 		count;
					Xfloat32	v;
					char		s[10]; 
					
					count = sample[smpl_idx];
					
					//ECE544 Students:
                    //Convert from count to 'volts'
                    //v = YOUR_FUNCTION(count);
					
					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);

					LCD_setcursor(2, 2);
					LCD_wrstring("   ");
					LCD_setcursor(2, 2);
					LCD_putnum(smpl_idx, 10);
					LCD_setcursor(2, 11);
					LCD_wrstring("     ");
					LCD_setcursor(2, 11);
					LCD_putnum(count, 10);
				}

				// stop the serial charter program				
				xil_printf("===ENDPLOT===\n\r");
				
				NX3_writeleds(0x00);
				old_btnsw = btnsw;								
				next_test = TEST_INVALID;
			}  // do the step test and dump data
			else
			{
				next_test = test;
			}
		} // Test 3 - Characterize Response
		else  // outside the current test range - blink LED's and hang
		{
			// should never get here but just in case
			NX3_writeleds(0xFF);
			delay_msecs(2000);
			NX3_writeleds(0x00);
		}
		// wait a bit and start again
		delay_msecs(100);			 								
	}  // while(1) loop
 }  // end main()

 
/*********************************************/
/*             Test Functions                */
/*********************************************/



/**************************************************************************************
 * The pushbuttons allow the user to control the offset values for each of the control
 * calculations (Proportional, Integral, and Derivative). This ultimately allows for
 * fine-tuning of the values to achieve the best results. 
 *
 * The set_PID_val() function reads the north pushbutton, changes the control method 
 * (P,I,D) as necessary,and increments/decrements that offset based on the east and 
 * west pushbuttons. It updates the global offset values and displays the new values
 * to the screen.
**************************************************************************************/


// TODO: resolution for set point should be based off .1 volts-ish
// TODO: make define masks for BTN_NORTH, ...

void set_PID_vals()
{

    // Set which control measurement we're using
    if (btnsw & BTN_NORTH)
        {
        // increment to the next selection.  If we're at the last enum, set it to the 1st (proportional)
        if (PID_current_sel == DERIVATIVE) PID_current_sel = PROPORTIONAL; 
        else PID_current_sel++;
        }
       
    if (btnsw & BTN_WEST)
    {
        // TODO make more compact
        // offset[select] += OFFSET_INCREMENT;
       if (PID_current_sel == PROPORTIONAL)     prop_offset     += OFFSET_INCREMENT;
       else if (PID_current_sel == INTEGRAL)    integral_offset += OFFSET_INCREMENT;
       else if (PID_current_sel == DERIVATIVE)  deriv_offset    += OFFSET_INCREMENT;
    }
    if (btnsw & BTN_EAST)
    {
        //make more compact
       if (PID_current_sel == PROPORTIONAL)     prop_offset     -= OFFSET_INCREMENT;
       else if (PID_current_sel == INTEGRAL)    integral_offset -= OFFSET_INCREMENT;
       else if (PID_current_sel == DERIVATIVE)  deriv_offset    -= OFFSET_INCREMENT;
    }

// TODO: update_lcd(...);

}

double duty_to_volts()
{
// want to change values more than 1 at a time or else would be too slow
    //readROT_count get what back as rotcnt?
rotcnt/MAX_ROT_CNT = volts/MAX_VOLTS;

}
void calc_prop()
{
    double frq_cnt;
    u16 volt_out;
    double setpoint;

    delay_msecs(1);
    // get count from light sensor and convert to voltage 
    frq_cnt = read_light_sensor();

    // TODO change these 2 functs to correct names
    volt_out = count_to_volts(frq_cnt);

    // Control offset is gotten from characterization
    volt_out = CONTROL_OFFSET + (setpoint - volt_out) * CONTROL_GAIN;

    // establish bounds
    if (volt_out < MIN_DUTY) volt_out = MIN_DUTY;
    if (volt_out > MAX_DUTY) volt_out = MAX_DUTY;

    // output voltage to PWM
    Status = PWM_SetParams(&PWMTimerInst, pwm_freq, volt_out);
    if (Status == XST_SUCCESS)
    {							
        PWM_Start(&PWMTimerInst);
    }
}


/*************************************************************************************
 * The calc_bang function performs one of the control methods for the PWM output.
 * It reads the value from the light sensor, scales it, and check that value against
 * the desired setpoint value (established before the start of the test).  If the
 * current reading is lower than the setpoint, it puts the PWM at the maximum duty
 * cycle.  IF it's above, it puts the PWM output to the minimum duty.
*************************************************************************************/

void calc_bang()
{
    double frq_cnt;
    u16 volt_out;

    delay_msecs(1);

    // get count from light sensor and convert to voltage 
    frq_cnt = read_light_sensor();
    volt_out = count_to_volts(frq_cnt);

    if (volt_out < setpoint)
    {
        Status = PWM_SetParams(&PWMTimerInst, pwm_freq, MAX_DUTY);
        if (Status == XST_SUCCESS)
        {							
            PWM_Start(&PWMTimerInst);
        }
    }
    else
    {
        Status = PWM_SetParams(&PWMTimerInst, pwm_freq, MIN_DUTY);
        if (Status == XST_SUCCESS)
        {							
            PWM_Start(&PWMTimerInst);
        }
    }
}



/*************************************************************************************
 * This method calculates the voltage that should be output to the PWM peripheral.  
 * It takes the reading from the light sensor, scales it appropriately, calculates
 * the P, I, and D values, and outputs the value to the PWM params.
**************************************************************************************/

void calc_PID()
{

    double frq_cnt;
    u16 deriv, integral; 
    double volt_out;    //make errors global?
    u16 error, prev_error;

        delay_msecs(1);

        // get count from light sensor and convert to voltage 
        frq_cnt = read_light_sensor();
        volt_out = count_to_volts(frq_cnt);
        error = setpoint - volt_out;

        // calculate derivative;
        deriv = error - prev_error;

        // calculate integral
        if (error < setpoint/10) integral += error;
        else integral = 0; 

        // if we don't want integral or derivative calculation, set it to 0
        if (!USE_INTEGRAL)  integral = 0;
        if (!USE_DERIV)     deriv    = 0; 

        // Control offset is gotten from characterization
        volt_out = CONTROL_OFFSET + (error * prop_offset) + (deriv * deriv_offset) + (integral * integral_offset);

        // establish bounds
        if (volt_out < 1) volt_out = 1;
        if (volt_out > 99)volt_out = 99;

        Status = PWM_SetParams(&PWMTimerInst, pwm_freq, volt_out);
        if (Status == XST_SUCCESS)
        {							
            PWM_Start(&PWMTimerInst);
        }
    
}


/*****
 * DoTest_Track() - Perform the Tracking test
 * 
 * This function uses the global "pwm_freq" and "pwm_duty" values to adjust the PWM
 * duty cycle and thus the intensity of the LED.  The function displays
 * the light detector reading as it tracks changes in the
 * LED intensity.  This test runs continuously until a different test is selected.
 * Returns XST_SUCCESS since this test can't fail.  Returns approximate sample interval
 * in the global variable "frq_sample_interval"
 *****/ 
XStatus DoTest_Track(void)
{
	static int		old_pwm_freq = 0;			// old pwm_frequency and duty cycle
	static int		old_pwm_duty = 200;			// these values will force the initial display	
	u16				frq_cnt;					// light detector counts to display
	XStatus			Status;						// Xilinx return status
	unsigned		tss;						// starting timestamp			

	if ((pwm_freq != old_pwm_freq) || (pwm_duty != old_pwm_duty))
	{	
		// set the new PWM parameters - PWM_SetParams stops the timer
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
		if (Status == XST_SUCCESS)
		{							
			PWM_Start(&PWMTimerInst);
		}

		tss = timestamp;	
		
		//ECE544 Students:
        //make the light sensor measurement
		//frq_cnt = YOUR FUNCTION HERE;
		
		delay_msecs(1);
		frq_smple_interval = timestamp - tss;
				
		// update the display and save the frequency and duty
		// cycle for next time
		update_lcd(pwm_duty, frq_cnt);
		old_pwm_freq = pwm_freq;
		old_pwm_duty = pwm_duty;
	}
	return XST_SUCCESS;
}



/*****
 * DoTest_Step() - Perform the Step test
 * 
 * This function stabilizes the duty cycle at "dc_start" for
 * about a second and a half and then steps the duty cycle from min to max or
 * max to min depending on the test. NUM_FRQ_SAMPLES are collected
 * into the global array sample[].  An approximate sample interval
 * is written to the global variable "frq_smpl_interval"
 *****/ 
XStatus DoTest_Step(int dc_start)
{	
	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	u16			frq_cnt;				// measured counts to display
		
	// stabilize the PWM output (and thus the lamp intensity) before
	// starting the test
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, dc_start);
	if (Status == XST_SUCCESS)
	{							
		PWM_Start(&PWMTimerInst);
	}
	else
	{
		return XST_FAILURE;
	}
	//Wait for the LED output to settle before starting
    delay_msecs(1500);
		
	if (dc_start > STEPDC_MAX / 2)
	{
		 Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MIN); 
	}
	else
	{
		 Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MAX); 
	}		
	if (Status == XST_SUCCESS)
	{							
		PWM_Start(&PWMTimerInst);
		pwm_duty = dc_start;
	}
	else
	{
		return XST_FAILURE;
	}
	
	// gather the samples
	smpl_idx = 0;
	tss = timestamp;
	while (smpl_idx < NUM_FRQ_SAMPLES)
	{
	
		//ECE544 Students:
        //make the light sensor measurement
		//sample[smpl_idx++] = YOUR FUNCTION HERE;
		
	}		
	frq_smple_interval = (timestamp - tss) / NUM_FRQ_SAMPLES;
	return XST_SUCCESS;
}
	
	
/*****
 * DoTest_Characterize() - Perform the Characterization test
 * 
 * This function starts the duty cycle at the minimum duty cycle and
 * then sweeps it to the max duty cycle for the test.
 * Samples are collected into the global array sample[].  
 * The function toggles the TEST_RUNNING signal high for the duration
 * of the test as a debug aid and adjusts the global "pwm_duty"
 *
 * The test also sets the global frequency count min and max counts to
 * help limit the counts to the active range for the circuit
 *****/
XStatus DoTest_Characterize(void)
{
	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	u16			frq_cnt;				// counts to display
	int			n;						// number of samples
	Xuint32		freq, dutyfactor;		// current frequency and duty factor


	// stabilize the PWM output (and thus the lamp intensity) at the
	// minimum before starting the test
	pwm_duty = PWM_STEPDC_MIN;
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (Status == XST_SUCCESS)
	{							
		PWM_Start(&PWMTimerInst);
	}
	else
	{
		return -1;
	}
	//Wait for the LED output to settle before starting
    delay_msecs(1500);
		
	// sweep the duty cycle from STEPDC_MIN to STEPDC_MAX
	smpl_idx = PWM_STEPDC_MIN;
	n = 0;
	tss = timestamp;
	while (smpl_idx <= PWM_STEPDC_MAX)
	{
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, smpl_idx);
		if (Status == XST_SUCCESS)
		{							
			PWM_Start(&PWMTimerInst);
		}
		else
		{
			return -1;
		}
		
        //ECE544 Students:
        // make the light sensor measurement
		//sample[smpl_idx++] = YOUR FUNCTION HERE;
		
		n++;
		delay_msecs(50);
	}		
	frq_smple_interval = (timestamp - tss) / smpl_idx;

    //ECE544 Students:
    //Find the min and max values and set the scaling/offset factors to use for your convert to 'voltage' function.
    //NOTE: It may also be useful to scale the actual 'count' values to a range of 0 - 4095 for the SerialCharter application to work correctly 
    
    // define above
    // fp values?
    u16 freq_max_cnt = 0;
    u16 freq_min_cnt = 0;
    for (i = 0; i < NUM_FRQ_SAMPLES; i++) 
    {
        if (sample[i] < freq_min_cnt)
        {
            freq_min_cnt = sample[i];
        }
        if (sample[i] > freq_max_cnt)
        {
            freq_max_cnt = sample[i];
        }
    }

    scale_count(freq_min_cnt,freq_max_cnt);
    return n;
}

	
	
/*********************************************/
/*            Support Functions              */
/*********************************************/

/*****
 * do_init() - initialize the system
 * 
 * This function is executed once at start-up and after a reset.  It initializes
 * the peripherals and registers the interrupt handlers
 *****/
XStatus do_init(void)
{
	XStatus 	Status;				// status from Xilinx Lib calls	
	
	// initialize the N3EIF hardware and driver
	// rotary encoder is set to increment duty cycle from 0 by 5 w/
	// no negative counts
 	N3EIF_init(N3EIF_BASEADDR);
 	ROT_init(DUTY_CYCLE_CHANGE, true);
	ROT_clear();

	// initialize the GPIO instance
	Status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit output port that your application can
	// use.  None of the bits are used by this program
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xF0);
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, gpio_port);
	
			
	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts
	Status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
    //ECE544 Students:
    //Initialize your peripheral here
			
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
	// all devices that cause interrupts, specifically real mode so that
	// the the  FIT can cause interrupts thru the interrupt controller.
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
 * NOTE:  Assumes that this loop is running faster than the fit_interval ISR (every msec)
 *****/
void delay_msecs(u32 msecs)
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
 * accepts an Xfloat32 voltage reading and turns it into a 5 character string
 * of the following format:
 *		(+/-)x.yy 
 * where (+/-) is the sign, x is the integer part of the voltage and yy is
 * the decimal part of the voltage.
 *	
 * NOTE:  Assumes that s points to an array of at least 6 bytes.	
 *****/
 void	voltstostrng(Xfloat32 v, char* s)
 {
	Xfloat32	dpf, ipf;
	Xuint32		dpi;	
	Xuint32		ones, tenths, hundredths;

	 // form the fixed digits 
	 dpf = modff(v, &ipf);
	 dpi = dpf * 100;
	 ones = abs(ipf) + '0';
	 tenths = (dpi / 10) + '0';
	 hundredths = (dpi - ((tenths - '0') * 10)) + '0';
	 
	 // form the string and return
	 *s++ = ipf == 0 ? ' ' : (ipf > 0 ? '+' : '-');
	 *s++ = (char) ones;
	 *s++ = '.';
	 *s++ = (char) tenths;
	 *s++ = (char) hundredths;
	 *s   = 0;
	 return;
 }
	  
	 
 /*****
  * update_lcd() - update the LCD display with a new count and voltage
  *
  * writes the display with new information.  "vin_dccnt" is the  unsigned PWM duty
  * cycle and "frqcnt" is the signed frq_count.  The function assumes that the
  * static portion of the display has been written and that the dynamic portion of
  * the display is the same for all tests
  *****/
 void update_lcd(int vin_dccnt, short frqcnt)
 {
 	Xfloat32	v;
 	char		s[10];

 	// update the PWM data
 	v = vin_dccnt * .01 * PWM_VIN;
 	voltstostrng(v, s);
 	LCD_setcursor(1, 11);
 	LCD_wrstring("      ");
 	LCD_setcursor(1, 11);
 	LCD_wrstring(s);

 	// update the data
    // ECE544 Students: Convert frequency count to 'volts'
    // v = YOUR_FUNCTION(frqcnt);
 	voltstostrng(v, s);
 	LCD_setcursor(2, 3);
 	LCD_wrstring("     ");
 	LCD_setcursor(2, 3);
 	LCD_wrstring(s);
 	LCD_setcursor(2, 11);
 	LCD_wrstring("     ");
 	LCD_setcursor(2, 11);
 	LCD_putnum(frqcnt, 10);
 	return;
 }


/*********************************************/
/*            Interrupt Handlers             */
/*********************************************/

/*****
 * FIT_Handler() - Fixed interval timer interrupt handler 
 *  
 * updates the global "timestamp" every millisecond.  "timestamp" is used for the delay_msecs() function
 * and as a time stamp for data collection and reporting
 *****/
void FIT_Handler(void)
{	
	static	int			ts_interval = 0;			// interval counter for incrementing timestamp
			
	// update timestamp	
	ts_interval++;	
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		ts_interval = 1;
	}
}	
