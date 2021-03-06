/*****
 * 
 * ECE544 Project 2
 * Erik Rhodes
 * Test Program for Control System Circuit used in Project 2
 * 5/5/14
 *  
 *  Modified by Erik Rhodes and Caren Zgheib.  Original version by Roy Kravitz, with modifications
 *  from Robin Marshall
 *
 * Revision History
 * ================
 * 18-Apr-13	RK		Created the first version from test_PmodCtlSys_r1
 * 23-Apr-13	RK		Moved PmodCtlSys functions to their own driver and header files
 * 24-Apr-14    RM      Modified to remove references to PmodCtlSys.h calls and use student-made peripheral instead
 * May 2014	  ER & CZ	Modified to be used for project 2 in ECE544.
 *
 * Description:
 * ============
 * This program implements a minimal test program for the Control System Pmod
 * that is used in ECE 544 Project 2.  The program uses a Xilinx timer/counter module in PWM mode
 * and a light sensor with a custom designed peripheral + driver.
 *
 *
 *		sw[1:0] = 00:		BangBang: Use the rotary encoder to set a setpoint. The program controls the system	
 *							in using a bang-bang or on-off method: if the sensor indicates that the output of the 
 *							LED is below the setpoint, turn the dutycycle to maximum. Otherwise drive it to minimum.
 *
 * 		sw[1:0] = 01:		Performs a PID control on the output of the LED.  Monitor/Control/Save the response
 *							of the system. Use the pushbuttons to set the P(I)(D) gain values as well as the offset value.
 *							Press and hold the rotary encoder pushbutton to start the test.
 *                          Release the button when the display indicates that it is sending the data to be uploaded 
 *                          via serial port to a PC. 
 *
 *`		sw[1:0] = 10:		*RESERVED*
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

// light sensor peripheral driver 
#include "lightsensor.h"

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
#define LIGHTSENSOR_BASEADDR	XPAR_LIGHTSENSOR_0_BASEADDR

// Settings for PID calculations
#define NUM_FRQ_SAMPLES					250
#define MIN_DUTY                        1
#define MAX_DUTY                        99 
#define GAIN_INCREMENT                  0.1

#define PROP_INIT_GAIN                  0
#define INT_INIT_GAIN                   0
#define DERIV_INIT_GAIN                 0
#define OFFSET_INIT                     0
#define LCD_DISPLAYONOFF 				0x09
#define LCD_CURSOR_ON 					0x0E
#define LCD_CURSOR_OFF 					0x0C
#define	VOLT_MAX                        3.3	
#define	VOLT_MIN                        0.1	
#define SETPOINT_SCALE                  40.96
// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/*****************************************************************************/	



/****************************** typedefs and structures **********************/
typedef enum {TEST_BANG = 0x0, TEST_PID = 0x01, TEST_T_CALLS= 0x02, 
    TEST_CHARACTERIZE = 0x03, TEST_INVALID = 0xFF} Test_t;


typedef enum {PROPORTIONAL = 0x0, INTEGRAL = 0x01, DERIVATIVE = 0x02, OFFSET = 0x03} Control_t;

/*****************************************************************************/	



/************************** Global Variables *********************************/
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;								// Interrupt Controller instance
XTmrCtr	PWMTimerInst;								// PWM timer instance
XGpio	GPIOInst;									// GPIO instance
//XSpi	SPIInst;									// SPI controller (master) instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile unsigned long	timestamp;					// timestamp since the program began
volatile u32			gpio_port = 0;				// GPIO port register - maintained in program

// The following variables are shared between the functions in the program
// such that they must be global
volatile u16			sample[NUM_FRQ_SAMPLES];	// sample array
int						smpl_idx;					// index into sample array
int						frq_smple_interval;			// approximate sample interval			

int						pwm_freq;					// PWM frequency 
int						pwm_duty;					// PWM duty cycle
int 					dc_start;					// Starting PWM duty cycle

// Light Sensor Peripheral parameters
double  setpoint;
double  prop_gain, integral_gain, deriv_gain;
double 	slope;
Xuint32 offset;
bool	is_scaled;
Xuint32  freq_min_cnt;

//enum for control selection
Control_t PID_current_sel;

/*---------------------------------------------------------------------------*/					
int						debugen = 0;				// debug level/flag
/*---------------------------------------------------------------------------*/

/*****************************************************************************/	



/************************** Function Prototypes ******************************/
XStatus			DoTest_Characterize(void);									// Perform Characterization test

XStatus			do_init(void);												// initialize system
void			delay_msecs(u32 msecs);										// busy-wait delay for "msecs" milliseconds
void			voltstostrng(Xfloat32 v, char* s);							// converts volts to a string
void			update_lcd(int vin_dccnt, short vout_frqcnt);				// update LCD display

void			FIT_Handler(void);											// fixed interval timer interrupt handler

// Count to voltage conversion
double count_to_volts(u16 count);

void set_PID_vals();
XStatus calc_bang();
XStatus calc_PID();
void no_test_LCD();
void param_select();


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
    is_scaled = false;
    bool lcd_initial = true;
    char 		sp[20];
    u16 row = 1;
    u16 col = 2;
    bool calc_done = false;
    freq_min_cnt = 3000;


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
    next_test = TEST_INVALID;

    prop_gain     = PROP_INIT_GAIN;
    integral_gain = INT_INIT_GAIN;
    deriv_gain    = DERIV_INIT_GAIN;


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
    LCD_wrstring("PWM Ctrl sys");
    LCD_setcursor(2,0);
    LCD_wrstring("Erik R, Caren Z");
    NX3_writeleds(0xFF);
    //delay_msecs(2500);

    LCD_clrd();
    LCD_setcursor(1,0);
    LCD_wrstring("Characterizing..");
    //Run the LED characterization routine to establish sensor min's and max's
    DoTest_Characterize();
    LCD_setcursor(2,0);
    LCD_wrstring("Done.");



    NX3_writeleds(0x00);

    delay_msecs(500);
    //set initial screen
    // main loop - there is no exit except by hardware reset
    while (1)
    { 
        //set Vin to min or max depending on switch 3 value 
        if (btnsw & msk_SWITCH3)
        	{
        		pwm_duty = MAX_DUTY;
        		dc_start = MAX_DUTY;
        	}
        else
        	{
        		pwm_duty = MIN_DUTY;
        		dc_start = MIN_DUTY;
        	}

        // Write values to display when in "idle" state
        if (lcd_initial)
        {
    		LCD_clrd();
    		LCD_setcursor(1,0);
    		LCD_wrstring("P:   I:   D:   ");
    		LCD_setcursor(1,2);
    		LCD_setcursor(2,0);
    		LCD_wrstring("SP:+     OFF:   ");
    		lcd_initial = false;
    		LCD_setcursor(1,2);
        }
        no_test_LCD();
            NX3_readBtnSw(&btnsw);
            // Set which control measurement we're using
            if (btnsw & msk_BTN_NORTH)
            {
                // increment to the next selection.  If we're at the last enum, set it to the 1st (proportional)
                if (PID_current_sel == OFFSET) PID_current_sel = PROPORTIONAL;
                else PID_current_sel = (Control_t)((int)(PID_current_sel+1));  //casting to allow increment

                // cursor control logic
                if (PID_current_sel == PROPORTIONAL)
                {
                	row = 1;
                	col = 2;
                }
                else if (PID_current_sel == INTEGRAL)
                {
                	row = 1;
                	col = 7;
                }
                else if (PID_current_sel == DERIVATIVE)
                {
                	row = 1;
                	col = 12;
                }
                else if (PID_current_sel == OFFSET)
                {
                	row  = 2;
                	col  = 13;
                }
            }

            delay_msecs(20);
            //set cursor location and turn on cursor
            LCD_setcursor(row,col);
            LCD_docmd(LCD_DISPLAYONOFF, LCD_CURSOR_ON);

            if (btnsw & msk_BTN_EAST)
            {
                if (PID_current_sel == PROPORTIONAL)     prop_gain     += GAIN_INCREMENT;
                else if (PID_current_sel == INTEGRAL)    integral_gain += GAIN_INCREMENT;
                else if (PID_current_sel == DERIVATIVE)  deriv_gain    += GAIN_INCREMENT;
                else                                     offset        += GAIN_INCREMENT;
            }
            if (btnsw & msk_BTN_WEST)
            {
                if (PID_current_sel == PROPORTIONAL)     prop_gain     -= GAIN_INCREMENT;
                else if (PID_current_sel == INTEGRAL)    integral_gain -= GAIN_INCREMENT;
                else if (PID_current_sel == DERIVATIVE)  deriv_gain    -= GAIN_INCREMENT;
                else                                     offset        -= GAIN_INCREMENT;
            }


        // read sw[1:0] to get the test to perform.
        NX3_readBtnSw(&btnsw);
        test = btnsw & (msk_SWITCH1 | msk_SWITCH0);

        ROT_readRotcnt(&rotcnt);
        if (rotcnt != old_rotcnt)
        {
            //scale rotary count to setpoint values
            LCD_docmd(LCD_DISPLAYONOFF, LCD_CURSOR_OFF);
            setpoint = MAX(VOLT_MIN, MIN((rotcnt/SETPOINT_SCALE), VOLT_MAX));
            voltstostrng(setpoint, sp);
            old_rotcnt = rotcnt;
            LCD_setcursor(2, 3);
            LCD_wrstring(sp);
            LCD_setcursor(2, 3);
            LCD_wrstring('+');
        }

        if (test == TEST_T_CALLS) //unused 
        {
            next_test = TEST_INVALID;
            lcd_initial = true;
            delay_msecs(3000);

        } 


        else if ((test == TEST_BANG ) || (test == TEST_PID))  // Test 1 & 2 - control methods Bang Bang and PID 
        {
            Xfloat32	v;
            char		s[20];	

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

            /*Running Test (rotary pushbutton pushed): Show which control algorithm is running and
              display instructions for running test and uploading data.*/

            	if (test != next_test)
                {
                    if (test == TEST_BANG)
                    {
                        strcpy(s, "|BANG|Long Press");
                    }
                    else
                    {
                        strcpy(s, "|PID|Long Press");
                    }
                    delay_msecs(100);
                    LCD_clrd();
                    LCD_setcursor(1,0);
                    LCD_wrstring(s);
                    LCD_setcursor(2,0);
                    LCD_wrstring("RotBtn to start");
                }

                NX3_writeleds(0x01);
                if (test == TEST_BANG)  // perform bang bang calculations 
                {				
                    calc_bang();
                }
                else  // perform PID tests 
                {
                    calc_PID();
                }
                NX3_writeleds(0x00);

                //FEATURE: wait for user input to send data over
                NX3_readBtnSw(&btnsw);
                if ((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw))
                {
                    // light "Transfer" LED to indicate that data is being transmitted
                    // Show the traffic on the LCD
                    NX3_writeleds(0x02);
                    LCD_clrd();
                    LCD_setcursor(1, 0);
                    LCD_wrstring("Sending Data....");
                    LCD_setcursor(2, 0);
                    LCD_wrstring("S:    DATA:     ");

                    // print the descriptive heading followed by the data
                    if (test == TEST_BANG)
                    {
                        xil_printf("\n\rBang Bang! Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
                    }
                    else
                    {
                        xil_printf("\n\rPID Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
                    }

                    // trigger the serial charter program)
                    xil_printf("===STARTPLOT===\n");

                    // start with the second sample.  The first sample is not representative of
                    // the data.  This will pretty-up the graph a bit								
                    for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++)
                    {
                        u16 count;

                        count = sample[smpl_idx];
                        if (count > 4096)
                        	count = 4095;

                        v = (-3.3 / 4095.0) * (count) + 3.3;

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
                    lcd_initial = true;
                    delay_msecs(3000);
                }
            }  // do the step test and dump data

        }
        else if (test == TEST_CHARACTERIZE)  // Test 3 - Characterize Response
        {

            // start the test on the rising edge of the Rotary Encoder button press
            // the test will write the samples into the global "sample[]"
            // the samples will be sent to stdout when the Rotary Encoder button
            // is released
            NX3_readBtnSw(&btnsw);
            if ((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw))
            {
            	LCD_docmd(LCD_DISPLAYONOFF, LCD_CURSOR_OFF);
                // light "Run" (rightmost) LED to show the test has begun
                // and do the test.  The test will return when the measured samples array
                // has been filled.  Turn off the rightmost LED after the data has been
                if (test != next_test)
                {
                    LCD_clrd();
                    LCD_setcursor(1,0);
                    LCD_wrstring("|CHAR|Long Press");
                    LCD_setcursor(2,0);
                    LCD_wrstring("RotBtn to start");
                }
                // captured to let the user know he/she can release the button
                NX3_readBtnSw(&btnsw);
                if ((msk_BTN_ROT & btnsw) && (!calc_done))
                {
					NX3_writeleds(0x01);
					LCD_clrd();
					LCD_setcursor(1,0);
					LCD_wrstring("Characterizing..");
					DoTest_Characterize();
					LCD_setcursor(2,0);
					LCD_wrstring("Done.");
					NX3_writeleds(0x00);
				    delay_msecs(750);
				    LCD_setcursor(2,8);
				    LCD_wrstring("<OK>");
					calc_done = true;
					next_test = test;
                }

                NX3_readBtnSw(&btnsw);
                if (msk_BTN_ROT & btnsw)
                {
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
                        if (count > 4096)
                        	count = 4095;

                        v = (-3.3 / 4095.0) * (count) + 3.3;

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
                    delay_msecs(1000);
                    lcd_initial = true;
                    delay_msecs(3000);
                }
            }  // do the step test and dump data

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

/* 
 * Move the cursor to the different control parameters based on the user's input from the EAST and
 * WEST buttons
 */


void no_test_LCD()
{
	u32 btnsw;
	NX3_readBtnSw(&btnsw);
	    // Set which control measurement we're using
	if (btnsw & msk_BTN_EAST || btnsw & msk_BTN_WEST)
	{
		// Write Proportional gain
		LCD_setcursor(1, 2);
		LCD_wrstring("   ");
		LCD_setcursor(1, 2);
		LCD_putnum(prop_gain, 10);

		// Write Integral gain
		LCD_setcursor(1, 7);
		LCD_wrstring("   ");
		LCD_setcursor(1, 7);
		LCD_putnum(integral_gain, 10);

		// Write Derivative gain
		LCD_setcursor(1, 12);
		LCD_wrstring("   ");
		LCD_setcursor(1, 12);
		LCD_putnum(deriv_gain, 10);

		// Write Offset
		LCD_setcursor(2, 13);
		LCD_wrstring("   ");
		LCD_setcursor(2, 13);
		LCD_putnum(offset, 10);

	}
}


/**************************************************************************************
 * The pushbuttons allow the user to control the offset values for each of the control
 * calculations (Proportional, Integral, and Derivative). This ultimately allows for
 * fine-tuning of the values to achieve the best results. 
 *
 * The set_PID_vals() function reads the north pushbutton, changes the control method 
 * (P,I,D) as necessary,and increments/decrements that offset based on the east and 
 * west pushbuttons. It updates the global offset values and displays the new values
 * to the screen.
 **************************************************************************************/

void set_PID_vals(row, col)
{

    u32 btnsw;

    NX3_readBtnSw(&btnsw);
    // Set which control measurement we're using
    if (btnsw & msk_BTN_NORTH)
    {
        // increment to the next selection.  If we're at the last enum, set it to the 1st (proportional)
        if (PID_current_sel == OFFSET) PID_current_sel = PROPORTIONAL; 
        else PID_current_sel = (Control_t)((int)(PID_current_sel+1));  //casting to allow increment

        // cursor control logic
        if (PID_current_sel == PROPORTIONAL)
        {
        	row = 1;
        	col = 2;
        }
        else if (PID_current_sel == INTEGRAL)
        {
        	row = 1;
        	col = 7;
        }
        else if (PID_current_sel == DERIVATIVE)
        {
        	row = 1;
        	col = 12;
        }
        else if (PID_current_sel == OFFSET)
        {
        	row  = 2;
        	col  = 13;
        }
    }

    delay_msecs(20);
    //set cursor location and turn on cursor 
    LCD_setcursor(row,col);
    LCD_docmd(LCD_DISPLAYONOFF, LCD_CURSOR_ON);


    if (btnsw & msk_BTN_EAST)
    {
        if (PID_current_sel == PROPORTIONAL)     prop_gain     += GAIN_INCREMENT;
        else if (PID_current_sel == INTEGRAL)    integral_gain += GAIN_INCREMENT;
        else if (PID_current_sel == DERIVATIVE)  deriv_gain    += GAIN_INCREMENT;
        else                                     offset        += GAIN_INCREMENT;
    }
    if (btnsw & msk_BTN_WEST)
    {
        if (PID_current_sel == PROPORTIONAL)     prop_gain     -= GAIN_INCREMENT;
        else if (PID_current_sel == INTEGRAL)    integral_gain -= GAIN_INCREMENT;
        else if (PID_current_sel == DERIVATIVE)  deriv_gain    -= GAIN_INCREMENT;
        else                                     offset        -= GAIN_INCREMENT;
    }
}

/*************************************************************************************
 * The calc_bang function performs one of the control methods for the PWM output.
 * It reads the value from the light sensor, scales it, and check that value against
 * the desired setpoint value (established before the start of the test).  If the
 * current reading is lower than the setpoint, it puts the PWM at the maximum duty
 * cycle.  IF it's above, it puts the PWM output to the minimum duty.
 *************************************************************************************/

XStatus calc_bang()
{
    Xfloat32 volt_out;
    XStatus		Status;					// Xilinx return status

    // stabilize the PWM output (and thus the lamp intensity) at the
    // minimum before starting the test
    Status = PWM_SetParams(&PWMTimerInst, pwm_freq, dc_start);
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

    for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++)
    {
        // get count from light sensor and convert to voltage 
        sample[smpl_idx] = LIGHTSENSOR_Capture(LIGHTSENSOR_BASEADDR, slope, offset, is_scaled, freq_min_cnt);

        volt_out = (-3.3 / 4095.0) * (sample[smpl_idx]) + 3.3;
    	       
        if (volt_out < setpoint)
        {
            Status = PWM_SetParams(&PWMTimerInst, pwm_freq, MAX_DUTY);
            delay_msecs(1);
            if (Status == XST_SUCCESS)
            {
                PWM_Start(&PWMTimerInst);
            }
        }
        else
        {
            Status = PWM_SetParams(&PWMTimerInst, pwm_freq, MIN_DUTY);
            delay_msecs(1);
            if (Status == XST_SUCCESS)
            {
                PWM_Start(&PWMTimerInst);
            }
        }
        delay_msecs(100);
    }
    return XST_SUCCESS;
}


/*************************************************************************************
 * This method calculates the voltage that should be output to the PWM peripheral.  
 * It takes the reading from the light sensor, scales it appropriately, calculates
 * the P, I, and D values, converts it to the duty cycle value,  and outputs the 
 * value to the PWM params.
 **************************************************************************************/
XStatus calc_PID()
{
    double deriv, integral, error, prev_error = 0;
    double volt_out;
    int duty_out;
    XStatus		Status;					// Xilinx return status

    	// stabilize the PWM output (and thus the lamp intensity) at the
        // minimum before starting the test
        Status = PWM_SetParams(&PWMTimerInst, pwm_freq, dc_start);
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

    for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++)
    {
        delay_msecs(100);

        // get count from light sensor and convert to voltage 
        sample[smpl_idx] = LIGHTSENSOR_Capture(LIGHTSENSOR_BASEADDR, slope, offset, is_scaled, freq_min_cnt);
        volt_out = (-3.3 / 4095.0) * (sample[smpl_idx]) + 3.3;


        // calculate derivative;
        error = setpoint - volt_out;
        deriv = error - prev_error;

        // calculate integral
        if (error < setpoint/10) integral += error;
        else integral = 0; 

        // Control offset is gotten from characterization
        volt_out = offset + (error * prop_gain) + (deriv * deriv_gain) + (integral * integral_gain);
        duty_out = (volt_out)* (MAX_DUTY+1)/VOLT_MAX;

        // establish bounds
        if (duty_out < 1) duty_out = 1;
        if (duty_out > 99)duty_out = 99;

        // activate PWM
        Status = PWM_SetParams(&PWMTimerInst, pwm_freq, duty_out);
        if (Status == XST_SUCCESS)
        {							
            PWM_Start(&PWMTimerInst);
        }
    } 
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
    int		n;					// number of samples
    Xuint32         freq_max_cnt = 1000;

    int             i = 0;
    double diff = 0;


    // stabilize the PWM output (and thus the lamp intensity) at the
    // minimum before starting the test
    pwm_duty = STEPDC_MIN;
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
    smpl_idx = STEPDC_MIN;
    n = 0;
    tss = timestamp;
    while (smpl_idx <= STEPDC_MAX)
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

        sample[smpl_idx++] = LIGHTSENSOR_Capture(LIGHTSENSOR_BASEADDR, slope, offset, is_scaled, freq_min_cnt);

        n++;
        delay_msecs(50);
    }		
    frq_smple_interval = (timestamp - tss) / smpl_idx;

    //Find min and max values to set scaling 
    for (i = 1; i < STEPDC_MAX ; i++)
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

    // Send min and max to set scaling and calculate slope and offset
    diff = freq_max_cnt - freq_min_cnt;
    slope = 4095.0 / diff;

    is_scaled = true;
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

    //Initialize LIGHTSENSOR peripheral
    Status = LIGHTSENSOR_Init(LIGHTSENSOR_BASEADDR);
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
    // all devices that cause interrupts, specifically real mode so that
    // the the  FIT can cause interrupts thru the interrupt controller.
    Status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    } 

    // start the lightsensor

    Status = LIGHTSENSOR_Start(LIGHTSENSOR_BASEADDR);
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


    v = (-3.3 / 4095.0) * (frqcnt) + 3.3;
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



