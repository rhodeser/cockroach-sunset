/* proj3_starter_app - ECE 544 Project #3 Application template
 *
 * Copyright (c) 2012, 2013, 2014, Portland State University by Roy Kravitz.  All rights reserved.
 *
 * Author:		Caren Zgheib
 * Date:		19-May-2014
 * Revision:	2.0
 *
 * Revision History:
 * -----------------
 * 	21-Feb-2012		RK		Created this program from the xilkernel sample application produced by SDK
 * 	16-May-2013		RK		Updated the program for Nexys3 board and revised homework 2 assignment
 *	12-May-2014		RK		Minor changes to ease support for both Nexys 3 and Nexys 4.  No functional changes
 *  19-May-2014		CZ		Edited the macros to match my implementation of the system and added the initialization code
 *
 * 	Description:
 * 	------------
 * 	This program implements ECE 544 Project #3.  It is a Xilinx-based design that is meant to be a learning tool
 * 	for incorporating several of the key functions provided by Xilkernel.   The application sets up 4 independent
 * 	threads.  Thread 1 periodically reads the switches on the FPGA development board and sends an update to the LED thread.
 * 	Thread 2 responds to pushbutton press interrupts and sends an update to the LED thread whenever the state of
 * 	any of the pushbuttons on the FPGA development board change.   Thread 3 writes the new pushbutton and switch state to the
 * 	LEDs.   The three threads communicate via a message queue.  Thread 1 communicates to its interrupt handler through
 * 	the use of a semaphore.  The application also makes use of the Xilinx watchdog timer.
 * 	The 4th thread (the master thread) sets up the Microblaze, initializes the peripherals, creates the other threads and
 * 	the semaphore and then enters a main loop where it wakes up periodically to reset the WDT.  Status messages are sent
 * 	via STDOUT through the USB serial port which can be connected to a PC running a terminal emulator such as putty.
 *
 * <your additional comments>
 *
 */

// Includes
#include "xmk.h"
#include "os_config.h"
#include "config/config_param.h"
#include "sys/ksched.h"
#include "sys/init.h"
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <semaphore.h>
#include <sys/intr.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>

#include "xparameters.h"
#include "platform_config.h"
#include "platform.h"
#include "stdbool.h"
#include "xgpio.h"
#include "xwdttb.h"
#include "xtmrctr.h"
#include "xstatus.h"


// Declarations
#define BTN_GPIO_DEVICEID		XPAR_PUSH_BUTTONS_4BITS_DEVICE_ID
#define SW_GPIO_DEVICEID		XPAR_XPS_GPIO_0_DEVICE_ID
#define LED_GPIO_DEVICEID		XPAR_LEDS_8BITS_DEVICE_ID
#define INTC_DEVICEID			XPAR_XPS_INTC_0_DEVICE_ID
#define WDT_DEVICEID			XPAR_XPS_TIMEBASE_WDT_0_DEVICE_ID
#define TMRCTR0_DEVICEID		XPAR_XPS_TIMER_0_DEVICE_ID
#define TMRCTR1_DEVICEID		XPAR_XPS_TIMER_1_DEVICE_ID

#define TMRCTR0_INTR_NUM		XPAR_XPS_INTC_0_XPS_TIMER_0_INTERRUPT_INTR
#define TMRCTR1_INTR_NUM		XPAR_XPS_INTC_0_XPS_TIMER_1_INTERRUPT_INTR
#define BTN_GPIO_INTR_NUM		XPAR_XPS_INTC_0_PUSH_BUTTONS_4BITS_IP2INTC_IRPT_INTR
#define WDT_INTR_NUM			XPAR_XPS_INTC_0_XPS_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

// Microblaze cache parameters
#define USE_ICACHE				XPAR_MICROBLAZE_0_USE_ICACHE
#define USE_DCACHE				XPAR_MICROBLAZE_0_USE_DCACHE
#define USE_DCACHE_WRITEBACK	XPAR_MICROBLAZE_DCACHE_USE_WRITEBACK

// macro functions
#define MIN(a, b)  				( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  				( ((a) >= (b)) ? (a) : (b) )

// Peripheral instances
XGpio BTNInst, SWInst, LEDInst;
XTmrCtr TMRCTR1Inst;
XWdtTb WDTInst;


// Message passing variables
typedef struct					// LED message definition
{
	int msg_src;
	int msg_value;
} t_LED_message, *t_LED_messageptr;

const int led_msg_key = 1;		// message key for LED message queue
struct msqid_ds	led_msgstats;	// statistics from message queue

// Synchronization variables
sem_t btn_press_sema;			// semaphore between clock tick ISR and the clock main thread
volatile u32 btn_state;			// button state - shared between button handler and button thread
volatile bool system_running;	// used for tickling the WDT


// Function declarations
void* master_thread(void *arg);
void* button_thread(void *arg);
void* switches_thread(void *arg);
void* leds_thread(void *arg);
void  button_handler(void);
void  wdt_handler(void);
XStatus init_peripherals(void);


int main()
{
    XStatus sts;

    // initialize the platform and the peripherals
    init_platform();
    sts = init_peripherals();
    if (sts != XST_SUCCESS)
    {
    	xil_printf("FATAL ERROR: Could not initialize the peripherals\n\r");
		xil_printf("Please power cycle or reset the system\n\r");
		return -1;
    }

	// check if WDT expired and caused the reset - if so, don't start
	if (XWdtTb_IsWdtExpired(&WDTInst))
	{
	 	// it's true, the WDT expired.
	 	//***** INSERT YOUR WDT RECOVERY CODE HERE ******//

		// Set the system_running flag ???????????????
		system_running = true;
		xil_printf("OH NOOOOOOOOOO\n\r");
	}

    // Initialize xilkernel
    xilkernel_init();

    // Create the master thread
    xmk_add_static_thread(master_thread, 0);
    
    // Start the kernel
    xilkernel_start();
    
    // Should never be reached
    cleanup_platform();
    
    return 0;
}


// The master thread
void* master_thread(void *arg)
{
    pthread_t button;
    pthread_t switches;
    pthread_t leds;

    pthread_attr_t attr;
    struct sched_param spar;

    int ret;

    xil_printf("----------------------------------------------------------------------------\r\n");
    xil_printf("ECE 544 Project 3 Starter Application \r\n");
    xil_printf("----------------------------------------------------------------------------\r\n");
    xil_printf("This Xilkernel based application reads the buttons and switches on the FPGA \r\n"
               "development board and displays them on the LEDs.  Even though the application is\r\n"
               "simple it uses several of the synchronization and interprocess communication\r\n"
               "capabilities offered in the Xilkernel\r\n\r\n"
               "To demonstrate, press any of the buttons and/or flip switches on the board.\r\n"
               "The current state of the buttons and switches should be displayed on the LEDs\r\n");
    xil_printf("----------------------------------------------------------------------------\r\n\r\n\r\n");;

    xil_printf("MASTER: Master Thread Starting\r\n");

    system_running = true;

    // set the priority of all but the master thread to 1.  The master thread runs at priority 0
    // because it is responsible for tickling the WDT.
    pthread_attr_init (&attr);              
    spar.sched_priority = 1;
    pthread_attr_setschedparam(&attr, &spar);

    // create the threads
    ret = pthread_create (&button, &attr, (void*) button_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "button thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
    ret = pthread_create (&switches, &attr, (void*) switches_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
    ret = pthread_create (&leds, &attr, (void*) leds_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }

    // initialize the button press semaphore
	ret = sem_init (&btn_press_sema, 0, 0);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not initialize %s\r\n", errno, "button press semaphore");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
	else
	{
		xil_printf ("MASTER: Button press semaphore has been initialized\n\r");
	}
	
	// Register the interrupt handlers
    ret = register_int_handler(WDT_INTR_NUM, (void*) wdt_handler, NULL);
    if (ret != XST_SUCCESS)
    {
    	return (void*) -4;
    }
    ret = register_int_handler(BTN_GPIO_INTR_NUM, (void*) button_handler, NULL);
    if (ret != XST_SUCCESS)
    {
    	return (void*) -4;
    }

    // Enable interrupts and start the WDT...we're off to the races
    enable_interrupt(BTN_GPIO_INTR_NUM);
	enable_interrupt(WDT_INTR_NUM);
	xil_printf("MASTER: Interrupts have been enabled\r\n");

	XWdtTb_Start(&WDTInst);
	xil_printf("MASTER: Watchdog timer has been started\r\n");

	// master thread main loop
	while(1)
	{
		//***** INSERT YOUR MASTER THREAD CODE HERE ******//
		system_running = true;
		enable_interrupt(BTN_GPIO_INTR_NUM);
		XGpio_InterruptEnable(&BTNInst, 1);

		enable_interrupt(WDT_INTR_NUM);

	}
	
	return NULL;
}

// The button thread
void* button_thread(void *arg)
{
	//***** INSERT YOUR BUTTON THREAD CODE HERE ******//

	return NULL;
}

// The switches thread
void* switches_thread(void *arg)
{
	//***** INSERT YOUR SWITCHES THREAD CODE HERE ******//
	
	return NULL;
}

// The leds thread
void* leds_thread(void *arg)
{
	//***** INSERT YOUR LEDS THREAD CODE HERE ******//
	
	return NULL;
}


// init_peripherals() - Initializes the peripherals
XStatus init_peripherals(void)
{
	XStatus sts;

	//***** INSERT YOUR PERIPHERAL INITIALIZATION CODE HERE ******//
		// initialize the GPIO instances
	    sts = XGpio_Initialize(&BTNInst, BTN_GPIO_DEVICEID);
	    if (sts != XST_SUCCESS)
	    {
	    	return XST_FAILURE;
	    }

	    sts = XGpio_Initialize(&SWInst, SW_GPIO_DEVICEID);
	    if (sts != XST_SUCCESS)
	    {
	      	return XST_FAILURE;
	    }

	    sts = XGpio_Initialize(&LEDInst, LED_GPIO_DEVICEID);
	    if (sts != XST_SUCCESS)
	    {
	    	return XST_FAILURE;
	    }

	    // Initialize the XPS timer
	    sts = XTmrCtr_Initialize(&TMRCTR1Inst, TMRCTR1_DEVICEID);
	    if (sts != XST_SUCCESS)
	    {
	    	return XST_FAILURE;
	    }

	    // Initialize the WDT
	    sts = XWdtTb_Initialize(&WDTInst, WDT_DEVICEID);
	    if (sts != XST_SUCCESS)
	    {
	    	return XST_FAILURE;
	    }


	    // Enable the Microblaze caches and kick off the
	    // processing by enabling the Microblaze interrupt.
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

	    // Start the XPS timer
	    XTmrCtr_Start(&TMRCTR1Inst, 0);

	    // Enable GPIO interrupts both globally and for the channel it uses (channel 1)
	    XGpio_InterruptEnable(&BTNInst, 1);
	    XGpio_InterruptGlobalEnable(&BTNInst);

	    microblaze_enable_interrupts();

	return XST_SUCCESS;
}

// button press interrupt handler
void button_handler(void)
{
	//***** INSERT YOUR BUTTON PRESS INTERRUPT HANDLER CODE HERE *****//
	// disable the interrupt
	disable_interrupt(BTN_GPIO_INTR_NUM);

	// Disable the GPIO interrupt
	XGpio_InterruptDisable(&BTNInst, 1);

	system_running = true;
	xil_printf("BUTTON Thread: Button Pushed\r\n");

	//Clear the interrupt such that it is no longer pending in the GPIO
	(void)XGpio_InterruptClear(&BTNInst, 1);


	// Acknowledge the interrupt
	acknowledge_interrupt(BTN_GPIO_INTR_NUM);
}

// WDT interrupt handler
void wdt_handler(void)
{

	//***** INSERT YOUR WATCHDOG TIMER INTERRUPT HANDLER CODE HERE *****//
	if (system_running)
	{
		disable_interrupt(WDT_INTR_NUM);

		//Restart the WDT as a normal application would
		XWdtTb_RestartWdt(&WDTInst);

		xil_printf("WDT Handler: Tickle, Tickle\r\n");

		// Reset the system_running flag
		system_running = false;

	}
	else
	{
		xil_printf("WDT Handler: First Timeout occurred, will reset CPU on next timeout\r\n");
		// Reset the system_running flag
		system_running = false;

		/*


		 * 	//Set the flag indicating that the WDT has expired
			WdtExpired = TRUE;
			 * Check whether the WatchDog Reset Status has been set.
			 * If this is set means then the test has failed

			if (XWdtTb_ReadReg(WdtTbInstancePtr->RegBaseAddress,
					XWT_TWCSR0_OFFSET) & XWT_CSR0_WRS_MASK) {

				 * Disable and disconnect the interrupt system

				WdtTbDisableIntrSystem(IntcInstancePtr, WdtTbIntrId);


				 * Stop the timer

				XWdtTb_Stop(WdtTbInstancePtr);

				return XST_FAILURE;*/
	}


	acknowledge_interrupt(WDT_INTR_NUM);

}

