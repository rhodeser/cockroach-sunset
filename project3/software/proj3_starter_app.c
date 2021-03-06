/* proj3_starter_app - ECE 544 Project 3
 *
 * Copyright (c) 2012, 2013, 2014, Portland State University by Caren Zgheib.  All rights reserved.
 *
 * Author:		Caren Zgheib
 * Date:		23-May-2014
 * Revision:	2.0
 *
 * Revision History:
 * -----------------
 * 	21-Feb-2012		RK		Created this program from the xilkernel sample application produced by SDK
 * 	16-May-2013		RK		Updated the program for Nexys3 board and revised homework 2 assignment
 *	12-May-2014		RK		Minor changes to ease support for both Nexys 3 and Nexys 4.  No functional changes
 *  19-May-2014		CZ		Edited the macros to match my implementation of the system and added the initialization code
 *	20-May-2014		CZ		Added the WDT functionality and safe recovery
 *	21-May-2014		CZ		Threads are alive and talking. Button state is shared between the interrupt handler and the button thread.
 *							Switches thread working and setting the force crash flag. LEDs thread working.
 *	23-May-2014		CZ		Added message queues and completed the project.
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
 * We did not add new functionality to the application.
 *  - Really cool project (at least in my opinion) -
 *
 * Note: This is a deliverable for both Erik Rhodes and Caren Zgheib. Even though Erik did not write any code, this is how we decided
 * to split the work. He started working on the final project while I focused on this project.
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

// SW mask
#define SW7						0x80

// Button masks
#define BTNU					1
#define BTNL					2
#define BTND					4
#define BTNR					8

// LED masks
#define LED7					0x80
#define LED6					0x40
#define LED5					0x20
#define LED4					0x10
#define LED3					0x08
#define LED2					0x04
#define LED1					0x02
#define LED0					0x01

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

// macros for message sources
#define SRC_BUTTONS			1
#define SRC_SWITCHES		2

const int led_msg_key = 1;		// message key for LED message queue
struct msqid_ds	led_msgstats;	// statistics from message queue

// Synchronization variables
sem_t btn_press_sema;			// semaphore between clock tick ISR and the clock main thread
volatile u32 btn_state;			// button state - shared between button handler and button thread
volatile bool system_running;	// used for tickling the WDT
volatile bool force_crash;		// force crash - shared between the switches thread and the master thread


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
    cleanup_platform();
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
	 	//***** WDT RECOVERY CODE ******//

		xil_printf("\n..........RESET in PROGRESS..........\n\r");

		sts = init_peripherals();
		if (sts != XST_SUCCESS)
		{
		   xil_printf("FATAL ERROR: Could not initialize the peripherals\n\r");
		   xil_printf("Please power cycle or reset the system\n\r");
		   return -1;
		}
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
    xil_printf("\t\t\tECE 544 Project 3  \r\n");
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
    force_crash = false;

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
    else
    {
    	xil_printf("MASTER: Button Thread Starting\r\n");
    }

    sleep (100);

    ret = pthread_create (&switches, &attr, (void*) switches_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
    else
    {
    	xil_printf("MASTER: Switches Thread Starting\r\n");
    }
    sleep(100);

    ret = pthread_create (&leds, &attr, (void*) leds_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
    else
    {
    	xil_printf("MASTER: Leds Thread Starting\r\n");
    }
    sleep(100);

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
		if (force_crash == false)
		{
			system_running = true;
		}
		else
		{
			xil_printf("MASTER: Force crash enabled\r\n");
			system_running = false;
		}

		enable_interrupt(BTN_GPIO_INTR_NUM);
		XGpio_InterruptEnable(&BTNInst, 1);

		enable_interrupt(WDT_INTR_NUM);

		sleep(100);

	}

	return NULL;
}

// The button thread
void* button_thread(void *arg)
{
	//***** BUTTON THREAD CODE ******//
	xil_printf("BUTTON Thread: We have lift-off\r\n");

	int msgid;							// Message ID
	t_LED_message message_from_b;		// Message from the buttons

	// open the message queue
	msgid = msgget (led_msg_key, IPC_CREAT ) ;

	if (msgid == -1)
	{
		xil_printf("BUTTON Thread: *** ERROR while opening message queue. Errno: %d\r\n",errno);
	}

	message_from_b.msg_src = SRC_BUTTONS;

	while(1)
	{
		// Wait for the button press
		sem_wait(&btn_press_sema);


		if (btn_state != 0)
		{
			xil_printf("BUTTON Thread: btn_state has changed. State is %d\r\n", btn_state);

			// SEND BUTTON STATE TO LEDS[7:4]
			message_from_b.msg_value = (int)btn_state;
			if (msgsnd(msgid, &message_from_b, 8 ,0) < 0) // Blocking send
			{
				xil_printf("BUTTON Thread: *** msgsnd ran into ERROR. Errno: %d.\r\n", errno);
			}
		}

		sleep(200);

		// SEND 0 to clear the LED
		message_from_b.msg_value = 0;
		if (msgsnd(msgid, &message_from_b, 8 ,0) < 0) // Blocking send
		{
			xil_printf("BUTTON Thread: *** msgsnd ran into ERROR. Errno: %d.\r\n", errno);
		}
	}
	return NULL;
}

// The switches thread
void* switches_thread(void *arg)
{
	//***** SWITCHES THREAD CODE ******//
	volatile u32 Switches;
	int msgid;						// Message ID
	t_LED_message message_from_s;	// Message from the switches

	xil_printf("SWITCHES Thread: online! \r\n");

	// open the message queue
	msgid = msgget (led_msg_key, IPC_CREAT ) ;

	if (msgid == -1)
	{
		xil_printf("SWITCHES Thread: *** ERROR while opening message queue. Errno: %d\r\n",errno);
	}

	message_from_s.msg_src = SRC_SWITCHES;

	while (1)
	{
		Switches = XGpio_DiscreteRead(&SWInst, 1);

		if(Switches & SW7)
		{
			force_crash = true;
		}
		else
		{
			// SEND SW STATE TO LEDS [3:0]
			force_crash = false;

			message_from_s.msg_value = (int)Switches;

			if (msgsnd(msgid, &message_from_s, 8 ,0) < 0) // Blocking send
			{
				xil_printf("SWITCHES Thread: *** msgsnd ran into ERROR. Errno: %d.\r\n", errno);
			}

		}

		sleep(100);
	}

	return NULL;
}

// The leds thread
void* leds_thread(void *arg)
{
	//***** LEDS THREAD CODE ******//
	xil_printf("LEDS Thread: We are ready\r\n");

	int msgid;							// Message ID
	static u32 button_value = 0;		// button value to be displayed on the LEDS
	static u32 switch_value = 0; 		// switch value to be displayed on the LEDS

	t_LED_message msg_to_read;

	// open the message queue
	msgid = msgget (led_msg_key, IPC_CREAT ) ;

	if (msgid == -1)
	{
		xil_printf("LEDS Thread: *** ERROR while opening message queue. Errno: %d\r\n",errno);
	}

	while (1)
	{
		// Read from the message queue and light the LEDs
		if(msgrcv(msgid, &msg_to_read, 8, 0, 0) != 8) // Blocking receive
			{
				xil_printf("LEDS Thread: *** msgrcv ran into ERROR. Errno: %d.\r\n", errno);
			}
		else
		{
			switch (msg_to_read.msg_src)
			{
				case SRC_BUTTONS:		// Lights LEDS[7:4] depending on the button
					if (msg_to_read.msg_value == BTNU)
					{
						button_value = LED7;
					}
					else if (msg_to_read.msg_value == BTNL)
					{
						button_value = LED6;
					}
					else if (msg_to_read.msg_value == BTND)
					{
						button_value = LED5;
					}
					else if (msg_to_read.msg_value == BTNR)
					{
						button_value = LED4;
					}
					else
					{
						button_value = 0;
					}
				break;

				case SRC_SWITCHES:		// Light LEDS[3:0] depending on the switch
					if (msg_to_read.msg_value < (0x10))
					{
						switch_value = msg_to_read.msg_value;
					}
				break;

				default:
					button_value = 0;
					switch_value = 0;
				break;
			}

			XGpio_DiscreteWrite(&LEDInst, 1, (button_value | switch_value));
		}
	}

	return NULL;
}


// init_peripherals() - Initializes the peripherals
XStatus init_peripherals(void)
{
	XStatus sts;

	//***** PERIPHERAL INITIALIZATION CODE ******//
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
	    XGpio_SetDataDirection(&LEDInst, 1, 0x00);

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
	    else if (sts == XST_DEVICE_IS_STARTED)
	    {
	    	xil_printf("WDT already started...\r\n");
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
	//***** BUTTON PRESS INTERRUPT HANDLER CODE *****//
	u32 Buttons;

	// disable the interrupt
	disable_interrupt(BTN_GPIO_INTR_NUM);

	// Disable the GPIO interrupt
	XGpio_InterruptDisable(&BTNInst, 1);


	Buttons = XGpio_DiscreteRead(&BTNInst, 1);
	if (Buttons != 0)
	{
		xil_printf("BUTTON Handler: Button Pushed\r\n");
		btn_state = Buttons;
		xil_printf("BUTTON Handler: Button state is %d\r\n", btn_state);
		sem_post(&btn_press_sema);
	}

	//Clear the interrupt such that it is no longer pending in the GPIO
	(void)XGpio_InterruptClear(&BTNInst, 1);


	// Acknowledge the interrupt
	acknowledge_interrupt(BTN_GPIO_INTR_NUM);
}

// WDT interrupt handler
void wdt_handler(void)
{

	//***** WATCHDOG TIMER INTERRUPT HANDLER CODE *****//
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
	}

	acknowledge_interrupt(WDT_INTR_NUM);

}
