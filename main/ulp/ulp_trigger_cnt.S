/*
    ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program checks
   the input on GPIO0. If the value is different from the previous one, the
   program "debounces" the input: on the next debounce_max_count wake ups,
   it expects to see the same value of input.
   If this condition holds true, the program increments edge_count and starts
   waiting for input signal polarity to change again.
   When the edge counter reaches certain value (set by the main program),
   this program running triggers a wake up from deep sleep.
   20250316 V1.0: Merge with version ee7951e to add GPIO Trigger
   20250330 V1.1: works with rainsensor_main.c Version 6.5.1
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "sdkconfig.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global next_edge
next_edge:
	.long 0

	/* Counter started when signal value changes.
	   Edge is "debounced" when the counter reaches zero. */
	.global debounce_counter
debounce_counter:
	.long 0

	/* Value to which debounce_counter gets reset.
	   Set by the main program. */
	.global debounce_max_count
debounce_max_count:
	.long 0

	/* Total number of signal edges acquired */
	.global edge_count
edge_count:
	.long 0

	/* Number of edges to acquire before waking up the SoC.
	   Set by the main program. */
	.global edge_count_to_wake_up
edge_count_to_wake_up:
	.long 0

	.global time_to_wake_CPU
time_to_wake_CPU:
	.long 0	

	/* RTC IO number used to sample the input signal.
	   Set by main program. */
	.global io_number
io_number:
	.long 0
//counter when timer is up
	.global timer_count
timer_count:
	.long 0

	.global timer_count_low_h
timer_count_low_h:
	.long 0

	.global timer_count_high
timer_count_high:
	.long 0

/* start time (48-Bit-timestamp, lower 16 bits not used) */

	.global start_time_low_h
start_time_low_h:
	.long 0

	.global start_time_high
start_time_high:
	.long 0

.global current_time_low
current_time_low:
	.long 0

	.global current_time_high
current_time_high:
	.long 0

	.global test_div
test_div:
	.long 0

return_address:
	.long 0


// --------------------------------
// Main Program
// --------------------------------
	.text
	.global entry
entry:
	//Read GPIO input
	/* Load io_number */
	move r3, io_number
	ld r3, r3, 0
	 /* ESP32S3 powers down RTC periph when entering deep sleep and thus by association SENS_SAR_PERI_CLK_GATE_CONF_REG */
    WRITE_RTC_FIELD(SENS_SAR_PERI_CLK_GATE_CONF_REG, SENS_IOMUX_CLK_EN, 1);
	/* Lower 16 IOs and higher need to be handled separately,
	 * because r0-r3 registers are 16 bit wide.
	 * Check which IO this is.
	 */
	move r0, r3
	jumpr read_io_high, 16, ge

	/* Read the value of lower 16 RTC IOs into R0 */
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
	rsh r0, r0, r3
	jump read_done

	/* Read the value of RTC IOs 16-17, into R0 */
read_io_high:
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2)
	sub r3, r3, 16
	rsh r0, r0, r3

read_done:
	and r0, r0, 1
	/* State of input changed? */
	move r3, next_edge
	ld r3, r3, 0
	add r3, r0, r3
	and r3, r3, 1
	jump changed, eq
	/* Not changed */
	/* Reset debounce_counter to debounce_max_count */
	move r3, debounce_max_count
	move r2, debounce_counter
	ld r3, r3, 0
	st r3, r2, 0
	jump handle_timer

changed:
	/* Input state changed */
	/* Has debounce_counter reached zero? */
	move r3, debounce_counter
	ld r2, r3, 0
	add r2, r2, 0 /* dummy ADD to use "jump if ALU result is zero" */
	jump edge_detected, eq
	/* Not yet. Decrement debounce_counter */
	sub r2, r2, 1
	st r2, r3, 0
	
	jump handle_timer

	.global edge_detected
edge_detected:
	/* Reset debounce_counter to debounce_max_count */
	move r3, debounce_max_count
	move r2, debounce_counter
	ld r3, r3, 0
	st r3, r2, 0
	/* Flip next_edge */
	move r3, next_edge
	ld r2, r3, 0
	add r2, r2, 1
	and r2, r2, 1
	st r2, r3, 0
	/* Increment edge_count */
	move r3, edge_count
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0
	/* Compare edge_count to edge_count_to_wake_up */
	move r3, edge_count_to_wake_up
	ld r3, r3, 0
	sub r3, r3, r2
	jump wake_up, eq
	move r0, test_div
	ld r1, r0, 0
	add r2, r1, 1
	st r2, r0, 0

	/* Not yet. End program */
	jump handle_timer

	
handle_timer:	
	//halt
	//Handle timer

	// set update bit to get a new timestamp
    WRITE_RTC_REG(RTC_CNTL_TIME_UPDATE_REG, 31, 1, 1)
  // register is 48 bit organized in lowReg 32 bit and high reg 16 bit we need only lowReg upper 16 bits and high reg

  // read upper 16 bit timer RTC_CNTL_TIME_LOW0_REG
	REG_RD DR_REG_RTCCNTL_BASE + 0x10, 31, 16
	move r1, r0
	//TEST TODO remove: To provoke overflow we add a constant 65000
	//add r1, r1, 65400
    move r2, timer_count_low_h  
	st r1, r2, 0 //store register in variable

 // read lower 16 Bit RTC_CNTL_TIME_HIGH0_REG
	REG_RD DR_REG_RTCCNTL_BASE + 0x14, 15, 0
	move r1, r0
    move r2, timer_count_high  
	st r1, r2, 0 //store in variable

	//initialize start time on first run
	//first time it will be 0
	move r2, start_time_low_h
	ld r0,r2, 0
	//if start time has a value > 1
	jumpr start_time_initialized, 1, gt //If start_time_low_h > 1, skip initialization
	// Store return address and jump to initialize_start_time
	move r1, start_time_initialized
	move r2, return_address
	st r1, r2, 0
	jump initialize_start_time

// --------------------------------
// Timer Comparison Logic
// --------------------------------
start_time_initialized:
//compare timer low_h with max time
    move r1, timer_count_low_h
    ld r1, r1, 0
    move r2, start_time_low_h
	ld r2, r2, 0
	sub r0, r1, r2  //r0 = diff Timer - Start = ticks since interval start, 
	//overflow may happen when timer count h exeeds 65535 -> 0 we get a negative result
	//then jump to "time is up" to set Start on new timer value. 
	//we might loose the time difference it is not so important for just one interval
	jump timer_low_h_overflow, ov
	//load interval from external variable
	move r3, time_to_wake_CPU
	ld r3, r3, 0
	sub r0, r0, r3 //r0(Timer - start time) - r3 (time_to_wake_CPU) if negative will lead to an overflow
	jump halt_execution, ov  //if timediff - time to wake < 0 time not up - halt

//--------------------------------------------------------------------------------
    // Time is up: increment timer_count and reset start time
	move r0, timer_count
	ld r1, r0, 0
	add r2, r1, 1
	st r2,  r0, 0
	//are there edges  to wake up?
	move r3, edge_count
	ld r0, r3, 0
	add r0, r0, 0 /* dummy ADD to use "jump if ALU result is zero" */
	jump no_edges_to_wake_up, eq
	//at least one edge detected
	//store return adress, to wake up CPU after initialisation
	move r1, end_and_wake_CPU
	move r2, return_address
	st r1, r2, 0
	jump initialize_start_time
		
no_edges_to_wake_up:	
	//just end program without waking up CPU
	move r1, halt_execution
	move r2, return_address
	st r1, r2, 0
	jump initialize_start_time
//-------------------------------------------- halt	
	

halt_execution:

	halt  //nothing more to do start with next cyclus

// Handle timer low_h overflow
timer_low_h_overflow:
	//we might loose the time difference it is not so important for just one interval

	// Reinitialize start time
	//store return adress, in this case we continue
	move r1, start_time_initialized
	move r2, return_address
	st r1, r2, 0
	jump initialize_start_time

end_and_wake_CPU:
	/* End program */
	jump wake_up

// Subroutine: Initialize start time
// --------------------------------
// Timer Initialization
// --------------------------------
initialize_start_time:
	//set start  time to current time so we can calculate the new time interval from there
	//copy timer low_h to start time_low_h
    move r1, timer_count_low_h
    ld r0, r1, 0
    move r2, start_time_low_h
    st r0, r2, 0

	//copy timer high to start 
    move r1, timer_count_high
    ld r0, r1, 0
    move r2, start_time_high
    st r0, r2, 0

	//back to return adress dynamic is set by caller to simulate a function call
	move r2, return_address
	ld r1, r2, 0
	jump r1



