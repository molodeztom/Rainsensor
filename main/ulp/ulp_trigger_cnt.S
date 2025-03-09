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

	.global pulse_count
pulse_count:
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




	/* Code goes into .text section */
	.text

	.global entry
entry:

	// Setze das Update-Bit, um das Ticks-Register zu aktualisieren
    WRITE_RTC_REG(RTC_CNTL_TIME_UPDATE_REG, 31, 1, 1)
	//read timer value in ms 48 bit
    /* read lower 16 bit RTC_CNTL_TIME_LOW0_REG not needed
   	REG_RD DR_REG_RTCCNTL_BASE + 0x10, 15, 0
	move r1, r0
    move r2, timer_count_low_l  
	st r1, r2, 0 
	*/

  // read upper 16 bit timer RTC_CNTL_TIME_LOW0_REG
	REG_RD DR_REG_RTCCNTL_BASE + 0x10, 31, 16
	move r1, r0
	//TEST TODO remove: To provoke overflow we add a constant 65000
	//add r1, r1, 65400
    move r2, timer_count_low_h  // Speichere adresse von variable in r2
	st r1, r2, 0 //speichere r1 in der variablen



 // read lower 16 Bit RTC_CNTL_TIME_HIGH0_REG
	REG_RD DR_REG_RTCCNTL_BASE + 0x14, 15, 0
	move r1, r0
    move r2, timer_count_high  // Speichere adresse von variable in r2
	st r1, r2, 0 //speichere r1 in der variablen

	//initialize start time on first run check if start time has a value > 1
	//first time it will be 0
	move r2, start_time_low_h
	ld r0,r2, 0
	jumpr start_time_initialized, 1, gt

initialize_start_time:

	
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
  

start_time_initialized:
//compare timer low_h with max time
    move r1, timer_count_low_h
    ld r1, r1, 0
    move r2, start_time_low_h
	ld r2, r2, 0
	sub r0, r1, r2
	//check: overflow and jump to time is up causing increment pulse timer always
	jump fixed_pulse, ov
	jumpr time_is_up, 100, gt
	halt

fixed_pulse:
	//set pulse to 999 so we find it later in the log and set start time to timer again
	//we might loose the time difference 
	move r0, pulse_count
	move r1, 999
	st r1, r0, 0
	jump initialize_start_time

no_overflow:


time_is_up:
//time is over max time
// increment test pulse count
	move r0, pulse_count
	ld r1, r0, 0
	add r2, r1, 1
	st r2,  r0, 0
	
//reset start time to current value


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
	jump end_program
	

end_program:
	/* End program */
	jump wake_up


