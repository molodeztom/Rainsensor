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

	/* RTC IO number used to sample the input signal.
	   Set by main program. */
	.global io_number
io_number:
	.long 0

	.global timer_count_low
timer_count_low:
	.long 0

	.global timer_count_high
timer_count_high:
	.long 0

	.global timer_count_upper
timer_count_upper:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	
	    // Setze das Update-Bit, um das Ticks-Register zu aktualisieren
    WRITE_RTC_REG(RTC_CNTL_TIME_UPDATE_REG, 31, 1, 1)
    // Warte, bis das Update abgeschlossen ist
//wait 10
    // Lese den unteren 32-Bit-Wert des Timers
   // READ_RTC_REG(RTC_CNTL_TIME_LOW0_REG, 0, 16)
   	REG_RD DR_REG_RTCCNTL_BASE + 0x10, 15, 0
	move r1, r0
    move r2, timer_count_low  // Speichere adresse von variable in r2
	st r1, r2, 0 //speichere r1 in der variablen
  // Lese den oberen 32-Bit-Wert des Timers
    //READ_RTC_REG(RTC_CNTL_TIME_LOW0_REG , 16, 16)
	REG_RD DR_REG_RTCCNTL_BASE + 0x10, 31, 16
	move r1, r0
	//move r1, 0x8
    move r2, timer_count_high  // Speichere adresse von variable in r2
	st r1, r2, 0 //speichere r1 in der variablen

	REG_RD DR_REG_RTCCNTL_BASE + 0x14, 15, 0
	move r1, r0
	//move r1, 0x8
    move r2, timer_count_upper  // Speichere adresse von variable in r2
	st r1, r2, 0 //speichere r1 in der variablen

	/* End program */
jump wake_up


