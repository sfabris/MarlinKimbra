#include "elements.h"
#if ENABLED(USE_WATCHDOG)
#include <avr/wdt.h>
#include "whatchdog.h"

//===========================================================================
//============================ private variables ============================
//===========================================================================

//===========================================================================
//================================ functions ================================
//===========================================================================


/// intialise watch dog with a 4 sec interrupt time
void watchdog_init()
{
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    //We enable the watchdog timer, but only for the interrupt.
    //Take care, as this requires the correct order of operation, with interrupts disabled. See the datasheet of any AVR chip for details.
    wdt_reset();
    _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
    _WD_CONTROL_REG = _BV(WDIE) | WDTO_4S;
  #else
    wdt_enable(WDTO_4S);
  #endif
}

/// reset watchdog. MUST be called every 1s after init or avr will reset.
void watchdog_reset() 
{
    wdt_reset();
}

//===========================================================================
//=================================== ISR ===================================
//===========================================================================

// Watchdog timer interrupt, called if main program blocks >1sec and manual reset is enabled.
#if ENABLED(WATCHDOG_RESET_MANUAL)
ISR(WDT_vect) {
  ECHO_LM(ER, MSG_WATCHDOG_RESET);
  kill(PSTR("ERR:Please Reset")); // kill blocks //16 characters so it fits on a 16x2 display
  while(1); // wait for user or serial reset
}
#endif // RESET_MANUAL

#endif
