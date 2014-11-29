#include "../../config.h"

#include <avr/interrupt.h>

// Hardware constants for timer 2.
#if (((F_CPU/32/POLL_TMR_FREQ) - 1) < 255)
#define halCLOCK_CONFIG     3
#define halCLOCK_COMPARE_VALUE ((F_CPU/32/POLL_TMR_FREQ)-1)
#elif (((F_CPU/64/POLL_TMR_FREQ) - 1) < 255)
#define halCLOCK_CONFIG     4
#define halCLOCK_COMPARE_VALUE ((F_CPU/64/POLL_TMR_FREQ)-1)
#elif (((F_CPU/128/POLL_TMR_FREQ) - 1) < 255)
#define halCLOCK_CONFIG     5
#define halCLOCK_COMPARE_VALUE ((F_CPU/128/POLL_TMR_FREQ)-1)
#elif (((F_CPU/256/POLL_TMR_FREQ) - 1) < 255)
#define halCLOCK_CONFIG     6
#define halCLOCK_COMPARE_VALUE ((F_CPU/256/POLL_TMR_FREQ)-1)
#elif (((F_CPU/1024/POLL_TMR_FREQ) - 1) < 255)
#define halCLOCK_CONFIG     7
#define halCLOCK_COMPARE_VALUE ((F_CPU/1024/POLL_TMR_FREQ)-1)
#else
#error Check F_CPU and POLL_TMR_FREQ
#endif


void StartSheduler(void)
{
    TCCR2A = (1<<WGM21);
    TCNT2 = 0;
    OCR2A = halCLOCK_COMPARE_VALUE;
    TIFR2 = (1<<OCF2A);
    TIMSK2 = (1<<OCIE2A);
    TCCR2B = halCLOCK_CONFIG;

    sei();
}

// Generate pseudo random uint16
uint16_t halRNG()
{
    static uint16_t rand16 = 0xA15E;

    // Galois LFSRs
    if(rand16 & 1)
    {
        rand16 >>= 1;
        rand16 ^= 0xB400;
    }
    else
        rand16 >>= 1;
  
    return rand16;
}

// Main program tick procedure
void SystemTick(void);

ISR(TIMER2_COMPA_vect)
{
    SystemTick();
}
