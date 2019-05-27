#ifndef __DWT_H
#define __DWT_H

#include "_main.h"

#ifdef __cplusplus
extern "C" {
#endif

// DWT tics in one microsecond
// for 72MHz:   72 000 000 / 1 000 000 = 72
// for 168MHz: 168 000 000 / 1 000 000 = 168
// for 216MHz: 216 000 000 / 1 000 000 = 216
#define SYSTEM_CORE_CLOCK 72000000
#define DWT_IN_MICROSEC (SYSTEM_CORE_CLOCK/1000000)

extern int SMALL_DELLAY;

void DWT_Init();
//void DWT_Delay_tics(uint32_t tics);  // dwt tics
void DWT_Delay_us(uint32_t us);      // microseconds
void DWT_Delay_ms(uint32_t ms);      // milliseconds
void DWT_Delay_With_Action(uint32_t us, int (*cond)(), void (*act)()); // microseconds
uint32_t DWT_Get_Current_Tick();
uint32_t DWT_GetDelta(uint32_t t0);
uint32_t DWT_Elapsed_Tick(uint32_t t0);

#define DWT_Get() DWT->CYCCNT

__STATIC_INLINE void DWT_Delay_tics(uint32_t tics) { // DWT tics
    uint32_t t0 = DWT_Get();
    while (DWT_GetDelta(t0) < tics) {}
}

#ifdef __cplusplus
}
#endif

#endif /* __DWT_H */
