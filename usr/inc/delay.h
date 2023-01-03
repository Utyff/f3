#ifndef F3_DELAY_H
#define F3_DELAY_H

#include "dwt.h"

#ifdef __cplusplus
extern "C" {
#endif

//void delay_us(uint32_t us);
#define delay_dwt(dwt) DWT_Delay_tics(dwt)
#define delay_us(us) DWT_Delay_us(us)
#define delay_ms(ms) DWT_Delay_ms(ms)
//void Delay(__IO uint32_t nTime);

//void TimingDelay_Decrement(void);
//static __IO uint32_t TimingDelay;

#ifdef __cplusplus
}
#endif

#endif // F3_DELAY_H
