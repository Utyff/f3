#include <_main.h>
#include <stdlib.h>
#include <stdio.h>
#include <delay.h>
#include <draw.h>
#include <keys.h>
#include <DataBuffer.h>
//#include <generator.h>
#include <adc.h>


void CORECheck();

void FPUCheck();

extern int ii;
extern float screenTime;


void mainInitialize() {
    DWT_Init();
    LCD_Init();

    //HAL_ADC_Start_DMA(&hadc1, (uint32_t *) samplesBuffer, BUF_SIZE);
    ADC_init();

    //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    GEN_setParams();

//    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
    KEYS_init();

    CORECheck();
    FPUCheck();
}

uint8_t prevBnt1 = 0;

void mainCycle() {
    drawScreen();
    KEYS_scan();

    if ((random() & 7) < 2) LED1_GPIO_Port->ODR ^= 1u << LED1_Pin;
#ifdef LED2_Pin
//    if ((random() & 7) < 3) LED2_GPIO_Port->ODR ^= 1u << LED2_Pin;
#endif
#ifdef LED3_Pin
    if ((random() & 7) < 3) HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
#endif

    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;
    LCD_ShowxNum(0, 227, TIM8->CNT, 5, 12, 0x0);
    LCD_ShowChar(22, 214, 'b', 12, 0x0);
    LCD_ShowxNum(30, 214, (u32) button1Count, 5, 12, 0x0);
    LCD_ShowxNum(60, 214, (u32) btns_state, 5, 12, 0x0);
    LCD_ShowChar(112, 214, 't', 12, 0x0);
//    LCD_ShowxNum(120, 214, (u32) screenTime, 5, 12, 0x0);
    LCD_ShowxNum(120, 214, (u32) new_state, 5, 12, 0x0);
    LCD_ShowChar(172, 214, 'g', 12, 0x0);
    LCD_ShowxNum(180, 214, (u32) enc_count, 5, 12, 0x0);
//    LCD_ShowxNum(180, 214, (u32) tim1Freq/100, 5, 12, 0x0);
//    LCD_ShowxNum(180, 214, (u32) ADCElapsedTick / DWT_IN_MICROSEC, 5, 12, 0x0);

    delay_ms(50);
    ADC_takeSamples();
}

#ifdef DEBUG_TRACE_SWO

void SWO_Trace(uint8_t *msg) {
    for (int i = 0; msg[i] != 0; i++) {
        ITM_SendChar(msg[i]);
    }
}

#endif

void FPUCheck(void) {
    char buf[120];
    uint32_t mvfr0;

    sprintf(buf, "%08X %08X %08X\n%08X %08X %08X\n",
            *(volatile uint32_t *) 0xE000EF34,   // FPCCR  0xC0000000
            *(volatile uint32_t *) 0xE000EF38,   // FPCAR
            *(volatile uint32_t *) 0xE000EF3C,   // FPDSCR
            *(volatile uint32_t *) 0xE000EF40,   // MVFR0  0x10110021 vs 0x10110221
            *(volatile uint32_t *) 0xE000EF44,   // MVFR1  0x11000011 vs 0x12000011
            *(volatile uint32_t *) 0xE000EF48);  // MVFR2  0x00000040
    DBG_Trace(buf);

    mvfr0 = *(volatile uint32_t *) 0xE000EF40;

    switch (mvfr0) {
        case 0x10110021 :
            sprintf(buf, "FPU-S Single-precision only\n");
            break;
        case 0x10110221 :
            sprintf(buf, "FPU-D Single-precision and Double-precision\n");
            break;
        default :
            sprintf(buf, "Unknown FPU");
    }
    DBG_Trace(buf);
}

void CORECheck(void) {
    char buf[120];
    uint32_t cpuid = SCB->CPUID;
    uint32_t var, pat;

    sprintf(buf, "CPUID %08X DEVID %03X\n", cpuid, DBGMCU->IDCODE & 0xFFF);
    DBG_Trace(buf);

    pat = (cpuid & 0x0000000F);
    var = (cpuid & 0x00F00000) >> 20;

    if ((cpuid & 0xFF000000) == 0x41000000) // ARM
    {
        switch ((cpuid & 0x0000FFF0) >> 4) {
            case 0xC20 :
                sprintf(buf, "Cortex M0 r%dp%d\n", var, pat);
                break;
            case 0xC60 :
                sprintf(buf, "Cortex M0+ r%dp%d\n", var, pat);
                break;
            case 0xC21 :
                sprintf(buf, "Cortex M1 r%dp%d\n", var, pat);
                break;
            case 0xC23 :
                sprintf(buf, "Cortex M3 r%dp%d\n", var, pat);
                break;
            case 0xC24 :
                sprintf(buf, "Cortex M4 r%dp%d\n", var, pat);
                break;
            case 0xC27 :
                sprintf(buf, "Cortex M7 r%dp%d\n", var, pat);
                break;

            default :
                sprintf(buf, "Unknown CORE");
        }
    } else
        sprintf(buf, "Unknown CORE IMPLEMENTER");
    DBG_Trace(buf);
}
