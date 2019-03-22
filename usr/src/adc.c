#include <_main.h>
#include <dwt.h>
#include <graph.h>
#include <DataBuffer.h>
#include "adc.h"


struct ADC_param {
    uint32_t ADC_Prescaler;
    uint32_t ADC_SampleTime;
    float SampleTime;    // microseconds
    float ScreenTime;    // microseconds
};
typedef struct ADC_param ADC_PARAM;

#define ADC_Parameters_Size 54
const ADC_PARAM ADC_Parameters[ADC_Parameters_Size] = {
        {RCC_CFGR2_ADCPRE12_DIV1,  ADC_SAMPLETIME_1CYCLE_5,   0.13,  42.22},
        {RCC_CFGR2_ADCPRE12_DIV1,  ADC_SAMPLETIME_2CYCLES_5,  0.15,  46.67},
        {RCC_CFGR2_ADCPRE12_DIV1,  ADC_SAMPLETIME_4CYCLES_5,  0.17,  55.56},
        {RCC_CFGR2_ADCPRE12_DIV1,  ADC_SAMPLETIME_7CYCLES_5,  0.22,  68.89},
        {RCC_CFGR2_ADCPRE12_DIV2,  ADC_SAMPLETIME_1CYCLE_5,   0.26,  84.44},
        {RCC_CFGR2_ADCPRE12_DIV2,  ADC_SAMPLETIME_2CYCLES_5,  0.29,  93.33},
        {RCC_CFGR2_ADCPRE12_DIV2,  ADC_SAMPLETIME_4CYCLES_5,  0.35,  111.11},
        {RCC_CFGR2_ADCPRE12_DIV1,  ADC_SAMPLETIME_19CYCLES_5, 0.38,  122.22},
        {RCC_CFGR2_ADCPRE12_DIV2,  ADC_SAMPLETIME_7CYCLES_5,  0.43,  137.78},
        {RCC_CFGR2_ADCPRE12_DIV4,  ADC_SAMPLETIME_1CYCLE_5,   0.53,  168.89},
        {RCC_CFGR2_ADCPRE12_DIV4,  ADC_SAMPLETIME_2CYCLES_5,  0.58,  186.67},
        {RCC_CFGR2_ADCPRE12_DIV4,  ADC_SAMPLETIME_4CYCLES_5,  0.69,  222.22},
        {RCC_CFGR2_ADCPRE12_DIV2,  ADC_SAMPLETIME_19CYCLES_5, 0.76,  244.44},
        {RCC_CFGR2_ADCPRE12_DIV6,  ADC_SAMPLETIME_1CYCLE_5,   0.79,  253.33},
        {RCC_CFGR2_ADCPRE12_DIV4,  ADC_SAMPLETIME_7CYCLES_5,  0.86,  275.56},
        {RCC_CFGR2_ADCPRE12_DIV6,  ADC_SAMPLETIME_2CYCLES_5,  0.88,  280.00},
        {RCC_CFGR2_ADCPRE12_DIV1,  ADC_SAMPLETIME_61CYCLES_5, 0.97,  308.89},
        {RCC_CFGR2_ADCPRE12_DIV6,  ADC_SAMPLETIME_4CYCLES_5,  1.04,  333.33},
        {RCC_CFGR2_ADCPRE12_DIV8,  ADC_SAMPLETIME_1CYCLE_5,   1.06,  337.78},
        {RCC_CFGR2_ADCPRE12_DIV8,  ADC_SAMPLETIME_2CYCLES_5,  1.17,  373.33},
        {RCC_CFGR2_ADCPRE12_DIV6,  ADC_SAMPLETIME_7CYCLES_5,  1.29,  413.33},
        {RCC_CFGR2_ADCPRE12_DIV10, ADC_SAMPLETIME_1CYCLE_5,   1.32,  422.22},
        {RCC_CFGR2_ADCPRE12_DIV8,  ADC_SAMPLETIME_4CYCLES_5,  1.39,  444.44},
        {RCC_CFGR2_ADCPRE12_DIV10, ADC_SAMPLETIME_2CYCLES_5,  1.46,  466.67},
        {RCC_CFGR2_ADCPRE12_DIV4,  ADC_SAMPLETIME_19CYCLES_5, 1.53,  488.89},
        {RCC_CFGR2_ADCPRE12_DIV12, ADC_SAMPLETIME_1CYCLE_5,   1.58,  506.67},
        {RCC_CFGR2_ADCPRE12_DIV8,  ADC_SAMPLETIME_7CYCLES_5,  1.72,  551.11},
        {RCC_CFGR2_ADCPRE12_DIV10, ADC_SAMPLETIME_4CYCLES_5,  1.74,  555.56},
        {RCC_CFGR2_ADCPRE12_DIV12, ADC_SAMPLETIME_2CYCLES_5,  1.75,  560.00},
        {RCC_CFGR2_ADCPRE12_DIV2,  ADC_SAMPLETIME_61CYCLES_5, 1.93,  617.78},
        {RCC_CFGR2_ADCPRE12_DIV12, ADC_SAMPLETIME_4CYCLES_5,  2.08,  666.67},
        {RCC_CFGR2_ADCPRE12_DIV16, ADC_SAMPLETIME_1CYCLE_5,   2.11,  675.56},
        {RCC_CFGR2_ADCPRE12_DIV10, ADC_SAMPLETIME_7CYCLES_5,  2.15,  688.89},
        {RCC_CFGR2_ADCPRE12_DIV6,  ADC_SAMPLETIME_19CYCLES_5, 2.29,  733.33},
        {RCC_CFGR2_ADCPRE12_DIV16, ADC_SAMPLETIME_2CYCLES_5,  2.33,  746.67},
        {RCC_CFGR2_ADCPRE12_DIV12, ADC_SAMPLETIME_7CYCLES_5,  2.58,  826.67},
        {RCC_CFGR2_ADCPRE12_DIV16, ADC_SAMPLETIME_4CYCLES_5,  2.78,  888.89},
        {RCC_CFGR2_ADCPRE12_DIV8,  ADC_SAMPLETIME_19CYCLES_5, 3.06,  977.78},
        {RCC_CFGR2_ADCPRE12_DIV16, ADC_SAMPLETIME_7CYCLES_5,  3.44,  1102.22},
        {RCC_CFGR2_ADCPRE12_DIV10, ADC_SAMPLETIME_19CYCLES_5, 3.82,  1222.22},
        {RCC_CFGR2_ADCPRE12_DIV4,  ADC_SAMPLETIME_61CYCLES_5, 3.86,  1235.56},
        {RCC_CFGR2_ADCPRE12_DIV32, ADC_SAMPLETIME_1CYCLE_5,   4.22,  1351.11},
        {RCC_CFGR2_ADCPRE12_DIV12, ADC_SAMPLETIME_19CYCLES_5, 4.58,  1466.67},
        {RCC_CFGR2_ADCPRE12_DIV32, ADC_SAMPLETIME_2CYCLES_5,  4.67,  1493.33},
        {RCC_CFGR2_ADCPRE12_DIV32, ADC_SAMPLETIME_4CYCLES_5,  5.56,  1777.78},
        {RCC_CFGR2_ADCPRE12_DIV6,  ADC_SAMPLETIME_61CYCLES_5, 5.79,  1853.33},
        {RCC_CFGR2_ADCPRE12_DIV16, ADC_SAMPLETIME_19CYCLES_5, 6.11,  1955.56},
        {RCC_CFGR2_ADCPRE12_DIV32, ADC_SAMPLETIME_7CYCLES_5,  6.89,  2204.44},
        {RCC_CFGR2_ADCPRE12_DIV8,  ADC_SAMPLETIME_61CYCLES_5, 7.72,  2471.11},
        {RCC_CFGR2_ADCPRE12_DIV10, ADC_SAMPLETIME_61CYCLES_5, 9.65,  3088.89},
        {RCC_CFGR2_ADCPRE12_DIV12, ADC_SAMPLETIME_61CYCLES_5, 11.58, 3706.67},
        {RCC_CFGR2_ADCPRE12_DIV32, ADC_SAMPLETIME_19CYCLES_5, 12.22, 3911.11},
        {RCC_CFGR2_ADCPRE12_DIV16, ADC_SAMPLETIME_61CYCLES_5, 15.44, 4942.22},
        {RCC_CFGR2_ADCPRE12_DIV32, ADC_SAMPLETIME_61CYCLES_5, 30.89, 9884.44}
};

uint32_t ADC_Prescaler = RCC_CFGR2_ADCPRE12_DIV1;
uint32_t ADC_SampleTime = ADC_SAMPLETIME_2CYCLES_5;

uint16_t ScreenTime = 0;      // index in ScreenTimes
uint16_t ScreenTime_adj = 0;  // 0-9 shift in ScreenTime
const float ScreenTimes[] = {50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};  // sweep screen, microseconds

uint32_t ADCStartTick;         // time when start ADC buffer fill
uint32_t ADCHalfElapsedTick;   // the last time half buffer fill
uint32_t ADCElapsedTick;       // the last time buffer fill


void ADC_Init() {
    MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12, ADC_Prescaler);

    // ADC voltage regulator
    if ((ADC1->CR &=ADC_CR_ADVREGEN_Msk) == ADC_CR_ADVREGEN_1) { // 10 - disabled state, reset state
        // Enable VREG
        ADC1->CR &= ~(ADC_CR_ADVREGEN_1 | ADC_CR_ADVREGEN_0);    // 00 - first stage
        ADC1->CR |= ADC_CR_ADVREGEN_0;                           // 01 - enabled state.
    }

    // ADC calibration
    // ADC enable

}

void ADC_DeInit() {

}

/**
 * Copy of MX_ADC1_Init()
 */
void ADC_setParams() {

    ADC_DeInit();
    ADC_Init();

//  ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /**Common config
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
//  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc1.Init.ContinuousConvMode = ENABLE;
//  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc1.Init.NbrOfConversion = 1;
//  hadc1.Init.DMAContinuousRequests = ENABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc1.Init.LowPowerAutoWait = DISABLE;
//  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /**Configure the ADC multi-mode
//  */
//  multimode.Mode = ADC_MODE_INDEPENDENT;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /**Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.SamplingTime = ADC_SampleTime;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) samplesBuffer, BUF_SIZE);
//
//    ADCStartTick = DWT_Get_Current_Tick();
}

uint32_t halfCount = 0;
uint32_t cpltCount = 0;
/**
  * @brief  Conversion complete callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    halfCount++;
//    firstHalf = 0;
//    ADCHalfElapsedTick = DWT_Elapsed_Tick(ADCStartTick);
//}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    cpltCount++;
////    firstHalf = 1;
//    samplesReady = 1;
//    ADCElapsedTick = DWT_Elapsed_Tick(ADCStartTick);
//    ADCStartTick = DWT_Get_Current_Tick();
//}

void ADC_step_up() {
    if (ScreenTime_adj < 9)
        ScreenTime_adj++;
    else if (ScreenTime < sizeof(ScreenTimes) / sizeof(ScreenTimes[0]) - 2) // last value forbidden to assign
        ScreenTime_adj = 0, ScreenTime++;
}

void ADC_step_down() {
    if (ScreenTime_adj > 0)
        ScreenTime_adj--;
    else if (ScreenTime > 0)
        ScreenTime_adj = 9, ScreenTime--;
}

float ADC_getTime() {
    float tm = ScreenTimes[ScreenTime];
    // next time always exist because last forbidden to assign
    float adj = (ScreenTimes[ScreenTime + 1] - tm) * ScreenTime_adj / 10;
    tm += adj;
    return tm;
}

float screenTime;
int ii;

void ADC_step(int16_t step) {
    if (step == 0) return;
    if (step > 0) ADC_step_up();
    else ADC_step_down();

    screenTime = ADC_getTime(); // get screen sweep time

    // looking last parameters set with ScreenTime less than required time
    int i = 1;
    while (ADC_Parameters[i].ScreenTime < screenTime) {
        i++;
        if (i >= ADC_Parameters_Size) break;
    }

    i--;
    ii = i;
    ADC_Prescaler = ADC_Parameters[i].ADC_Prescaler;
    ADC_SampleTime = ADC_Parameters[i].ADC_SampleTime;

    // set X scale
    scaleX = ADC_Parameters[i].ScreenTime / screenTime;
    ADC_setParams();
}

/*uint16_t ICount = 0;

// dma2 stream 0 irq handler
void DMA2_Stream0_IRQHandler() {
    ICount++;
    // Test on DMA Stream HalfTransfer Complete interrupt
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0)) {
        // Clear Stream0 HalfTransfer
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);

        // count time for half circle
        ADCHalfElapsedTick = DWT_Elapsed_Tick(ADCStartTick);
        half = 0;
    }

    // Test on DMA Stream Transfer Complete interrupt
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
        // Clear Stream0 Transfer Complete
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

        // count time for one circle
        ADCElapsedTick = DWT_Elapsed_Tick(ADCStartTick);
        ADCStartTick = DWT_Get_Current_Tick();
        half = 1;
    }
} //*/
