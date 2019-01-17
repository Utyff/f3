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

#define ADC_Parameters_Size  6
const ADC_PARAM ADC_Parameters[ADC_Parameters_Size] = {
        {ADC_CLOCK_SYNC_PCLK_DIV1, ADC_SAMPLETIME_1CYCLE_5,  1.0f,  1.0f},
        {ADC_CLOCK_SYNC_PCLK_DIV2, ADC_SAMPLETIME_1CYCLE_5,  2.0f,  2.0f},
        {ADC_CLOCK_SYNC_PCLK_DIV4, ADC_SAMPLETIME_1CYCLE_5,  4.0f,  4.0f},
        {ADC_CLOCK_SYNC_PCLK_DIV4, ADC_SAMPLETIME_2CYCLES_5,  6.0f,  6.0f},
        {ADC_CLOCK_SYNC_PCLK_DIV4, ADC_SAMPLETIME_4CYCLES_5,  8.0f,  8.0f},
        {ADC_CLOCK_SYNC_PCLK_DIV4, ADC_SAMPLETIME_7CYCLES_5,  10.0f, 10.0f}
};

uint32_t ADC_Prescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
uint32_t ADC_SampleTime = ADC_SAMPLETIME_1CYCLE_5;

uint16_t ScreenTime = 0;      // index in ScreenTimes
uint16_t ScreenTime_adj = 0;  // 0-9 shift in ScreenTime
const float ScreenTimes[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000};  // sweep screen, microseconds

uint32_t ADCStartTick;         // time when start ADC buffer fill
uint32_t ADCHalfElapsedTick;   // the last time half buffer fill
uint32_t ADCElapsedTick;       // the last time buffer fill

/**
 * Copy of MX_ADC1_Init()
 */
void ADC_setParams() {

  HAL_ADC_DeInit(&hadc1);
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /**Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_Prescaler;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SampleTime;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) samplesBuffer, BUF_SIZE);

    ADCStartTick = DWT_Get_Current_Tick();
}

uint32_t halfCount =0;
uint32_t cpltCount =0;
/**
  * @brief  Conversion complete callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    halfCount++;
    firstHalf = 0;
    ADCHalfElapsedTick = DWT_Elapsed_Tick(ADCStartTick);
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    cpltCount++;
    firstHalf = 1;
    samplesReady = 1;
    ADCElapsedTick = DWT_Elapsed_Tick(ADCStartTick);
    ADCStartTick = DWT_Get_Current_Tick();
}

uint16_t paramNum = 2;

void ADC_step_up() {
//    if (ScreenTime_adj < 9)
//        ScreenTime_adj++;
//    else if (ScreenTime < sizeof(ScreenTimes) / sizeof(ScreenTimes[0]) - 2) // last value forbidden to assign
//        ScreenTime_adj = 0, ScreenTime++;
    if (paramNum < ADC_Parameters_Size - 1) paramNum++;
}

void ADC_step_down() {
//    if (ScreenTime_adj > 0)
//        ScreenTime_adj--;
//    else if (ScreenTime > 0)
//        ScreenTime_adj = 9, ScreenTime--;
    if (paramNum > 0) paramNum--;
}

float ADC_getTime() {
    float time = ScreenTimes[ScreenTime];
    // next time always exist because last forbidden to assign
    float adj = (ScreenTimes[ScreenTime + 1] - time) * ScreenTime_adj / 10;
    time += adj;
    return time;
}

s16 sStep;
float time;
int ii;

void ADC_step(int16_t step) {
    if (step == 0) return;
    if (step > 0) ADC_step_up();
    else ADC_step_down();
    sStep = step;

/*    time = ADC_getTime(); // get screen sweep time

    // looking last parameters set with ScreenTime less than required time
    int i = 1;
    while (ADC_Parameters[i].ScreenTime < time) {
        i++;
        if (i >= ADC_Parameters_Size) break;
    }

    i--;
    ii = i;
    ADC_Prescaler = ADC_Parameters[i].ADC_Prescaler;
    ADC_SampleTime = ADC_Parameters[i].ADC_SampleTime;

    // set X scale
    scaleX = ADC_Parameters[i].ScreenTime / time;
//*/
    ADC_Prescaler = ADC_Parameters[paramNum].ADC_Prescaler;
    ADC_SampleTime = ADC_Parameters[paramNum].ADC_SampleTime;
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
