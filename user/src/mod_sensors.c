/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_sensors.c
** Descriptions:            Sensors service for various measurements
**
**--------------------------------------------------------------------------------------------------------
** Created by:              
** Created date:            
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mod_lighting.h"
#include "mod_orientsensor.h"
#include "mod_sensors.h"

#define SENSORS_TASK_INTERVAL_MS    50
#define SENSORS_ADC_TIMER_STEPS     100
#define SENSORS_ADC_TIMER_FREQ      200
#define SENSORS_DIST_TIMER_STEPS    50000
#define SENSORS_DIST_TIMER_FREQ     20
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

#define SEN_PROX_WARN_DIST_CM       100
#define SEN_PROX_ALERT_DIST_CM      25

static void vAnalogSensors_configuration(void);
static void vDistSensors_configuration(void);
static void vVoltageDetector_configuration(void);

uint16_t ADC_ConvertedData[8] = {0};
double SensorsMeasurements[2] = {0};
double distance[2]            = {0};
sensorDistance_t sensorInUse      = SENSOR_DIST_FIRST;


static inline void supp_ticks_delay(volatile register uint16_t ticks)
{
    while(ticks--);
}

sensorProximity_t get_front_proximity_estimation(void)
{
    sensorProximity_t fresult = SEN_PROX_NONE;
    int32_t front_dist_cm = get_front_distance();

    if(SEN_PROX_WARN_DIST_CM < front_dist_cm) fresult = SEN_PROX_NONE;
    else fresult = SEN_PROX_ALERT_DIST_CM < front_dist_cm ? \
                                    SEN_PROX_WARN:SEN_PROX_ALERT;
    return fresult;
}

#define SENSOR_GET_LIMITED_DISTANCE(dist,limit) dist < 0 ? (uint16_t)(-1): \
                                dist > limit ? (uint16_t)(-1):(uint16_t)dist
uint16_t get_front_distance(void)
{
    return SENSOR_GET_LIMITED_DISTANCE(distance[SENSOR_DIST_FRONT], 500);
}

uint16_t get_back_distance(void)
{
    return SENSOR_GET_LIMITED_DISTANCE(distance[SENSOR_DIST_REAR], 500);
}



sensorBattery_t get_battery_status(void)
{
    sensorBattery_t fresult = SEN_BAT_LOW;
    sensorVolatage_t supplyVoltage = (sensorVolatage_t)PWR_PVDLevelGet();

    if(supplyVoltage < SEN_VOLT_2V7)  fresult = SEN_BAT_LOW;
    else if(supplyVoltage < SEN_VOLT_2V9) fresult = SEN_BAT_HALF;
    else  fresult = SEN_BAT_FULL;
    return fresult;
}

static uint32_t get_13b_oversampled_adc(uint8_t first, uint8_t elems)
{
    uint32_t adcAvgVal = 0;
    for(uint8_t i=first; i < (first + elems); i++) \
                    adcAvgVal += ADC_ConvertedData[i];
    return adcAvgVal>>1;
}

static uint32_t get_12b_average_adc(uint8_t first, uint8_t elems)
{
    uint32_t adcAvgVal = 0;
    for(uint8_t i=first; i < (first + elems); i++) \
                    adcAvgVal += ADC_ConvertedData[i];
    return adcAvgVal/elems;
}

double get_internal_temp(void)
{
//    uint32_t v_adc = get_13b_oversampled_adc(4, 4);
//    double v_25 = 1.41f, avg_slope = 0.0043f, v_sense = 3.3f*v_adc/8191.0f,
    uint32_t v_adc = get_12b_average_adc(4, 4);
    double v_25 = 1.41f, avg_slope = 0.0043f, v_sense = 3.3f*v_adc/4095.0f,
           tempVal = 25 + (v_25 - v_sense)/avg_slope;
    return tempVal;
}

double get_illumination(void)
{
//    uint16_t v_adc = get_13b_oversampled_adc(0, 4);
//    double l_factor = 34.5, l_offset = 10, v_rev = 3.3f*(8191-v_adc)/8191.0f,
    uint16_t v_adc = get_12b_average_adc(0, 4);
    double l_factor = 34.5, l_offset = 10, v_rev = 3.3f*(4095-v_adc)/4095.0f,
            lightVal = l_factor * v_rev - l_offset;
    return lightVal;
}

void trigger_dist_sensor(sensorDistance_t sensor)
{
    static const double undetected_val = -1;
    uint16_t trigger_src = SENSOR_DIST_FRONT == sensor ? TIM_TS_TI2FP2:TIM_TS_TI1FP1,
             trigger_pin = SENSOR_DIST_FRONT == sensor ? GPIO_Pin_10:GPIO_Pin_11,
             reading_delay_ms = SENSOR_DIST_FRONT == sensor ? 30:15;

    TIM_SelectInputTrigger(TIM4, trigger_src);
    GPIO_ResetBits(GPIOB, trigger_pin);
    GPIO_SetBits(GPIOB, trigger_pin);
    supp_ticks_delay(1000);
    GPIO_ResetBits(GPIOB, trigger_pin);
    TIM_SetCounter(TIM4, 0);
    sensorInUse = sensor;

    if(pdFALSE == xTaskNotifyWait(0x00, 0x00, NULL, pdMS_TO_TICKS(reading_delay_ms))) {
        distance[sensor] = undetected_val;  // no echo within given time
    }
}

void trigger_all_dist_sensors(void)
{
    for(sensorDistance_t sensor = SENSOR_DIST_FIRST; sensor < SENSOR_DIST_LAST; sensor++) {
        trigger_dist_sensor(sensor);
    }
}

uint8_t PWR_PVDLevelGet(void)
{ 
    return (uint8_t)((PWR->CR >> 5) & 0x07); 
}

void vCheckSupplyVoltage(FlagStatus voltage_drop)
{
    sensorVolatage_t supplyVoltage = (sensorVolatage_t)PWR_PVDLevelGet();
    if(SET == voltage_drop) { // PVDO set -> voltage going down
        switch(supplyVoltage) {
            case SEN_VOLT_2V9: PWR_PVDLevelConfig(PWR_PVDLevel_2V8); break;
            case SEN_VOLT_2V8: PWR_PVDLevelConfig(PWR_PVDLevel_2V7); break;
            case SEN_VOLT_2V7: PWR_PVDLevelConfig(PWR_PVDLevel_2V6); break;
            case SEN_VOLT_2V6: PWR_PVDLevelConfig(PWR_PVDLevel_2V5); break;
            case SEN_VOLT_2V5: PWR_PVDLevelConfig(PWR_PVDLevel_2V4); break;
            case SEN_VOLT_2V4: PWR_PVDLevelConfig(PWR_PVDLevel_2V3); break;
            case SEN_VOLT_2V3: PWR_PVDLevelConfig(PWR_PVDLevel_2V2); break;
            case SEN_VOLT_2V2: // no break
            default:  break;
        }
    }
    else { // PVDO cleared -> voltage is going up
        switch(supplyVoltage) {
            case SEN_VOLT_2V8: PWR_PVDLevelConfig(PWR_PVDLevel_2V9); break;
            case SEN_VOLT_2V7: PWR_PVDLevelConfig(PWR_PVDLevel_2V8); break;
            case SEN_VOLT_2V6: PWR_PVDLevelConfig(PWR_PVDLevel_2V7); break;
            case SEN_VOLT_2V5: PWR_PVDLevelConfig(PWR_PVDLevel_2V6); break;
            case SEN_VOLT_2V4: PWR_PVDLevelConfig(PWR_PVDLevel_2V5); break;
            case SEN_VOLT_2V3: PWR_PVDLevelConfig(PWR_PVDLevel_2V4); break;
            case SEN_VOLT_2V2: PWR_PVDLevelConfig(PWR_PVDLevel_2V3); break;
            case SEN_VOLT_2V9: // no break
            default: break;
        }
    }
}

TickType_t auxil_get_ticks_duration_ms(TickType_t* timepoint_prev)
{
    TickType_t timepoint_curr = xTaskGetTickCount(),
                timeslot = pdTICKS_TO_MS(timepoint_curr - *timepoint_prev);
    *timepoint_prev = timepoint_curr;
    return timeslot;
}

/*******************************************************************************
* Function Name  : vSensorsServiceTask
* Description    : Sensors service routine
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vSensorsTask(void * pvArg)
{
    TickType_t xLastTimePoint = 0, timepoint_prev = 0;
    const TickType_t interval = pdMS_TO_TICKS(SENSORS_TASK_INTERVAL_MS);

    vAnalogSensors_configuration();
    vDistSensors_configuration();
    vVoltageDetector_configuration();
    vOrientation_sensor_configuration();

    xLastTimePoint = timepoint_prev = xTaskGetTickCount();
    while(1) {
        TickType_t timeslot = auxil_get_ticks_duration_ms(&timepoint_prev);
        gyro_get_data(timeslot);
        accl_get_data(timeslot);
        trigger_all_dist_sensors();
        vTaskDelayUntil(&xLastTimePoint, interval);
    }
}

void vVoltageDetector_configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  // detect voltage droping only
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    PWR_PVDLevelConfig(PWR_PVDLevel_2V9);
    PWR_PVDCmd(ENABLE);
}

void vAnalogSensors_config_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // Configure PA.04 (ADC Channel_4) as analog input
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

void vAnalogSensors_config_timer(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    RCC_ClocksTypeDef RCC_Clocks = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_GetClocksFreq(&RCC_Clocks); 
    TIM_TimeBaseStructure.TIM_Period = \
        TIMER_STEPS_TO_PERIOD(SENSORS_ADC_TIMER_STEPS);
    TIM_TimeBaseStructure.TIM_Prescaler = \
        TIMER_FREQHZ_TO_PRESCAL(RCC_Clocks, SENSORS_ADC_TIMER_STEPS, SENSORS_ADC_TIMER_FREQ);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = SENSORS_ADC_TIMER_STEPS/2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void vAnalogSensors_config_adc_dma(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = SIZEOF_ARRAY(ADC_ConvertedData);;
    ADC_Init(ADC1, &ADC_InitStructure);  

    // setup sequencer order and sampling time for specific tasks
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  2, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  3, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  4, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 5,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 6,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 7,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 8,  ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(SET == ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(SET == ADC_GetCalibrationStatus(ADC1));
    
    ADC_DMACmd(ADC1, ENABLE);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SIZEOF_ARRAY(ADC_ConvertedData);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    //DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);  
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void vAnalogSensors_configuration(void)
{
    vAnalogSensors_config_gpio();
    vAnalogSensors_config_timer();
    vAnalogSensors_config_adc_dma();
}

void vDistSensors_configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    TIM_ICInitTypeDef  TIM_ICInitStructure = {0};
    RCC_ClocksTypeDef  RCC_Clocks = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // TRIGGER configuration of sensor_1 and sensor_2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ECHO configuration of both sensors
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_GetClocksFreq(&RCC_Clocks);
    //set timer with 20HZ freq using 50000 steps, 1step ~ 1us -> max 50ms
    TIM_TimeBaseStructure.TIM_Period = \
        TIMER_STEPS_TO_PERIOD(SENSORS_DIST_TIMER_STEPS);
    TIM_TimeBaseStructure.TIM_Prescaler = \
        TIMER_FREQHZ_TO_PRESCAL(RCC_Clocks, SENSORS_DIST_TIMER_STEPS, SENSORS_DIST_TIMER_FREQ);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Gated);
//    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    TIM_ITConfig(TIM4, TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
