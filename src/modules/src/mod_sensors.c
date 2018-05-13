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
#include <stdlib.h>

#include "mod_sensors.h"
#include "stm32f10x_it.h"
#include "mod_lighting.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mod_orientation_sensor.h"



#define ADC1_DR_Address    ((uint32_t)0x4001244C)

typedef enum {
    SEN_ILLUM_NONE = 10,
    SEN_ILLUM_MIN  = 20,
    SEN_ILLUM_MID  = 45,
    SEN_ILLUM_MAX  = 70,
    SEN_ILLUM_HIS  = 2,
} sensorIllumination_t;

typedef enum {
    SEN_VOLT_2V2 = 0,
    SEN_VOLT_2V3,
    SEN_VOLT_2V4,
    SEN_VOLT_2V5,
    SEN_VOLT_2V6,
    SEN_VOLT_2V7,
    SEN_VOLT_2V8,
    SEN_VOLT_2V9
} sensorVolatage_t;

static void vAnalogSensors_configuration(void);
static void vDistSensors_configuration(void);
static void vVoltageDetector_configuration(void);
static double get_internal_temp(void);
static double get_illumination(void);
static uint8_t PWR_PVDLevelGet(void);

uint16_t ADC_ConvertedData[8] = {0};
double SensorsMeasurements[2] = {0};
double distance[2]            = {0};
sensorType_t sensorInUse      = SENSOR_FRONT;



extern bool acclValsFlag;
//extern int8_t acclVals[];
sensorProximity_t get_front_proximity_estimation(void)
{
    sensorProximity_t fresult = SEN_PROX_NONE;
    uint32_t distValue = (uint32_t)distance[SENSOR_FRONT];
    
    if(distValue > 150 || distValue == 0) {
        // no obstacle detected or distance to it is safe
        // do nothing
    }
    else if(distValue > 100) {
        fresult = SEN_PROX_WRN_1;
    }
    else if(distValue > 30) {
        fresult = SEN_PROX_WRN_2;
    }
    else {
        //acclValsFlag = true;
        fresult = SEN_PROX_ALERT;
    }
    
    return fresult;
}

void adjust_auto_lighting(void)
{
    static sensorIllumination_t currIllumination = SEN_ILLUM_MAX;
    
    if(isLightingAutoModeEnabled() == true) {
        double illumination = get_illumination();

        //printf("illumiantion: %f, %d\r\n", illumination, currIllumination);
        if(illumination > SEN_ILLUM_MAX) {
            if(currIllumination != SEN_ILLUM_MAX) {
                if(illumination > (SEN_ILLUM_MAX + SEN_ILLUM_HIS)) {
                    currIllumination = SEN_ILLUM_MAX;
                    vLighting_Auto_Control(LIGHT_AUTO_NONE);
                }
                else {
                    // fluctuations in max and mid levels, do nothing
                }
            }
            else {
                // lightning disable peak case, do nothing
            }
        }
        else if(illumination > SEN_ILLUM_MID) {
            if(currIllumination > SEN_ILLUM_MID) {
                currIllumination = SEN_ILLUM_MID;
                vLighting_Auto_Control(LIGHT_AUTO_MIN);
            }
            else if(currIllumination == SEN_ILLUM_MID) {
                // lightning minimum case, do nothing
            }
            else if(illumination > (SEN_ILLUM_MID + SEN_ILLUM_HIS)) {
                // case for illumination below medium
                currIllumination = SEN_ILLUM_MID;
                vLighting_Auto_Control(LIGHT_AUTO_MIN);
            }
            else {
                // fluctuations in mid and min levels, do nothing
            }
        }
        else if(illumination > SEN_ILLUM_MIN) {
            if(currIllumination > SEN_ILLUM_MIN) {
                currIllumination = SEN_ILLUM_MIN;
                vLighting_Auto_Control(LIGHT_AUTO_MEDIUM);
            }
            else if(currIllumination == SEN_ILLUM_MIN) {
                // lightning medium case, do nothing
            }
            else if(illumination > (SEN_ILLUM_MIN + SEN_ILLUM_HIS)) {
                // case for illumination below minimum
                currIllumination = SEN_ILLUM_MIN;
                vLighting_Auto_Control(LIGHT_AUTO_MEDIUM);
            }
            else {
                // fluctuations in min and none levels, do nothing
            }
        }
        else {
            if(currIllumination != SEN_ILLUM_NONE) {
                currIllumination = SEN_ILLUM_NONE;
                vLighting_Auto_Control(LIGHT_AUTO_MAX);
            }
            else {
                // lightning off case, do nothing
            }
        }
    }
    else {
        currIllumination = SEN_ILLUM_MAX;
    }
}

sensorBattery_t get_battery_status(void)
{
    sensorBattery_t fresult = SEN_BAT_LOW;
    sensorVolatage_t supplyVoltage = (sensorVolatage_t)PWR_PVDLevelGet();
    
    if(supplyVoltage < SEN_VOLT_2V7) {
        fresult = SEN_BAT_LOW;
    }
    else if(supplyVoltage < SEN_VOLT_2V9) {
        fresult = SEN_BAT_HALF;
    }
    else {
        // battery >= 2,9V
        fresult = SEN_BAT_FULL;
    }
    
    return fresult;
}

static double get_internal_temp(void)
{
    uint8_t i = 0;
    uint16_t adcAvgVal = 0;
    double voltVal = 0, tempVal = 0;
    
    for(i=4; i<8; i++) {
        adcAvgVal += ADC_ConvertedData[i];
    }
    adcAvgVal >>= 1;
    
    voltVal = (3.3 * adcAvgVal) / 8191U;
    tempVal = 25 + (1.41 - voltVal)/0.0043;

    return tempVal;
}

static double get_illumination(void)
{
    uint8_t i = 0;
    uint16_t adcAvgVal = 0;
    double voltVal = 0, lightVal = 0;
    
    for(i=0; i<4; i++) {
        adcAvgVal += ADC_ConvertedData[i];
    }
    adcAvgVal >>= 1;
    
    voltVal = (3.3 * (8191U - adcAvgVal)) / 8191U;
    lightVal = 34.5 * voltVal - 10U;

    return lightVal;
}

void trigger_front_sensor(void)
{
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    vTaskDelay(1);  
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    TIM_SetCounter(TIM4, 0);

    distMeasReady = false;
    sensorInUse = SENSOR_FRONT;
    vTaskDelay(30);
    if(distMeasReady == false) {
        distance[SENSOR_FRONT] = 0;  // no obstacle detected, set to 0
    }
}

void trigger_rear_sensor(void)
{
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);

    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    vTaskDelay(1);  
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    TIM_SetCounter(TIM4, 0);

    distMeasReady = false;  
    sensorInUse = SENSOR_REAR;
    vTaskDelay(10);
    if(distMeasReady == false) {
        distance[SENSOR_REAR] = 0;  // no obstacle detected, set to 0
    }
}

static uint8_t PWR_PVDLevelGet(void) 
{ 
    return (uint8_t)((PWR->CR >> 5) & 0x07); 
}

void vCheckSupplyVoltage(FlagStatus voltageDropping)
{
    sensorVolatage_t supplyVoltage = (sensorVolatage_t)PWR_PVDLevelGet();
    if(voltageDropping == SET) {
        // PVDO is set -> voltage going down
        switch(supplyVoltage) {
            case SEN_VOLT_2V9:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V8);
                break;
            case SEN_VOLT_2V8:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V7);
                break;
            case SEN_VOLT_2V7:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V6);
                break;
            case SEN_VOLT_2V6:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V5);
                break;
            case SEN_VOLT_2V5:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V4);
                break;
            case SEN_VOLT_2V4:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V3);
                break;
            case SEN_VOLT_2V3:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V2);
                break;
            case SEN_VOLT_2V2:
                break;
            default: {
                //case should not occur
            }
        }
    }
    else {
        // PVDO is cleared -> voltage is going up
        switch(supplyVoltage) {
            case SEN_VOLT_2V9:
                break;
            case SEN_VOLT_2V8:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V9);
                break;
            case SEN_VOLT_2V7:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V8);
                break;
            case SEN_VOLT_2V6:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V7);
                break;
            case SEN_VOLT_2V5:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V6);
                break;
            case SEN_VOLT_2V4:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V5);
                break;
            case SEN_VOLT_2V3:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V4);
                break;
            case SEN_VOLT_2V2:
                PWR_PVDLevelConfig(PWR_PVDLevel_2V3);
                break;
            default: {
                //case should not occur
            }
        }
    }
}


/*******************************************************************************
* Function Name  : vSensorsServiceTask
* Description    : Sensors service routine
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vSensorsServiceTask(void * pvArg)
{
    vAnalogSensors_configuration();
    vDistSensors_configuration();
    vVoltageDetector_configuration();

    while(1)
    {

        //printf("Voltage: %d,%d,%d\r\n", timer, power_pvd, PWR_PVDLevelGet());
        //
        //printf("Nb: %d, power PVD: %d\r\n", timer, power_pvd);

        //printf("ADC1: %.2f, %.2f\n\r", get_internal_temp(), get_illumination());
        //printf("Dist: %.2fcm, %.2fcm*/\n\r", distance[SENSOR_FRONT], distance[SENSOR_REAR]);
        adjust_auto_lighting();
        trigger_front_sensor();
        trigger_rear_sensor();
   
        //vTaskDelay(10);
        
        if(acclValsFlag == true) {
            int i = 0;
            
            printf("\r\n--start--\r\n");
            for(i=0; i < SEN_ACCL_VALS_SIZE; i++) {
//                printf("%d, ", acclVals[i]);
            }
            printf("\r\n--end--\r\n");
            vTaskDelay(1000);
        }
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

void vAnalogSensors_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_ClocksTypeDef  RCC_Clocks;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
    /* Configure PA.04 (ADC Channel_4) as analog input */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_GetClocksFreq(&RCC_Clocks); 
    /* Time base configuration */ 
    TIM_TimeBaseStructure.TIM_Period        = 65000U - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/1000000UL - 1;  //for 1MHz frequency
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    /* Mode configuration: Channel1 */ 
    TIM_OCInitStructure.TIM_OCMode          = TIM_OCMode_Timing; 
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Disable; 
    TIM_OCInitStructure.TIM_Pulse           = 50000;
    TIM_OCInitStructure.TIM_OCPolarity      = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 8;
    ADC_Init(ADC1, &ADC_InitStructure);  

    /* setup sequencer order and sampling time for specific tasks */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  2, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  3, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  4, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 5,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 6,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 7,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 8,  ADC_SampleTime_239Cycles5);
    /* activate ADC module and calibrate it */
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);
    
    /* Enable DMA controller for storing data in a defined memory space for consecutive measurememts */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable internal temperature sensor for measuring ambient temperature */
    ADC_TempSensorVrefintCmd(ENABLE);
    
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    /* Enable analog data processing through determined sequencer */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 8;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    //DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);  
    /* Enable DMA Channel1 for ADC */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    

}

void vDistSensors_configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
    //TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    RCC_ClocksTypeDef  RCC_Clocks;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    /* Sensor_1 */
    /* TRIGGER configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ECHO configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Sensor_2 */
    /* TRIGGER configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ECHO configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_GetClocksFreq(&RCC_Clocks); 
    /* Time base configuration */ 
    TIM_TimeBaseStructure.TIM_Period        = 65000U - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/1000000UL - 1;  //for 1MHz frequency
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
//    TIM_ICInitStructure.TIM_Channel         = TIM_Channel_2;
//    TIM_ICInitStructure.TIM_ICPolarity      = TIM_ICPolarity_Rising;
//    TIM_ICInitStructure.TIM_ICSelection     = TIM_ICSelection_DirectTI;
//    TIM_ICInitStructure.TIM_ICPrescaler     = TIM_ICPSC_DIV1;
//    TIM_ICInitStructure.TIM_ICFilter        = 0x0; 
//    TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
  
    TIM_ICInitStructure.TIM_Channel         = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity      = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection     = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler     = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter        = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Gated);
    //TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    
    TIM_Cmd(TIM4, ENABLE); 
    //TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
    //TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Trigger, ENABLE);   
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
