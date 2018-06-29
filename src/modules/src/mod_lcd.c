/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_lcd.c
** Descriptions:            Lcd interface for visual information
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
#include <stdbool.h>
#include <string.h>

#include "mod_lcd.h"
#include "mod_lcd_consts.h"
#include "mod_i2c.h"
#include "mod_sensors.h"
#include "mod_orientation_sensor.h"
#include "stm32f10x_it.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define ZTSCI2CMX_DADDRESS    0x51
#define ZTSCI2CMX_ADDRESS     0x27

#define REG_CMD          0x01
#define REG_DAT          0x02
#define REG_RESET        0x03
#define RESET_OLED      0x06
#define REG_VERSION      0x1F
#define REG_VCOMH         0x05
#define REG_STATUS        0x06
#define STATUS_RUN            0x00
#define STATUS_SET_ADDRESS    0x02
#define STATUS_BUSY           0x10

#define REG_ADDRESS      0x08
#define REG_BRIGHTNESS   0x0A
#define REG_8X16STR      0x52
#define REG_OLED_XY      0x60
#define REG_FILL_AREA    0x61
#define REG_SCROHOR      0x62
#define REG_SCROVER      0x63
#define REG_SCROVERHOR   0x64

#define SCROLL_UP       0x01
#define SCROLL_DOWN     0x00
#define SCROLL_RIGHT    0x26
#define SCROLL_LEFT     0x27
#define SCROLL_VR       0x00
#define SCROLL_VL       0x01

#define FRAMS_2         0x07
#define FRAMS_3         0x04
#define FRAMS_4         0x05
#define FRAMS_5        0x06
//#define FRAMS_5         0x00
//#define FRAMS_25        0x06
#define FRAMS_64        0x01
#define FRAMS_128       0x02
#define FRAMS_256       0x03

#define LCD_DATA_QUEUE_SIZE    10U

static bool lcdStartScrSaver = false;
static bool lcdClearScreen   = false;

xQueueHandle xQueueLcdControl = NULL;


//static void vLCD_Configuration(void);

static void vLCD_Configuration(void);;
static int ZtScI2cMxReset(void);
static int ZtScI2cMxSetAddress(uint8_t addr);
static int ZtScI2cMxReadState(uint8_t* state);
static int ZtScI2cMxReadVersion(uint8_t * vbuff);
static int ZtScI2cMxDisplay8x16Str(uint8_t page, uint8_t column, const char *str);
static int ZtScI2cMxFillArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, uint8_t filldata);
static int ZtScI2cMxClearScreen(void);
static int ZtScI2cMxScrollingHorizontal(uint8_t lr, uint8_t spage, uint8_t epage, uint8_t frames);
static int ZtScI2cMxScrollingVertical(uint8_t scrollupdown, uint8_t rowsfixed, uint8_t rowsscroll, uint8_t scrollstep, uint8_t stepdelay);
static int ZtScI2cMxScrollingVerticalHorizontal(uint8_t Sdirection, uint8_t spage, uint8_t epage, uint8_t fixedarea, uint8_t scrollarea, uint8_t offset, uint8_t frames);
static int ZtScI2cMxDeactivateScroll(void);
static int ZtScI2cMxSetLocation(uint8_t page, uint8_t column);


static bool ZtScI2cMxWaitLcdReady(void)
{
    bool fresult = false;
    int counter = 1000;
    uint8_t status = 0xFF;

    while(1) {
        ZtScI2cMxReadState(&status);
        if(status == STATUS_RUN) {
            fresult = true;
            break;
        }
        else if(counter == 0) {
            break;
        }
        else {
            counter--;
            vTaskDelay(1);
        }
    }

    return fresult;
}

static int ZtScI2cMxReset(void)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_RESET,
        .data = NULL,
        .dataSize = 1,
    };
    uint8_t data = RESET_OLED;

    params.data = &data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

//static int ZtScI2cMxReset(void)
//{
//    uint8_t addr = 0;
//    uint8_t buff[2] = {0};
//
//    addr = ZTSCI2CMX_ADDRESS;
//    buff[0] =  REG_RESET;
//    buff[1] =  RESET_OLED;
//
//
//    return xI2C_write_sequence(addr, buff, 2);
//}

static int ZtScI2cMxSetAddress(uint8_t addr)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_ADDRESS,
        .data = NULL,
        .dataSize = 1,
    };
    uint8_t data = addr;

    params.data = &data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxReadState(uint8_t* state)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_STATUS,
        .data = NULL,
        .dataSize = 1,
    };

    params.data = state;
    return xI2C_read_sequence(&params);
}

static int ZtScI2cMxReadVersion(uint8_t* vbuff)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_VERSION,
        .data = NULL,
        .dataSize = 16,
    };

    params.data = vbuff;
    return xI2C_read_sequence(&params);
}

static int ZtScI2cMxDisplay8x16Str(uint8_t page, uint8_t column, const char *str)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_8X16STR,
        .data = NULL,
        .dataSize = 2,
    };
    uint8_t data[18] =
    {
        [0] = page,
        [1] = column,
    };
    uint8_t i = 0;
    uint8_t* dataStr = &data[2];

    params.dataSize = 2;
    for(i=0; i < 16 && str[i] != '\0'; i++) {
        dataStr[i] = str[i];
        params.dataSize++;
    }

    params.data = data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxFillArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, uint8_t filldata)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_FILL_AREA,
        .data = NULL,
        .dataSize = 5,
    };
    uint8_t data[] =
    {
        [0] = spage,
        [1] = epage,
        [2] = scolumn,
        [3] = ecolumn,
        [4] = filldata
    };

    params.data = data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxClearScreen(void)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_FILL_AREA,
        .data = NULL,
        .dataSize = 5,
    };
    static uint8_t data[] =
    {
        [0] = 0x00,
        [1] = 0x08,
        [2] = 0x00,
        [3] = 0x80,
        [4] = 0x00
    };

    params.data = data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxScrollingHorizontal(uint8_t lr, uint8_t spage, uint8_t epage, uint8_t frames)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_CMD,
        .data = NULL,
        .dataSize = 8,
    };
    uint8_t data[] =
    {
        [0] = lr,
        [1] = 0x00,
        [2] = spage,
        [3] = frames,
        [4] = epage,
        [5] = 0x00,
        [6] = 0xFF,
        [7] = 0x2F
    };

    params.data = data;
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxScrollingVertical(uint8_t scrollupdown, uint8_t rowsfixed, uint8_t rowsscroll, uint8_t scrollstep, uint8_t stepdelay)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_SCROVER,
        .data = NULL,
        .dataSize = 5,
    };
    uint8_t data[] =
    {
        [0] = scrollupdown,
        [1] = rowsfixed,
        [2] = rowsscroll,
        [3] = scrollstep,
        [4] = stepdelay
    };

    params.data = data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxScrollingVerticalHorizontal(uint8_t Sdirection, uint8_t spage, uint8_t epage, uint8_t fixedarea, uint8_t scrollarea, uint8_t offset, uint8_t frames)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_SCROVERHOR,
        .data = NULL,
        .dataSize = 7
    };
    uint8_t data[] =
    {
        [0] = Sdirection,
        [1] = spage,
        [2] = epage,
        [3] = fixedarea,
        [4] = scrollarea,
        [5] = offset,
        [6] = frames
    };

    params.data = data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxDeactivateScroll(void)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_CMD,
        .data = NULL,
        .dataSize = 1
    };
    uint8_t data = 0x2E;

    params.data = &data;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static int ZtScI2cMxSetLocation(uint8_t page, uint8_t column)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_CMD,
        .data = NULL,
        .dataSize = 3
    };
    uint8_t data[] =
    {
        [0] = 0xB0|page,
        [1] = column%16,
        [2] = column/16+0x10
    };

    params.data = data;
    return xI2C_write_sequence1(&params);
}

static void ZtScI2cMxDisplayDot16x16(uint8_t page, uint8_t column, const char *valf)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_DAT,
        .data = NULL,
        .dataSize = 16
    };


    params.data = &valf[0];
    ZtScI2cMxSetLocation(page, column);
    xI2C_write_sequence1(&params);

    params.data = &valf[16];
    ZtScI2cMxSetLocation(page+1, column);
    xI2C_write_sequence1(&params);
}

/* 
 * Function ScI2cMxSetBrightness
 * Desc     Set ZT.SC-I2CMx Brightness
 * Input    val: Brightness 0~0xFF
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
static int ZtScI2cMxSetBrightness(uint8_t val)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_BRIGHTNESS,
        .data = NULL,
        .dataSize = 1,
    };

    params.data = &val;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

/*
* Function ScI2cMxSetVcomH
* Desc Set ZT.SC-I2CMx VcomH
* Input val: Brightness 0~7
* Output 0 .. success
* 1 .. length to long for buffer
* 2 .. address send, NACK received
* 3 .. data send, NACK received
* 4 .. other twi error (lost bus arbitration, bus error, ..)
*/
static int ZtScI2cMxSetVcomH(uint8_t val)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_VCOMH,
        .data = NULL,
        .dataSize = 1,
    };

    params.data = &val;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence1(&params);
}

static void ZtScI2cMxDisplayArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, const char *pt)
{
    I2cParams_t params =
    {
        .devAddr = ZTSCI2CMX_ADDRESS<<1,
        .ctrlReg = REG_DAT,
        .data = NULL,
        .dataSize = 0,
    };

    uint8_t i = 0;
    uint8_t height = epage - spage;
    uint8_t width = ecolumn - scolumn;

    ZtScI2cMxWaitLcdReady();
    for(i=0; i < height; i++, spage++) {
        ZtScI2cMxSetLocation(spage, scolumn);

        params.data = pt;
        params.dataSize = width;
        pt += width;

        xI2C_write_sequence1(&params);
    }
}

static int isLcdMenuButtonPressed(void)
{
    register int pressed = 0;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == Bit_SET) {
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == Bit_SET) {
            vTaskDelay(25);
        }
        pressed = 1;
    }

    return pressed;
}

static int isLcdSelectionButtonPressed(void)
{
    register int pressed = 0;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_SET) {
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_SET) {
            vTaskDelay(25);
        }
        pressed = 1;
    }

    return pressed;
}

void vLcdScrSaverCallback(xTimerHandle pxTimer)
{
    lcdStartScrSaver = true;
}

void vLcdClearScrCallback(xTimerHandle pxTimer)
{
    lcdClearScreen = true;
}

/*******************************************************************************
* Function Name  : vLcdInterfaceTask
* Description    : Lcd interface main routine
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void vLcdInterfaceTask(void * pvArg)
{
    xTimerHandle xLcdScrSaverTimer;
    xTimerHandle xLcdClearScreenTimer;
    vLCD_Configuration();
    xQueueLcdControl = xQueueCreate(LCD_DATA_QUEUE_SIZE, sizeof(lcdControlData_t));

//    vTaskDelay(1000);
//    while(1) {
//        char ver1[20] = {0};
//
//        ZtScI2cMxReadVersion(ver1);
//        vTaskDelay(1500);
//    }

    ZtScI2cMxDisplay8x16Str(2, 0, ">AMUv2 prototype");
    //vTaskDelay(10);

    int welcomeCnt = 0;
    while(1) {
        ZtScI2cMxDisplay8x16Str(4, 0, "   Welcome :)   ");
        if(++welcomeCnt > 2) {
            break;
        }
        else {
            vTaskDelay(1000);
            ZtScI2cMxDisplay8x16Str(4, 0, "           ;)   ");
            vTaskDelay(1000);
        }
    }


    xLcdScrSaverTimer = xTimerCreate((signed char *)"Lcd SS timer", 1000, pdFALSE, (void *)4, vLcdScrSaverCallback);
    xLcdClearScreenTimer = xTimerCreate((signed char *)"Lcd clear screen timer", 3000, pdFALSE, (void *)5, vLcdClearScrCallback);
    xTimerStart(xLcdScrSaverTimer, 0);
    
    while(1)
    { 
        lcdControlData_t lcdData = {0};

        if(xQueueReceive(xQueueLcdControl, &lcdData, 50U) == pdTRUE) {
            xTimerStart(xLcdScrSaverTimer, 0);

            if(lcdData.operation == LCD_OP_DRIVE) {
                if(lcdData.state == LCD_MV_IN_MOTION) {
                    if(xTimerIsTimerActive(xLcdClearScreenTimer) == pdFALSE) {
                        ZtScI2cMxClearScreen();
                        ZtScI2cMxDisplay8x16Str(3, 0, "    <<<<<<<<    ");
                        vTaskDelay(2);
                        ZtScI2cMxDeactivateScroll();
                        ZtScI2cMxScrollingHorizontal(SCROLL_LEFT, 3, 4, FRAMS_2);
                        xTimerStop(xLcdScrSaverTimer, 0);
                        xTimerStop(xLcdClearScreenTimer, 0);
                    }
                    else {
                        vTaskDelay(10);
                        xQueueSend(xQueueLcdControl, (void *)&lcdData, 0);
                    }
                }
                else if(lcdData.state == LCD_MV_STOPPED) {
                    ZtScI2cMxDeactivateScroll();
                    ZtScI2cMxClearScreen();
                    xQueueReset(xQueueLcdControl);
                }
                else {
                    // unsupported state, do nothing
                }
            }
            else if(lcdData.operation == LCD_OP_SOUND_SIG) {
                if(lcdData.state == LCD_SOUND_ON) {
                    ZtScI2cMxClearScreen();
                    ZtScI2cMxDisplay8x16Str(3, 0, "  HORN ENABLED  ");
                    vTaskDelay(2);
                    ZtScI2cMxDeactivateScroll();
                    ZtScI2cMxScrollingHorizontal(SCROLL_LEFT, 3, 4, FRAMS_2);
                    xTimerStop(xLcdScrSaverTimer, 0);
                    xTimerStop(xLcdClearScreenTimer, 0);
                }
                else if(lcdData.state == LCD_SOUND_OFF) {
                    ZtScI2cMxDeactivateScroll();
                    ZtScI2cMxClearScreen();
                }
                else {
                    // unsupported state, do nothing
                }
            }
            else if(lcdData.operation == LCD_OP_COLLISION) {
                if(lcdData.state == LCD_COLLIS_ON) {
                    ZtScI2cMxClearScreen();
                    ZtScI2cMxDisplay8x16Str(3, 0, "  COLLISION !!  ");
                    ZtScI2cMxScrollingVertical(SCROLL_UP, 0, 64, 1, 250);
                    xTimerStart(xLcdClearScreenTimer, 0);
                }
            }
            else if(lcdData.operation == LCD_OP_LIGHTING) {
                if(lcdData.state == LCD_LIGHT_AUTO) {
                    ZtScI2cMxClearScreen();
                    ZtScI2cMxDisplay8x16Str(3, 0, "   AUTO LIGHT   ");
                    vTaskDelay(2);
                    ZtScI2cMxDeactivateScroll();
                    ZtScI2cMxScrollingHorizontal(SCROLL_LEFT, 3, 4, FRAMS_2);
                    xTimerStart(xLcdClearScreenTimer, 0);
                }
                else if(lcdData.state == LCD_LIGHT_MANUAL) {
                    ZtScI2cMxClearScreen();
                    ZtScI2cMxDisplay8x16Str(3, 0, "  MANUAL LIGHT  ");
                    vTaskDelay(2);
                    ZtScI2cMxDeactivateScroll();
                    ZtScI2cMxScrollingHorizontal(SCROLL_LEFT, 3, 4, FRAMS_2);
                    xTimerStart(xLcdClearScreenTimer, 0);
                }
                else {
                    // unsupported case, do nothing
                }
            }
            else {
                // unsupported operation, do nothing
            }
        }
        else {
            // no lcd action pending, do nothing
            char text[20+1] = {0};

            snprintf(text, 20, "Front: %.2f", distance[SENSOR_FRONT]);
            //ZtScI2cMxDisplay8x16Str(3, 0, text);

            extern AcclData_t acclData;
            char acclBuff[10+1] = {0};

            int tmp = acclData.vect.x * 10;
            if(tmp < 0) {
                snprintf(acclBuff, 10, "X:%.1f", acclData.vect.x);
            }
            else {
                snprintf(acclBuff, 10, "X: %.1f", acclData.vect.x);
            }
            ZtScI2cMxDisplay8x16Str(2, 78, acclBuff);

            tmp = acclData.vect.y * 10;
            if(tmp < 0) {
                snprintf(acclBuff, 10, "Y:%.1f", acclData.vect.y);
            }
            else {
                snprintf(acclBuff, 10, "Y: %.1f", acclData.vect.y);
            }
            ZtScI2cMxDisplay8x16Str(4, 78, acclBuff);

            tmp = acclData.vect.z * 10;
            if(tmp < 0) {
                snprintf(acclBuff, 10, "Z:%.1f", acclData.vect.z);
            }
            else {
                snprintf(acclBuff, 10, "Z: %.1f", acclData.vect.z);
            }
            ZtScI2cMxDisplay8x16Str(6, 78, acclBuff);


            if(isLcdMenuButtonPressed()) {
                printf("[%s] Menu button pressed!!\n", __func__);

                xTimerStop(xLcdScrSaverTimer, 0);

                ZtScI2cMxClearScreen();
                ZtScI2cMxDisplay8x16Str(2, 0, "      MENU      ");
                ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 1000);
                vTaskDelay(2000);
                ZtScI2cMxDeactivateScroll();

                while(1) {
                    static int menu = 0, sel = 0;

                    if(menu != 1) {
                        ZtScI2cMxDisplay8x16Str(0, 0, "    option 1    ");
                    }
                    else {
                        if(sel) {
                            sel = 0;
                            ZtScI2cMxClearScreen();
                            ZtScI2cMxDisplay8x16Str(2, 0, "Selected opt. 1");
                            ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 1000);
                            vTaskDelay(2000);

                            continue;
                        }
                        else {
                            ZtScI2cMxDisplay8x16Str(0, 0, "*    option 1   ");
                        }
                    }

                    if(menu != 2) {
                        ZtScI2cMxDisplay8x16Str(2, 0, "    option 2    ");
                    }
                    else {
                        if(sel) {
                            sel = 0;
                            ZtScI2cMxClearScreen();
                            ZtScI2cMxDisplay8x16Str(2, 0, "Selected opt. 2");
                            ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 1000);
                            vTaskDelay(2000);
                            continue;
                        }
                        else {
                            ZtScI2cMxDisplay8x16Str(2, 0, "*    option 2   ");
                        }
                    }

                    if(menu != 3) {
                        ZtScI2cMxDisplay8x16Str(4, 0, "    option 3    ");
                    }
                    else {
                        if(sel) {
                            sel = 0;
                            ZtScI2cMxClearScreen();
                            ZtScI2cMxDisplay8x16Str(2, 0, "Selected opt. 3");
                            ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 1000);
                            vTaskDelay(2000);
                            continue;
                        }
                        else {
                            ZtScI2cMxDisplay8x16Str(4, 0, "*    option 3   ");
                        }
                    }

                    if(menu != 4) {
                        ZtScI2cMxDisplay8x16Str(6, 0, "      EXIT      ");
                    }
                    else {
                        if(sel) {
                            sel = 0;
                            ZtScI2cMxClearScreen();
                            ZtScI2cMxDisplay8x16Str(2, 0, " EXITTING MENU! ");
                            ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 1000);
                            vTaskDelay(2000);
                            break;
                        }
                        else {
                            ZtScI2cMxDisplay8x16Str(6, 0, "*      EXIT     ");
                        }
                    }

                    sel = 0;
                    while(1) {
                        if(isLcdMenuButtonPressed()) {
                            menu = (menu < 4) ? menu+1 : 1;
                            break;
                        }
                        if(isLcdSelectionButtonPressed()) {
                            sel = 1;
                            break;
                        }
                        vTaskDelay(100);
                    }
                }

                xTimerStart(xLcdScrSaverTimer, 0);
            }

            if(isLcdSelectionButtonPressed()) {
                printf("[%s] Selection button pressed!!\n", __func__);
            }
        }

        if(lcdStartScrSaver == true) {
            static int robotNb = 1;

            lcdStartScrSaver = false;
            xTimerStart(xLcdScrSaverTimer, 0);

            if(welcomeCnt) {
                welcomeCnt = 0;
                ZtScI2cMxClearScreen();

                int i = 0;
                for(i = 0; i < 10; i++) {
                    uint8_t state = 0xFF;
                    ZtScI2cMxReadState(&state);
                    printf("[%s] state: %#x\n", __func__, state);
                    vTaskDelay(1);
                }

                //vTaskDelay(10);
                char ver[50] = {0};
                ZtScI2cMxReadVersion(ver);
                printf("[%s] version: %s\n", __func__, ver);

                //vTaskDelay(10);
            }

            if(robotNb == 1) {
                ZtScI2cMxDisplayArea(0, 8, 0, 64, robot_3);
                robotNb = 2;
                //vTaskDelay(1000);
                //ZtScI2cMxDisplayArea(0, 8, 0, 64, robot_4);
                //vTaskDelay(1000);
                //ZtScI2cMxDeactivateScroll();
                //ZtScI2cMxScrollingHorizontal(SCROLL_RIGHT, 0, 1, FRAMS_128);
                //ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 100);
                //ZtScI2cMxDeactivateScroll();
                //vTaskDelay(1000);

                //ZtScI2cMxDisplayArea(0, 8, 0, 64, robot_3);
                //vTaskDelay(1000);
                //ZtScI2cMxDisplayArea(0, 8, 0, 64, robot_4);
                //vTaskDelay(1000);
                //ZtScI2cMxScrollingVerticalHorizontal(0x00, 0, 7, 0, 64, 1, FRAMS_2);


                //while(1) vTaskDelay(10000);
                //ZtScI2cMxDeactivateScroll();
                //ZtScI2cMxClearScreen();
            }
            else {
                ZtScI2cMxDisplayArea(0, 8, 0, 64, robot_4);
                robotNb = 1;
            }

            switch(get_battery_status()) {
                case SEN_BAT_FULL:
                    ZtScI2cMxDisplayArea(0, 2, 104, 128, battery_full);
                    break;
                case SEN_BAT_HALF:
                    ZtScI2cMxDisplayArea(0, 2, 104, 128, battery_half);
                    break;
                case SEN_BAT_LOW:
                    ZtScI2cMxDisplayArea(0, 2, 104, 128, battery_low);
                    break;
                default: {}
                }


        }
        
        if(lcdClearScreen == true) {
            lcdClearScreen = false;
            ZtScI2cMxDeactivateScroll();
            ZtScI2cMxClearScreen();
            // introduce 250ms pause in cleared screen between consecutive events
            vTaskDelay(250);
        }
    }
}

static void vLCD_Configuration(void)
{
    // LCD initialization sequence should be the first on I2C line and I2C line
    // should be released for some time to prevent demo mode from being started

    vTaskDelay(100);
    ZtScI2cMxReset();
    vTaskDelay(10);
    ZtScI2cMxSetBrightness(0xFF);
    ZtScI2cMxSetVcomH(7);
    
    uint8_t ZtScI2cMxVersion[16+1] = {0};
    ZtScI2cMxReadVersion(ZtScI2cMxVersion);
    ZtScI2cMxDisplay8x16Str(0, 0, "OLED INITIALIZED");
    ZtScI2cMxDisplay8x16Str(3, 0, "   FW VERSION   ");
    ZtScI2cMxDisplay8x16Str(5, 0, ZtScI2cMxVersion);

    vTaskDelay(2000);
    ZtScI2cMxClearScreen();
    vTaskDelay(1000);
    //xSemaphoreGive(xSemaphI2CLcdInitDone);

    static GPIO_InitTypeDef GPIO_InitStructure =
    {
        .GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_8,
        .GPIO_Mode  = GPIO_Mode_IPD
    };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
