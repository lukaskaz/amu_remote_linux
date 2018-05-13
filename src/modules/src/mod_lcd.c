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
#define SCROLL_VR       0x29
#define SCROLL_VL       0x2A

#define FRAMS_2         0x07
#define FRAMS_3         0x04
#define FRAMS_4         0x05
#define FRAMS_5         0x00
#define FRAMS_25        0x06
#define FRAMS_64        0x01
#define FRAMS_128       0x02
#define FRAMS_256       0x03

#define LCD_DATA_QUEUE_SIZE    10U

static bool lcdStartScrSaver = false;
static bool lcdClearScreen   = false;

xQueueHandle xQueueLcdControl = NULL;


static void vLCD_Configuration(void);

static int ZtScI2cMxReset(void);
static int ZtScI2cMxSetAddress(void);
static int ZtScI2cMxReadState(void);
static int ZtScI2cMxReadVersion(uint8_t * vbuff);
static int ZtScI2cMxDisplay8x16Str(uint8_t page, uint8_t column, const char *str);
static int ZtScI2cMxFillArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, uint8_t filldata);
static int ZtScI2cMxClearScreen(void);
static int ZtScI2cMxScrollingHorizontal(uint8_t lr, uint8_t spage, uint8_t epage, uint8_t frames);
static int ZtScI2cMxScrollingVertical(uint8_t scrollupdown, uint8_t rowsfixed, uint8_t rowsscroll, uint8_t scrollstep, uint8_t stepdelay);
static int ZtScI2cMxScrollingVerticalHorizontal(uint8_t Sdirection, uint8_t spage, uint8_t epage, uint8_t fixedarea, uint8_t scrollarea, uint8_t offset, uint8_t frames);
static int ZtScI2cMxDeactivateScroll(void);
static int ZtScI2cMxSetLocation(uint8_t page, uint8_t column);


static int ZtScI2cMxReset(void)
{
    uint8_t addr = 0;
    uint8_t buff[2] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    buff[0] =  REG_RESET;
    buff[1] =  RESET_OLED;
    return xI2C_write_sequence(addr, buff, 2);
}

static int ZtScI2cMxSetAddress(void)
{
    uint8_t addr = 0;
    unsigned char buff[2] = {0};
    
    addr = ZTSCI2CMX_ADDRESS;
    buff[0] = REG_ADDRESS;    
    buff[1] = addr;    
    return xI2C_write_sequence(ZTSCI2CMX_DADDRESS, buff, 2);
}

static int ZtScI2cMxReadState(void)
{
    uint8_t addr = 0;
    uint8_t reg[2] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    reg[0] = REG_STATUS;
    xI2C_read_sequence(addr, reg, 1, &reg[1], 1);
    return reg[1];
}

static int ZtScI2cMxReadVersion(uint8_t* vbuff)
{
    uint8_t addr = 0;
    uint8_t reg[1] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    reg[0] = REG_VERSION;
    return xI2C_read_sequence(addr, reg, 1, vbuff, 16);
}

static int ZtScI2cMxDisplay8x16Str(uint8_t page, uint8_t column, const char *str)
{
    uint8_t addr = 0;
    uint8_t buff[19] = {0};
    uint8_t i = 0;
        
    addr = ZTSCI2CMX_ADDRESS;
    buff[0] = REG_8X16STR;
    buff[1] = page;
    buff[2] = column;
    
    i=0;
    while ((*str != '\0') && (i<16))
    {
       buff[i+3] = (uint8_t)*str++;
       i++;
    }
    return xI2C_write_sequence(addr, buff, i+3);
}

static int ZtScI2cMxFillArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, uint8_t filldata)
{
    uint8_t addr = 0;
    uint8_t buff[6] = {0};
    
    addr = ZTSCI2CMX_ADDRESS;
    buff[0] = REG_FILL_AREA;
    buff[1] = spage;
    buff[2] = epage;
    buff[3] = scolumn;
    buff[4] = ecolumn;
    buff[5] = filldata;
    return xI2C_write_sequence(addr, buff, 6);
}

static int ZtScI2cMxClearScreen(void)
{
    ZtScI2cMxFillArea(0, 8, 0, 128, 0x00);
}

static int ZtScI2cMxScrollingHorizontal(uint8_t lr, uint8_t spage, uint8_t epage, uint8_t frames)
{
    uint8_t addr = 0;
    uint8_t buff[9] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    buff[0] = REG_CMD;
    buff[1] = lr;
    buff[2] = 0x00;
    buff[3] = spage;
    buff[4] = frames;
    buff[5] = epage;
    buff[6] = 0x00;
    buff[7] = 0xFF;
    buff[8] = 0x2F;
    return xI2C_write_sequence(addr, buff, 9);
}

static int ZtScI2cMxScrollingVertical(uint8_t scrollupdown, uint8_t rowsfixed, uint8_t rowsscroll, uint8_t scrollstep, uint8_t stepdelay)
{
    uint8_t addr = 0;
    uint8_t buff[6] = {0};

    addr = ZTSCI2CMX_ADDRESS;;
    buff[0] = REG_SCROVER;
    buff[1] = scrollupdown;
    buff[2] = rowsfixed;
    buff[3] = rowsscroll;
    buff[4] = scrollstep;
    buff[5] = stepdelay;
    return xI2C_write_sequence(addr, buff, 6);
}

static int ZtScI2cMxScrollingVerticalHorizontal(uint8_t Sdirection, uint8_t spage, uint8_t epage, uint8_t fixedarea, uint8_t scrollarea, uint8_t offset, uint8_t frames)
{
    uint8_t addr = 0;
    uint8_t buff[8] = {0};
    
    addr = ZTSCI2CMX_ADDRESS;;
    buff[0] = REG_SCROVERHOR;
    buff[1] = Sdirection;
    buff[2] = spage;
    buff[3] = epage;
    buff[4] = fixedarea;
    buff[5] = scrollarea;
    buff[6] = offset;
    buff[7] = frames;
    return xI2C_write_sequence(addr, buff, 8);
}

static int ZtScI2cMxDeactivateScroll(void)
{
    uint8_t addr = 0;
    uint8_t buff[2] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    buff[0] =  REG_CMD;
    buff[1] =  0x2E;
    return xI2C_write_sequence(addr, buff, 2);
}

static int ZtScI2cMxSetLocation(uint8_t page, uint8_t column)
{
    uint8_t addr = 0;
    uint8_t buff[4] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    buff[0] =  REG_CMD;
    buff[1] =  0xB0|page;
    buff[2] =  column%16;
    buff[3] =  column/16+0x10;

    return xI2C_write_sequence(addr, buff, 4);
}

static void ZtScI2cMxDisplayDot16x16(uint8_t page, uint8_t column, const char *valf)
{
    uint8_t addr = 0;
    uint8_t buff[17] = {0};
    uint8_t i = 0;
    
    addr = ZTSCI2CMX_ADDRESS;
    buff[0] = REG_DAT;
    ZtScI2cMxSetLocation(page, column);
    for (i=0; i<16; i++)
    {
       buff[i+1] = valf[i];
    }
    xI2C_write_sequence(addr, buff, 17);
    ZtScI2cMxSetLocation(page+1, column);
    for (i=0; i<16; i++)
    {
       buff[i+1] = valf[i+16];
    }
    xI2C_write_sequence(addr, buff, 17);
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
    uint8_t addr = 0;
    uint8_t buff[2] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    buff[0] =  REG_BRIGHTNESS;
    buff[1] =  val;
    return xI2C_write_sequence(addr, buff, 2);
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
    uint8_t addr = 0;
    uint8_t buff[2] = {0};

    addr = ZTSCI2CMX_ADDRESS;
    buff[0] =  REG_VCOMH;
    buff[1] =  val;
    return xI2C_write_sequence(addr, buff, 2);
}

static void ZtScI2cMxDisplayArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, const char *pt)
{
    uint8_t j = 0;
    uint8_t h = 0;
    uint8_t w = 0;
    uint16_t cnt = 0;
    uint8_t addr = 0;
    uint8_t buff[32] = {0};

    if(xSemaphoreTakeRecursive(xRecMutexI2CSequence, portMAX_DELAY) == pdTRUE) {
        addr = ZTSCI2CMX_ADDRESS;
        buff[0] = REG_DAT;

        h = epage - spage;
        w = ecolumn - scolumn;
        while ( j<h ) {
            uint8_t p = w;
            ZtScI2cMxSetLocation(spage + j, scolumn);

            while(p) {
                if(p>=31) {
                    uint8_t n = 0;
                    for (n=0; n<31; n++) {
                        buff[1+n] = pt[cnt++];
                    }
                    xI2C_write_sequence(addr, buff, 32);
                    p -= 31;
                }
                else {
                    int n;
                    for (n=0; n<p; n++) {
                        buff[1+n] = pt[cnt++];
                    }
                    xI2C_write_sequence(addr, buff, n+1);
                    p -= n;
                }
            }
            j++;
        }

        xSemaphoreGiveRecursive(xRecMutexI2CSequence);
    }
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

    ZtScI2cMxDisplay8x16Str(2, 0, ">AMUv2 prototype");
    ZtScI2cMxDisplay8x16Str(4, 0, "   Welcome :)   ");

    xLcdScrSaverTimer = xTimerCreate((signed char *)"Lcd SS timer", 10000, pdFALSE, (void *)4, vLcdScrSaverCallback);
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
            ZtScI2cMxDisplay8x16Str(3, 0, text);
        }

        if(lcdStartScrSaver == true) {
            lcdStartScrSaver = false;
            ZtScI2cMxDisplayArea(0, 8, 0, 128, robot);
            
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
    // LCD initialisation sequence should be the first on I2C line and I2C line 
    // should be released for some time to prevent demo mode from being started
    ZtScI2cMxReset();
    ZtScI2cMxSetBrightness(0xFF);
    ZtScI2cMxSetVcomH(7);
    
    vTaskDelay(10);
    xSemaphoreGive(xSemaphI2CLcdInitDone);
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
