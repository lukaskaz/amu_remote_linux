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

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "mod_led.h"
#include "mod_i2c.h"
#include "mod_drive.h"
#include "mod_lighting.h"
#include "mod_sound.h"
#include "mod_sensors.h"
#include "mod_orientsensor.h"
#include "mod_lcd_consts.h"
#include "mod_lcd.h"

#define ZTSCI2CMX_DADDRESS    0x51
#define ZTSCI2CMX_ADDRESS     0x27
#define LCD_SHIFTED_ADDR      (ZTSCI2CMX_ADDRESS<<1)

#define LCD_PARAMS(reg, data, size) \
    &(I2cParams_t){LCD_SHIFTED_ADDR, reg, data, size}

#define REG_CMD          0x01
#define REG_DAT          0x02
#define REG_RESET        0x03
#define RESET_OLED      0x06
#define REG_VERSION      0x1F
#define REG_VCOMH         0x05
#define REG_STATUS        0x06
#define STATUS_READY        0x00
#define STATUS_SET_ADDRESS  0x02
#define STATUS_BUSY         0x10

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

#define LCD_ROWS_PER_CHAR       2
#define LCD_COLS_PER_CHAR       8
#define LCD_STRING_SIZE_MAX     16
#define LCD_PRESS_KEYS_PORT     GPIOB
#define LCD_PRESS_KEY_MENU      GPIO_Pin_2
#define LCD_PRESS_KEY_SELECT    GPIO_Pin_8
#define LCD_MENU_SEL_SYM        ">"

#define LCD_TASK_INTERVAL_MS         100
#define LCD_MENU_INTERVAL_MS         25

#define LCD_ELEM_FIRST(menu)    LCD_##menu##_FIRST
#define LCD_ELEM_LAST(menu)     LCD_##menu##_LAST
#define LCD_ITEM_EXIT(menu)     LCD_##menu##_EXIT
#define MENU_NAME_TO_DISPLAY(name) \
    " "name"          " : LCD_MENU_SEL_SYM" "name"          "
#define MENU_NAME_NORMAL(name) \
    " "name"          "
#define MENU_NAME_SELECTED(name) \
    LCD_MENU_SEL_SYM" "name"          "
#define SUBMENU_OPTION_TO_DISPLAY(name, isset) \
    (true == isset)?name"[*]":name"    ":LCD_MENU_SEL_SYM" "name"  "
#define MENU_TITLE_5CHAR(name) \
    "#  "#name" MENU  #"
#define MENU_TITLE_6CHAR(name) \
    "# "#name" MENU  #"
#define MENU_TITLE_4CHAR(name) \
    "## "#name" MENU  ##"
#define MENU_NAME_TO_DISPLAY(name) \
    " "name"          " : LCD_MENU_SEL_SYM" "name"          "
#define CALL_SUBMENU_FUNC(func,param) if(NULL != func) func(param)


#define PREDEFINE_MAINMENU_STRUCT(name,idname,scrolldef)                      \
    .normname = MENU_NAME_NORMAL(name), .scrollname = scrolldef(idname),      \
    .exit_item = LCD_ITEM_EXIT(idname),                                       \
    .first_elem = LCD_ELEM_FIRST(idname), .last_elem = LCD_ELEM_LAST(idname)

#define PREDEFINE_SUBMENU_STRUCT(name,scrolldef,idname)                       \
    PREDEFINE_MAINMENU_STRUCT(name,scrolldef,idname),                         \
    .selname = MENU_NAME_SELECTED(name), .exit_ref = &exitmenu

typedef enum{
    ROBOT_HAND_DOWN = 0,
    ROBOT_HAND_UP,
} lcdRobotScreen_t;

typedef enum {
    LCD_MAIN_FIRST = 0,
    LCD_MAIN_NONE = LCD_MAIN_FIRST,
    LCD_MAIN_MOV,
    LCD_MAIN_LIGHT,
    LCD_MAIN_SIGNAL,
    LCD_MAIN_ORIENT,
    LCD_MAIN_ENVIRO,
    LCD_MAIN_STATUS,
    LCD_MAIN_EXIT,
    LCD_MAIN_LAST,
} lcdMainMenuItems_t;

typedef enum {
    LCD_MOVEMT_FIRST = 0,
    LCD_MOVEMT_NONE = LCD_MOVEMT_FIRST,
    LCD_MOVEMT_EXIT,
    LCD_MOVEMT_LAST
} lcdMovMenuItems_t;

typedef enum {
    LCD_LIGHT_FIRST = 0,
    LCD_LIGHT_NONE = LCD_LIGHT_FIRST,
    LCD_LIGHT_AUTO,
    LCD_LIGHT_MANU,
    LCD_LIGHT_EXIT,
    LCD_LIGHT_LAST
} lcdLightMenuItems_t;

typedef enum {
    LCD_SIGNAL_FIRST = 0,
    LCD_SIGNAL_NONE = LCD_SIGNAL_FIRST,
    LCD_SIGNAL_EXIT,
    LCD_SIGNAL_LAST
} lcdSignalMenuItems_t;

typedef enum {
    LCD_ORIENT_FIRST = 0,
    LCD_ORIENT_NONE = LCD_ORIENT_FIRST,
    LCD_ORIENT_EXIT,
    LCD_ORIENT_LAST
} lcdOrinetMenuItems_t;

typedef enum {
    LCD_ENVIRO_FIRST = 0,
    LCD_ENVIRO_NONE = LCD_ENVIRO_FIRST,
    LCD_ENVIRO_EXIT,
    LCD_ENVIRO_LAST
} lcdEnviroMenuItems_t;

typedef enum {
    LCD_STATUS_FIRST = 0,
    LCD_STATUS_NONE = LCD_STATUS_FIRST,
    LCD_STATUS_REBOOT,
    LCD_STATUS_BOOTLD,
    LCD_STATUS_EXIT,
    LCD_STATUS_LAST
} lcdStatusMenuItems_t;

typedef enum {
    LCD_EXIT_FIRST = 0,
    LCD_EXIT_NONE = LCD_EXIT_FIRST,
    LCD_EXIT_EXIT,
    LCD_EXIT_LAST
} exitMenuItems_t;

typedef struct submenu submenu_t;
typedef void(*lcd_submenu_func)(uint8_t* __restrict);

typedef struct submenu {
    char* normname;
    char* selname;
    char* scrollname;
    uint8_t first_elem;
    uint8_t last_elem;
    uint8_t exit_item;
    lcd_submenu_func func_options;
    lcd_submenu_func func_confirm;
    lcd_submenu_func func_status;
    const submenu_t* exit_ref;
} submenu_t;

typedef struct {
    char* normname;
    char* scrollname;
    uint8_t first_elem;
    uint8_t last_elem;
    uint8_t exit_item;
    const submenu_t** submenus;
} mainmenu_t;

static void vLCD_Configuration(void);
static int ZtScI2cMxReset(void);
static int ZtScI2cMxSetAddress(uint8_t);
static uint8_t ZtScI2cMxReadState(uint8_t*);
static int ZtScI2cMxReadVersion(char*);
static int ZtScI2cMxDisplay8x16Str(uint8_t, uint8_t, char *);
static int ZtScI2cMxFillArea(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
static int ZtScI2cMxClearScreen(void);
static int ZtScI2cMxScrollingHorizontal(uint8_t, uint8_t, uint8_t, uint8_t);
static int ZtScI2cMxScrollingVertical(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
static int ZtScI2cMxScrollingVerticalHorizontal(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
static int ZtScI2cMxDeactivateScroll(void);
static int ZtScI2cMxSetLocation(uint8_t, uint8_t);
static void lcd_mov_display_state(uint8_t* __restrict);
static void lcd_light_display_options(uint8_t*);
static void lcd_light_on_confirm(uint8_t* __restrict);
static void lcd_light_display_state(uint8_t*);
static void lcd_signal_on_confirm(uint8_t* __restrict);
static void lcd_signal_display_state(uint8_t* __restrict);
static void lcd_orient_display_state(uint8_t* __restrict);
static void lcd_enviro_display_state(uint8_t* __restrict);
static void lcd_status_display_options(uint8_t* __restrict);
static void lcd_status_on_confirm(uint8_t* __restrict);
static void lcd_status_display_state(uint8_t* __restrict);
static void supp_jump_bootloader(void);

static const submenu_t exitmenu = {
    PREDEFINE_SUBMENU_STRUCT("EXIT MENU", EXIT, MENU_TITLE_4CHAR),
    .func_options = NULL,
    .func_confirm = NULL,
    .func_status = NULL
};

static const submenu_t movmenu = {
    PREDEFINE_SUBMENU_STRUCT("Movement", MOVEMT, MENU_TITLE_6CHAR),
    .func_options = NULL,
    .func_confirm = NULL,
    .func_status = lcd_mov_display_state
};

static const submenu_t ligmenu = {
    PREDEFINE_SUBMENU_STRUCT("Lighting", LIGHT, MENU_TITLE_5CHAR),
    .func_options = lcd_light_display_options,
    .func_confirm = lcd_light_on_confirm,
    .func_status = lcd_light_display_state
};

static const submenu_t sigmenu = {
    PREDEFINE_SUBMENU_STRUCT("Sound signal", SIGNAL, MENU_TITLE_6CHAR),
    .func_options = NULL,
    .func_confirm = lcd_signal_on_confirm,
    .func_status = lcd_signal_display_state
};

static const submenu_t orientmenu = {
    PREDEFINE_SUBMENU_STRUCT("Orientation", ORIENT, MENU_TITLE_6CHAR),
    .func_options = NULL,
    .func_confirm = NULL,
    .func_status = lcd_orient_display_state
};

static const submenu_t enviromenu = {
    PREDEFINE_SUBMENU_STRUCT("Environment", ENVIRO, MENU_TITLE_6CHAR),
    .func_options = NULL,
    .func_confirm = NULL,
    .func_status = lcd_enviro_display_state
};

static const submenu_t statusmenu = {
    PREDEFINE_SUBMENU_STRUCT("Status", STATUS, MENU_TITLE_6CHAR),
    .func_options = lcd_status_display_options,
    .func_confirm = lcd_status_on_confirm,
    .func_status = lcd_status_display_state
};

static const mainmenu_t mainmenu = {
    PREDEFINE_MAINMENU_STRUCT("MAIN MENU", MAIN, MENU_TITLE_4CHAR),
    .submenus = (const submenu_t*[]) {
        [LCD_MAIN_NONE]   = NULL,       [LCD_MAIN_MOV]    = &movmenu,     \
        [LCD_MAIN_LIGHT]  = &ligmenu,   [LCD_MAIN_SIGNAL] = &sigmenu,     \
        [LCD_MAIN_ORIENT] = &orientmenu,[LCD_MAIN_ENVIRO] = &enviromenu,  \
        [LCD_MAIN_STATUS] = &statusmenu,[LCD_MAIN_EXIT]   = &exitmenu
    }
};

static bool lcdShowScrSaver = true;


static inline void supp_ticks_delay(volatile register uint16_t ticks)
{
    while(ticks--);
}

static bool ZtScI2cMxWaitLcdReady(void)
{
    static const uint16_t cnt_max = 1000;
    uint8_t status = STATUS_BUSY;

    for(uint16_t cnt=0; cnt < cnt_max; cnt++) {
        ZtScI2cMxReadState(&status);
        if(status == STATUS_READY) break;
        else supp_ticks_delay(250);
    }

    return status == STATUS_READY ? true:false;
}

static int ZtScI2cMxReset(void)
{
    static uint8_t data = RESET_OLED;

    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_RESET, &data, 1));
}

static int ZtScI2cMxSetAddress(uint8_t addr)
{
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_ADDRESS, &addr, 1));
}

static uint8_t ZtScI2cMxReadState(uint8_t* state)
{
    return xI2C_read_sequence_slow(LCD_PARAMS(REG_STATUS, state, 1));
}

static int ZtScI2cMxReadVersion(char* vbuff)
{
    ZtScI2cMxWaitLcdReady();
    return xI2C_read_sequence_slow(LCD_PARAMS(REG_VERSION, (uint8_t*)vbuff, 16));
}

static int ZtScI2cMxDisplay8x16Str(uint8_t line, uint8_t column, char *str)
{
    uint8_t data[2+LCD_STRING_SIZE_MAX] = { LCD_ROWS_PER_CHAR*line, LCD_COLS_PER_CHAR*column },
        pos = SIZEOF_ARRAY(data) - LCD_STRING_SIZE_MAX, size = SIZEOF_ARRAY(data) - column;

    for(; pos < size && *str != '\0'; pos++) data[pos] = *str++;
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_8X16STR, data, pos));
}

static int ZtScI2cMxFillArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, uint8_t filldata)
{
    uint8_t data[5] = \
        { spage, epage, scolumn, ecolumn, filldata };

    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_FILL_AREA, data, SIZEOF_ARRAY(data)));
}

static int ZtScI2cMxClearScreen(void)
{
    return ZtScI2cMxFillArea(0, 8, 0, 128, 0x00);
}

static int ZtScI2cMxClearLines(uint8_t sline, uint8_t eline)
{
    return ZtScI2cMxFillArea(sline, eline, 0, 128, 0x00);
}

static int ZtScI2cMxScrollingHorizontal(uint8_t lr, uint8_t spage, uint8_t epage, uint8_t frames)
{
    uint8_t data[8] = \
        { lr, 0x00, spage, frames, epage, 0x00, 0xFF, 0x2F };

    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_CMD, data, SIZEOF_ARRAY(data)));
}

static int ZtScI2cMxScrollingVertical(uint8_t scrollupdown, uint8_t rowsfixed, uint8_t rowsscroll, uint8_t scrollstep, uint8_t stepdelay)
{
    uint8_t data[5] = \
        { scrollupdown, rowsfixed, rowsscroll, scrollstep, stepdelay };

    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_SCROVER, data, SIZEOF_ARRAY(data)));
}

static int ZtScI2cMxScrollingVerticalHorizontal(uint8_t Sdirection, uint8_t spage, uint8_t epage, uint8_t fixedarea, uint8_t scrollarea, uint8_t offset, uint8_t frames)
{
    uint8_t data[7] = \
        { Sdirection, spage, epage, fixedarea, scrollarea, offset, frames };

    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_SCROVERHOR, data, SIZEOF_ARRAY(data)));
}

static int ZtScI2cMxDeactivateScroll(void)
{
    static uint8_t data = 0x2E;

    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_CMD, &data, 1));
}

static int ZtScI2cMxSetLocation(uint8_t page, uint8_t column)
{
    uint8_t data[3] = { 0xB0|page, column%16, column/16+0x10 };
    return xI2C_write_sequence(LCD_PARAMS(REG_CMD, data, SIZEOF_ARRAY(data)));
}

static void ZtScI2cMxDisplayDot16x16(uint8_t page, uint8_t column, char *valf)
{
    ZtScI2cMxSetLocation(page, column);
    xI2C_write_sequence(LCD_PARAMS(REG_DAT, (uint8_t*)&valf[0], 16));
    ZtScI2cMxSetLocation(page+1, column);
    xI2C_write_sequence(LCD_PARAMS(REG_DAT, (uint8_t*)&valf[16], 16));
}


// ScI2cMxSetBrightness: Brightness 0~0xFF
static int ZtScI2cMxSetBrightness(uint8_t val)
{
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_BRIGHTNESS, &val, 1));
}

// ScI2cMxSetVcomH: Brightness 0~7
static int ZtScI2cMxSetVcomH(uint8_t val)
{
    ZtScI2cMxWaitLcdReady();
    return xI2C_write_sequence(LCD_PARAMS(REG_VCOMH, &val, 1));
}

static void ZtScI2cMxDisplayArea(uint8_t spage, uint8_t epage, uint8_t scolumn, uint8_t ecolumn, const uint8_t *pt)
{
    const uint16_t width = ecolumn-scolumn;

    ZtScI2cMxWaitLcdReady();
    for(uint8_t page = spage; page < epage; page++, pt+=width) {
        ZtScI2cMxSetLocation(page, scolumn);
        xI2C_write_sequence(LCD_PARAMS(REG_DAT, (uint8_t*)pt, width));
    }
}

static inline bool is_key_pressed(uint16_t key)
{
    static const TickType_t delay_ms = pdMS_TO_TICKS(25);
    register const bool key_pressed = \
        Bit_SET == GPIO_ReadInputDataBit(LCD_PRESS_KEYS_PORT, key) ? true:false;

    if(true == key_pressed) {
        while(Bit_SET == GPIO_ReadInputDataBit(LCD_PRESS_KEYS_PORT, key)) {
            vTaskDelay(delay_ms);
        }
        vTaskDelay(delay_ms);
    }
    return key_pressed;

}

static inline bool is_lcd_menu_key_pressed(void)
{
    return is_key_pressed(LCD_PRESS_KEY_MENU);
}

static inline bool is_lcd_select_key_pressed(void)
{
    return is_key_pressed(LCD_PRESS_KEY_SELECT);
}

static void vLcdScrSaverCallback(xTimerHandle pxTimer)
{
    lcdShowScrSaver = true;
}

static void lcd_string_fill_and_display(uint8_t line, uint8_t col, char* str, uint16_t size)
{
    uint16_t len = strlen(str), pos = len;
    for(; pos < size - 1; pos++) str[pos] = ' ';
    str[pos] = '\0';
    ZtScI2cMxDisplay8x16Str(line, col, str);
}

static void lcd_string_fill_and_display_degree(uint8_t line, uint8_t col, char* str, uint16_t size)
{
    uint16_t len = strlen(str), pos = len;
    uint8_t linesym = 2*line, colsym = 8*col + 8*len;
    for(; pos < size - 1; pos++) str[pos] = ' ';
    str[len] = '\0';
    str[pos] = '\0';
    ZtScI2cMxDisplay8x16Str(line, col, str);
    ZtScI2cMxDisplay8x16Str(line, col+len+1, &str[len+1]);
    ZtScI2cMxDisplayArea(linesym, linesym+2, colsym, colsym+8, degree_symbol);
}

static void lcd_string_fill_and_display_celc_degree(uint8_t line, uint8_t col, char* str, uint16_t size)
{
    uint16_t len = strlen(str), pos = len;
    uint8_t linesym = 2*line, colsym = 8*col + 8*len;
    for(; pos < size - 1; pos++) str[pos] = ' ';
    str[len] = '\0';
    str[len+1] = 'C';
    str[pos] = '\0';
    ZtScI2cMxDisplay8x16Str(line, col, str);
    ZtScI2cMxDisplay8x16Str(line, col+len+1, &str[len+1]);
    ZtScI2cMxDisplayArea(linesym, linesym+2, colsym, colsym+8, degree_symbol);
}

static void lcd_submenu_scroll_title(char* __restrict name)
{
    ZtScI2cMxClearScreen();
    ZtScI2cMxDisplay8x16Str(2, 0, name);
    ZtScI2cMxScrollingHorizontal(SCROLL_LEFT, 4, 5, FRAMS_2);
    vTaskDelay(pdMS_TO_TICKS(2900));
    ZtScI2cMxDeactivateScroll();
    vTaskDelay(pdMS_TO_TICKS(2000));
    ZtScI2cMxClearScreen();
}

static void lcd_mainmenu_scroll_title(char* __restrict name)
{
    ZtScI2cMxClearScreen();
    ZtScI2cMxDisplay8x16Str(2, 0, name);
    ZtScI2cMxScrollingVertical(SCROLL_DOWN, 0, 64, 1, 200);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ZtScI2cMxDeactivateScroll();
    ZtScI2cMxClearScreen();
}

static void lcd_mov_display_screen(uint8_t line, uint8_t drive_dir)
{
    const uint8_t* screen = NULL, screen_lines = 6;
    switch(drive_dir) {
        case DRIVE_OP_FORWARD:
        case DRIVE_OP_JOY_FORWARD: screen = mov_forward; break;
        case DRIVE_OP_BACKWARD:
        case DRIVE_OP_JOY_BACKWARD: screen = mov_backward; break;
        case DRIVE_OP_LEFT:
        case DRIVE_OP_JOY_FW_LEFT:
        case DRIVE_OP_JOY_BW_LEFT: screen = mov_left; break;
        case DRIVE_OP_RIGHT:
        case DRIVE_OP_JOY_FW_RIGHT:
        case DRIVE_OP_JOY_BW_RIGHT: screen = mov_right; break;
        case DRIVE_OP_STOPPED:
        case DRIVE_OP_JOY_STOPPED: screen = mov_stopped; break;
        default: break;
    }

    if(NULL != screen) \
        ZtScI2cMxDisplayArea(line, line+screen_lines, 68, 128, screen);
}

static void lcd_mov_display_state(uint8_t* __restrict state_prev)
{
    static const char *dirInfo[] = { [DRIVE_OP_STOPPED]  = "STP",
        [DRIVE_OP_FORWARD] = "FWD",  [DRIVE_OP_BACKWARD] = "BWD",
        [DRIVE_OP_LEFT]    = "LFT",  [DRIVE_OP_RIGHT]    = "RGT",
        [DRIVE_OP_JOY_FORWARD] = "FWD", [DRIVE_OP_JOY_BACKWARD] = "BWD",
        [DRIVE_OP_JOY_FW_LEFT]    = "LFT", [DRIVE_OP_JOY_FW_RIGHT]    = "RGT",
        [DRIVE_OP_JOY_BW_LEFT]    = "LFT", [DRIVE_OP_JOY_BW_RIGHT]    = "RGT"
    };
    char infoTxt[8+1] = {0};
    const movement_settings_t state_curr = drive_get_status();

    if(state_curr.direction != *state_prev) {
        if(DRIVE_OP_WAIT != state_curr.direction) {
            *state_prev = state_curr.direction;
            snprintf(infoTxt, SIZEOF_ARRAY(infoTxt), \
                    "Dir: %s", dirInfo[state_curr.direction]);
            lcd_string_fill_and_display(0, 0, infoTxt, SIZEOF_ARRAY(infoTxt));
            lcd_mov_display_screen(0, state_curr.direction);
        }
    }

    snprintf(infoTxt, SIZEOF_ARRAY(infoTxt), "SL: %d%%", state_curr .speed_lft);
    lcd_string_fill_and_display(1, 0, infoTxt, SIZEOF_ARRAY(infoTxt));
    snprintf(infoTxt, SIZEOF_ARRAY(infoTxt), "SR: %d%%", state_curr .speed_rgt);
    lcd_string_fill_and_display(2, 0, infoTxt, SIZEOF_ARRAY(infoTxt));
}

static void lcd_light_display_options(uint8_t* __restrict sel)
{
    bool is_auto = light_is_auto();
    char* light_auto_name = NULL,* light_manu_name = NULL;

    if(LCD_LIGHT_AUTO == *sel && true == is_auto) (*sel)++;
    if(LCD_LIGHT_MANU == *sel && false == is_auto) (*sel)++;

    light_auto_name = LCD_LIGHT_AUTO != *sel ? \
            SUBMENU_OPTION_TO_DISPLAY("Auto", is_auto);
    light_manu_name = LCD_LIGHT_MANU != *sel ? \
            SUBMENU_OPTION_TO_DISPLAY("Manu", !is_auto);

    ZtScI2cMxDisplay8x16Str(2, 0, light_auto_name);
    ZtScI2cMxDisplay8x16Str(2, 8, light_manu_name);
}

static void lcd_light_on_confirm(uint8_t* __restrict sel)
{
    static uint8_t led_pos = 0;

    if(LCD_LIGHT_AUTO == *sel) light_set_auto(true);
    if(LCD_LIGHT_MANU == *sel) {
        light_set_auto(false);
        light_off_all();
        led_pos = 0;
    }
    if(LCD_LIGHT_NONE == *sel && false == light_is_auto()) {
        led_pos = LIGHT_LED_BIT_POS_MAX > led_pos ? led_pos+1:0;
        light_set_led_on(auxil_get_bit_from_pos(led_pos));
    }
    *sel = LCD_LIGHT_NONE;
}

static void lcd_light_display_state(uint8_t* __restrict state_prev)
{
    const uint8_t state_curr = light_get_leds_on(),* screen = NULL;

    if(state_curr != *state_prev) {
        *state_prev = state_curr;
        switch(state_curr) {
            case LIGHT_RF_LEFT:  screen = lighting_left;  break;
            case LIGHT_RF_RIGHT: screen = lighting_right; break;
            case LIGHT_RF_INNER: screen = lighting_inner; break;
            case LIGHT_RF_OUTER: screen = lighting_outer; break;
            case LIGHT_RF_LEFT|LIGHT_RF_RIGHT: screen = lighting_left_right;
                break;
            case LIGHT_RF_INNER|LIGHT_RF_LEFT:
                screen = lighting_left_inner;
                break;
            case LIGHT_RF_INNER|LIGHT_RF_RIGHT:
                screen = lighting_right_inner;
                break;
            case LIGHT_RF_INNER|LIGHT_RF_LEFT|LIGHT_RF_RIGHT:
                screen = lighting_left_right_inner;
                break;
            case LIGHT_RF_INNER|LIGHT_RF_OUTER:
                screen = lighting_inner_outer;
                break;
            case LIGHT_RF_INNER|LIGHT_RF_OUTER|LIGHT_RF_LEFT:
                screen = lighting_left_inner_outer;
                break;
            case LIGHT_RF_INNER|LIGHT_RF_OUTER|LIGHT_RF_RIGHT:
                screen = lighting_right_inner_outer;
                break;
            case LIGHT_RF_OUTER|LIGHT_RF_LEFT:
                screen = lighting_left_outer;
                break;
            case LIGHT_RF_OUTER|LIGHT_RF_RIGHT:
                screen = lighting_right_outer;
                break;
            case LIGHT_RF_OUTER|LIGHT_RF_LEFT|LIGHT_RF_RIGHT:
                screen = lighting_left_right_outer;
                break;
            case LIGHT_RF_OUTER|LIGHT_RF_INNER|LIGHT_RF_LEFT|LIGHT_RF_RIGHT:
                screen = lighting_all;
                break;
            case LIGHT_RF_NONE: //no break
            default: screen = lighting_none;
                break;
        }
        ZtScI2cMxDisplayArea(0, 4, 24, 104, screen);
    }
}

static void lcd_signal_on_confirm(uint8_t* __restrict sel)
{
    if(LCD_SIGNAL_NONE == *sel) {
        uint8_t state = true == signal_is_on() ? SOUND_RF_NONE:SOUND_RF_TOGGLE;
        signal_set_state(state);
    }
}

static void lcd_signal_display_state(uint8_t* __restrict state_prev)
{
    const uint8_t state_curr = signal_is_on();

    if(state_curr != *state_prev) {
        *state_prev = state_curr;
        ZtScI2cMxDisplayArea(0, 6, 32, 96, state_curr?buzzer_on:buzzer_off);
    }
}

static void lcd_orient_display_state(uint8_t* __restrict state_prev)
{
    char acclTxt[9+1] = {0}, gyroTxt[7+1] = {0};
    VectorInt_t accl = { sensor_get_tiltx(), sensor_get_tilty(), sensor_get_tiltz()};

    snprintf(acclTxt, SIZEOF_ARRAY(acclTxt), "Y: %d", accl.y);
    lcd_string_fill_and_display_degree(0, 0, acclTxt, SIZEOF_ARRAY(acclTxt));
    snprintf(acclTxt, SIZEOF_ARRAY(acclTxt), "X: %d", accl.x);
    lcd_string_fill_and_display_degree(1, 0, acclTxt, SIZEOF_ARRAY(acclTxt));
    snprintf(acclTxt, SIZEOF_ARRAY(acclTxt), "Z: %d", accl.z);
    lcd_string_fill_and_display_degree(2, 0, acclTxt, SIZEOF_ARRAY(acclTxt));

    VectorInt_t gyro = { sensor_get_roll(), sensor_get_pitch(), sensor_get_yaw()};
    snprintf(gyroTxt, SIZEOF_ARRAY(gyroTxt), "R: %d", gyro.x);
    lcd_string_fill_and_display_degree(0, 9, gyroTxt, SIZEOF_ARRAY(gyroTxt));
    snprintf(gyroTxt, SIZEOF_ARRAY(gyroTxt), "P: %d", gyro.y);
    lcd_string_fill_and_display_degree(1, 9, gyroTxt, SIZEOF_ARRAY(gyroTxt));
    snprintf(gyroTxt, SIZEOF_ARRAY(gyroTxt), "Y: %d", gyro.z);
    lcd_string_fill_and_display_degree(2, 9, gyroTxt, SIZEOF_ARRAY(gyroTxt));
}

static void lcd_enviro_display_state(uint8_t* __restrict state_prev)
{
    char distTxt[15+1] = {0}, illumTxt[15+1] = {0};
    uint16_t dist_front = get_front_distance(), dist_back = get_back_distance();

    snprintf(distTxt, SIZEOF_ARRAY(distTxt), dist_front == 0xFFFF ? \
                            "Front: UNDETECT":"Front: %dcm", dist_front);
    lcd_string_fill_and_display(0, 0, distTxt, SIZEOF_ARRAY(distTxt));
    snprintf(distTxt, SIZEOF_ARRAY(distTxt), dist_back == 0xFFFF ? \
                            "Back: UNDETECT":"Back: %dcm", dist_back);
    lcd_string_fill_and_display(1, 0, distTxt, SIZEOF_ARRAY(distTxt));

    double illum_lvl = get_illumination();
    snprintf(illumTxt, SIZEOF_ARRAY(illumTxt), "Illum: %.1f%%", illum_lvl);
    lcd_string_fill_and_display(2, 0, illumTxt, SIZEOF_ARRAY(illumTxt));
}

static void lcd_status_display_options(uint8_t* __restrict sel)
{
    char* status_reboot_name = LCD_STATUS_REBOOT != *sel ? \
            SUBMENU_OPTION_TO_DISPLAY("Reboot", 0),
        * status_bootld_name = LCD_STATUS_BOOTLD != *sel ? \
            SUBMENU_OPTION_TO_DISPLAY("Bootld", 0);
    uint8_t bootld_name_pos = LCD_STATUS_BOOTLD != *sel ? 9:8;
    ZtScI2cMxDisplay8x16Str(2, 0, status_reboot_name);
    ZtScI2cMxDisplay8x16Str(2, bootld_name_pos, status_bootld_name);
}

static void lcd_status_on_confirm(uint8_t* __restrict sel)
{
    if(LCD_STATUS_REBOOT == *sel) NVIC_SystemReset();
    if(LCD_STATUS_BOOTLD == *sel) {
        ZtScI2cMxClearScreen();
        ZtScI2cMxDisplay8x16Str(0, 0, "+--------------+");
        ZtScI2cMxDisplay8x16Str(1, 0, "|BOOTLDR JUMPED|");
        ZtScI2cMxDisplay8x16Str(2, 0, "|START FLASHING|");
        ZtScI2cMxDisplay8x16Str(3, 0, "+--------------+");
        led_set_brightness_level(0);
        supp_jump_bootloader();
    }
}

static void lcd_status_display_state(uint8_t* __restrict state_prev)
{
    char tempTxt[13+1] = {0}, voltTxt[16+1] = {0};
    double tempVal = get_internal_temp();

    snprintf(tempTxt, SIZEOF_ARRAY(tempTxt), tempVal < 0 ? \
                            "Tjct:%.1f":"Tjct: %.1f", tempVal);
    lcd_string_fill_and_display_celc_degree(0, 0, tempTxt, SIZEOF_ARRAY(tempTxt));

    switch(PWR_PVDLevelGet()) {
        case SEN_VOLT_2V9: strncpy(voltTxt, "Vlvl: >2.9 [OK]",  SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V8: strncpy(voltTxt, "Vlvl: ~2.8 [OK]",  SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V7: strncpy(voltTxt, "Vlvl: ~2.7 [Med]", SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V6: strncpy(voltTxt, "Vlvl: ~2.6 [Med]", SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V5: strncpy(voltTxt, "Vlvl: ~2.5 [Low]", SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V4: strncpy(voltTxt, "Vlvl: ~2.4 [Low]", SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V3: strncpy(voltTxt, "Vlvl: ~2.3 [CRI]", SIZEOF_ARRAY(voltTxt)); break;
        case SEN_VOLT_2V2: strncpy(voltTxt, "Vlvl: <2.2 [CRI]", SIZEOF_ARRAY(voltTxt)); break;
        default: strncpy(voltTxt, "UNKNOWN!", SIZEOF_ARRAY(voltTxt)); break;
    }
    lcd_string_fill_and_display(1, 0, voltTxt, SIZEOF_ARRAY(voltTxt));
}

static void supp_jump_bootloader(void)
{
    // system memory address with UART1 bootloader for STM32F10x controller
    volatile const uint32_t sysmem_addr = 0x1FFFF000, bootld_code_offset = 4;
    void (*const forced_bootloader_jump)(void) = (void(*const)(void)) \
                            *(uint32_t*)(sysmem_addr + bootld_code_offset);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
    RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __disable_irq();
    __set_MSP(*(uint32_t*)sysmem_addr);
    forced_bootloader_jump();
}

static void lcd_submenu_options(uint8_t* __restrict sel, const submenu_t* submenu)
{
    char* exit_menu_name = NULL;

    CALL_SUBMENU_FUNC(submenu->func_options, sel);
    exit_menu_name = submenu->exit_item != *sel ? \
        submenu->exit_ref->normname:submenu->exit_ref->selname;
    ZtScI2cMxDisplay8x16Str(3, 1, exit_menu_name);
}

static void lcd_submenu(const submenu_t* submenu)
{
    if(NULL != submenu) {
        uint8_t sel = submenu->first_elem, state = 0xFF;

        lcd_submenu_scroll_title(submenu->scrollname);
        lcd_submenu_options(&sel, submenu);
        while(1) {
            if(true == is_lcd_menu_key_pressed()) {
                sel = submenu->last_elem > sel+1 ? sel+1:submenu->first_elem;
                lcd_submenu_options(&sel, submenu);
            }
            if(true == is_lcd_select_key_pressed()) {
                if(submenu->exit_item == sel) break;
                else CALL_SUBMENU_FUNC(submenu->func_confirm, &sel);
                lcd_submenu_options(&sel, submenu);
            }
            CALL_SUBMENU_FUNC(submenu->func_status, &state);
            vTaskDelay(pdMS_TO_TICKS(LCD_MENU_INTERVAL_MS));
        }
        ZtScI2cMxClearScreen();
    }
}

static void lcd_mainmenu_options(uint8_t* __restrict sel, const mainmenu_t* menu)
{
    static const uint8_t lcd_line_max = 4;
    uint8_t line = 0, elem_first = lcd_line_max >= *sel ? 0:*sel-lcd_line_max;

    for(uint8_t elem = elem_first + LCD_MAIN_NONE+1; \
            line < lcd_line_max && elem < menu->last_elem; elem++)
    {
        char* menu_txt = *sel == elem ? \
            menu->submenus[elem]->selname : menu->submenus[elem]->normname;
        ZtScI2cMxDisplay8x16Str(line++, 1, menu_txt);
    }
}

static void lcd_mainmenu(const mainmenu_t* mainmenu)
{
    uint8_t sel = mainmenu->first_elem;

    lcd_mainmenu_scroll_title(mainmenu->scrollname);
    lcd_mainmenu_options(&sel, mainmenu);
    while(1) {
        if(true == is_lcd_menu_key_pressed()) {
            sel = mainmenu->last_elem > sel+1 ? sel+1:mainmenu->first_elem;
            lcd_mainmenu_options(&sel, mainmenu);
        }
        if(true == is_lcd_select_key_pressed()) {
            if(mainmenu->exit_item == sel) {
                lcd_mainmenu_scroll_title(mainmenu->submenus[sel]->scrollname);
                break;
            }
            else lcd_submenu(mainmenu->submenus[sel]);
            lcd_mainmenu_options(&sel, mainmenu);
        }
        vTaskDelay(pdMS_TO_TICKS(LCD_MENU_INTERVAL_MS));
    }
    ZtScI2cMxClearScreen();
}

static void lcd_display_welcome_msg(void)
{
    uint16_t welcome_cnt = 6, welcome_delay_ms = 1000;

    ZtScI2cMxDisplay8x16Str(1, 0, ">AMUv2 prototype");
    while(--welcome_cnt) {
        char* welcome_txt = NULL;
        if(welcome_cnt%2) welcome_txt = "Welcome :)";
        else welcome_txt = "        ;)";
        ZtScI2cMxDisplay8x16Str(2, 3, welcome_txt);
        vTaskDelay(pdMS_TO_TICKS(welcome_delay_ms));
    }
    ZtScI2cMxClearScreen();
}

/*******************************************************************************
* Function Name  : vLcdInterfaceTask
* Description    : Lcd interface main routine
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void vLcdTask(void * pvArg)
{
    lcdRobotScreen_t robot_screen_t = ROBOT_HAND_DOWN;
    xTimerHandle xLcdScrSaverTimer = \
        xTimerCreate((char *)"Lcd SS timer", pdMS_TO_TICKS(1000), pdFALSE, (void *)4, vLcdScrSaverCallback);


    vLCD_Configuration();
    xTimerStart(xLcdScrSaverTimer, 0);

    lcd_display_welcome_msg();
    while(1) {
        if(true == is_lcd_menu_key_pressed()) {
            xTimerStop(xLcdScrSaverTimer, 0);
            lcdShowScrSaver = true;
            lcd_mainmenu(&mainmenu);
        }

        if(true == lcdShowScrSaver) {
            const uint8_t* screen = ROBOT_HAND_DOWN == robot_screen_t ? \
                                    robot_hand_down:robot_hand_up;

            robot_screen_t = ROBOT_HAND_DOWN == robot_screen_t ? \
                                ROBOT_HAND_UP:ROBOT_HAND_DOWN;
            ZtScI2cMxDisplayArea(0, 8, 0, 64, screen);

            switch(get_battery_status()) {
                case SEN_BAT_FULL: screen = battery_full; break;
                case SEN_BAT_HALF: screen = battery_half; break;
                case SEN_BAT_LOW:
                default: screen = battery_low; break;
            }
            ZtScI2cMxDisplayArea(0, 2, 104, 128, screen);

            lcdShowScrSaver = false;
            xTimerStart(xLcdScrSaverTimer, 0);
        }
        lcd_mov_display_screen(2, drive_get_direction());
        vTaskDelay(pdMS_TO_TICKS(LCD_TASK_INTERVAL_MS));
    }
}

static void vLCD_Configuration(void)
{
    // Menu selection press buttons
    GPIO_InitTypeDef GPIO_InitStructure = \
        { .GPIO_Pin   = GPIO_Pin_2|GPIO_Pin_8, .GPIO_Mode  = GPIO_Mode_IPD };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // LCD initialization is be done once I2C is ready, applies more when PWROFF
    TickType_t lcd_startup_delay = \
            RESET == RCC_GetFlagStatus(RCC_FLAG_SFTRST) ? 500:10;

    vTaskDelay(pdMS_TO_TICKS(lcd_startup_delay));
    ZtScI2cMxReset();
    vTaskDelay(pdMS_TO_TICKS(10));
    ZtScI2cMxSetBrightness(0xFF);
    ZtScI2cMxSetVcomH(7);

    char ZtScI2cMxVersion[16+1] = {0};
    ZtScI2cMxReadVersion(ZtScI2cMxVersion);
    ZtScI2cMxDisplay8x16Str(0, 0, "OLED INITIALIZED");
    ZtScI2cMxDisplay8x16Str(2, 0, "   FW VERSION   ");
    ZtScI2cMxDisplay8x16Str(3, 0, ZtScI2cMxVersion);

    vTaskDelay(pdMS_TO_TICKS(2000));
    ZtScI2cMxClearScreen();

}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
