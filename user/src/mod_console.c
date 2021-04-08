/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_console.c
** Descriptions:            Interface for console operations
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
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usb_lib.h"
#include "hw_config.h"

#include "mod_drive.h"
#include "mod_sound.h"
#include "mod_lighting.h"
#include "mod_auxiliary.h"
#include "mod_console.h"

#define CONSOLE_NL                      "\n\r"
#define CONSOLE_MENU_SEPARATOR          "+--------------------------+"
#define CONSOLE_MENU_SEPARATOR_NL       CONSOLE_MENU_SEPARATOR CONSOLE_NL
#define CONSOLE_MENU_SEPARATOR_NLNL     CONSOLE_NL CONSOLE_MENU_SEPARATOR CONSOLE_NL
#define CONSOLE_MENU_HEADER_TXT_SIZE    (strlen(CONSOLE_MENU_SEPARATOR) - 2)
#define CONSOLE_MENU_HEADER             CONSOLE_MENU_SEPARATOR_NLNL         \
                                        "|%*s%s%*s|"                        \
                                        CONSOLE_MENU_SEPARATOR_NLNL
#define CONSOLE_MENU_FOOTER             CONSOLE_MENU_SEPARATOR_NL           \
                                        " Selection/> "
#define CONSOLE_MENU_LINE               "| %2d. %-20s |" CONSOLE_NL
#define CONSOLE_MENU_LINE_SEL           "| > %2d. %-18s |" CONSOLE_NL
#define CONSOLE_MENU_UNAVAIL            \
    CONSOLE_NL "sorry, menu '%c' not available here"CONSOLE_NL

#define CONSOLE_ENTER_CHAR_CODE     0x0A
#define CONSOLE_ESCAPE_CHAR_CODE    0x1B
#define CONSOLE_SHOW_MENU_SYM       CONSOLE_ESCAPE_CHAR_CODE
#define CONSOLE_IS_SHOW_MENU(ch)    (CONSOLE_SHOW_MENU_SYM == ch)
#define CONSOLE_IS_ENTER_MENU(ch)   (CONSOLE_ENTER_CHAR_CODE == ch)
#define CONSOLE_REF_FROM_ELEM(num)  ('0' + num)
#define CONSOLE_ELEM_FROM_CHAR(ch)  (ch >= '0' ? ch - '0':0xFF)
#define CONSOLE_IS_ARROW_SEQ        (3 == USB_Rx_Counter && 0x5B == getchar())
#define CONSOLE_IS_ARROW_UP(ch)     (0x41 == ch)
#define CONSOLE_IS_ARROW_DOWN(ch)   (0x42 == ch)
#define CONS_ELEM_FIRST(menu)       CONS_##menu##_FIRST
#define CONS_ELEM_LAST(menu)        CONS_##menu##_LAST

#define CONSOLE_IS_ELEM_IN_MENU(menu, elem) \
    (menu->first_elem < elem && menu->last_elem > elem)
#define PREDEFINE_MAINMENU_STRUCT(idname)                      \
    .first_elem = CONS_ELEM_FIRST(idname), .last_elem = CONS_ELEM_LAST(idname)
#define PREDEFINE_SUBMENU_STRUCT(idname)                       \
    PREDEFINE_MAINMENU_STRUCT(idname), .storage_var = (-1)

#define console_print_menu(menu, pos) _Generic((*menu),                 \
                            mainmenu_t:console_print_main_menu,         \
                            submenu_t:console_print_sub_menu)(menu, pos)


typedef void(*cons_menu_func)(const char*);
typedef void(*cons_submenu_func)(uint8_t, int32_t* __restrict);
typedef enum
{
    CONS_MAIN_FIRST = 0,
    CONS_MAIN_NAME = CONS_MAIN_FIRST,
    CONS_MAIN_MOVEMENT,
    CONS_MAIN_LIGHT,
    CONS_MAIN_SOUND,
    CONS_MAIN_LAST,
    CONS_MAIN_ORIENT,
    CONS_MAIN_ENVIRO,
    CONS_MAIN_STATUS,
} mainMenuElems_t;

typedef enum {
    CONS_MOV_FIRST = 0,
    CONS_MOV_NAME = CONS_MOV_FIRST,
    CONS_MOV_FWD,
    CONS_MOV_BKW,
    CONS_MOV_LFT,
    CONS_MOV_RGT,
    CONS_MOV_SPEED,
    CONS_MOV_EXIT,
    CONS_MOV_LAST
} movMenuElems_t;

typedef enum {
    CONS_LIG_FIRST = 0,
    CONS_LIG_NAME = CONS_LIG_FIRST,
    CONS_LIG_LFT,
    CONS_LIG_RGT,
    CONS_LIG_INN,
    CONS_LIG_OUT,
    CONS_LIG_MODE,
    CONS_LIG_EXIT,
    CONS_LIG_LAST
} ligMenuElems_t;

typedef enum {
    CONS_SIG_FIRST = 0,
    CONS_SIG_NAME = CONS_SIG_FIRST,
    CONS_SIG_PRESS,
    CONS_SIG_TOGGLE,
    CONS_SIG_EXIT,
    CONS_SIG_LAST
} sigMenuElems_t;

typedef struct {
    char ref;
    const char* name;
    cons_submenu_func func;
} submenu_elem_t;

typedef struct {
    uint8_t first_elem;
    uint8_t last_elem;
    int32_t storage_var;
    submenu_elem_t* elems;
} submenu_t;

typedef struct {
    char ref;
    const char* name;
    submenu_t* submenu;
} mainmenu_elem_t;

typedef struct {
    uint8_t first_elem;
    uint8_t last_elem;
    mainmenu_elem_t* elems;
} mainmenu_t;

static void vMainMenu_Console(void);
static void vSubmenu_Console(const char*, submenu_t*);
static void vDrive_setup(uint8_t, int32_t* __restrict);
static void vDrive_speed(uint8_t, int32_t* __restrict);
static void vLight_mode(uint8_t, int32_t* __restrict);
static void vLight_setup(uint8_t, int32_t* __restrict);
static void vSound_setup(uint8_t, int32_t* __restrict);
static void vSound_mode(uint8_t, int32_t* __restrict);
static void console_clear_screen(void);

static submenu_t movmenu = {
    PREDEFINE_SUBMENU_STRUCT(MOV),
    .storage_var = DRIVE_SPEED_MAX/2,
    .elems = (submenu_elem_t[]){
        { CONS_MOV_NAME,  NULL,            NULL         },
        { CONS_MOV_FWD,   "go forward",    vDrive_setup },
        { CONS_MOV_BKW,   "go backward",   vDrive_setup },
        { CONS_MOV_LFT,   "go left",       vDrive_setup },
        { CONS_MOV_RGT,   "go right",      vDrive_setup },
        { CONS_MOV_SPEED, "set speed",     vDrive_speed },
        { CONS_MOV_EXIT,  "to exit",       NULL         },
    }
};

static submenu_t ligmenu = {
    PREDEFINE_SUBMENU_STRUCT(LIG),
    .elems = (submenu_elem_t[]){
        { CONS_LIG_NAME,  NULL,               NULL         },
        { CONS_LIG_LFT,   "toggle left",      vLight_setup },
        { CONS_LIG_RGT,   "toggle right",     vLight_setup },
        { CONS_LIG_INN,   "toggle inner",     vLight_setup },
        { CONS_LIG_OUT,   "toggle outer",     vLight_setup },
        { CONS_LIG_MODE,  "toggle auto/manu", vLight_mode  },
        { CONS_LIG_EXIT,  "to exit",          NULL         },
    }
};

static submenu_t sigmenu = {
    PREDEFINE_SUBMENU_STRUCT(SIG),
    .elems = (submenu_elem_t[]){
        { CONS_SIG_NAME,    NULL,             NULL         },
        { CONS_SIG_PRESS,   "press to sound", vSound_setup },
        { CONS_SIG_TOGGLE,  "toggle on/off",  vSound_mode  },
        { CONS_SIG_EXIT,    "to exit",        NULL         },
    }
};

static const mainmenu_t mainmenu = {
     PREDEFINE_MAINMENU_STRUCT(MAIN),
    .elems = (mainmenu_elem_t[]){
        { CONS_MAIN_NAME,     "MAIN MENU",        NULL      },
        { CONS_MAIN_MOVEMENT, "movement control", &movmenu  },
        { CONS_MAIN_LIGHT,    "light control",    &ligmenu  },
        { CONS_MAIN_SOUND,    "sound control",    &sigmenu  },
    }
};

static void console_print_menu_header(const char* __restrict name)
{
    size_t name_len = strlen(name),
        space_len = CONSOLE_MENU_HEADER_TXT_SIZE - name_len,
        front_len = space_len/2, back_len = space_len%2?front_len+1:front_len;
    console_clear_screen();
    printf(CONSOLE_MENU_HEADER, front_len, "", name, back_len, "");
}

static void console_print_main_menu(const mainmenu_t* menu, uint8_t pos)
{
    mainmenu_elem_t* elems = menu->elems;
    console_print_menu_header(elems->name);
    for(uint8_t idx = 1; idx < menu->last_elem; idx++)
        if(pos != idx) printf(CONSOLE_MENU_LINE, elems[idx].ref, elems[idx].name);
        else printf(CONSOLE_MENU_LINE_SEL, elems[idx].ref, elems[idx].name);
    printf(CONSOLE_MENU_FOOTER);
}

static void console_print_sub_menu(const submenu_t* submenu, uint8_t pos)
{
    submenu_elem_t* elems = submenu->elems;
    console_print_menu_header(elems->name);
    for(uint8_t idx = 1; idx < submenu->last_elem; idx++) {
        if(NULL == elems[idx].func) printf(CONSOLE_MENU_SEPARATOR_NL);
        if(pos != idx) printf(CONSOLE_MENU_LINE, elems[idx].ref, elems[idx].name);
        else printf(CONSOLE_MENU_LINE_SEL, elems[idx].ref, elems[idx].name);
    }
    printf(CONSOLE_MENU_FOOTER);
}

/*******************************************************************************
* Function Name  : vConsoleInterfaceTask
* Description    : Console operations routine
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vConsoleTask(void * pvArg)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    vMainMenu_Console();
}

static void vMainMenu_Console(void)
{
    char selection = 0;
    const mainmenu_t* menu = &mainmenu;
    mainMenuElems_t menupos = CONS_MAIN_FIRST;

    while(1) {
        fflush(stdin);
        selection = getchar();
        if(true == CONSOLE_IS_ENTER_MENU(selection)) \
            selection = CONSOLE_REF_FROM_ELEM(menupos);
        if(true == CONSOLE_IS_SHOW_MENU(selection)) {
            if(true == CONSOLE_IS_ARROW_SEQ) {
                const uint8_t arrow = getchar();
                if(true == CONSOLE_IS_ARROW_UP(arrow))
                    menupos = menupos-1 < menu->first_elem ? \
                                        menu->first_elem:menupos-1;
                if(true == CONSOLE_IS_ARROW_DOWN(arrow))
                    menupos = menupos+1 >= menu->last_elem ? menupos:menupos+1;
            }
            console_print_menu(menu, menupos);
        }
        else {
            const uint8_t idx = CONSOLE_ELEM_FROM_CHAR(selection);
            if(true == CONSOLE_IS_ELEM_IN_MENU(menu, idx)) \
                vSubmenu_Console(menu->elems[idx].name, menu->elems[idx].submenu);
            else printf(CONSOLE_MENU_UNAVAIL, selection);
            console_print_menu(menu, menupos);
        }
    }
}

static void vSubmenu_Console(const char* name, submenu_t* submenu)
{
    char selection = 0;
    uint8_t menupos = submenu->first_elem;

    submenu->elems->name = name;
    console_print_menu(submenu, menupos);
    while(1) {
        fflush(stdin);
        selection = getchar();
        if(true == CONSOLE_IS_ENTER_MENU(selection)) \
            selection = CONSOLE_REF_FROM_ELEM(menupos);
        if(true == CONSOLE_IS_SHOW_MENU(selection)) {
            if(true == CONSOLE_IS_ARROW_SEQ) {
                const uint8_t arrow = getchar();
                if(true == CONSOLE_IS_ARROW_UP(arrow))
                    menupos = menupos-1 < submenu->first_elem ? \
                                        submenu->first_elem:menupos-1;
                if(true == CONSOLE_IS_ARROW_DOWN(arrow))
                    menupos = menupos+1 >= submenu->last_elem ? menupos:menupos+1;
            }
            console_print_menu(submenu, menupos);
        }
        else {
            const uint8_t idx = CONSOLE_ELEM_FROM_CHAR(selection);
            if(true == CONSOLE_IS_ELEM_IN_MENU(submenu, idx))
                if(NULL != submenu->elems[idx].func) \
                    submenu->elems[idx].func(idx, &submenu->storage_var);
                else break;
            else printf(CONSOLE_MENU_UNAVAIL, selection);
        }
    }
}

static void vDrive_setup(uint8_t sel, int32_t* __restrict speed)
{
    movement_settings_t status = {0};
    status.direction = sel;
    status.speed_lft = status.speed_rgt = (uint8_t)(*speed);
    drive_set_status(&status);
}

static void vDrive_speed(uint8_t dir, int32_t* __restrict speed)
{
    const uint32_t speed_prev = *speed;
    printf(CONSOLE_NL"Current speed: %lu%%, set value or ESC"CONSOLE_NL, *speed);
    scanf("%lu", speed);
    if(speed_prev != *speed) {
        *speed = *speed <= DRIVE_SPEED_MAX ? *speed:DRIVE_SPEED_MAX;
        printf("Speed updated to: %lu%%"CONSOLE_NL, *speed);
    }
    else printf("No update, speed is still: %lu%%"CONSOLE_NL, *speed);
}

static void vLight_mode(uint8_t sel, int32_t* __restrict unused)
{
    const bool auto_mode = !light_is_auto();
    light_set_auto(auto_mode);
    if(false == auto_mode) light_off_all();
    printf(CONSOLE_NL"Mode switched to: %s", true == auto_mode?"auto":"manual");
}

static void vLight_setup(uint8_t sel, int32_t* __restrict unused)
{
    const bool auto_mode = light_is_auto();
    if(false == auto_mode) light_toggle_led(auxil_get_bit_from_pos(sel));
    else printf(CONSOLE_NL"Blocked in auto light mode");
}

static void vSound_setup(uint8_t sel, int32_t* __restrict unused)
{
    uint8_t state = SOUND_RF_PRESS;
    if(SOUND_RF_TOGGLE == sel) state = true == signal_is_on() ? \
                                    SOUND_RF_NONE:SOUND_RF_TOGGLE;
    signal_set_state(state);
}

static void vSound_mode(uint8_t sel, int32_t* __restrict unused)
{
    vSound_setup(sel, unused);
    vTaskDelay(pdMS_TO_TICKS(100));
    printf(CONSOLE_NL"Mode switched to: %s", \
        false == signal_is_on() ? "silent":"sound");
}

void vUSB_configuration(void)
{
    Set_System();
    Set_USBClock();
    //USB NVIC settings setup in main file
    //USB_Interrupts_Config();
    USB_Init();
}

static void console_clear_screen(void)
{
    static const char *clear_screen_seq = "\e[1;1H\e[2J";
    write(fileno(stdout), clear_screen_seq, strlen(clear_screen_seq));
}


// retargeting std io data streams to go via USB CDC
int __io_write(int file, char *data, int len)
{
    int i = 0;

    for(i=0; i < len; i++, USB_Tx_ptr_in++) {
        USB_Tx_Buffer[USB_Tx_ptr_in] = (uint8_t)data[i];
        if(USB_TX_DATA_SIZE <= USB_Tx_ptr_in) USB_Tx_ptr_in = 0;
    }
    return i;
}

int __io_read(int file, char *data, int len)
{
    int i = 0;

    for(i=0; i < len; i++, USB_Rx_ptr_out++) {
        if(USB_Rx_ptr_out >= USB_Rx_Counter) USB_Rx_Counter = 0;
        while(USB_Rx_Counter == 0) vTaskDelay(pdMS_TO_TICKS(10));
        data[i] = USB_Rx_Buffer[USB_Rx_ptr_out];
    }
    if(USB_Rx_ptr_out >= USB_Rx_Counter) USB_Rx_Counter = 0;
    return i;
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
