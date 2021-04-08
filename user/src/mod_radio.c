#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "mod_radio.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mod_auxiliary.h"

#define RADIO_USART                     USART1
#define RADIO_FRAME_GUARD_FIRST         0x66
#define RADIO_FRAME_GUARD_SECOND        0x4D
#define RADIO_FRAME_SIZE                25U
#define RADIO_PAYLOAD_SIZE              (RADIO_FRAME_SIZE - 1)
#define RADIO_BYTES_FOR_CHKSUM          (RADIO_PAYLOAD_SIZE - 1)

#define RADIO_UART_ADDRESS              0x0A
#define RADIO_UART_REMOTE_ADDRESS       (0x80|RADIO_UART_ADDRESS)
#define RADIO_UART_CONTROL_ADDRESS      (0x80|0x05)
#define RADIO_UART_END_SIGNATURE        (0x80|0x7F)

#define RADIO_SETT_RAW_SIZE             240

#define RADIO_FRAME_SEND_INIT \
    .start_byte = RADIO_UART_CONTROL_ADDRESS, .end_byte = RADIO_UART_END_SIGNATURE, \
    .guard_first = RADIO_FRAME_GUARD_FIRST, .guard_second = RADIO_FRAME_GUARD_SECOND,

#define ADD_VAL_UNIT(ret, val, unit) \
    if(SUCCESS == ret) { strcat(val, unit); }

#define ADD_NUM_VAL(ret, val, num) \
    if(SUCCESS == ret) { char tmp[] = "000"; strcat(val, "("); \
        snprintf(tmp, sizeof(tmp), "%d", num); strcat(val, tmp); \
        strcat(val, ")"); }

typedef enum {
    FIRSTCMD = 0,
    CMDMODE = FIRSTCMD,
    OPMODE,
    LASTCMD
} radio_cmds_t;

typedef enum {
    FIRSTCFG = 0,
    UARTBAUD = FIRSTCFG,
    DATARATE,
    RADIOFREQ,
    DATAPAYLOAD,
    MODULATION,
    TXRXLED,
    LASTCFG
} radio_cfgs_t;

typedef struct {
    char* cmd;
    char* info;
    char value[32];
} radio_setting_t;

typedef ErrorStatus (*radio_init)(radio_setting_t* const, uint8_t*);

void vRadio_configuration(void);
static ErrorStatus radio_frame_verify(const radio_frame_t* __restrict);
static uint8_t radio_data_get_checksum(const uint8_t* __restrict, uint16_t);
static void radio_send_data(const uint8_t* const __restrict, uint16_t);
static ErrorStatus radio_enter_command_mode(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_enter_operating_mode(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_get_uart_baudrate(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_get_rfdata_rate(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_get_freq(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_get_modulation(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_get_payload(radio_setting_t* const, uint8_t*);
static ErrorStatus radio_get_txrx_led(radio_setting_t* const, uint8_t*);

static const radio_init radio_init_cmds[LASTCMD] =
{
    [CMDMODE]     = radio_enter_command_mode,
    [OPMODE]      = radio_enter_operating_mode
};
static const radio_init radio_init_cfgs[LASTCFG] =
{
    [UARTBAUD]    = radio_get_uart_baudrate,
    [DATARATE]    = radio_get_rfdata_rate,
    [RADIOFREQ]   = radio_get_freq,
    [DATAPAYLOAD] = radio_get_payload,
    [TXRXLED]     = radio_get_txrx_led,
    [MODULATION]  = radio_get_modulation
};

typedef struct {
    radio_setting_t cmds[LASTCMD];
    radio_setting_t cfgs[LASTCFG];
} radio_settings_t;

static radio_settings_t radio_settings = {0};


static inline void radio_reset_data(char* raw)
{
    memset(raw, '\0', RADIO_SETT_RAW_SIZE);
}

static ErrorStatus radio_send_command(radio_setting_t* sett, uint8_t* raw)
{
    ErrorStatus ret = ERROR;
    const char* respok = "OK";
    char* const rxd = (char*)raw;

    radio_reset_data(rxd);
    radio_send_data((uint8_t*)sett->cmd, strlen(sett->cmd));
    vTaskDelay(pdMS_TO_TICKS(10));

    char* nlptr = strchr((char*)rxd, '\n');
    if(NULL != nlptr) {
        *nlptr = '\0';
        if(0 == strncmp(respok, rxd, sizeof(respok))) ret = SUCCESS;
        strncpy(sett->value, rxd, SIZEOF_ARRAY(sett->value));
    }
    else strncpy(sett->value, "FAILURE", SIZEOF_ARRAY(sett->value));
    return ret;
}

static ErrorStatus radio_get_config(radio_setting_t* sett, uint8_t* raw)
{
    ErrorStatus ret = ERROR;
    char* const rxd = (char*)raw;

    radio_reset_data(rxd);
    radio_send_data((uint8_t*)sett->cmd, strlen(sett->cmd));
    vTaskDelay(pdMS_TO_TICKS(10));

    char* nlptr = strchr((char*)rxd, '\n');
    if(NULL != nlptr) {
        const char* value = strchr((char*)rxd, '=');
        *nlptr = '\0';
        if(NULL != value) {
            value += 1;
            strncpy(sett->value, value, SIZEOF_ARRAY(sett->value));
            ret = SUCCESS;
        }
        else strncpy(sett->value, rxd, SIZEOF_ARRAY(sett->value));
    }
    else printf("> Cannot fetch config value\n");
    return ret;
}

static ErrorStatus radio_enter_command_mode(radio_setting_t* const settings, uint8_t* raw)
{
    settings->cmd = "+++";
    settings->info = "Go CMD";
    return radio_send_command(settings, raw);
}

static ErrorStatus radio_enter_operating_mode(radio_setting_t* const settings, uint8_t* raw)
{
    settings->cmd = "ATO\r";
    settings->info = "Go OPER";
    return radio_send_command(settings, raw);
}

static ErrorStatus radio_get_uart_baudrate(radio_setting_t* const settings, uint8_t* raw)
{
    static const char* unit = "bps";
    ErrorStatus ret = ERROR;
    settings->cmd = "ATS00?\r";
    settings->info = "UART";
    ret = radio_get_config(settings, raw);
    ADD_VAL_UNIT(ret, settings->value, unit);
    return ret;
}

static ErrorStatus radio_get_rfdata_rate(radio_setting_t* const settings, uint8_t* raw)
{
    static const char* unit = "bps";
    ErrorStatus ret = ERROR;
    settings->cmd = "ATS02?\r";
    settings->info = "Radio";
    ret = radio_get_config(settings, raw);
    ADD_VAL_UNIT(ret, settings->value, unit);
    return ret;
}

static ErrorStatus radio_get_freq(radio_setting_t* const settings, uint8_t* raw)
{
    static const char* unit = "MHz";
    ErrorStatus ret = ERROR;
    settings->cmd = "ATS01?\r";
    settings->info = "Freq";
    ret = radio_get_config(settings, raw);
    if(SUCCESS == ret) {
        double numvalue = atol(settings->value)/1000000.0f;
        snprintf(settings->value, sizeof(settings->value), "%.3f", numvalue);
        ADD_VAL_UNIT(ret, settings->value, unit);
    }
    return ret;
}

static ErrorStatus radio_get_modulation(radio_setting_t* const settings, uint8_t* raw)
{
    ErrorStatus ret = ERROR;
    settings->cmd = "ATS03?\r";
    settings->info = "Modul";
    ret = radio_get_config(settings, raw);
    if(SUCCESS == ret) {
        uint8_t numvalue = atoi(settings->value);
        const char* value = NULL;

        switch(numvalue) {
            case 0: value = "2-FSK";  break;
            case 1: value = "GFSK05"; break;
            case 2: value = "GFSK1";  break;
            case 3: value = "GMSK";   break;
            case 4: value = "OOK";    break;
            case 5: value = "ASK";    break;
            default: value = "Unknown"; break;
        }
        strncpy(settings->value, value, sizeof(settings->value));
        ADD_NUM_VAL(ret, settings->value, numvalue);
    }
    return ret;
}

static ErrorStatus radio_get_payload(radio_setting_t* const settings, uint8_t* raw)
{
    static const char* unit = "bytes";
    ErrorStatus ret = ERROR;
    settings->cmd = "ATS28?\r";
    settings->info = "Payld";
    ret = radio_get_config(settings, raw);
    ADD_VAL_UNIT(ret, settings->value, unit);
    return ret;
}

static ErrorStatus radio_get_txrx_led(radio_setting_t* const settings, uint8_t* raw)
{
    ErrorStatus ret = ERROR;
    settings->cmd = "ATS24?\r";
    settings->info = "Led";
    ret = radio_get_config(settings, raw);
    if(SUCCESS == ret) {
        uint8_t numvalue = atoi(settings->value);
        const char* value = NULL;

        switch(numvalue) {
            case 0: value = "Disabled";  break;
            case 1: value = "AcL/OpDr"; break;
            case 2: value = "Push/pul";  break;
            default: value = "Unknown"; break;
        }
        strncpy(settings->value, value, sizeof(settings->value));
        ADD_NUM_VAL(ret, settings->value, numvalue);
    }
    return ret;
}

void print_results(radio_settings_t* settings)
{
    for(radio_cmds_t i = FIRSTCMD; i < LASTCMD; i++)
        printf("%s: %s\n", settings->cmds[i].info, settings->cmds[i].value);
    for(radio_cfgs_t i = FIRSTCFG; i < LASTCFG; i++)
        printf("%s: %s\n", settings->cfgs[i].info, settings->cfgs[i].value);
}


void vRadio_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | \
                                            RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate   = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(RADIO_USART, &USART_InitStructure);

    USART_SetAddress(RADIO_USART, RADIO_UART_REMOTE_ADDRESS);

    USART_ITConfig(RADIO_USART, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(RADIO_USART,USART_FLAG_RXNE);
//    USART_ClearFlag(RADIO_USART,USART_FLAG_TC);
    USART_Cmd(RADIO_USART, ENABLE);
}

static void radio_send_data(const uint8_t* const __restrict txd, uint16_t size)
{
    for(uint16_t pos = 0; pos < size; pos++) {
        while(RESET == USART_GetFlagStatus(RADIO_USART, USART_FLAG_TXE));
        USART_SendData(RADIO_USART, txd[pos]);
    }
}

static void radio_recv_handler(uint16_t uart_data, radio_frame_t* frame, uint8_t* config)
{
    static uint8_t pos = 0,* rxd_data = NULL,* rxd_config = NULL;

    if(NULL != rxd_data) {
        if(RADIO_UART_REMOTE_ADDRESS == uart_data || RADIO_FRAME_SIZE <= pos) pos = 0;
        rxd_data[pos++] = (uint8_t)uart_data;
        if(RADIO_PAYLOAD_SIZE == pos)
            xTaskNotifyFromISR(xTaskGetHandle("Radio"), (uint32_t){0}, eNoAction, NULL);
    }
    else if(NULL != frame) rxd_data = (uint8_t*)frame;
    else
        if(NULL != rxd_config)
            if('\r' == uart_data || RADIO_SETT_RAW_SIZE <= pos) pos = 0;
            else rxd_config[pos++] = uart_data;
        else if(NULL != config) rxd_config = config;

}

void radio_recv_handler_irq(uint16_t uart_data)
{
    radio_recv_handler(uart_data, NULL, NULL);
}

static void radio_recv_handler_init(radio_frame_t* frame)
{
    radio_recv_handler(0, frame, NULL);
}

static void radio_recv_handler_config_init(uint8_t* config)
{
    radio_recv_handler(0, NULL, config);
}

void radio_send_status_frame(radio_frame_t* radio_frame)
{
    radio_get_status_curr(radio_frame);
    radio_frame->chksum = radio_data_get_checksum( \
        (uint8_t*)radio_frame, RADIO_BYTES_FOR_CHKSUM);
    radio_send_data((uint8_t*)radio_frame, sizeof(*radio_frame));
}

void vRadioTask(void *pvArg)
{
    radio_frame_t frame_recv = {0}, frame_send = { RADIO_FRAME_SEND_INIT };
    uint8_t config_raw[RADIO_SETT_RAW_SIZE] = {0};

    vRadio_configuration();
    vTaskDelay(pdMS_TO_TICKS(1000));

    radio_recv_handler_config_init(config_raw);
    if(SUCCESS == radio_init_cmds[CMDMODE](&radio_settings.cmds[CMDMODE], config_raw)) {
//        vTaskDelay(pdMS_TO_TICKS(100));
        for(radio_cfgs_t op = FIRSTCFG; op < LASTCFG; op++) {
            radio_init_cfgs[op](&radio_settings.cfgs[op], config_raw);
        }
        radio_init_cmds[OPMODE](&radio_settings.cmds[OPMODE], config_raw);
        printf("=== %s radio operate ===\n", __func__);
        print_results(&radio_settings);
        printf("Radio size: %d\n", sizeof(radio_frame_t));
    }
    else printf("=== %s radio failure ===\n", __func__);


    USART_WakeUpConfig(RADIO_USART, USART_WakeUp_AddressMark);
    radio_recv_handler_init(&frame_recv);
    while(1) {
        do {
            xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
        } while(SUCCESS != radio_frame_verify(&frame_recv));
        radio_send_status_frame(&frame_send);
        radio_set_status_curr(&frame_recv);
    }
}

static uint8_t radio_data_get_checksum(const uint8_t* __restrict data, uint16_t size)
{
    uint32_t checksum = 0;

    for(uint32_t i = 0; i < size; i++) checksum += data[i];
    checksum = (((checksum^0xFF) + 1) & 0x7F);
    return (uint8_t)checksum;
}

static ErrorStatus radio_frame_verify(const radio_frame_t* __restrict frame)
{
    ErrorStatus fresult = ERROR;

    if(RADIO_FRAME_GUARD_FIRST == frame->guard_first &&
        RADIO_FRAME_GUARD_SECOND == frame->guard_second)
    {
        uint8_t checksum = radio_data_get_checksum((uint8_t*)frame, RADIO_BYTES_FOR_CHKSUM);
        fresult = frame->chksum == checksum ? SUCCESS:ERROR;
    }

    return fresult;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
