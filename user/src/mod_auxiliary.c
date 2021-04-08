#include "stm32f10x_conf.h"
#include "mod_lighting.h"
#include "mod_auxiliary.h"
#include <string.h>

SemaphoreHandle_t sys_data_mutex = NULL;

uint8_t auxil_get_bit_from_pos(uint8_t pos)
{
    return pos > 8 ? 0:(uint16_t)(1<<pos)>>1;
}

ErrorStatus manage_state_info(system_state_t* data, state_data_op_t op)
{
    static system_state_t state_info = {0};
    BaseType_t ret = pdFAIL;
    ErrorStatus fresult = ERROR;

    if(NULL != data) {
        switch(op) {
            case DATA_GET_ONLY:
                if(pdPASS == xSemaphoreTake(sys_data_mutex, portMAX_DELAY)) {
                    memcpy(data, &state_info, sizeof(system_state_t));
                    ret = xSemaphoreGive(sys_data_mutex);
                    fresult = pdPASS == ret ? SUCCESS:ERROR;
                }
                break;
            case DATA_LOCK_AND_GET:
                if(pdPASS == xSemaphoreTake(sys_data_mutex, portMAX_DELAY)) {
                    memcpy(data, &state_info, sizeof(system_state_t));
                    fresult = SUCCESS;
                }
                break;
            case DATA_SET_AND_UNLOCK:
                memcpy(&state_info, data, sizeof(system_state_t));
                ret = xSemaphoreGive(sys_data_mutex);
                fresult = pdPASS == ret ? SUCCESS:ERROR;
                break;
            default:
                // should never occur
                break;
        }
    }

    return fresult;
}

static system_state_t system_info = {0};

ErrorStatus radio_get_status_curr(radio_frame_t* __restrict radio_frame)
{
    if(SUCCESS == manage_state_info(&system_info, DATA_GET_ONLY)){
        radio_frame->exec = system_info.radio_frame_send.exec;
        radio_frame->orientation = system_info.radio_frame_send.orientation;
        radio_frame->enviro = system_info.radio_frame_send.enviro;
        return SUCCESS;
    }
    return ERROR;
}

ErrorStatus radio_set_status_curr(radio_frame_t* __restrict radio_frame)
{
    ErrorStatus fresult = ERROR;

    if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
        exec_settings_t* curr = &system_info.radio_frame_send.exec;
        uint8_t light = curr->light.all & 0x0F;
        *curr = radio_frame->exec;
        if(true == curr->light.autodetect) \
            curr->light.all = light|(curr->light.all & 0xF0);
        fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
        return SUCCESS;
    }
    return fresult;
}



ErrorStatus exec_group_set_status(const exec_settings_t* __restrict status)
{
    ErrorStatus fresult = ERROR;

    if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
        system_info.radio_frame_send.exec = *status;
        fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
    }
    return fresult;
}

exec_settings_t exec_group_get_status(void)
{
    exec_settings_t fresult = {0};

    if(SUCCESS == manage_state_info(&system_info, DATA_GET_ONLY)){
        fresult = system_info.radio_frame_send.exec;
    }
    return fresult;
}

movement_settings_t drive_get_status(void)
{
    movement_settings_t fresult = {0};

    if(SUCCESS == manage_state_info(&system_info, DATA_GET_ONLY)){
        fresult = system_info.radio_frame_send.exec.movement;
    }
    return fresult;
}

uint8_t drive_get_direction(void)
{
    return drive_get_status().direction;
}

ErrorStatus drive_set_status(const movement_settings_t* __restrict status)
{
    ErrorStatus fresult = ERROR;

    if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
        system_info.radio_frame_send.exec.movement = *status;
        fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
    }
    return fresult;
}

light_settings_t light_get_status(void)
{
    light_settings_t fresult = {0};

    if(SUCCESS == manage_state_info(&system_info, DATA_GET_ONLY)){
        fresult = system_info.radio_frame_send.exec.light;
    }
    return fresult;
}

ErrorStatus light_set_status(const light_settings_t* status)
{
    ErrorStatus fresult = ERROR;

    if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
        system_info.radio_frame_send.exec.light = *status;
        fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
    }
    return fresult;
}

uint8_t light_get_leds_on(void)
{
    return light_get_status().all & 0x0F;
}

bool light_is_auto(void)
{
    return light_get_status().autodetect;
}

ErrorStatus light_set_auto(bool is_auto)
{
    ErrorStatus fresult = ERROR;

    if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
        system_info.radio_frame_send.exec.light.autodetect = is_auto;
        fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
    }
    return fresult;
}

ErrorStatus light_toggle_led(uint8_t bit)
{
    ErrorStatus fresult = ERROR;

    if(bit < LIGHT_RF_LAST) {
        if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
            uint8_t *status = &system_info.radio_frame_send.exec.light.all;
            *status = 0 != (*status & bit) ? *status & ~bit:*status | bit;
            fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
        }
    }
    return fresult;
}

ErrorStatus light_set_led_on(uint8_t bit)
{
    ErrorStatus fresult = ERROR;

    if(bit < LIGHT_RF_LAST) {
        if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
            uint8_t* status = &system_info.radio_frame_send.exec.light.all;
            *status = 0 != bit? *status|bit:*status&0xF0;
            fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
        }
    }
    return fresult;
}

ErrorStatus light_off_all(void)
{
    return light_set_led_on(0);
}

uint8_t signal_get_state(void)
{
    uint8_t fresult = false;

    if(SUCCESS == manage_state_info(&system_info, DATA_GET_ONLY)){
        fresult = system_info.radio_frame_send.exec.signal.honk;
    }
    return fresult;
}

bool signal_is_on(void)
{
    return 0 != signal_get_state() ? true:false;
}

ErrorStatus signal_set_state(uint8_t state)
{
    ErrorStatus fresult = ERROR;

    if(SUCCESS == manage_state_info(&system_info, DATA_LOCK_AND_GET)){
        system_info.radio_frame_send.exec.signal.honk = state;
        fresult = manage_state_info(&system_info, DATA_SET_AND_UNLOCK);
    }
    return fresult;
}
