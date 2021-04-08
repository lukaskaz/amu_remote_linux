#ifndef __MOD_SOUND_SIGNAL_H__
#define __MOD_SOUND_SIGNAL_H__

typedef enum {
    SOUND_RF_NONE = 0,
    SOUND_RF_PRESS,
    SOUND_RF_TOGGLE,
    SOUND_RF_WAIT,
    SOUND_RF_LAST,
} SoundRFOperation_t;

extern void vSound_configuration(void);
extern void vSound_Signal_Control(const uint8_t);
extern void vSound_Signal_RF_Control(const uint8_t status);
extern bool vSound_is_signal(void);

#endif  // __MOD_SOUND_SIGNAL_H__
