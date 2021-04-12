/****************************************Copyright (c)****************************************************
**
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_orientation_sensor.c
** Descriptions:            GY85 sensor service for orientation measurements
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
#include <math.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "mod_i2c.h"
#include "mod_lcd.h"
#include "mod_sound.h"
#include "mod_orientsensor.h"

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define ADXL345_ADDRESS         0x53 // Assumes ALT address pin low
#define ACCL_SHIFTED_ADDR       (ADXL345_ADDRESS<<1)
#define ACCL_PARAMS(reg, data, size) \
    &(I2cParams_t){ACCL_SHIFTED_ADDR, reg, data, size}

/*=========================================================================*/

/*=========================================================================
REGISTERS
-----------------------------------------------------------------------*/
#define ADXL345_REG_DEVID               (0x00)    // Device ID
#define ADXL345_REG_THRESH_TAP          (0x1D)
#define ADXL345_REG_OFSX                (0x1E)
#define ADXL345_REG_OFSY                (0x1F)
#define ADXL345_REG_OFSZ                (0x20)
#define ADXL345_REG_DUR                 (0x21)
#define ADXL345_REG_LATENT              (0x22)
#define ADXL345_REG_WINDOW              (0x23)
#define ADXL345_REG_TAP_AXES            (0x2A)
#define ADXL345_REG_BW_RATE             (0x2C)    // Data rate
#define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
#define ADXL345_REG_INT_ENABLE          (0x2E)
#define ADXL345_REG_INT_MAP             (0x2F)
#define ADXL345_REG_INT_SOURCE          (0x30)
#define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
#define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
#define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
#define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
#define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
#define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
#define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1

#define ACCL_DEVID                      0xE5
#define ADXL345_FULL_RESOLUTION         0x08
#define ADXL345_INT_STAP                0x40 // Enable interrupt when device is tapped once
#define ADXL345_INT_DTAP                0x20 // Enable interrupt when device is tapped once
#define ADXL345_INT_DATA                0x80 // Enable interrupt when data is available
#define ADXL345_X_TAP_AXIS              0x04
#define ADXL345_Y_TAP_AXIS              0x02
#define ADXL345_Z_TAP_AXIS              0x01
#define ADXL345_DTAP_SUPPRESS_BIT       0x08

#define ADXL345_OFFSET_X                (-0.08f)
#define ADXL345_OFFSET_Y                0.00f
#define ADXL345_OFFSET_Z                (-0.2f)
#define ADXL345_MG2G_MULTIPLIER         (0.004f)   // 4mg per lsb
//#define ADXL345_MG2G_MULTIPLIER         (0.032)   // 4mg per lsb
/* Constants */
#define SENSORS_GRAVITY_EARTH           (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON            (1.6F) /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN             (275.0F) /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_SYMBOLIC        1
//#define SENSORS_GRAVITY_STANDARD        (SENSORS_GRAVITY_EARTH)
#define SENSORS_GRAVITY_STANDARD        (SENSORS_GRAVITY_SYMBOLIC)


#define GYRO_ADDRESS                    0x68
#define ITG3205_ADDRESS                 0x68
#define GYRO_ID(dat)                    (dat & 0x7E)
#define GYRO_SHIFTED_ADDR               (ITG3205_ADDRESS<<1)
#define GYRO_PARAMS(reg, data, size) \
    &(I2cParams_t){GYRO_SHIFTED_ADDR, reg, data, size}

#define GYRO_REG_WHOAMI                 0x00
#define GYRO_REG_SMPLRT_DIV             0x15
#define GYRO_REG_DLPF_FS                0x16
#define GYRO_REG_INT_CFG                0x17
#define GYRO_REG_INT_STS                0X1A

#define GYRO_REG_TEMP_H                 0x1B
#define GYRO_REG_TEMP_L                 0x1C
#define GYRO_REG_X_H                    0x1D
#define GYRO_REG_X_L                    0x1E
#define GYRO_REG_Y_H                    0x1F
#define GYRO_REG_Y_L                    0x20
#define GYRO_REG_Z_H                    0x21
#define GYRO_REG_Z_L                    0x22
#define GYRO_REG_PWR_MGM                0x3E

#define GYRO_DEVID                      GYRO_ID(GYRO_ADDRESS)
#define GYRO_INT_READY                  0x04
#define GYRO_INT_DATA                   0x01
#define GYRO_INTERNAL_OSC               0x00
#define GYRO_LPFILTER_5HZ               0x06
#define GYRO_SCALE_FULL                 0x18

#define GYRO_OFFSET_X                   5
#define GYRO_OFFSET_Y                   (-6)
#define GYRO_OFFSET_Z                   41
#define GYRO_SENSITIVITY                14.375F

#define ORIENT_TASK_INTERVAL_MS         50

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G = 3, // +/- 16g
  ADXL345_RANGE_8_G  = 2, // +/- 8g
  ADXL345_RANGE_4_G  = 1, // +/- 4g
  ADXL345_RANGE_2_G  = 0  // +/- 2g (default value)
} range_t;

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ = 0x0F, // 1600Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_1600_HZ = 0x0E, // 800Hz Bandwidth 90�A IDD
  ADXL345_DATARATE_800_HZ  = 0x0D, // 400Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_400_HZ  = 0x0C, // 200Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_200_HZ  = 0x0B, // 100Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_100_HZ  = 0x0A, // 50Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_50_HZ   = 0x09, // 25Hz Bandwidth 90�A IDD
  ADXL345_DATARATE_25_HZ   = 0x08, // 12.5Hz Bandwidth 60�A IDD
  ADXL345_DATARATE_12_5_HZ = 0x07, // 6.25Hz Bandwidth 50�A IDD
  ADXL345_DATARATE_6_25HZ  = 0x06, // 3.13Hz Bandwidth 45�A IDD
  ADXL345_DATARATE_3_13_HZ = 0x05, // 1.56Hz Bandwidth 40�A IDD
  ADXL345_DATARATE_1_56_HZ = 0x04, // 0.78Hz Bandwidth 34�A IDD
  ADXL345_DATARATE_0_78_HZ = 0x03, // 0.39Hz Bandwidth 23�A IDD
  ADXL345_DATARATE_0_39_HZ = 0x02, // 0.20Hz Bandwidth 23�A IDD
  ADXL345_DATARATE_0_20_HZ = 0x01, // 0.10Hz Bandwidth 23�A IDD
  ADXL345_DATARATE_0_10_HZ = 0x00  // 0.05Hz Bandwidth 23�A IDD (default value)
} dataRate_t;

static bool ITG3205_setReady(const bool* __restrict);
static bool ITG3205_isReady(void);
static uint8_t ITG3205_getDeviceID(void);
static uint8_t ITG3205_powerMgmt(uint8_t);
static uint8_t ITG3205_SampleRateDiv(uint8_t);
static uint8_t ITG3205_LowPassFilter(uint8_t);
static uint8_t ITG3205_InterruptConf(uint8_t);
static bool ITG3205_isInterruptRawDataReady(void);
static bool ITG3205_isInterruptDeviceReady(void);
static void ITG3205_getXYZ(VectorInt_t *);
static int16_t ITG3205_getX(void);
static int16_t ITG3205_getY(void);
static int16_t ITG3205_getZ(void);

static bool ADXL_setReady(const bool* __restrict);
static bool ADXL_isReady(void);
static uint8_t ADXL_getDeviceID(void);
static uint8_t ADXL_powerMgmt(uint8_t);
static uint8_t ADXL_getRange(void);
static void ADXL_setRange(uint8_t);
dataRate_t ADXL_getDataRate(void);
static uint8_t ADXL_setDataRate(dataRate_t);
static uint8_t ADXL_setInterrupts(uint8_t);
static uint8_t ADXL_getInterruptStatus(void);
static void ADXL_getXYZ(VectorInt_t *);
static int16_t ADXL_getX(void);
static int16_t ADXL_getY(void);
static int16_t ADXL_getZ(void);
static uint8_t ADXL_setDTapLatent(double);
static uint8_t ADXL_setDTapWindow(double);
static uint8_t ADXL_setTapThreshold(double);
static double ADXL_getTapThreshold(void);
static uint8_t ADXL_setTapDuration(double);
static double ADXL_getTapDuration(void);
static uint8_t ADXL_setAxesTapDetection(uint8_t);
static uint8_t ADXL_getAxesTapDetection(void);
static uint8_t ADXL_setAxesOffset(const VectorDbl_t *);
static void ADXL_getAxesOffset(VectorDbl_t *v);

static uint32_t constrain(int32_t, int32_t, int32_t);

uint16_t sensor_update_yaw(const double* __restrict);
uint16_t sensor_update_pitch(const double* __restrict);
uint16_t sensor_update_roll(const double* __restrict);
double sensor_update_acclx(const VectorDbl_t*);
double sensor_update_accly(const VectorDbl_t*);
double sensor_update_acclz(const VectorDbl_t*);
uint8_t sensor_set_events(const uint8_t* __restrict, bool);

#define SEN_ACCL_VECTY_AVG_SIZE         20U

bool is_robot_stopped(void)
{
    bool fresult = false;
    return fresult;
}

void vOrientation_sensor_configuration(void)
{
    if(ACCL_DEVID == ADXL_getDeviceID()) {
        VectorDbl_t xyzOffsetVect = { ADXL345_OFFSET_X, ADXL345_OFFSET_Y, ADXL345_OFFSET_Z };

        ADXL_powerMgmt(0x08);
        ADXL_setRange(ADXL345_RANGE_16_G);
        ADXL_setDataRate(ADXL345_DATARATE_100_HZ);
        ADXL_setInterrupts(ADXL345_INT_DATA|ADXL345_INT_STAP|ADXL345_INT_DTAP);
        ADXL_setDTapLatent(0.01);
        ADXL_setDTapWindow(0.18);
        ADXL_setTapThreshold(8.0);
        ADXL_setTapDuration(0.01);
        ADXL_setAxesTapDetection(ADXL345_X_TAP_AXIS|ADXL345_Y_TAP_AXIS|ADXL345_Z_TAP_AXIS);
        ADXL_setAxesOffset(&xyzOffsetVect);
        ADXL_setReady(&(const bool){true});
    }
    else {
        // sensor not detected
        // ToDo: implement error service for this case
    }

    if(GYRO_DEVID == ITG3205_getDeviceID()) {
        ITG3205_powerMgmt(GYRO_INTERNAL_OSC);
        ITG3205_SampleRateDiv(0x13);
        ITG3205_LowPassFilter(GYRO_LPFILTER_5HZ|GYRO_SCALE_FULL);
        ITG3205_InterruptConf(GYRO_INT_READY|GYRO_INT_DATA);
        ITG3205_setReady(&(const bool){true});
    }
    else {
        // sensor is not detected
        // ToDo: implement error service for the case
    }
}

#define GYRO_IS_VALUE_ABOVE_THRESHOLD(val) ((-0.5f) >= (val) || 0.5f <= (val))
#define GYRO_GET_ANGULAR_GAIN(speed, timediff_ms) (speed * ((double)timediff_ms/1000.0f))
#define GYRO_GET_PRECISE_REV(vint, axis) ((vint + GYRO_OFFSET_##axis)/GYRO_SENSITIVITY)
#define GYRO_GET_PRECISE_VECT_REV(vd, vi) vd.x = GYRO_GET_PRECISE_REV(vi.x, X); \
    vd.y = GYRO_GET_PRECISE_REV(vi.y, Y); vd.z = GYRO_GET_PRECISE_REV(vi.z, Z);
void gyro_get_data(TickType_t timediff_ms)
{
    if(true == ITG3205_isReady()) {
        VectorInt_t vi = {0};
        VectorDbl_t vd = {0};

        ITG3205_getXYZ(&vi);
        GYRO_GET_PRECISE_VECT_REV(vd, vi);
        if(true == GYRO_IS_VALUE_ABOVE_THRESHOLD(vd.x)) {
            sensor_update_yaw(&(double){GYRO_GET_ANGULAR_GAIN(vd.x, timediff_ms)});
        }
        if(true == GYRO_IS_VALUE_ABOVE_THRESHOLD(vd.y)) {
            sensor_update_pitch(&(double){GYRO_GET_ANGULAR_GAIN(vd.y, timediff_ms)});
        }
        if(true == GYRO_IS_VALUE_ABOVE_THRESHOLD(vd.z)) {
            sensor_update_roll(&(double){GYRO_GET_ANGULAR_GAIN(vd.z, timediff_ms)});
        }
    }
}

#define ACCL_IS_GRAVITY_AXIS(val) ((-0.975f) >= (val) || 0.975f <= (val))
#define ACCL_IS_VALUE_ABOVE_THRESHOLD(val) ((-0.02f) >= (val) || 0.02f <= (val))
#define ACCL_GET_PRECISE_VAL(vint) (vint*ADXL345_MG2G_MULTIPLIER*SENSORS_GRAVITY_STANDARD)
#define ACCL_GET_PRECISE_VECT_VALS(vd, vi) vd.x = ACCL_GET_PRECISE_VAL(vi.x); \
        vd.y = ACCL_GET_PRECISE_VAL(vi.y); vd.z = ACCL_GET_PRECISE_VAL(vi.z);
void accl_get_data(TickType_t timediff_ms)
{
    if(true == ADXL_isReady()) {
        static VectorDbl_t stored = {0};
        VectorDbl_t curr = {0};
        VectorInt_t vi = {0};

        ADXL_getXYZ(&vi);
        ACCL_GET_PRECISE_VECT_VALS(curr, vi);

        if(true == ACCL_IS_VALUE_ABOVE_THRESHOLD(curr.x-stored.x)) stored.x = curr.x;
        if(true == ACCL_IS_VALUE_ABOVE_THRESHOLD(curr.y-stored.y)) stored.y = curr.y;
        if(true == ACCL_IS_VALUE_ABOVE_THRESHOLD(curr.z-stored.z)) stored.z = curr.z;

        sensor_update_acclx(&stored);
        sensor_update_accly(&stored);
        sensor_update_acclz(&stored);
        sensor_set_events(&(uint8_t){ADXL_getInterruptStatus()}, false);

    }
}

static bool ITG3205_setReady(const bool* __restrict is_ready)
{
    static bool ready = false;

    if(NULL != is_ready) ready = *is_ready;
    return ready;
}

static bool ITG3205_isReady(void)
{
    return ITG3205_setReady(NULL);
}

static uint8_t ITG3205_getDeviceID(void)
{
     uint8_t data = 0xFF;
     return 1 == xI2C_read_sequence(GYRO_PARAMS(GYRO_REG_WHOAMI, &data, 1)) ? \
                                                             GYRO_ID(data):0xFF;
}

static uint8_t ITG3205_powerMgmt(uint8_t sett)
{
    return xI2C_write_sequence(GYRO_PARAMS(GYRO_REG_PWR_MGM, &sett, 1));
}

static uint8_t ITG3205_SampleRateDiv(uint8_t sett)
{
    return xI2C_write_sequence(GYRO_PARAMS(GYRO_REG_SMPLRT_DIV, &sett, 1));
}

static uint8_t ITG3205_LowPassFilter(uint8_t sett)
{
    return xI2C_write_sequence(GYRO_PARAMS(GYRO_REG_DLPF_FS, &sett, 1));
}

static uint8_t ITG3205_InterruptConf(uint8_t sett)
{
    uint8_t data = sett & 0xF5;
    return xI2C_write_sequence(GYRO_PARAMS(GYRO_REG_INT_CFG, &data, 1));
}

static uint8_t ITG3205_getInterruptStatus(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(GYRO_PARAMS(GYRO_REG_INT_STS, &data, 1)) ? \
                                                                    data:0xFF;
}

static bool ITG3205_isInterruptRawDataReady(void)
{
    uint8_t status = ITG3205_getInterruptStatus();
    return (status & GYRO_INT_DATA) == GYRO_INT_DATA;
}

static bool ITG3205_isInterruptDeviceReady(void)
{
    uint8_t status = ITG3205_getInterruptStatus();
    return (status & GYRO_INT_READY) == GYRO_INT_READY;
}

#define GYRO_CONV_XYZ_RAW_TO_VECT(vect, data) \
    vect->x = data[1] | (data[0]<<8); \
    vect->y = data[3] | (data[2]<<8); \
    vect->z = data[5] | (data[4]<<8);

static void ITG3205_getXYZ(VectorInt_t *vi)
{
    uint8_t data[6] = {0};
    if(sizeof(data) == xI2C_read_sequence(GYRO_PARAMS(GYRO_REG_X_H, data, sizeof(data)))) {
        GYRO_CONV_XYZ_RAW_TO_VECT(vi, data);
    }
}

static int16_t ITG3205_getX(void)
{
    uint8_t data[2] = {0};
    return sizeof(data) == xI2C_read_sequence(GYRO_PARAMS(GYRO_REG_X_H, data, sizeof(data))) ? \
                                                                    data[1]|(data[0]<<8):0xFF;
}

static int16_t ITG3205_getY(void)
{
    uint8_t data[2] = {0};
    return sizeof(data) == xI2C_read_sequence(GYRO_PARAMS(GYRO_REG_Y_H, data, sizeof(data))) ? \
                                                                    data[1]|(data[0]<<8):0xFF;
}

static int16_t ITG3205_getZ(void)
{
    uint8_t data[2] = {0};
    return sizeof(data) == xI2C_read_sequence(GYRO_PARAMS(GYRO_REG_Z_H, data, sizeof(data))) ? \
                                                                    data[1]|(data[0]<<8):0xFF;
}

static bool ADXL_setReady(const bool* __restrict is_ready)
{
    static bool ready = false;

    if(NULL != is_ready) ready = *is_ready;
    return ready;
}

static bool ADXL_isReady(void)
{
    return ADXL_setReady(NULL);
}

static uint8_t ADXL_getDeviceID(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DEVID, &data, 1)) \
                                                                    ? data:0xFF;
}

static uint8_t ADXL_powerMgmt(uint8_t sett)
{
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_POWER_CTL, &sett, 1));
}

static uint8_t ADXL_getRange(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DATA_FORMAT, &data, 1)) ? \
                                                                        data&0x03:0xFF;
}

static void ADXL_setRange(uint8_t range)
{
    uint8_t data = 0xFF;

    xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DATA_FORMAT, &data, 1));
    data &= (~0x0F);
    data |= range | ADXL345_FULL_RESOLUTION;
    xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_DATA_FORMAT, &data, 1));
}

dataRate_t ADXL_getDataRate(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_BW_RATE, &data, 1)) ? \
                                                    (dataRate_t)(data&0x0F):0xFF;
}

static uint8_t ADXL_setDataRate(dataRate_t dataRate)
{
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_BW_RATE, &dataRate, 1));
}

static uint8_t ADXL_setInterrupts(uint8_t interrupts)
{
    uint8_t data = 0x00;
    xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_INT_MAP, &data, 1));
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_INT_ENABLE, &interrupts, 1));
}

static uint8_t ADXL_getInterruptStatus(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_INT_SOURCE, &data, 1)) ? \
                                                                            data:0xFF;
}

static bool ADXL_isInterruptRawDataReady(void)
{
    uint8_t status = ADXL_getInterruptStatus();
    return (status & ADXL345_INT_DATA) == ADXL345_INT_DATA;
}

#define ACCL_CONV_XYZ_RAW_TO_VECT(vect, data) \
    vect->x = data[0] | (data[1]<<8); \
    vect->y = data[2] | (data[3]<<8); \
    vect->z = data[4] | (data[5]<<8);

static void ADXL_getXYZ(VectorInt_t *vi)
{
    uint8_t data[6] = {0};
    if(sizeof(data) == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DATAX0, data, sizeof(data)))) {
        ACCL_CONV_XYZ_RAW_TO_VECT(vi, data);
    }
}

static int16_t ADXL_getX(void)
{
    uint8_t data[2] = {0};
    return sizeof(data) == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DATAX0, data, sizeof(data))) ? \
                                                                        data[0] | (data[1]<<8):0xFFFF;
}

static int16_t ADXL_getY(void)
{
    uint8_t data[2] = {0};
    return sizeof(data) == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DATAY0, data, sizeof(data))) ? \
                                                                        data[0] | (data[1]<<8):0xFFFF;
}

static int16_t ADXL_getZ(void)
{
    uint8_t data[2] = {0};
    return sizeof(data) == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DATAZ0, data, sizeof(data))) ? \
                                                                        data[0] | (data[1]<<8):0xFFFF;
}

static uint8_t ADXL_setDTapLatent(double value)
{
    uint8_t data = constrain(value / 0.00125f, 0, 255);
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_LATENT, &data, 1));
}

static uint8_t ADXL_setDTapWindow(double value)
{
    uint8_t data = constrain(value / 0.00125f, 0, 255);
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_WINDOW, &data, 1));
}

static uint8_t ADXL_setTapThreshold(double threshold)
{
    uint8_t data = constrain(threshold / 0.0625f, 0, 255);
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_THRESH_TAP, &data, 1));
}

// Get Tap Threshold (62.5mg / LSB)
double ADXL_getTapThreshold(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_THRESH_TAP, &data, 1)) ? \
                                                                data * 0.0625f:(-1);
}

// Set Tap Duration (625us / LSB)
static uint8_t ADXL_setTapDuration(double duration)
{
    uint8_t data = constrain(duration / 0.000625f, 0, 255);
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_DUR, &data, 1));
}

// Get Tap Duration (625us / LSB)
double ADXL_getTapDuration(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_DUR, &data, 1)) ? \
                                                        data * 0.000625f:(-1);
}

static uint8_t ADXL_setAxesTapDetection(uint8_t axes)
{
    //uint8_t data = axes;
    //data |= ADXL345_DTAP_SUPPRESS_BIT
    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_TAP_AXES, &axes, 1));
}

uint8_t ADXL_getAxesTapDetection(void)
{
    uint8_t data = 0xFF;
    return 1 == xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_TAP_AXES, &data, 1)) ? \
                                                                          data:0xFF;
}

static uint8_t ADXL_setAxesOffset(const VectorDbl_t *v)
{
    int32_t value = 0;
    uint8_t data[3] = {0};

    value = (int32_t)(v->x/0.015625f);
    data[0] = constrain((uint8_t)value, 0, 255);
    value = (int32_t)(v->y/0.015625f);
    data[1] = constrain((uint8_t)value, 0, 255);
    value = (int32_t)(v->z/0.015625f);
    data[2] = constrain((uint8_t)value, 0, 255);

    return xI2C_write_sequence(ACCL_PARAMS(ADXL345_REG_OFSX, data, sizeof(data)));
}

void ADXL_getAxesOffset(VectorDbl_t *v)
{
    uint8_t data[3] = {0};

    xI2C_read_sequence(ACCL_PARAMS(ADXL345_REG_OFSX, data, sizeof(data)));
    v->x = (int8_t)data[0] * 0.015625f;
    v->y = (int8_t)data[1] * 0.015625f;
    v->z = (int8_t)data[2] * 0.015625f;
}

#define POS_ANGLE_MIN   0
#define POS_ANGLE_MAX   360
uint16_t sensor_update_yaw(const double* __restrict value)
{
    static double yaw = 0.0;

    if(NULL != value) {
        yaw += *value;
        yaw = yaw >= POS_ANGLE_MAX ? yaw - POS_ANGLE_MAX:yaw;
        yaw = yaw <  POS_ANGLE_MIN ? POS_ANGLE_MAX + yaw:yaw;
    }
    return (uint16_t)yaw;
}

uint16_t sensor_get_yaw(void)
{
    return sensor_update_yaw(NULL);
}

uint16_t sensor_update_pitch(const double* __restrict value)
{
    static double pitch = 0.0;

    if(NULL != value) {
        pitch += *value;
        pitch = pitch >= POS_ANGLE_MAX ? pitch - POS_ANGLE_MAX:pitch;
        pitch = pitch <  POS_ANGLE_MIN ? POS_ANGLE_MAX + pitch:pitch;
    }
    return (uint16_t)pitch;
}

uint16_t sensor_get_pitch(void)
{
    return sensor_update_pitch(NULL);
}

uint16_t sensor_update_roll(const double* __restrict value)
{
    static double roll = 0.0;

    if(NULL != value) {
        roll += *value;
        roll = roll >= POS_ANGLE_MAX ? roll - POS_ANGLE_MAX:roll;
        roll = roll <  POS_ANGLE_MIN ? POS_ANGLE_MAX + roll:roll;
    }
    return (uint16_t)roll;
}

uint16_t sensor_get_roll(void)
{
    return sensor_update_roll(NULL);
}


#define PI_VAL 3.14159265
#define SENSOR_CALC_ATAN2(v1,v2)    (atan2(v1, v2)*180.0f/PI_VAL);
#define SENSOR_ROUND_ATAN(val)      (uint16_t)((val) < 0 ? 360.0f+(val):(val));

uint16_t sensor_update_acclx_tilt(const VectorDbl_t* accl)
{
    static uint16_t tiltx = 0;

    if(NULL != accl) {
        double result = 0;
        if(accl->x != 0) result = SENSOR_CALC_ATAN2(accl->z, accl->x);
        tiltx = SENSOR_ROUND_ATAN(result);
    }
    return tiltx;
}

double sensor_update_acclx(const VectorDbl_t* accl)
{
    static double acclx = 0;

    if(NULL != accl) {
        acclx = accl->x;
        sensor_update_acclx_tilt(accl);
    }
    return acclx;
}

double sensor_get_acclx(void)
{
    return sensor_update_acclx(NULL);
}

uint16_t sensor_get_tiltx(void)
{
    return sensor_update_acclx_tilt(NULL);
}

uint16_t sensor_update_accly_tilt(const VectorDbl_t* accl)
{
    static uint16_t tilty = 0;

    if(NULL != accl) {
        double result = 0;
        if(accl->x != 0) result = SENSOR_CALC_ATAN2(accl->y, accl->x);
        tilty = SENSOR_ROUND_ATAN(result);
        tilty = 0 == tilty ? 0:360-tilty;
    }
    return tilty;
}

double sensor_update_accly(const VectorDbl_t* accl)
{
    static double accly = 0;

    if(NULL != accl) {
        accly = accl->y;
        sensor_update_accly_tilt(accl);
    }

    return accly;
}

double sensor_get_accly(void)
{
    return sensor_update_accly(NULL);
}

uint16_t sensor_get_tilty(void)
{
    return sensor_update_accly_tilt(NULL);
}

uint16_t sensor_update_acclz_tilt(const VectorDbl_t* accl)
{
    static uint16_t tiltz = 0;

    if(NULL != accl) {
        double result = 0;
        if(accl->z != 0) result = SENSOR_CALC_ATAN2(accl->y, accl->z);
        tiltz = true == ACCL_IS_GRAVITY_AXIS(accl->x) \
                            ? 0:SENSOR_ROUND_ATAN(result);
    }
    return tiltz;
}

double sensor_update_acclz(const VectorDbl_t* accl)
{
    static double acclz = 0;

    if(NULL != accl) {
        acclz = accl->z;
        sensor_update_acclz_tilt(accl);
    }
    return acclz;
}

double sensor_get_acclz(void)
{
    return sensor_update_acclz(NULL);
}

uint16_t sensor_get_tiltz(void)
{
    return sensor_update_acclz_tilt(NULL);
}

uint8_t sensor_set_events(const uint8_t* __restrict status, bool clear)
{
    static uint8_t events = false;

    events = (true == clear) ? 0:events;
    if(NULL != status) events |= *status;
    return events;
}

uint8_t sensor_get_events(void)
{
    return sensor_set_events(NULL, false);
}

#define ACCL_IS_WEAK_TAPPED(st) ((st & ADXL345_INT_STAP) == ADXL345_INT_STAP)
#define ACCL_IS_HARD_TAPPED(st) ((st & ADXL345_INT_DTAP) == ADXL345_INT_DTAP)
uint8_t sensor_get_minor_collision_event(void)
{
    return ACCL_IS_WEAK_TAPPED(sensor_set_events(NULL, false));
}

uint8_t sensor_get_major_collision_event(void)
{
    return ACCL_IS_HARD_TAPPED(sensor_set_events(NULL, false));
}

static uint32_t constrain(int32_t x, int32_t a, int32_t b)
{
    if(x < a) return a;
    else if(b < x) return b;
    else return x;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
