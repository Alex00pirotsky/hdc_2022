//
// Created by alexp on 06/07/2021.
//  alex_pirotsky@outlook.com

#include "../Inc/hdc_2022.h"
#include <stdint.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "../Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_gpio.h"
/*#include "stm32f0xx_hal.h"
#include "../Inc/main.h"*/


#define POW_2_IN_8     (256.0  )
#define POW_2_IN_16    (65536.0)

#define TEMP_CALCULATION(x)               ((x / 65536.0) * 165.0 - 40.0)
#define HUMI_CALCULATION(x)               ((x / 65536.0) * 100         )
#define MAX_TEMP_CALCULATION(x)           ((x / 256.0  ) * 165.0 - 40.0)
#define MAX_HUMI_CALCULATION(x)           (x  * (100.0   / 256.0)      )
#define TEMP_THRESHOLD_CALCULATION(x)     ((x / 256.0  ) * 165.0 - 40.0)
#define HUMI_THRESHOLD_CALCULATION(x)     ((x / 256.0  ) * 100.0       )

union {
    uint8_t val;
    union {
        struct {
            uint8_t : 3; //Reserved
            uint8_t HL_ENABLE: 1; //Humidity threshold LOW Interrupt enable
            uint8_t HH_ENABLE: 1; //Humidity threshold HIGH Interrupt enable
            uint8_t TL_ENABLE: 1; //Temperature threshold LOW Interrupt enable
            uint8_t TH_ENABLE: 1; //Temperature threshold HIGH Interrupt enable
            uint8_t DRDY_ENABLE: 1; //DataReady Interrupt enable
        };
    } bits;
} INTERRUPT_ENABLE; /* The Interrupt Enable register enables or disables interrupt asserstion on the DRDY/INT pin  */

union {
    uint8_t val;
    union {
        struct {
            uint8_t bit0: 1; // +0.16°C
            uint8_t bit1: 1; // +0.32°C
            uint8_t bit2: 1; // +0.64°C
            uint8_t bit3: 1; // +1.29°C
            uint8_t bit4: 1; // +2.58°C
            uint8_t bit5: 1; // +5.16°C
            uint8_t bit6: 1; // +10.31°C
            uint8_t bit7: 1; // –20.63°C
        };
    } bits;
} TEMPERATURE_OFFSET_ADJUSTMENT; /*  Temperature offset adjustment value. The value is added to the converted temperature data.  */
union {
    uint8_t val;
    union {
        struct {
            uint8_t bit0: 1;  /*  +0.2%RH   */
            uint8_t bit1: 1;  /*  +0.4%RH   */
            uint8_t bit2: 1;  /*  +0.8%RH   */
            uint8_t bit3: 1;  /*  +1.6%RH   */
            uint8_t bit4: 1;  /*  +3.1%RH   */
            uint8_t bit5: 1;  /*  +6.3%RH   */
            uint8_t bit6: 1;  /*  +12.5%RH  */
            uint8_t bit7: 1;  /*  –25%RH    */
        };
    } bits;
} HUMIDITY_OFFSET_ADJUSTMENT; /* Humidity offset adjustment value. The value is added to the converted humidity data.  */
union {
    uint8_t val;
    union {
        struct {
            uint8_t INT_MODE: 1;     /*  Interrupt mode                : 0 = Clear-on-read mode, 1 = Comparator mode       */
            uint8_t INT_POL: 1;      /*  Interrupt polarity            : 0 = Active Low, 1 = Active High                   */
            uint8_t DRDY__INT_EN: 1; /*  DRDY/INT_EN pin configuration : 0 = High Z, 1 = Enable                            */
            uint8_t HEAT_EN: 1;      /*  0 = Heater off, 1 = Heater on                                                     */
            uint8_t CC: 3;           /*  Configure the measurement mode to one-shot or continuous conversion               */
            uint8_t SOFT_RES: 1;     /*  0 = Normal Operation, 1 = Trigger a Soft Reset. This bit self-clears after RESET. */
        };
    } bits;
} DEVICE_CONFIGURATION;

union {
    uint8_t val;
    union {
        struct {
            uint8_t MEAS_TRIG: 1;  /*  Measurement trigger         - 0: No action, 1: Start measurement                                */
            uint8_t MEAS_CONF: 2;  /*  Measurement configuration   - 00: Humidity + Temperature, 01: Temperature only, 10: NA, 11: NA  */
            uint8_t : 1;           /*  Reserved                                                                                        */
            uint8_t HACC: 2;       /*  Humidity accuracy option    - 00 : 14 bit, 01 : 11 bit, 10 : 9 bit, 11 : NA                     */
            uint8_t TACC: 2;       /*  Temperature accuracy option - 00 : 14 bit, 01 : 11 bit, 10 : 9 bit, 11 : NA                     */
        };
    } bits;
} MEASUREMENT_CONFIGURATION;
union {
    uint8_t val;
    union {
        struct {
            uint8_t : 3;            /*  Reserved                                   */
            uint8_t HL_STATUS: 1;   /* Humidity threshold LOW Interrupt status     */
            uint8_t HH_STATUS: 1;   /* Humidity threshold HIGH Interrupt status    */
            uint8_t TL_STATUS: 1;   /* Temperature threshold LOW Interrupt status  */
            uint8_t TH_STATUS: 1;   /* Temperature threshold HIGH Interrupt status */
            uint8_t DRDY_STATUS: 1; /* DataReady bit status                        */
        };
    } bits;
} STATUS;


I2C_HandleTypeDef i2c;

uint8_t i2c_timeout = 100;
uint8_t DeviceID;
uint8_t DeviceIDHigh = 0x41 << 1;
uint8_t DeviceIDLow = 0x40 << 1;
uint8_t buffer_8[5];

uint8_t TEMPERATURE_LOW;  /* Temperature data- lower byte  */
uint8_t TEMPERATURE_HIGH; /* Temperature data- higher byte */

uint8_t HUMIDITY_LOW;     /*  Humidity data- lower byte    */
uint8_t HUMIDITY_HIGH;    /*  Humidity data- higher byte   */

uint8_t TEMPERATURE_THRESHOLD_LOW;  /*  Temperature threshold LOW value   */
uint8_t TEMPERATURE_THRESHOLD_HIGH; /*  Temperature threshold HIGH value  */

uint8_t HUMIDITY_THRESHOLD_LOW;     /*  Humidity threshold LOW value      */
uint8_t HUMIDITY_THRESHOLD_HIGH;    /*  Humidity threshold HIGH value     */

int8_t TEMPERATURE_MAX; /* Maximum temperature measurement data (one-shot mode only) */
uint8_t HUMIDITY_MAX;    /* Maximum humidity measurement data (one-shot mode only)    */



uint8_t MANUFACTURER_ID_LOW;  /*  The Manufacturer ID Low and Manufacturer ID High registers contain a factory-programmable identification value that identifies this device as being manufactured by Texas Instruments.  */
uint8_t MANUFACTURER_ID_HIGH; /*  The Manufacturer ID Low and Manufacturer ID High registers contain a factory-programmable identification value that identifies this device as being manufactured by Texas Instruments.  */

uint8_t DEVICE_ID_LOW;        /*  The Device ID Low and Device ID High registers contain a factory-programmable identification value that identifies this device as a HDC2022.  */
uint8_t DEVICE_ID_HIGH;       /*  The Device ID Low and Device ID High registers contain a factory-programmable identification value that identifies this device as a HDC2022.  */



float res = 0;
float device_configs[] = {0, 1.0 / 120.0, 0.1, 0.2, 1.0, 2.0, 5.0};

static uint8_t i2C_getByte(e_hdc_22_data_adr reg);

static void i2C_setByte(e_hdc_22_data_adr reg, uint8_t val);


void init_hdc_22(I2C_HandleTypeDef I2C_Handler, uint8_t timeout) {

    i2c = I2C_Handler;
    i2c_timeout = timeout;
    DeviceID = DeviceIDLow;

    i2C_setByte(ADDR_DEVICE_CONFIGURATION, DEVICE_CONFIGURATION.val);
    i2C_setByte(ADDR_RH_THR_H, HUMIDITY_THRESHOLD_HIGH);
    i2C_setByte(ADDR_HUM_OFFSET_ADJUST, HUMIDITY_OFFSET_ADJUSTMENT.val);
    i2C_setByte(ADDR_RH_THR_L, HUMIDITY_THRESHOLD_LOW);
    i2C_setByte(ADDR_INTERRUPT_ENABLE, INTERRUPT_ENABLE.val);
    i2C_setByte(ADDR_TEMP_OFFSET_ADJUST, TEMPERATURE_OFFSET_ADJUSTMENT.val);
    i2C_setByte(ADDR_TEMP_THR_L, TEMPERATURE_THRESHOLD_LOW);
    i2C_setByte(ADDR_TEMP_THR_H, TEMPERATURE_THRESHOLD_HIGH);
    i2C_setByte(ADDR_MEASUREMENT_CONFIGURATION, MEASUREMENT_CONFIGURATION.val);

}

void deInit_hdc_22(void) {
    TEMPERATURE_LOW = 0x00;
    TEMPERATURE_HIGH = 0x00;
    HUMIDITY_LOW = 0x00;
    HUMIDITY_HIGH = 0x00;
    STATUS.val = 0x00;
    TEMPERATURE_MAX = 0x00;
    HUMIDITY_MAX = 0x00;
    INTERRUPT_ENABLE.val = 0x00;
    TEMPERATURE_OFFSET_ADJUSTMENT.val = 0x00;
    HUMIDITY_OFFSET_ADJUSTMENT.val = 0x00;
    TEMPERATURE_THRESHOLD_LOW = 0x01;
    TEMPERATURE_THRESHOLD_HIGH = 0xFF;
    HUMIDITY_THRESHOLD_LOW = 0x00;
    HUMIDITY_THRESHOLD_HIGH = 0xFF;
    DEVICE_CONFIGURATION.val = 0x00;
    MEASUREMENT_CONFIGURATION.val = 0x00;
    MANUFACTURER_ID_LOW = 0x49;
    MANUFACTURER_ID_HIGH = 0x54;
    DEVICE_ID_LOW = 0xD0;
    DEVICE_ID_HIGH = 0X07;
}

static void i2C_setByte(e_hdc_22_data_adr reg, uint8_t val) {
    uint8_t buf_setI2C[2];

    buf_setI2C[0] = reg;
    buf_setI2C[0] = val;
    HAL_I2C_Master_Transmit(&i2c, DeviceID, buf_setI2C, 2, i2c_timeout);

}

static uint8_t i2C_getByte(e_hdc_22_data_adr reg) {

    uint8_t buf_getI2C[2];

    buf_getI2C[0] = reg;
    HAL_I2C_Master_Transmit(&i2c, DeviceID, buf_getI2C, 1, i2c_timeout);
    HAL_I2C_Master_Receive(&i2c, DeviceID, buf_getI2C, 1, i2c_timeout);
    return buf_getI2C[0];
}


/**
 * @brief  Get Temperature Values
 * @note   read sensor values as byte as and calculate temperature value as a Celsius (°C)
 * @param  None
 * @retval float
 */

float get_Temperature() {

    buffer_8[0] = i2C_getByte(ADDR_TEMPERATURE_LOW);
    buffer_8[1] = i2C_getByte(ADDR_TEMPERATURE_HIGH);

    res = (buffer_8[1] << 8) | (buffer_8[0]);
    res = TEMP_CALCULATION(res);

    return (res); //  * 165
}

/**
 * @brief  Get Maximum Temperature
 * @note	None
 * @param  None
 * @retval uint8_t
 */
float get_MAXTemperature() {

    res = (float) i2C_getByte(ADDR_TEMPERATURE_MAX);
    res = MAX_TEMP_CALCULATION(res);

    return (res);
}

/**
 * @brief  Get Humidity Values
 * @note   read sensor values as byte as and calculate humidity value as a  Relative Humidity (RH)
 * @param  None
 * @retval float
 */

float get_Humidity() {

    buffer_8[0] = i2C_getByte(ADDR_HUMIDITY_LOW);
    buffer_8[1] = i2C_getByte(ADDR_HUMIDITY_HIGH);

    res = (buffer_8[1] << 8) | (buffer_8[0]);
    res = HUMI_CALCULATION(res);

    return (res); // * 100

}

/**
 * @brief  Get Maximum HUM
 * @note	None
 * @param  None
 * @retval uint8_t
 */
float get_MAXHumidity() {
    float res = (float) i2C_getByte(ADDR_HUMIDITY_MAX);

    res = MAX_HUMI_CALCULATION(res);

    return (res); //  * 165

}

/**
 * @brief  Get Temperature Offset Adjustment
 * @note	None
 * @param  None
 * @retval uint8_t
 */

uint8_t get_TemperatureOffset() {

    return i2C_getByte(ADDR_TEMP_OFFSET_ADJUST);

}

/**
 * @brief  Get Humidity Offset Register Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
uint8_t get_HumidityOffset() {

    return i2C_getByte(ADDR_HUM_OFFSET_ADJUST);

}

/**
 * @brief  Get Temperature Low Threshold Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
float get_TemperatureLOWThreshold() {
    res = (float) i2C_getByte(ADDR_TEMP_THR_L);
    res = TEMP_THRESHOLD_CALCULATION(res);
    return (res);
}

/**
 * @brief  Get Temperature High Threshold Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
float get_TemperatureHIGHThreshold() {
    res = (float) i2C_getByte(ADDR_TEMP_THR_H);
    res = TEMP_THRESHOLD_CALCULATION(res);
    return (res);

}

/**
 * @brief  Get Humidity Low Threshold Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
float get_HumidityLOWThreshold() {
    res = (float) i2C_getByte(ADDR_RH_THR_L);
    res = HUMI_THRESHOLD_CALCULATION(res);
    return (res);
}

/**
 * @brief  Get Humidity High Threshold Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
float get_HumidityHIGHThreshold() {
    res = (float) i2C_getByte(ADDR_RH_THR_H);
    res = HUMI_THRESHOLD_CALCULATION(res);
    return (res);
}


/**
 * @brief  Get Device Configuration
 *          Configure the measurement mode to one-shot or continuous
            conversion. The bits also allow sampling frequency to be
            programmed in continuous conversion mode.
            000 = Continuous conversion disabled (one-shot mode)
            001 = 1/120Hz (1 samples every 2 minutes)
            010 = 1/60Hz (1 samples every minute)
            011 = 0.1Hz (1 samples every 10 seconds)
            100 = 0.2 Hz (1 samples every 5 second)
            101 = 1Hz (1 samples every second)
            110 = 2Hz (2 samples every second)
            111 = 5Hz (5 samples every second)
 * @note	None
 * @param  None
 * @retval HZ
 */
float get_DeviceConfiguration() {
    buffer_8[0] = i2C_getByte(ADDR_DEVICE_CONFIGURATION);

    if (buffer_8[0] < sizeof(device_configs) / sizeof(device_configs[0])) {
        return (device_configs[buffer_8[0]]);
    }
    return (0);

}


/**
 * @brief  Get Measurement Configuration Register Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
uint8_t get_MeasurementConfiguration() {

    return i2C_getByte(ADDR_MEASUREMENT_CONFIGURATION);

}


/**
 * @brief  Get Manufacturer ID Low Value
 * @note	None
 * @param  None
 * @retval uint8_t
 */
uint8_t get_ManufacturerIDLOW() {

    return i2C_getByte(ADDR_MANUFACTURER_ID_LOW);

}


//float get_data_hdc_22(e_hdc_22_data_adr adr)
//{
//    if()
//}





