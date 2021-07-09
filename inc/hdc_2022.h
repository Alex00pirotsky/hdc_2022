//
// Created by alexp on 06/07/2021.
//

#ifndef M0FIRMWARE_HDC_2022_H
#define M0FIRMWARE_HDC_2022_H

#include "main.h"
void      init_hdc_22(I2C_HandleTypeDef I2C_Handler, uint8_t timeout);
void      deInit_hdc_22(void );

typedef enum
{
    ADDR_TEMPERATURE_LOW = 0x00,      /*  R    Temperature data [7:0]                                   */
    ADDR_TEMPERATURE_HIGH,            /*  R    Temperature data [15:8]                                  */
    ADDR_HUMIDITY_LOW,                /*  R    Humidity data [7:0]                                      */
    ADDR_HUMIDITY_HIGH,               /*  R    Humidity data [15:8]                                     */
    ADDR_STATUS,                      /*  R    DataReady and threshold status                           */
    ADDR_TEMPERATURE_MAX,             /*  R    Maximum measured temperature (one-shot mode only)        */
    ADDR_HUMIDITY_MAX,                /*  R    Maximum measured humidity (one-shot mode only)           */
    ADDR_INTERRUPT_ENABLE,            /*  R/W  Interrupt enable                                         */
    ADDR_TEMP_OFFSET_ADJUST,          /*  R/W  Temperature offset adjustment                            */
    ADDR_HUM_OFFSET_ADJUST,           /*  R/W  Humidity offset adjustment                               */
    ADDR_TEMP_THR_L,                  /*  R/W  Temperature threshold low                                */
    ADDR_TEMP_THR_H,                  /*  R/W  Temperature threshold high                               */
    ADDR_RH_THR_L,                    /*  R/W  Humidity threshold low                                   */
    ADDR_RH_THR_H,                    /*  R/W  Humidity threshold high                                  */
    ADDR_DEVICE_CONFIGURATION,        /*  R/W  Soft reset and interrupt reporting configuration         */
    ADDR_MEASUREMENT_CONFIGURATION,   /*  R/W  Device measurement configuration                         */
    ADDR_MANUFACTURER_ID_LOW = 0xFC,  /*  R    Manufacturer ID lower-byte                               */
    ADDR_MANUFACTURER_ID_HIGH,        /*  R    Manufacturer ID higher-byte                              */
    ADDR_DEVICE_ID_LOW,               /*  R    Device ID lower-byte                                     */
    ADDR_DEVICE_ID_HIGH,              /*  R    Device ID higher-byte                                    */
}e_hdc_22_data_adr;

#endif //M0FIRMWARE_HDC_2022_H
