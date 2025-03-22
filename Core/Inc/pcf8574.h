/*
 * pcf8574.h
 *
 *  Created on: Mar 13, 2025
 *      Author: cypri
 */

#ifndef PCF8574_H_
#define PCF8574_H_

#include "stm32wbxx_hal.h"


#define PCF8574_ADDRESS_BASE 0x40


typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint8_t portState;
} PCF8574_HandleTypeDef;

void PCF8574_Init(PCF8574_HandleTypeDef *pcf8574, I2C_HandleTypeDef *hi2c, uint8_t a0, uint8_t a1, uint8_t a2);
void PCF8574_WritePin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin, GPIO_PinState state);
GPIO_PinState PCF8574_ReadPin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin);
void PCF8574_TogglePin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin);
uint8_t PCF8574_ReadPort(PCF8574_HandleTypeDef *pcf8574);
void  PCF8574_WritePort(PCF8574_HandleTypeDef *pcf8574, uint8_t value);

#endif // PCF8574_H
