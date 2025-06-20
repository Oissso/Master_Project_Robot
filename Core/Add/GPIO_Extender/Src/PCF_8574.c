#include "../Inc/PCF_8574.h"
#include "stm32wbxx_hal.h"

#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;
#define PCF8574_ADDRESS (0x39 << 1)  // Adresse I2C correcte

bool GPIO_EXTENDER_ProcessCommand(uint8_t *data) {
    uint8_t TxData = *data;  // Convertir la donnée en octet unique

    if (HAL_I2C_Master_Transmit(&hi2c1, PCF8574_ADDRESS, &TxData, 1, 1000) == HAL_OK) {
        return true;  // Transmission réussie
    } else {
        return false;  // Erreur de transmission
    }
}
