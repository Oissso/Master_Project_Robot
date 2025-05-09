#include "../Inc/ble_control.h"
#include "../../GPIO_Extender/Inc/PCF_8574.h"

#include <stdbool.h>
#include <stdio.h>

extern bool GPIO_EXTENDER_ProcessCommand(uint8_t *data);

bool check_command(uint8_t *data) {
	if (data[0] == 0x01) {
		return true;  // Commande valide
	}
	return false;  // Commande invalide
}

//bool BLE_ProcessCommand(uint8_t *data, uint8_t length) {
//    if (length < 2) return false;  // Vérifie que la commande a au moins 2 octets
//
//    // Extrait la commande et la valeur
//    uint8_t command = data[0];
//    uint8_t value = data[1];
//
//    // Vérifie si la commande correspond à un contrôle GPIO
//    if (command == 0x01) {
//        return GPIO_EXTENDER_ProcessCommand(&value);  // Envoie la valeur au PCF8574
//    }
//
//    return false;
//}

//bool BLE_ProcessCommand(uint8_t *data, uint8_t length) {
//    uint8_t command = data[0];
//
//    if (command == 0x01) {
//            // On prépare un tableau de 3 octets à partir de data[1], data[2], data[3]
//            uint8_t gpio_values[3];
//            for (int i = 0; i < 3; i++) {
//                gpio_values[i] = data[i + 1];
//            }
//
//            // On envoie le tableau complet
//            return GPIO_EXTENDER_ProcessCommand(gpio_values);
//        }
//    return false;
//}

bool BLE_ProcessCommand(uint8_t *data, uint8_t length) {

    uint8_t command = data[0];
    uint8_t gpio_value = 0;
    printf("GPIO Value: 0x%01X 0x%01X 0x%01X 0x%01X \r\n", data[0],data[1], data[2], data[3]);  // Hexa + décimal);

    if (command == 0x01) {
        for (int i = 0; i < 4; i++) {
            gpio_value |= (data[i + 1] & 0x01) << i;  // bits 1 à 4
            printf("GPIO Value: 0x%01X (%d)\r\n", gpio_value, gpio_value);  // Hexa + décimal
        }

        printf("GPIO Value: 0x%02X (%d)\r\n", gpio_value, gpio_value);  // Hexa + décimal

        return GPIO_EXTENDER_ProcessCommand(&gpio_value);
    }

    return false;
}

