#ifndef BLE_CONTROL_H
#define BLE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Déclaration de la fonction qui traitera les données BLE reçues
bool BLE_ProcessCommand(uint8_t *data, uint8_t length);

#endif /* BLE_CONTROL_H */
