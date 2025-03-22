#include "../Inc/ble_control.h"

#include <stdbool.h>  // Pour utiliser bool

bool BLE_ProcessCommand(uint8_t *data, uint8_t length)
{
    if (length >= 4)
    {
        if (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00 && data[3] == 0x01)
        {
            return true;  // La commande est valide
        }
    }
    return false;  // La commande est invalide
}
