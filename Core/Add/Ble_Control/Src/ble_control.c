#include "../Inc/ble_control.h"
#include "../../GPIO_Extender/Inc/PCF_8574.h"
#include "../../cJSON/Inc/cJSON.h"
#include "../../Motor_Control/Inc/Motor_Control.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern bool GPIO_EXTENDER_ProcessCommand(uint8_t *data);

bool check_command(uint8_t *data) {
	if (data[0] == 0x01) {
		return true;  // Commande valide
	}
	return false;  // Commande invalide
}

// Fonction pour traiter la commande BLE
// Format des données a envoyé : {"GPS_mode":0,"forward":0,"spine":0} de avancé de 50 à 100, reculé de 0 à 50
bool BLE_ProcessCommand(uint8_t *data, uint8_t length) {
    char json_string[256] = {0};
    if (length >= sizeof(json_string)) return false;
    memcpy(json_string, data, length);
    json_string[length] = '\0';

    printf(">> JSON reçu : %s\r\n", json_string);

    // Parse le JSON
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        printf("⚠️ JSON invalide\n");
        return false;
    }

    // Extraction des champs moteurs
    cJSON *forward = cJSON_GetObjectItem(root, "forward");
    cJSON *spine = cJSON_GetObjectItem(root, "spine");
    cJSON *GPS_mode = cJSON_GetObjectItem(root, "GPS_mode");

    if (!cJSON_IsNumber(forward) || !cJSON_IsNumber(spine) || !cJSON_IsNumber(GPS_mode)) {
        printf("⚠️ Un ou plusieurs champs ne sont pas des nombres\n");
        cJSON_Delete(root);
        return false;
    }

    printf("Moteurs : forward=%d, spine=%d, GPS_mode=%d\n",
           forward->valueint, spine->valueint, GPS_mode->valueint);

    cJSON_Delete(root);

    motorTraduction(forward->valueint, spine->valueint);
    return true;
}
