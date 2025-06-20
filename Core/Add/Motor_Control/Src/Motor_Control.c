#include <stdio.h>
#include <stdlib.h>
#include "../Inc/Motor_Control.h"
#include "stm32wbxx_hal.h"
#include "../../GPIO_Extender/Inc/PCF_8574.h"

extern TIM_HandleTypeDef htim2;

MotorState motors[4] = {0};  // index 0 → moteur 1, index 1 → moteur 2, etc.

// Structure pour stocker l’état des pins
typedef struct {
    uint8_t Pin;
} Pins_Typedef;

// Tableau contenant l'état courant des GPIO (8 bits pour PCF8574)
Pins_Typedef pins[] = {
    { .Pin = 0x00 }
};
size_t pins_size = sizeof(pins) / sizeof(Pins_Typedef);

void print_binary(uint8_t value) {
    printf("Etat binaire des pins : ");
    for (int i = 7; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
    }
    printf("\n");
}

float map_power_to_speed(int power) {
    return (power / 100.0f) * 34.0f; // linéaire de 0 à 35 tr/s
}

int compute_pid(PID_Controller *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

//	printf(
//			"PID - Setpoint: %.2f, Measurement: %.2f, Error: %.2f, Output: %.2f\n",
//			setpoint, measurement, error, output);
//	printf("PID - Integral: %.2f, Derivative: %.2f\n", pid->integral, derivative);
//	printf("PID - Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid->kp, pid->ki, pid->kd);


	if (output > 100.0f) output = 100.0f;
	if (output < -100.0f) output = -100.0f;

    return (int)output;
}

void motorTraduction(int forward, int spine, float p, float i, float d) {
	for(int it = 0; it < 4; it++) {
		motors[it].pid.kp = p;  // Gain proportionnel
		motors[it].pid.ki = i;  // Gain intégral
		motors[it].pid.kd = d;// Gain dérivé
		motors[it].pid.max_integral = 50.0f;  // Réinitialisation de l'intégrale
	}

    // Centre les valeurs autour de 0 (50 = stop)
    int fwd = forward - 50;  // [-50 ; +50]
    int turn = spine - 50;   // [-50 ; +50]

    // Gains (peuvent être ajustés pour modifier réactivité)
    float Kf = 2.0f;  // Avancer/Reculer
    float Kt = 2.0f;  // Rotation

    // Calcule les composantes
    int speed_forward = (int)(Kf * fwd);   // [-100 ; +100]
    int speed_turn    = (int)(Kt * turn);  // [-100 ; +100]

    // Applique la logique différentielle pour chaque moteur
    motors[0].setpoint = speed_forward - speed_turn;
    motors[1].setpoint = speed_forward + speed_turn;
    motors[2].setpoint = speed_forward - speed_turn;
    motors[3].setpoint = speed_forward + speed_turn;

//    setMotor(1, compute_pid(&motors[0].pid, map_power_to_speed(motors[0].setpoint), motors[0].measured_speed, 1.0f));
//    setMotor(2, compute_pid(&motors[1].pid, map_power_to_speed(motors[1].setpoint), motors[1].measured_speed, 1.0f));
//    setMotor(3, compute_pid(&motors[2].pid, map_power_to_speed(motors[2].setpoint), motors[2].measured_speed, 1.0f));
//    setMotor(4, compute_pid(&motors[3].pid, map_power_to_speed(motors[3].setpoint), motors[3].measured_speed, 1.0f))

    setMotor(1, compute_pid(&motors[0].pid, motors[0].setpoint, motors[0].measured_speed, 1.0f));
    setMotor(2, compute_pid(&motors[1].pid, motors[1].setpoint, motors[1].measured_speed, 1.0f));
    setMotor(3, compute_pid(&motors[2].pid, motors[2].setpoint, motors[2].measured_speed, 1.0f));
    setMotor(4, compute_pid(&motors[3].pid, motors[3].setpoint, motors[3].measured_speed, 1.0f));

	print_binary(pins[0].Pin);
	GPIO_EXTENDER_ProcessCommand(&pins[0].Pin);
}

void setMotor(int id, int vitesse) {
	switch (id) {
	case 1:
		setDirection(id,vitesse);
		Set_PWM_DutyCycle(&htim2, TIM_CHANNEL_1, abs(vitesse));
		break;
	case 2:
		setDirection(id,vitesse);
		Set_PWM_DutyCycle(&htim2, TIM_CHANNEL_2, abs(vitesse));
		break;
	case 3:
		setDirection(id,vitesse);
		Set_PWM_DutyCycle(&htim2, TIM_CHANNEL_3, abs(vitesse));
		break;
	case 4:
		setDirection(id,vitesse);
		Set_PWM_DutyCycle(&htim2, TIM_CHANNEL_4, abs(vitesse));
		break;
	default:
		printf("Erreur : ID de moteur invalide\n");
		return;
	}
//	printf("Moteur %d regle a la vitesse %d\n", id, vitesse);
}

void setDirection(int id, int vitesse) {
    if (id < 1 || id > 4) return;

    // Calcul de l'offset dans l'octet (0 pour M1, 2 pour M2, etc.)
    uint8_t shift = (id - 1) * 2;

    uint8_t last = pins[0].Pin; // Sauvegarde de l'état avant modification

    // Reset des deux bits correspondant au moteur (met à 00)
    pins[0].Pin &= ~(0b11 << shift);

    if (vitesse > 0) {
//    	printf("Avant 01 pour M%d\n",id);
        // Avant → 01
        pins[0].Pin |= (0b01 << shift);
    } else if (vitesse < 0) {
//    	printf("Arrière 10 pour M%d\n",id);
        // Arrière → 10
        pins[0].Pin |= (0b10 << shift);
    }
//
//    if (last != pins[0].Pin) {
//		printf("Changement de direction pour M%d, nouvel etat : \n", id);
//		print_binary(pins[0].Pin);
//		GPIO_EXTENDER_ProcessCommand(&pins[0].Pin);
//	} else {
//		printf("Aucun changement de direction pour M%d, etat inchange : \n", id);
//	}

}

void Set_PWM_DutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float dutyCycle) {
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t pulse = (uint32_t)((dutyCycle / 100.0f) * (float)(period));
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
