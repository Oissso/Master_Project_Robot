/*
 * Motor_Control.h
 *
 *  Created on: May 15, 2025
 *      Author: cypri
 */

#ifndef ADD_MOTOR_CONTROL_INC_MOTOR_CONTROL_H_
#define ADD_MOTOR_CONTROL_INC_MOTOR_CONTROL_H_

#include "stm32wbxx_hal.h"

// Structure pour le contrôleur PID
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float current_value;
    float max_integral; // Limite pour l'intégrale
} PID_Controller;


typedef struct {
    float setpoint;
    float measured_speed;
    int   pwm_output;
    PID_Controller pid;
} MotorState;

extern MotorState motors[4];

// Déclarations des fonctions
void motorTraduction(int forward, int spine, float p, float i, float d);
void setMotor(int id, int vitesse);
void setDirection(int id, int vitesse);
void Set_PWM_DutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float dutyCycle);
int compute_pid(PID_Controller *pid, float setpoint, float measurement, float dt);
void print_binary(uint8_t value);
float map_power_to_speed(int power);

#endif /* ADD_MOTOR_CONTROL_INC_MOTOR_CONTROL_H_ */
