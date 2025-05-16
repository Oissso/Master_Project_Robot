#include <stdio.h>
#include "../Inc/Motor_Control.h"

// Prototype de la fonction setMotor (à définir ailleurs ou simuler pour test)
extern void setMotor(int id, int vitesse);

void motorTraduction(int forward, int spine) {
    // Centre les valeurs autour de 0 (50 = stop)
    int fwd = forward - 50;  // [-50 ; +50]
    int turn = spine - 50;   // [-50 ; +50]

    // Gains (peuvent être ajustés pour modifier réactivité)
    float Kf = 1.0f;  // Avancer/Reculer
    float Kt = 1.0f;  // Rotation

    // Calcule les composantes
    int speed_forward = (int)(Kf * fwd);   // [-100 ; +100]
    int speed_turn    = (int)(Kt * turn);  // [-100 ; +100]

    // Applique la logique différentielle pour chaque moteur
    int m1 = speed_forward - speed_turn;
    int m2 = speed_forward + speed_turn;
    int m3 = speed_forward - speed_turn;
    int m4 = speed_forward + speed_turn;

    // Affichage debug
    printf("Vitesses moteurs : m1=%d, m2=%d, m3=%d, m4=%d\n", m1, m2, m3, m4);
}
