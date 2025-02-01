/*
 * PI.h
 *
 *  Created on: 23 de dez de 2024
 *      Author: gusta
 */

#ifndef PI_H_
#define PI_H_

typedef volatile struct {
    float Kp;       // Ganho proportcional
    float Ki;       // Ganho Integral
    float Integral; // Soma da integral
    float Umax;     // Limite de saturação superior
    float Umin;     // Limite de saturação inferior
} PI;

void PI_init(PI *pi, float Kp_, float Ki_, float Umax_, float Umin_){
    pi->Kp = Kp_;
    pi->Ki = Ki_;
    pi->Integral = 0.0;
    pi->Umax = Umax_;
    pi->Umin = Umin_;
}

float PI_run(PI *pi, float rk, float yk, float fam){
    float erro = rk-yk;
    float uk = 0.0;

    pi->Integral += erro/fam;

    uk = pi->Kp*erro + pi->Ki * pi->Integral;

    if (uk > pi->Umax){
        uk = pi->Umax;
        pi->Integral -= erro * (1/fam);
    }
    else if (uk < pi->Umin){
        uk = pi->Umin;
        pi->Integral -= erro * (1/fam);
    }

    return(uk);
}


#endif /* PI_H_ */
