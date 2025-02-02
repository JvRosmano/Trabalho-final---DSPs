/*
 *  Utils.h
 *  Copyright (C) 2025
 *  JoÃ£o Vitor de M.G. Rosmaninho <jvrosmaninho@ufmg.br>,
 *  Gustavo Miranda Auler <gustavoauler@ufmg.br>,
 *
 *  Version 1.0 - API with the following implemented function:
 *  float CondicionaSinal(Uint16 sinal, Uint16 offset, float gain);
 *  void clarkeTransform(*Clarke clarke, float a, float b, float c);
 *  void parkTransform(Park *park, Clarke clarke, float angulo);
 *  void inverseClarkeTransform(Park park, Clarke *clarke);
 *  void inverseParkTransform(Clarke clarke, float *a, float *b, float *c);
 *  void executePLL(Park park, float *omega, float *angulo);
 *  Based on notes from professor Gabriel Fogli <gabrielfogli@ufmg.br> for real time control with DSPs course.
 *
 *
 *  Created on: 2025/01/31
 *  Institution: UFMG
 */

#ifndef UTILS_H_
#define UTILS_H_
#include "F28x_Project.h"
#include "math.h"

// PerÃ­odo de amostragem
#define DELT 0.0001
// Definicoes da geracao do PWM
#define PWM_TBPRD 5000 // Define a freq. de amostragem PWMTBPRD = (10000*2500)/freq_desejada   (10kHz => 2500) [pag. 1914]
#define PWM_CMPR25 0   // PWM initial duty cycle = 25%
// PWM
#define Periodo_PWM 100000000 * 0.5 * 0.5 * DELT // 0.5(up_down)*Freq do CPU * Period da interrupcao (em sec)
#define DUTYCYCLE 750
// PLL
#define Kp_PLL 2.93
#define Ki_PLL 793
// Controle
#define Id_REF 20
#define Iq_REF 0
#define Kp_CONT 0.1
#define Ki_CONT 100

typedef struct
{
    float alfa;
    float beta;
} Clarke;

typedef struct
{
    float D;
    float Q;
} Park;

float CondicionaSinal(Uint16 sinal, Uint16 offset, float gain);
void clarkeTransform(Clarke *clarke, float a, float b, float c);
void parkTransform(Park *park, Clarke *clarke, float angulo);
void inverseParkTransform(Park *park, Clarke *clarke);
void inverseClarkeTransform(Clarke *clarke, float *a, float *b, float *c);
void executePLL(Park *park, float *omega, float *angulo);

#endif /* UTILS_H_ */
