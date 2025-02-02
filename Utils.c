/*
 *  Utils.c
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

#include "Utils.h"

// VariÃ¡veis de interesse nas transformaÃ§Ãµes
float cos_angulo = 0, sin_angulo = 0;
float int_srf = 0;
float int_id = 0, int_iq = 0;

float CondicionaSinal(Uint16 sinal, Uint16 offset, float gain)
{
    return (sinal - offset) * gain;
}

void clarkeTransform(Clarke *clarke, float a, float b, float c)
{
    clarke->alfa = 0.6666666666 * (a - b * 0.5 - c * 0.5);
    clarke->beta = 0.5773502691896 * (b - c);
}
void parkTransform(Park *park, Clarke *clarke, float angulo)
{
    cos_angulo = cos(angulo);
    sin_angulo = sin(angulo);
    park->D = (clarke->alfa * cos_angulo) + (clarke->beta * sin_angulo);
    park->Q = -(clarke->alfa * sin_angulo) + (clarke->beta * cos_angulo);
}

void inverseClarkeTransform(Clarke *clarke, float *a, float *b, float *c)
{
    *a = clarke->alfa;
    *b = -clarke->alfa * 0.5 + 0.8660254037844 * clarke->beta;
    *c = -clarke->alfa * 0.5 - 0.8660254037844 * clarke->beta;
}

void inverseParkTransform(Park *park, Clarke *clarke)
{
    float erro_id = Id_REF - park->D;
    float erro_iq = Iq_REF - park->Q;
    // Integradores
    int_id += erro_id * DELT;
    int_iq += erro_iq * DELT;
    // Acao de controle
    float ud_ref = Kp_CONT * erro_id + Ki_CONT * int_id; // Possibilidade de feedfoward
    float uq_ref = Kp_CONT * erro_iq + Ki_CONT * int_iq;
    // Transformada de Park inversa
    clarke->alfa = (ud_ref * cos_angulo) - (uq_ref * sin_angulo);
    clarke->beta = (ud_ref * sin_angulo) + (uq_ref * cos_angulo);
}

void executePLL(Park *park, float *omega, float *angulo)
{
    int_srf += park->Q * DELT; // Integral de V_q
    *omega = Kp_PLL * park->Q + Ki_PLL * int_srf + 377;
    *angulo += DELT * (*omega);

    // Garante dente de serra
    if (*angulo < 0)
    {
        *angulo = 0;
    }
    if (*angulo > 6.28318530)
    {
        *angulo -= 6.28318530;
    }
}