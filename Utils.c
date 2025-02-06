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
float int_id = 0, int_iq = 0, int_vdc = 0;

float CondicionaSinal(Uint16 sinal, int offset, float gain)
{
    int aux = sinal - offset;
    return (float) aux * gain;
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

void inverseParkTransform(Clarke *clarke, float vdc, float udRef, float uqRef)
{
    clarke->alfa = (2./vdc) * ((udRef * cos_angulo) - (uqRef * sin_angulo));
    clarke->beta = (2./vdc) * ((udRef * sin_angulo) + (uqRef * cos_angulo));
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

Uint16 convertSine2PWM(float u){

    return (Uint16)((u + 0.5) * PWM_TBPRD);

}


void dcVoltageControl(float *idRef, float *Vdc_U, float vdc, float vdcRef){
    *Vdc_U = (*Vdc_U < vdcRef) ? (*Vdc_U + 0.001) : vdcRef;

    float erro_vdc = -1 * ((*Vdc_U) * (*Vdc_U) - vdc * vdc);
    int_vdc += erro_vdc * DELT;
    *idRef = Kp_VDC*(erro_vdc) + Ki_VDC * (int_vdc);
}


void currentControl(Park *park, float idRef, float iqRef, float *udRef, float *uqRef){
    float erro_id = idRef - park->D;
    float erro_iq = iqRef - park->Q;
    // Integradores
    int_id += erro_id * DELT;
    int_iq += erro_iq * DELT;
    // A��o de controle
    *udRef = Kp_CONT * erro_id + Ki_CONT * int_id; //Possibilidade de feedfoward
    *uqRef = Kp_CONT * erro_iq + Ki_CONT * int_iq;
}


Uint16 pwmConvert(float u){
    return (Uint16)((-u + 1) * PWM_TBPRD/2.);
}


void LPFilter(float *anterior, float atual, float alfa){
    *anterior = alfa*(*anterior) + (1-alfa)*(atual);
}
