/*
 * Parameters.h
 *
 *  Created on: 31 de jan de 2025
 *      Author: Gustavo Barbosa
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_
#include <math.h>

// Frequencies
extern float f_amostragem;
extern float f_Hbridge;
extern float f_shuntLC;

// Setpoints
extern float VDC_REF;

//PI parameters
extern float VDC_KP;
extern float VDC_KI;
extern float VDC_saturation;

extern float I_KP;
extern float I_KI;
extern float I_saturation;


// Function to calculate TBPRD
int TBPRD_calculate (float frequency);

#endif /* PARAMETERS_H_ */
