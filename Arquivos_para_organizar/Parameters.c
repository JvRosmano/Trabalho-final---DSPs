/*
 * Parameters.c
 *
 *  Created on: 31 de jan de 2025
 *      Author: Gustavo Barbosa
 */
#include "Parameters.h"

// Frequencies
float f_amostragem = 120000;
float f_Hbridge = 6000;
float f_shuntLC = 60000;

// Setpoints
float VDC_REF = 500.0;

//PI parameters
float VDC_KP = 4.0;
float VDC_KI = 10.0;
float VDC_saturation = 10000.0;

float I_KP = 1.0;
float I_KI = 120.0;
float I_saturation = 3.0;

// Function to calculate TBPRD
int TBPRD_calculate (float frequency){
    return(round(200000000/(4*frequency)));
}
