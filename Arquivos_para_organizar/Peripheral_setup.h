/*
 * Peripheral_setup.h
 *
 *  Created on: 5 de dez de 2024
 *      Author: gusta
 */

#ifndef PERIPHERAL_SETUP_H_
#define PERIPHERAL_SETUP_H_
#include "F28x_Project.h"

void Setup_GPIO(void); // Configurar os pinos GPIO

void Setup_EPwm1(void); // PWM do braço 1 da ponte H
void Setup_EPwm2(void); // PWM do braço 2 da ponte H
void Setup_EPwm3(void); // PWM do shunt_LC
void Setup_EPwm8(void); // Amostragem e interrupção

void Setup_ADCA(void);   // Sensores

#endif /* PERIPHERAL_SETUP_H_ */
