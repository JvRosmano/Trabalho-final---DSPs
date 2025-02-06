/*
 *  Peripherals.h
 *  Copyright (C) 2024
 *  João Vitor de M.G. Rosmaninho <jvrosmaninho@ufmg.br>,
 *  Gustavo Miranda Auler <gustavoauler@ufmg.br>,
 *
 *  Version 1.0 - API with the following implemented function:
 *  void ConfiguraDAC(void);
 *  void ConfiguraADC(void);
 *  void ConfiguraADCA(void);
 *  void ConfiguraADCB(void);
 *  void ConfiguraADCC(void);
 *  void ConfiguraADCD(void);
 *  void ConfiguraPWM1(void);
 *  void ConfiguraPWM2(void);
 *  void ConfiguraPWM3(void);
 *
 *  Based on notes from professor Gabriel Fogli <gabrielfogli@ufmg.br> for real time control with DSPs course.
 *
 *
 *  Created on: 2025/01/31
 *  Institution: UFMG
 *  This API contain functions to provide use of some hardware resources
 *  from LAUNCHXL-F28379D available at:
 *  https://www.ti.com/tool/LAUNCHXL-F28379D
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_
#include "F28x_Project.h"

void ConfiguraDAC(void);
void ConfiguraADC(void);
void ConfiguraADCA(void);
void ConfiguraADCB(void);
void ConfiguraADCC(void);
void ConfiguraADCD(void);
void ConfiguraPWM1(void);
void ConfiguraPWM2(void);
void ConfiguraPWM3(void);

#endif /* PERIPHERALS_H_ */
