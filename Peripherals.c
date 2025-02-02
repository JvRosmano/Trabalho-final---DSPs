/*
 *  Peripherals.c
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

#include "Peripherals.h"

void ConfiguraDAC()
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // Use ADC references
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DAC

    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // Use ADC references
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DAC
    EDIS;
}

void ConfiguraADC(void)
{
    EALLOW;
    // Habilita interrupcao
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; // Enable INT1 flag
    // Configura cada canal AD
    ConfiguraADCA();
    ConfiguraADCB();
    ConfiguraADCC();
    ConfiguraADCD();

    EDIS;
}

void ConfiguraADCA()
{
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;   // ADCINA2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;  // sample duration of 20 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;   // SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;   // SOC2 will convert pin A4
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5;   // SOC3 will convert pin A5
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // ePWM1 SOCA
}

void ConfiguraADCB()
{
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;   // SOC0 will convert pin A2
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;   // SOC1 will convert pin A3
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4;   // SOC2 will convert pin A4
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5;   // SOC3 will convert pin A5
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // ePWM1 SOCA
}

void ConfiguraADCC(void)
{
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;   // SOC0 will convert pin A2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;   // SOC1 will convert pin A3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;   // SOC2 will convert pin A4
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5;   // SOC3 will convert pin A5
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // ePWM1 SOCA
}

void ConfiguraADCD()
{
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 14;  // SOC0 will convert pin A2
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA
}

void ConfiguraPWM1()
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // Ver pag. 355! e  o default...

    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK pre-scaler = /1

    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Modo Up-down (simetrico)

    EPwm1Regs.TBPRD = PWM_TBPRD; // Set timer period

    EPwm1Regs.ETSEL.bit.SOCASEL = 2; // Select SOCA on period match
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;  // Generate pulse on 1st event
    EDIS;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // TROCADO
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Trocar para DB_ACTV_HIC
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.OUTSWAP = 0;

    EPwm1Regs.DBFED.all = 250;
    EPwm1Regs.DBRED.all = 250;
}

void ConfiguraPWM2()
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // Ver pag. 355! e  o default...

    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK pre-scaler = /1

    EPwm2Regs.TBCTL.bit.CTRMODE = 2; // Modo Up-down (simetrico)

    EPwm2Regs.TBPRD = PWM_TBPRD; // Set timer period

    EPwm2Regs.ETSEL.bit.SOCASEL = 2; // Select SOCA on period match
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm2Regs.ETPS.bit.SOCAPRD = 1;  // Generate pulse on 1st event
    EDIS;

    // Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Trocar para DB_ACTV_HIC
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.OUTSWAP = 0;

    EPwm2Regs.DBFED.all = 250;
    EPwm2Regs.DBRED.all = 250;
}

void ConfiguraPWM3()
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // Ver pag. 355! e  o default...

    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK pre-scaler = /1

    EPwm3Regs.TBCTL.bit.CTRMODE = 2; // Modo Up-down (simetrico)

    EPwm3Regs.TBPRD = PWM_TBPRD; // Set timer period

    EPwm3Regs.ETSEL.bit.SOCASEL = 2; // Select SOCA on period match
    EPwm3Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm3Regs.ETPS.bit.SOCAPRD = 1;  // Generate pulse on 1st event
    EDIS;

    // Set actions
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Trocar para DB_ACTV_HIC
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.OUTSWAP = 0;

    EPwm3Regs.DBFED.all = 250;
    EPwm3Regs.DBRED.all = 250;
}