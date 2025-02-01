/*
 * Peripheral_setup.c
 *
 *  Created on: 5 de dez de 2024
 *      Author: gusta
 */

#include "Peripheral_setup.h"
#include "Parameters.h"

void Setup_GPIO(void) {
        EALLOW;

        // PWM

        // Seleciona o pino GPIO 0 como ePWM-1A
        GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;

        // Seleciona o pino GPIO 1 como ePWM-1B
        GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;

        // Seleciona o pino GPIO 2 como ePWM-2A
        GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;

        // Seleciona o pino GPIO 3 como ePWM-2B
        GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;

        // Seleciona o pino GPIO 4 como ePWM-3A
        GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;

        // Seleciona o pino GPIO 5 como ePWM-3B
        GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;

        // Seleciona o pino GPIO 14 como ePWM-8B
        GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;

        EDIS;
}

void Setup_EPwm1(void){
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Desabilita clock do contador
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;

    // Contagem do PWM
    EPwm1Regs.TBPRD = (uint16_t)TBPRD_calculate(f_Hbridge);                            // Período do PWM (PRD = 200000000/(4*fsw))
    //EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD >> 1;

    EPwm1Regs.TBPHS.bit.TBPHS = 0;                      // Deslocamento de fase (trifásico)
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;     // Sincronismo dos PWMs
    EPwm1Regs.TBCTR = 0;                                // Zerando a contagem
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // UP DOWN
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase Loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Atualização do duty cycle
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // Ações do PWM
    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    // Dead Band
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;           // Pulso complementar
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // Enable dead-band module
    EPwm1Regs.DBFED.bit.DBFED = 80;                     // FED
    EPwm1Regs.DBRED.bit.DBRED = 80;                     // RED

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Habilita clock do contador

    EDIS;
}

void Setup_EPwm2(void){
       EALLOW;
       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Desabilita clock do contador
       CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;

       // Contagem do PWM
       EPwm2Regs.TBPRD = (uint16_t)TBPRD_calculate(f_Hbridge);                            // Período do PWM (PRD = 200000000/(4*fsw))
       //EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD >> 1;

       EPwm2Regs.TBPHS.bit.TBPHS = 0;                      // Deslocamento de fase (trifásico)
       EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;     // Sincronismo dos PWMs
       EPwm2Regs.TBCTR = 0;                                // Zerando a contagem
       EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // UP DOWN
       EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase Loading
       EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Clock ratio to SYSCLKOUT
       EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

       // Atualização do duty cycle
       EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
       EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
       EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
       EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

       // Ações do PWM
       EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
       EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
       EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
       EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

       // Dead Band
       EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;           // Pulso complementar
       EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // Enable dead-band module
       EPwm2Regs.DBFED.bit.DBFED = 80;                     // FED
       EPwm2Regs.DBRED.bit.DBRED = 80;                     // RED

       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Habilita clock do contador

       EDIS;
}

void Setup_EPwm3(void){
       EALLOW;
       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Desabilita clock do contador
       CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;

       // Contagem do PWM
       EPwm3Regs.TBPRD = (uint16_t)TBPRD_calculate(f_shuntLC);                            // Período do PWM (PRD = 200000000/(4*fsw))
       //EPwm3Regs.CMPA.bit.CMPA = EPwm3Regs.TBPRD >> 1;

       EPwm3Regs.TBPHS.bit.TBPHS = 0;                      // Deslocamento de fase (trifásico)
       EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;     // Sincronismo dos PWMs
       EPwm3Regs.TBCTR = 0;                                // Zerando a contagem
       EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // UP DOWN
       EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase Loading
       EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Clock ratio to SYSCLKOUT
       EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

       // Atualização do duty cycle
       EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
       EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
       EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
       EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

       // Ações do PWM
       EPwm3Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
       EPwm3Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
       EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
       EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

       EPwm3Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
       EPwm3Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;
       EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
       EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Habilita clock do contador

       EDIS;
}

void Setup_EPwm8(void){
       EALLOW;
       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Desabilita clock do contador
       CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;

       // Contagem do PWM
       EPwm8Regs.TBPRD = (uint16_t)TBPRD_calculate(f_amostragem);                            // Período do PWM (PRD = 200000000/(4*fsw))
       //EPwm8Regs.CMPA.bit.CMPA = EPwm8Regs.TBPRD >> 1;

       EPwm8Regs.TBPHS.bit.TBPHS = 0;                      // Deslocamento de fase (trifásico)
       EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;     // Sincronismo dos PWMs
       EPwm8Regs.TBCTR = 0;                                // Zerando a contagem
       EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // UP DOWN
       EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase Loading
       EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Clock ratio to SYSCLKOUT
       EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

       // Trigger ADC
       EPwm8Regs.ETSEL.bit.SOCAEN = 1;                     // Enable SOC on A group
       EPwm8Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;       // Dispara ADC PRD e zero
       EPwm8Regs.ETPS.bit.SOCAPRD = ET_1ST;                // Trigger no primeiro evento

       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Habilita clock do contador

       EDIS;
}

void Setup_ADCA(void){
    Uint16 acqps;
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14;             // 75ns
    else
        acqps = 63;             // 320ns

    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // escala
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Setar pulso antes da saída
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up ADC
    DELAY_US(1000);                             // Delay para o ADC energizar

    // Utilização do ADC
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;          // Utilizando A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x13;     // Epwm8

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;          // Utilizando A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x13;     // Epwm8

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;          // Utilizando A4
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x13;     // Epwm8

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x02;   // Dispara a interrupção depois do Soc2
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable interrupt
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Cleaer flag

    EDIS;
}
