#include "Peripheral_setup.h"
#include "PI.h"
#include "Parameters.h"

// VARIÁVEIS
float ma_shuntLC;
float ma_hbridge;

int Ligar_ShuntLC = 0;

float ADCA_2 = 0.0;
float ADCA_3 = 0.0;
float ADCA_4 = 0.0;

float VAC = 0.0;
float VDC = 0.0;
float I_out = 0.0;

// PI tensão
PI Controlador_VDC;
float Out_Cvdc = 0.0;

// PI corrente
PI Controlador_I;
float Out_Ci = 0.0;

// FUNÇÃO DE INTERRUPÇÃO PWM1
__interrupt void isr_adc(void);

void main(void) {
    InitSysCtrl();

    DINT;
    InitPieCtrl();
    IER = 0X0000;
    IFR = 0X0000;
    InitPieVectTable();

    Setup_GPIO();
    Setup_EPwm1();
    Setup_EPwm2();
    Setup_EPwm3();
    Setup_EPwm8();
    Setup_ADCA();

    PI_init(&Controlador_VDC, VDC_KP, VDC_KI, VDC_saturation, -VDC_saturation);
    PI_init(&Controlador_I, I_KP, I_KI, I_saturation, -I_saturation);

    EALLOW;
    PieVectTable.ADCA1_INT = &isr_adc;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADC
    EDIS;
    IER |= M_INT1;

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global real-time interrupt DBGM

    while(1){

    }
}

// Interrupção para leitura do ADC-A
__interrupt void isr_adc(void){

    // Lê os sensores
    ADCA_2 = AdcaResultRegs.ADCRESULT0;  // ADCINA 2
    ADCA_3 = AdcaResultRegs.ADCRESULT1;  // ADCINA 3
    ADCA_4 = AdcaResultRegs.ADCRESULT2;  // ADCINA 4

    VAC = 1.0/2047.0*(float)((int)ADCA_2 - 2047);
    VDC = VDC_REF/2047.0*(float)((int)ADCA_3);
    I_out = 100.0/2047.0*(float)((int)ADCA_4 - 2047);

    Out_Cvdc = PI_run(&Controlador_VDC, (-VDC_REF*0.01), (-VDC*0.01), f_amostragem);
    Out_Ci = PI_run(&Controlador_I, (Out_Cvdc*VAC), 0.1*I_out, f_amostragem);

    ma_hbridge = (float)(3.0 + Out_Ci)/6.0;
    ma_shuntLC = 0.018*(VDC - VDC_REF);


    // Atualizar PWM

    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)(ma_hbridge*EPwm1Regs.TBPRD);
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)((1.0-ma_hbridge)*EPwm2Regs.TBPRD);

    if(Ligar_ShuntLC == 1) {
        EPwm3Regs.CMPA.bit.CMPA = (uint16_t)((ma_shuntLC)*EPwm3Regs.TBPRD);
        EPwm3Regs.CMPB.bit.CMPB = (uint16_t)((1.0 - (1.0 + ma_shuntLC))*EPwm3Regs.TBPRD);
    }
    else {
        EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(0);
        EPwm3Regs.CMPB.bit.CMPB = (uint16_t)(0);
    }


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
