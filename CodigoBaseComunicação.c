/**
 * main.c
 */

#include "F28x_Project.h"
#include "math.h"

// Definitions for PWM generation
#define PWM_TBPRD 5000 // Define a freq. de amostragem PWMTBPRD = (10000*2500)/freq_desejada   (10kHz => 2500) [pag. 1914]
#define PWM_CMPR25 0   // PWM initial duty cycle = 25%

// Variaveis novas - Plecs
float sensorA = 0, sensorB = 0, sensorC = 0, sensorD = 0;
float sensorE = 0, sensorF = 0, sensorG = 0, sensorH = 0;
float sensorI = 0, sensorJ = 0, sensorK = 0, sensorL = 0;
float sensorM = 0;

float igA = 0, igB = 0, igC = 0;
float vcapA = 0, vcapB = 0, vcapC = 0;
float itA = 0, itB = 0, itC = 0;
float vpacA = 0, vpacB = 0, vpacC = 0;
float vdc = 0;

float vAlfa = 0, vBeta = 0, iAlfa = 0, iBeta = 0;
float I_d = 0, I_q = 0;
float Flag_PWM = 0;
float erro_id = 0, erro_iq = 0, inte_id = 0, inte_iq = 0, ud_ref = 0, uq_ref = 0;

// Variables
Uint16 ToggleCount1 = 0, dutyCycle = 750;

__interrupt void adca1_isr(void);
// void ConfigureTimer(void);
void ConfigureEPWM1(void);
void ConfigureDAC(void);
void ConfigureADC(void);
void ConfiguraPWM1(void);
void ConfiguraPWM2(void);
void ConfiguraPWM3(void);

#define Tamanho 256
int16 Vetor0[Tamanho];
// SOGI
int16 V_teste;
float z = 0, k = 1, V = 0, V_linha = 0, qV_linha = 0, inte_a = 0;
float w = 377;

// Transformada dq
float V_d = 0, V_q = 0, cos_angulo = 0, sin_angulo = 0;

// PLL
float int_srf = 0, omega = 0, angulo = 0, Kp_pll = 2.97, Ki_pll = 793;
int16 angulo_saida;

// PWM
Uint16 Periodo_PWM = 100000000 * 0.5 * 0.5 * 0.0001; // 0.5(up_down)*Freq do CPU * Period da interrupção (em sec)
Uint16 dutycycle1 = 1000;

void main(void)
{

    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    //    EALLOW;
    //    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
    //    EDIS;

    // Step 2. Initialize GPIO:
    InitGpio();
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; // output
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0; // input
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0; // Habilita Pull-up
    GpioCtrlRegs.GPBINV.bit.GPIO59 = 0; // Não inverter
    EDIS;

    InitEPwm1Gpio();
    InitEPwm1Gpio(); // Lembrar do F2837xD_EPwm.c (EPWM1)
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    // Configura PWM1
    ConfiguraPWM1();

    // Configura PWM2
    ConfiguraPWM2();

    // Configura PWM3
    ConfiguraPWM3();
    // ConfigureTimer();
    // ConfigureEPWM1();
    ConfigureDAC();
    ConfigureADC();

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    InitPieCtrl();
    IER = 0;
    IFR = 0;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    EDIS;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Mapeia a int para ADCA1

    IER |= M_INT1;

    EINT;
    ERTM;

    while (1)
    {
    }
}

__interrupt void adca1_isr(void)
{
    static int InterruptCount = 0, resultsIndex = 0;
    static float tempo = 0, delt = 0.0001; // delt é o período de amostragem
    tempo += delt;
    InterruptCount++;

    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    if (GpioDataRegs.GPBDAT.bit.GPIO59 == 1)
    {
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    }

    if (GpioDataRegs.GPBDAT.bit.GPIO59 == 0)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    }

    DacbRegs.DACVALS.all = (Uint16)(2048 + 2000 * sin(2 * 3.141592 * 60 * tempo));

    sensorA = AdcaResultRegs.ADCRESULT0; // ADCINA2
    sensorB = AdcaResultRegs.ADCRESULT1; // ADCINA3
    sensorC = AdcaResultRegs.ADCRESULT2; // ADCINA4
    sensorD = AdcaResultRegs.ADCRESULT3; // ADCINA5

    sensorE = AdcbResultRegs.ADCRESULT0; // ADCINB2
    sensorF = AdcbResultRegs.ADCRESULT1; // ADCINB3
    sensorG = AdcbResultRegs.ADCRESULT2; // ADCINB4
    sensorH = AdcbResultRegs.ADCRESULT3; // ADCINB5

    sensorI = AdccResultRegs.ADCRESULT0; // ADCINC2
    sensorJ = AdccResultRegs.ADCRESULT1; // ADCINC3
    sensorK = AdccResultRegs.ADCRESULT2; // ADCINC4
    sensorL = AdccResultRegs.ADCRESULT3; // ADCINC5

    sensorM = AdcdResultRegs.ADCRESULT0; // ADCIN14

    // V_teste = (AdcaResultRegs.ADCRESULT0 - 2048);
    // V = (float)(V_teste)*0.09765625;  //(180/2048)

    int Conversao_OFFSET = 0;
    float Conversao_Ganho = 0;

    Conversao_OFFSET = (sensorA - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.048828125;
    igA = Conversao_Ganho;

    Conversao_OFFSET = (sensorB - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.048828125;
    igB = Conversao_Ganho;

    Conversao_OFFSET = (sensorC - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.048828125;
    igC = Conversao_Ganho;

    Conversao_OFFSET = (sensorD - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.09765625;
    vcapA = Conversao_Ganho;

    Conversao_OFFSET = (sensorE - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.09765625;
    vcapB = Conversao_Ganho;

    Conversao_OFFSET = (sensorF - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.09765625;
    vcapC = Conversao_Ganho;

    Conversao_OFFSET = (sensorG - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.048828125;
    itA = Conversao_Ganho;

    Conversao_OFFSET = (sensorH - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.048828125;
    itB = Conversao_Ganho;

    Conversao_OFFSET = (sensorI - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.048828125;
    itC = Conversao_Ganho;

    Conversao_OFFSET = (sensorJ - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.09765625;
    vpacA = Conversao_Ganho;

    Conversao_OFFSET = (sensorK - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.09765625;
    vpacB = Conversao_Ganho;

    Conversao_OFFSET = (sensorL - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET) * 0.09765625;
    vpacC = Conversao_Ganho;

    vdc = (float)(sensorM) * 0.244140625;

    //

    // Transformada de clarke
    vAlfa = 0.6666666666 * (vpacA - vpacB * 0.5 - vpacC * 0.5);
    vBeta = 0.5773502691896 * (vpacB - vpacC);

    iAlfa = 0.6666666666 * (igA - igB * 0.5 - igC * 0.5);
    iBeta = 0.5773502691896 * (igB - igC);
    // Transformada dq
    cos_angulo = cos(angulo);
    sin_angulo = sin(angulo);
    V_d = (vAlfa * cos_angulo) + (vBeta * sin_angulo);
    V_q = -(vAlfa * sin_angulo) + (vBeta * cos_angulo);
    I_d = (iAlfa * cos_angulo) + (iBeta * sin_angulo);
    I_q = -(iAlfa * sin_angulo) + (iBeta * cos_angulo);

    // PLL
    int_srf += V_q * delt;
    omega = Kp_pll * V_q + Ki_pll * int_srf + 377;
    angulo += delt * omega;

    if (angulo < 0)
    {
        angulo = 0;
    }
    if (angulo > 6.28318530)
    {
        angulo -= 6.28318530;
    }
    angulo_saida = angulo * 600;

    DacaRegs.DACVALS.all = (Uint16)(angulo_saida);

    // Realiza controle
    // Erros
    erro_id = 20 - I_d;
    erro_iq = 0 - I_q;
    // Integradores
    inte_id += erro_id * delt;
    inte_iq += erro_iq * delt;
    // Ação de controle
    ud_ref = 2 * erro_id + 600 * inte_id - I_q * 0.00125 * 377 + V_d;
    uq_ref = 2 * erro_iq + 600 * inte_iq + I_d * 0.00125 * 377 + V_q;

    vAlfa = ud_ref * cos_angulo - uq_ref * sin_angulo;
    vBeta = ud_ref * sin_angulo + uq_ref * cos_angulo;

    // Update PWMs

    EPwm1Regs.CMPA.bit.CMPA = dutyCycle;
    EPwm2Regs.CMPA.bit.CMPA = dutyCycle;
    EPwm3Regs.CMPA.bit.CMPA = dutyCycle;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear ADC INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void ConfigureTimer(void)
{
    CpuTimer0Regs.PRD.all = 200000000 * 0.0001; // Freq do CPU * Period da interrupção (em sec)
    CpuTimer0Regs.TCR.bit.TSS = 1;              // 1 = Stop timer, 0 = Start/Restart Timer
    CpuTimer0Regs.TCR.bit.TRB = 1;              // 1 = reload timer
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0; // Timer Free Run Disabled
    CpuTimer0Regs.TCR.bit.TIE = 1;  // 0 = Disable/ 1 = Enable Timer Interrupt

    CpuTimer0Regs.TCR.bit.TSS = 0; // 1 = Stop timer, 0 = Start/Restart Timer
}

void ConfigureDAC(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // Use ADC references
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DAC

    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // Use ADC references
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DAC
    EDIS;
}

void ConfigureADC(void)
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;   // ADCINA2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;  // sample duration of 20 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;   // SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;   // SOC2 will convert pin A4
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5;   // SOC3 will convert pin A5
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA
    //

    //    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; // Enable INT1 flag
    EDIS;

    EALLOW;

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;   // SOC0 will convert pin A2
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;   // SOC1 will convert pin A3
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4;   // SOC2 will convert pin A4
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5;   // SOC3 will convert pin A5
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    EDIS;

    EALLOW;

    AdccRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;   // SOC0 will convert pin A2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;   // SOC1 will convert pin A3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;   // SOC2 will convert pin A4
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5;   // SOC3 will convert pin A5
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    EDIS;

    EALLOW;

    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 14;  // SOC0 will convert pin A2
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

    EDIS;
}

void ConfigureEPWM1(void)
{
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm1Regs.TBPRD = Periodo_PWM;

    EPwm1Regs.ETSEL.bit.SOCASEL = 2; // Select SOCA on period match
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;  // Generate pulse on 1st event

    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Un-freeze and enter up-count mode

    EDIS;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm1Regs.DBCTL.bit.IN_MODE = 0x00;
    EPwm1Regs.DBCTL.bit.POLSEL = 0x01;
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x03;
    EPwm1Regs.DBCTL.bit.OUTSWAP = 0x00;
}

void ConfiguraPWM1(void)
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

void ConfiguraPWM2(void)
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

void ConfiguraPWM3(void)
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
