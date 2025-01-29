#include "F28x_Project.h" // Device Header File and Examples Include File
#include "math.h"

void ConfiguraGpio(void);
void ConfiguraDAC(void);
void ConfiguraADC(void);
void ConfiguraADCA(void);
void ConfiguraADCB(void);
void ConfiguraADCC(void);
void ConfiguraADCD(void);
void ConfiguraPWM1(void);
void ConfiguraPWM2(void);
void ConfiguraPWM3(void);
__interrupt void adca1_isr(void);

// Definitions for PWM generation
#define PWM_TBPRD 5000 // Define a freq. de amostragem PWMTBPRD = (10000*2500)/freq_desejada   (10kHz => 2500) [pag. 1914]
#define PWM_CMPR25 0   // PWM initial duty cycle = 25%

// Variables
Uint16 ToggleCount1 = 0, dutyCycle = 750;
// static int InterruptCount=0;

// Configura��es de funcionamento
float Period_Timer = 0.0001; // (em sec)
float tempo = 0, senoide = 0;

int V_teste = 0;
float V = 0;

float z = 0, k = 1, V_linha = 0, qV_linha = 0, inte_a = 0;
float w = 377;

// Transformada
float V_d = 0, V_q = 0, cos_angulo = 0, sin_angulo = 0;

// PLL
float int_srf = 0, omega = 0, angulo = 0, Kp_pll = 2.97, Ki_pll = 793;
int16 angulo_saida;

// Vari�veis novas - Plecs
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

void main(void)
{
    // Initialize System Control
    InitSysCtrl();

    // Initialize GPIO
    InitGpio();

    // Readequa os pinos conforme interesse
    ConfiguraGpio();
    InitEPwm1Gpio(); // Lembrar do F2837xD_EPwm.c (EPWM1)
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    // Configura DAC
    ConfiguraDAC();

    // Configura ADC
    ConfiguraADC();

    // Configura PWM1
    ConfiguraPWM1();

    // Configura PWM2
    ConfiguraPWM2();

    // Configura PWM3
    ConfiguraPWM3();

    // Disable CPU interrupts and initialize PIE control registers
    DINT;
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table
    InitPieVectTable();

    // Configura��o da interrup��o
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; // Function for ADCA interrupt 1
    EDIS;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    IER |= M_INT1;

    // Enable global interrupts and higher priority real-time debug events
    EINT; // Enable Global interrupt INTM
    ERTM; // Enable Global realtime interrupt DBGM

    // Sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    // Desabilitar Dead time - Garantir que todas as chaves estar o abertas no in cio do c digo (incluir apenas quando tiver controle)
    EPwm1Regs.DBCTL.bit.OUTSWAP = 0x01;
    EPwm2Regs.DBCTL.bit.OUTSWAP = 0x01;
    EPwm3Regs.DBCTL.bit.OUTSWAP = 0x01;

    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPA.bit.CMPA = 0;
    // Idle loop
    for (;;)
    {
        DELAY_US(1000 * 500); // ON delay
                              //  GpioDataRegs.GPASET.bit.GPIO31=1;
        DELAY_US(1000 * 500); // OFF delay
                              // GpioDataRegs.GPACLEAR.bit.GPIO31=1;
                              // GpioDataRegs.GPBTOGGLE.bit.GPIO34=1;

        //   CpuTimer0Regs.PRD.all;
    }
}

void ConfiguraGpio(void)
{
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; // Drives LED LD1
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; // Drives LED LD2 on controlCARD

    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0; // input
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0; // Habilita Pull-up
    GpioCtrlRegs.GPBINV.bit.GPIO59 = 0; // N�o inverter

    EDIS;

    GpioDataRegs.GPASET.bit.GPIO31 = 1; // Devido a conex�o dos LEDs
    GpioDataRegs.GPBSET.bit.GPIO34 = 1; // Ativar os pinos apagam os LEDs
}

void ConfiguraDAC(void)
{
    EALLOW;

    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // Use ADC references
    DacaRegs.DACCTL.bit.LOADMODE = 0;   // Load on next SYSCLK
    DacaRegs.DACVALS.all = 0x0800;      // Set mid-range
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DAC

    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // Use ADC references
    DacbRegs.DACCTL.bit.LOADMODE = 0;   // Load on next SYSCLK
    DacbRegs.DACVALS.all = 0x0800;      // Set mid-range
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable DAC
    EDIS;
}

void ConfiguraADC(void)
{

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    EALLOW;
    // ADC-A
    ConfiguraADCA();

    // ADC-B
    ConfiguraADCB();

    // ADC-C
    ConfiguraADCC();

    // ADC-D
    ConfiguraADCD();

    EDIS;
}

void ConfiguraADCA(void)
{
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;   // SOC0 will convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA

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

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC
    DELAY_US(1000);                    // Delay for 1ms to allow ADC time to power up
}

void ConfiguraADCB(void)
{
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

    // Checar essa parte
    // AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;   // SOC0 will convert pin A2
    // AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19;  // Sample window is 20 SYSCLK cycles
    // AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on Timer0; SOCA/C

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC
    DELAY_US(1000);                    // Delay for 1ms to allow ADC time to power up
}

void ConfiguraADCC(void)
{
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

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC
    DELAY_US(1000);                    // Delay for 1ms to allow ADC time to power up
}

void ConfiguraADCD(void)
{
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 14;  // SOC0 will convert pin D3
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on ePWM2 SOCA/C

    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC
    DELAY_US(1000);                    // Delay for 1ms to allow ADC time to power up
}

void ConfiguraPWM1(void)
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // Ver pag. 355! e � o default...

    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK pre-scaler = /1

    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Modo Up-down (sim�trico)

    EPwm1Regs.TBPRD = PWM_TBPRD; // Set timer period

    EPwm1Regs.ETSEL.bit.SOCASEL = 2; // Select SOCA on period match
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;  // Generate pulse on 1st event
    EDIS;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
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
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // Ver pag. 355! e � o default...

    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK pre-scaler = /1

    EPwm2Regs.TBCTL.bit.CTRMODE = 2; // Modo Up-down (sim�trico)

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
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1; // Ver pag. 355! e � o default...

    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK pre-scaler = /1

    EPwm3Regs.TBCTL.bit.CTRMODE = 2; // Modo Up-down (sim�trico)

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

__interrupt void adca1_isr(void)
{

    tempo += Period_Timer;

    if (tempo > 2 && Flag_PWM == 0)
    {
        EPwm1Regs.DBCTL.bit.OUTSWAP = 0x00;
        EPwm2Regs.DBCTL.bit.OUTSWAP = 0x00;
        EPwm3Regs.DBCTL.bit.OUTSWAP = 0x00;
        Flag_PWM = 1;
        dutyCycle = PWM_TBPRD / 2;
    }

    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    if (GpioDataRegs.GPBDAT.bit.GPIO59 == 1)
    {
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    }

    else
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    }

    senoide = 2048 * cos(2 * 3.141592 * 60 * tempo) + 2048;
    DacbRegs.DACVALS.all = (Uint16)(senoide);

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

    igA = sensorA;
    igB = sensorB;
    igC = sensorC;
    vcapA = sensorD;
    vcapB = sensorE;
    vcapC = sensorF;
    itA = sensorG;
    itB = sensorH;
    itC = sensorI;
    vpacA = sensorJ;
    vpacB = sensorK;
    vpacC = sensorL;
    vdc = sensorM;

    // Update PWMs

    EPwm1Regs.CMPA.bit.CMPA = dutyCycle;
    EPwm2Regs.CMPA.bit.CMPA = dutyCycle;
    EPwm3Regs.CMPA.bit.CMPA = dutyCycle;

    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;

    // Transformada de clarke
    VAlfa = 0.6666666666 * (vpacA - vpacB * 0.5 - vpacC * 0.5);
    VBeta = 0.5773502691896 * (vpacB - vpacC);

    iAlfa = 0.6666666666 * (igA - igB * 0.5 - igC * 0.5);
    iBeta = 0.5773502691896 * (igB - igC);
    // Transformada dq
    cos_angulo = cos(angulo);
    sin_angulo = sin(angulo);
    V_d = (VAlfa * cos_angulo) + (VBeta * sin_angulo);
    V_q = -(VAlfa * sin_angulo) + (VBeta * cos_angulo);
    I_d = (iAlfa * cos_angulo) + (iBeta * sin_angulo);
    I_q = -(iAlfa * sin_angulo) + (iBeta * cos_angulo);

    // PLL
    int_srf += V_q * Period_Timer;
    omega = Kp_pll * V_q + Ki_pll * int_srf + 377;
    angulo += Period_Timer * omega;

    if (angulo < 0)
    {
        angulo = 0;
    }
    if (angulo > 6.28318530)
    {
        angulo -= 6.28318530;
    }
    //    angulo_saida = angulo*600;
    //
    //    DacaRegs.DACVALS.all = (Uint16)(angulo_saida);
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear ADC INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
