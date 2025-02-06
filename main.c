#include "F28x_Project.h"
#include "math.h"
#include "Peripherals.h"
#include "Utils.h"
// #include "stateMachine.h"

// Sensores (Leem do ADCResultsRegs)
Uint16 sensorA = 0, sensorB = 0, sensorC = 0, sensorD = 0;
Uint16 sensorE = 0, sensorF = 0, sensorG = 0, sensorH = 0;
Uint16 sensorI = 0, sensorJ = 0, sensorK = 0, sensorL = 0;
Uint16 sensorM = 0;

// Variaveis medidas da planta
float igA = 0, igB = 0, igC = 0;
float vcapA = 0, vcapB = 0, vcapC = 0;
float itA = 0, itB = 0, itC = 0;
float vpacA = 0, vpacB = 0, vpacC = 0;
float vdc = 0, vdcAnterior = 0;
float vdc_filter = 0;
// Variaveis das transformadas
Clarke I_Clarke, V_Clarke, U_Clarke;
Park I_Park, V_Park;
// Variaveis manipuladas no controle
float Vdc_U = 311, Vdc_Ref = 450;
float iq_ref = 0, id_ref = 0;
float uq_ref = 0, ud_ref = 0;
// PLL
float omega = 0, angulo = 0;
int16 angulo_saida;
// Transformada inversa de Clarke
float ua = 0, ub = 0, uc = 0;

//// Máquina de estados
// StateMachine stateMachine;
//// Declaracao de Funcoes
// void App_init(StateMachine *machine);
// void App_transPLL2SHUNT(void);
// void App_transSHUNT2BYPASS(void);
// void App_transBYPASS2DCBUS(void);
// void App_transDCBUS2WORKING(void);
// void App_transAnyToError(void);
__interrupt void adca1_isr(void);

void main(void)
{

    // Passo 1 - Inicializa sistema de controle

    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    // Passo 2 - Inicializa GPIO
    InitGpio();

    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1; // Corrente
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1; // Switch rede
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1; // Switch Bypass
    EDIS;

    InitEPwm1Gpio(); // Lembrar do F2837xD_EPwm.c (EPWM1)
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    ConfiguraPWM1();
    ConfiguraPWM2();
    ConfiguraPWM3();
    ConfiguraDAC();
    ConfiguraADC();

    // Passo 3 - Clear nas interrupcoes e inicializar PIE vector table
    // Disabilita interrupcoes da CPU
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

    // Loop infinito
    while (1)
    {
    }
}

__interrupt void adca1_isr(void)
{
    static int InterruptCount = 0; // Variavel de debug
    InterruptCount++;

    static float tempo = 0;
    tempo += DELT; // Incrementa tempo
    // Executa estados
    //    SM_main(&stateMachine);
    // Gera senoide
    DacbRegs.DACVALS.all = (Uint16)(2048 + 2000 * sin(2 * 3.141592 * 60 * tempo));

    // Sensores recebem os valores do ADCs
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

    // Conversao de bit para valor real das variaveis

    igA = CondicionaSinal(sensorA, 2048, 0.48828125);

    igB = CondicionaSinal(sensorB, 2048, 0.48828125);
    igC = CondicionaSinal(sensorC, 2048, 0.48828125);

    vcapA = CondicionaSinal(sensorD, 2048, 0.09765625);
    vcapB = CondicionaSinal(sensorE, 2048, 0.09765625);
    vcapC = CondicionaSinal(sensorF, 2048, 0.09765625);

    itA = CondicionaSinal(sensorG, 2048, 0.48828125);
    itB = CondicionaSinal(sensorH, 2048, 0.48828125);
    itC = CondicionaSinal(sensorI, 2048, 0.48828125);

    vpacA = CondicionaSinal(sensorJ, 2048, 0.09765625);
    vpacB = CondicionaSinal(sensorK, 2048, 0.09765625);
    vpacC = CondicionaSinal(sensorL, 2048, 0.09765625);

    vdc = CondicionaSinal(sensorM, 0, 0.2442002);

    LPFilter(&vdc_filter, vdc, 0.99252);

    // Transformada de Clarke
    clarkeTransform(&V_Clarke, vpacA, vpacB, vpacC);
    clarkeTransform(&I_Clarke, igA, igB, igC);

    // Transformada de Park
    parkTransform(&V_Park, &V_Clarke, angulo);
    parkTransform(&I_Park, &I_Clarke, angulo);

    // PLL
    executePLL(&V_Park, &omega, &angulo);

    // Realiza controle
    // PLL estabilizou
    if (V_Park.D > 170)
    {
        GpioDataRegs.GPASET.bit.GPIO7 = 1; // Conectou a rede

        if (vdc_filter > 300 && GpioDataRegs.GPADAT.bit.GPIO8 == 0)
        {

            GpioDataRegs.GPASET.bit.GPIO8 = 1; // Conectou a rede
        }
        if (tempo > 15 && GpioDataRegs.GPADAT.bit.GPIO6 == 0)
        {
            GpioDataRegs.GPASET.bit.GPIO6 = 1;
        }
        //        Mudar de estado

        dcVoltageControl(&id_ref, &Vdc_U, vdc, Vdc_Ref);
        currentControl(&I_Park, id_ref, iq_ref, &ud_ref, &uq_ref);
        inverseParkTransform(&U_Clarke, vdc, ud_ref, uq_ref);
        inverseClarkeTransform(&U_Clarke, &ua, &ub, &uc);

        // Update PWMs
        EPwm1Regs.CMPA.bit.CMPA = pwmConvert(ua);
        EPwm2Regs.CMPA.bit.CMPA = pwmConvert(ub);
        EPwm3Regs.CMPA.bit.CMPA = pwmConvert(uc);
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear ADC INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// void App_init(StateMachine *machine)
//{
//     // check parameters
//     assert(NULL != machine);
//     SM_init(machine);
//     machine->transPLL2SHUNT = ((TransitionFunc)((void *)App_transPLL2SHUNT));
//     machine->transSHUNT2BYPASS = ((TransitionFunc)((void *)App_transSHUNT2BYPASS));
//     machine->transBYPASS2DCBUS = ((TransitionFunc)((void *)App_transBYPASS2DCBUS));
//     machine->transDCBUS2WORKING = ((TransitionFunc)((void *)App_transDCBUS2WORKING));
//     machine->transAnyToError = ((TransitionFunc)((void *)App_transAnyToError));
// }
// void App_transPLL2SHUNT(void)
//{
// }
// void App_transSHUNT2BYPASS(void)
//{
// }
// void App_transBYPASS2DCBUS(void)
//{
// }
// void App_transDCBUS2WORKING(void)
//{
// }
// void App_transAnyToError(void)
//{
// }
