#include "F28x_Project.h"
#include "math.h"

// Definicoes da geracao do PWM
#define PWM_TBPRD 5000 // Define a freq. de amostragem PWMTBPRD = (10000*2500)/freq_desejada   (10kHz => 2500) [pag. 1914]
#define PWM_CMPR25 0   // PWM initial duty cycle = 25%

// PWM
Uint16 Periodo_PWM = 100000000*0.5*0.5*0.0001;           // 0.5(up_down)*Freq do CPU * Period da interrupção (em sec)
Uint16 dutyCycle = 750;

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
float vdc = 0;

// Variaveis manipuladas no controle

// PLL
float int_srf=0, omega=0, angulo=0, Kp_pll=2.97,Ki_pll=793;
int16 angulo_saida;

// Transformada de Clarke
float vAlfa = 0, vBeta = 0, iAlfa = 0, iBeta = 0;

// Transformdada de Park
float I_d = 0, I_q = 0;
float V_d=0, V_q=0, cos_angulo=0, sin_angulo=0;

// Transformada inversa de Park
float uAlfa= 0, uBeta = 0;

// Transformada inversa de Clarke
float ua = 0, ub = 0, uc = 0;

// Malha de controle
float erro_id = 0, erro_iq = 0, inte_id = 0, inte_iq = 0, ud_ref = 0, uq_ref = 0, erro_vdc = 0, inte_vdc = 0;

// Set-points
float id_ref = 0;
float iq_ref = 0;
float vdc_ref1 = 311;
float vdc_ref2 = 450;

// Ganhos controlador PI
float Ki_cont = 200; // 0.1ohm
float Kp_cont = 2.2;//0.1;  tau_i = 0.5ms

float Ki_vdc = 0.0004*100;
float Kp_vdc = 0.00001396*10;

// Filtro VDC SM
float vdc_1 = 0, vdc_filter = 0;


// Conversao PWM
Uint16 cmpA_value = 0, cmpB_value = 0, cmpC_value = 0;

// Teste normalização
float teste=1;

// Declaracao de Funcoes
__interrupt void adca1_isr(void);
void ConfigureEPWM1(void);
void ConfigureDAC(void);
void ConfigureADC(void);
void ConfiguraPWM1(void);
void ConfiguraPWM2(void);
void ConfiguraPWM3(void);

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

    ConfigureDAC();

    ConfigureADC();


    // Passo 3 - Clear nas interrupcoes e inicializar PIE vector table
    // Disabilita interrupcoes da CPU
    DINT;

    InitPieCtrl();
    IER=0;
    IFR=0;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    EDIS;


    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Mapeia a int para ADCA1

    IER |= M_INT1;

    EINT;
    ERTM;

    // Loop infinito
    while(1)
    {

    }
}

__interrupt void adca1_isr(void)
{
    static int InterruptCount=0; // Variavel de debug
    InterruptCount++;

    static float tempo = 0, delt = 0.0001;  //delt é o período de amostragem
    tempo += delt; // Incrementa tempo


    // Gera senoide
    //DacbRegs.DACVALS.all = (Uint16)(2048 + 2000*sin(2*3.141592*60*tempo));


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
    int Conversao_OFFSET = 0;
    float Conversao_Ganho = 0;


    Conversao_OFFSET = (sensorA - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.48828125;//0.244140625;
    igA = Conversao_Ganho;

    Conversao_OFFSET = (sensorB - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.48828125;
    igB = Conversao_Ganho;

    Conversao_OFFSET = (sensorC - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.48828125;
    igC = Conversao_Ganho;


    Conversao_OFFSET = (sensorD - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.09765625;
    vcapA = Conversao_Ganho;

    Conversao_OFFSET = (sensorE - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.09765625;
    vcapB = Conversao_Ganho;

    Conversao_OFFSET = (sensorF - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.09765625;
    vcapC = Conversao_Ganho;

    Conversao_OFFSET = (sensorG - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.48828125;
    itA = Conversao_Ganho;

    Conversao_OFFSET = (sensorH - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.48828125;
    itB = Conversao_Ganho;

    Conversao_OFFSET = (sensorI - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.48828125;
    itC = Conversao_Ganho;


    Conversao_OFFSET = (sensorJ - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.09765625;
    vpacA = Conversao_Ganho;

    Conversao_OFFSET = (sensorK - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.09765625;
    vpacB = Conversao_Ganho;

    Conversao_OFFSET = (sensorL - 2048);
    Conversao_Ganho = (float)(Conversao_OFFSET)*0.09765625;
    vpacC = Conversao_Ganho;

    vdc = (float)(sensorM)*0.2442002;


    // Filtra VDC

    vdc_filter = 0.99252*vdc_1 + 0.00748*vdc;
    vdc_1 = vdc_filter;

    // Transformada de Clarke
    vAlfa = 0.6666666666 * (vpacA - vpacB * 0.5 - vpacC * 0.5);
    vBeta = 0.5773502691896 * (vpacB - vpacC);

    iAlfa = 0.6666666666 * (igA - igB * 0.5 - igC * 0.5);
    iBeta = 0.5773502691896 * (igB - igC);

    // Transformada de Park
    cos_angulo = cos(angulo);
    sin_angulo = sin(angulo);
    V_d = (vAlfa * cos_angulo) + (vBeta * sin_angulo);
    V_q = -(vAlfa * sin_angulo) + (vBeta * cos_angulo);
    I_d = (iAlfa * cos_angulo) + (iBeta * sin_angulo);
    I_q = -(iAlfa * sin_angulo) + (iBeta * cos_angulo);

    // PLL
    int_srf += V_q * delt; //Integral de V_q
    omega = Kp_pll * V_q + Ki_pll * int_srf + 377;
    angulo += delt * omega;

    // Garante dente de serra
    if (angulo < 0)
    {
       angulo = 0;
    }
    if (angulo > 6.28318530)
    {
       angulo -= 6.28318530;
    }
       angulo_saida = angulo*600;

    // Extrai resultado do angulo do PLL
    //DacaRegs.DACVALS.all = (Uint16)(angulo_saida);


   // Realiza controle




   if (V_d > 170){ // PLL Estabilizou

       GpioDataRegs.GPASET.bit.GPIO7 = 1; // Conectou a rede

      if (vdc_filter > 300 && GpioDataRegs.GPADAT.bit.GPIO8 == 0 ){

          GpioDataRegs.GPASET.bit.GPIO8 = 1; // Conectou a rede
      }

       if (tempo > 15 && GpioDataRegs.GPADAT.bit.GPIO6 == 0){
           GpioDataRegs.GPASET.bit.GPIO6 = 1;
       }
       if(vdc_ref1 < vdc_ref2){

           vdc_ref1 += 0.001;
       }
       else if(vdc_ref1 >= vdc_ref2)
       {
           vdc_ref1 = vdc_ref2;
       }

       erro_vdc = -1 * (vdc_ref1 * vdc_ref1 - vdc * vdc);
       inte_vdc += erro_vdc * delt;

       id_ref = erro_vdc * Kp_vdc + inte_vdc * Ki_vdc;

       // Erros
       erro_id = id_ref - I_d;
       erro_iq = iq_ref - I_q;
       // Integradores
       inte_id += erro_id * delt;
       inte_iq += erro_iq * delt;
       // Ação de controle
       ud_ref = Kp_cont * erro_id + Ki_cont * inte_id; //Possibilidade de feedfoward
       uq_ref = Kp_cont * erro_iq + Ki_cont * inte_iq;

       //ud_ref = V_d;
       //uq_ref = V_q;

       // Transformada de Park inversa

       uAlfa = (2./vdc) * ((ud_ref * cos_angulo) - (uq_ref * sin_angulo));
       uBeta = (2./vdc) * ((ud_ref * sin_angulo) + (uq_ref * cos_angulo));

       // Transformda de Clarke inversa
       ua = uAlfa;
       ub = -uAlfa * 0.5 + 0.8660254037844 * uBeta;
       uc = -uAlfa * 0.5 - 0.8660254037844 * uBeta;



       DacaRegs.DACVALS.all = (Uint16)vdc_filter*5;//(Uint16)(I_q*10);
       DacbRegs.DACVALS.all = (Uint16)vdc*5;
       // Update PWMs

       // Normaliza para o intervalo de [0, PWM_TBPRD] sabendo que a tensão máxima de saída é +-0.5
       //cmpA_value = (Uint16)((ua + 1) * PWM_TBPRD/2.);
       //cmpB_value = (Uint16)((ub + 1) * PWM_TBPRD/2.);
       //cmpC_value = (Uint16)((uc + 1) * PWM_TBPRD/2.);

       cmpA_value = (Uint16)((-ua + 1) * PWM_TBPRD/2.);
       cmpB_value = (Uint16)((-ub + 1) * PWM_TBPRD/2.);
       cmpC_value = (Uint16)((-uc + 1) * PWM_TBPRD/2.);

       EPwm1Regs.CMPA.bit.CMPA = cmpA_value;
       EPwm2Regs.CMPA.bit.CMPA = cmpB_value;
       EPwm3Regs.CMPA.bit.CMPA = cmpC_value;


  }


   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear ADC INT1 flag
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;


}


void ConfigureDAC(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

    DacbRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC
    EDIS;
}

void ConfigureADC(void)
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  // ADCINA2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19; // sample duration of 20 SYSCLK cycles
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
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable INT1 flag
    EDIS;

    EALLOW;

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

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

        AdccRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

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

                AdcdRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
                AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
                AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

                AdcdRegs.ADCSOC0CTL.bit.CHSEL = 14;   // SOC0 will convert pin A2
                    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;  // Sample window is 100 SYSCLK cycles
                    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Trigger on ePWM8 SOCA



         EDIS;


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
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; //TROCADO
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

