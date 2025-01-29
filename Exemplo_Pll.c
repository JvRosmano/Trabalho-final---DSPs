#include <Stdlib.h>
#include <String.h>

int g_nInputNodes = 0;
int g_nOutputNodes = 0;

int g_nStepCount = 0;

static double fs = 21000;      // Frequencia de comutacao
static double fa = 21000;      // Frequencia de amostragem
static double delta = 1. / fa; // Per odo de amostragem
static double Start = 0;
static double periodo_amostragem = 0, pi = 3.1415926535;

//----- Entradas
static double ia = 0, ib = 0, ic = 0, Va = 0, Vb = 0, Vc = 0, Vdc = 400, triangular_PSIM = 0;

//-----Transformada Clarke
static double Valfa = 0, Vbeta = 0, Ialfa = 0, Ibeta = 0;

//----- SOGI-BPF
static double z1 = 0, k = 1.41, Valfa_sogi = 0, Vqalfa_sogi = 0, wr = 377, inte_a = 0;
static double z2 = 0, Vbeta_sogi = 0, inte_b = 0, Vqbeta_sogi = 0;

//----- Detec  o sequencias
static double Valfa_pos = 0, Vbeta_pos = 0, Valfa_neg = 0, Vbeta_neg = 0;
static double Ialfa_pos = 0, Ibeta_pos = 0;

//-----Transformada Park
static double cos_angulo = 0, sin_angulo = 0;
static double Vd = 0, Vq = 0, Id = 0, Iq = 0, Id_pos = 0, Iq_pos = 0;
static double Vd_pos = 0, Vq_pos = 0, Vd_neg = 0, Vq_neg = 0;

//----- Correntes de refer ncia
static double Id_ref = 0, Iq_ref = 0;
static double L = 1e-3, R = 0.3, tau_i = 0.5e-3;
static double Kp_i = L / tau_i, Ki_i = R / tau_i;

//---- PLL
static double Kp_pll = 2.97, Ki_pll = 791, int_srf = 0, omega = 377, angulo = 0;

//---- Controlador de tensão CC
static double Vdc_ref = 400, erro_Vdc = 0, inte_Vdc = 0, omega_n = 10, xi = 1, C_dc = 6e-3, Kp_Vdc = 1, Ki_Vdc = 1;

//----- Controladores de corrente
static double inte_id, erro_id, ud_ref, inte_iq, erro_iq, uq_ref;
static double aux_VDC = 1, Valfa_norm, Vbeta_norm;

//----- SVM
static double v_alfa, v_beta, Va_svm, Vb_svm, Vc_svm;
static double Sector = 1, Ta, Tb, Tc, t1, t2, Ta1, Tb1, Tc1;
static double S1 = 0, S2 = 0, S3 = 0, S4 = 0, S5 = 0, S6 = 0;
static double square = 1, triang = 0, TESTE = 0;

/////////////////////////////////////////////////////////////////////
// FUNCTION: SimulationStep
//   This function runs at every time step.
// double t: (read only) time
// double delt: (read only) time step as in Simulation control
// double *in: (read only) zero based array of input values. in[0] is the first node, in[1] second input...
// double *out: (write only) zero based array of output values. out[0] is the first node, out[1] second output...
// int *pnError: (write only)  assign  *pnError = 1;  if there is an error and set the error message in szErrorMsg
//    strcpy(szErrorMsg, "Error message here...");
// DO NOT CHANGE THE NAME OR PARAMETERS OF THIS FUNCTION
void SimulationStep(
    double t, double delt, double *in, double *out,
    int *pnError, char *szErrorMsg,
    void **reserved_UserData, int reserved_ThreadIndex, void *reserved_AppPtr)
{
    g_nStepCount++;

    // In case of error, uncomment next two lines. Set *pnError to 1 and copy Error message to szErrorMsg
    //*pnError=1;
    // strcpy(szErrorMsg, "Place Error description here.");

    // -----------------------------------------Entradas

    Va = in[0];
    Vb = in[1];
    Vc = in[2];
    ia = in[3];
    ib = in[4];
    ic = in[5];
    Vdc = in[6];

    Start = 0.01; // Para esperar o PLL entrar em regime

    // Uso esse if ao inv s do bloco ZOH para fazer a amostragem, pois desse jeito o time step (no relogio)
    // n o   modificado com a mudan a do per odo de amostragem

    if (periodo_amostragem >= delta)
    { //--------------------------------------------------------------------------//

        //---------------------------------------- Transformada de Clarke

        // Invariante em tens o
        Valfa = 0.6666666666 * (Va - Vb * 0.5 - Vc * 0.5);
        Vbeta = 0.5773502691896 * (Vb - Vc);

        Ialfa = 0.6666666666 * (ia - ib * 0.5 - ic * 0.5);
        Ibeta = 0.5773502691896 * (ib - ic);

        //---------------------------------------- DSOGI

        z1 = k * (Valfa * 0.5 - Valfa_sogi);
        Valfa_sogi += (z1 - Vqalfa_sogi) * wr * delta;
        inte_a += Valfa_sogi * delta;
        Vqalfa_sogi = wr * inte_a;

        z2 = k * (Vbeta * 0.5 - Vbeta_sogi);
        Vbeta_sogi += (z2 - Vqbeta_sogi) * wr * delta;
        inte_b += Vbeta_sogi * delta;
        Vqbeta_sogi = wr * inte_b;

        //---------------------------------------- Detec  o das Sequ ncias

        Valfa_pos = Valfa_sogi - Vqbeta_sogi;
        Vbeta_pos = Vqalfa_sogi + Vbeta_sogi;
        Valfa_neg = Vbeta_sogi - Vqalfa_sogi;
        Vbeta_neg = Vqbeta_sogi + Valfa_sogi;

        //----------------------------------------  Transformada de Park (AlfaBeta/dq)

        cos_angulo = cos(angulo);
        sin_angulo = sin(angulo);

        Vd = (Valfa * cos_angulo) + (Vbeta * sin_angulo);
        Vq = -(Valfa * sin_angulo) + (Vbeta * cos_angulo);
        Id = (Ialfa * cos_angulo) + (Ibeta * sin_angulo);
        Iq = -(Ialfa * sin_angulo) + (Ibeta * cos_angulo);

        Vd_pos = (Valfa_pos * cos_angulo) + (Vbeta_pos * sin_angulo);
        Vq_pos = -(Valfa_pos * sin_angulo) + (Vbeta_pos * cos_angulo);
        Vd_neg = (cos_angulo * Valfa_neg) + (-sin_angulo * Vbeta_neg);
        Vq_neg = -(sin_angulo * Valfa_neg) + (cos_angulo * Vbeta_neg);

        //-----  PLL

        Kp_pll = 2.93;
        Ki_pll = 791;

        int_srf += Vq_pos * delta;
        omega = Kp_pll * Vq_pos + Ki_pll * int_srf + 377;
        angulo += delta * omega;

        if (angulo < 0)
        {
            angulo = 0;
        }
        if (angulo > 6.28318530)
        {
            angulo -= 6.28318530;
        }

        //------ Malha de controle

        if (t > Start)
        {

            //------ C lculo dos ganhos
            C_dc = 6e-3;
            xi = 1;
            omega_n = 2 * pi * 10;
            L = 1e-3;
            R = 0.3;
            tau_i = 0.5e-3;

            Kp_Vdc = (2 * C_dc * xi * omega_n) / (3 * 180); // 13.96e-04
            Ki_Vdc = C_dc * omega_n * omega_n / (3 * Vd);   // 0.04

            Kp_i = L / tau_i;
            Ki_i = R / tau_i;

            //------ Malha de controle da tensão CC

            erro_Vdc = -1 * (Vdc_ref * Vdc_ref - Vdc * Vdc); // Invertido por causa do sentido da modelagem
            inte_Vdc += erro_Vdc * delta;

            Id_ref = erro_Vdc * Kp_Vdc + inte_Vdc * Ki_Vdc;

            //------ Controladores de Corrente (PI)

            // Referencias
            if (t > 0.02)
            {
                Iq_ref = 20;
            }

            if (t > 0.1)
            {
                Iq_ref = 20;
            }

            if (t > 0.15)
            {
                Vdc_ref = 420;
            }

            erro_id = Id_ref - Id; // C lculo erro Id
            erro_iq = Iq_ref - Iq; // C lculo erro Iq

            //----- PI Id Fundamental
            inte_id += erro_id * delta;
            ud_ref = Kp_i * erro_id + Ki_i * inte_id - Iq * 0.00125 * 377 + Vd;

            //----- PI Iq Fundamental
            inte_iq += erro_iq * delta;
            uq_ref = Kp_i * erro_iq + Ki_i * inte_iq + Id * 0.00125 * 377 + Vq;

            aux_VDC = 1. / Vdc;
            Valfa_norm = 1.732050807 * aux_VDC * (ud_ref * cos_angulo - uq_ref * sin_angulo);
            Vbeta_norm = 1.732050807 * aux_VDC * (ud_ref * sin_angulo + uq_ref * cos_angulo);

            // In cio do SVPWM
            // --------------------------------------Transformada inversa de Clarke
            v_alfa = Valfa_norm;
            v_beta = Vbeta_norm;

            Va_svm = v_beta;
            Vb_svm = -v_beta * 0.5 + 0.8660254037844 * v_alfa;
            Vc_svm = -v_beta * 0.5 - 0.8660254037844 * v_alfa;

            // -------------------------------------- Determina  o do SETOR
            Sector = 0;
            if (Va_svm > 0)
                Sector = 1;
            if (Vb_svm > 0)
                Sector += 2;
            if (Vc_svm > 0)
                Sector += 4;

            Va_svm = v_beta;
            Vb_svm = v_beta * 0.5 + 0.8660254037844 * v_alfa;
            Vc_svm = v_beta * 0.5 - 0.8660254037844 * v_alfa;

            // -------------------------------------- Atribui  o do tempo de chaveamento de acordo com o SETOR
            if (Sector == 0)
            {
                Ta = 0.5;
                Tb = 0.5;
                Tc = 0.5;
            }

            if (Sector == 1)
            {
                t1 = Vc_svm;
                t2 = Vb_svm;
                Tb = (1. - t1 - t2) * 0.5;
                Ta = Tb + t1;
                Tc = Ta + t2;
            }

            if (Sector == 2)
            {
                t1 = Vb_svm;
                t2 = -Va_svm;
                Ta = (1. - t1 - t2) * 0.5;
                Tc = Ta + t1;
                Tb = Tc + t2;
            }

            if (Sector == 3)
            {
                t1 = -Vc_svm;
                t2 = Va_svm;
                Ta = (1. - t1 - t2) * 0.5;
                Tb = Ta + t1;
                Tc = Tb + t2;
            }

            if (Sector == 4)
            {
                t1 = -Va_svm;
                t2 = Vc_svm;
                Tc = (1. - t1 - t2) * 0.5;
                Tb = Tc + t1;
                Ta = Tb + t2;
            }

            if (Sector == 5)
            {
                t1 = Va_svm;
                t2 = -Vb_svm;
                Tb = (1. - t1 - t2) * 0.5;
                Tc = Tb + t1;
                Ta = Tc + t2;
            }

            if (Sector == 6)
            {
                t1 = -Vb_svm;
                t2 = -Vc_svm;
                Tc = (1. - t1 - t2) * 0.5;
                Ta = Tc + t1;
                Tb = Ta + t2;
            }

            Ta1 = -v_alfa;
            Tb1 = -(-v_alfa * 0.5 + 0.8660254037844 * v_beta);
            Tc1 = -(-v_alfa * 0.5 - 0.8660254037844 * v_beta);

            Ta1 = 2 * (Ta - 0.5);
            Tb1 = 2 * (Tb - 0.5);
            Tc1 = 2 * (Tc - 0.5);
        }

        periodo_amostragem = 0;
    } // final da amostragem
    else
    {
        periodo_amostragem += delt;
    } //-------------------------------------------------------------------------//

    if (t > Start)
    {

        // -------------------------------------- Gera  o de uma Onda triangular (poderia ter sido feita fora do C)

        if ((sin(2 * pi * fs * t - pi * 0.5)) > 0)
        {
            square = +1;
        }
        else
        {
            square = -1;
        }

        triang += 4 * fs * square * delt;

        // --------------------------------------Gera  o dos Pulsos de Disparo

        if (triang > Ta1)
        {
            S1 = 1;
            S4 = 0;
        }
        else
        {
            S1 = 0;
            S4 = 1;
        }

        if (triang > Tb1)
        {
            S3 = 1;
            S6 = 0;
        }
        else
        {
            S3 = 0;
            S6 = 1;
        }

        if (triang > Tc1)
        {
            S5 = 1;
            S2 = 0;
        }
        else
        {
            S5 = 0;
            S2 = 1;
        }
    }

    // Atribui  o das sa das
    // ----------------------------------------- Sa das

    out[0] = S1;
    out[1] = S2;
    out[2] = S3;
    out[3] = S4;
    out[4] = S5;
    out[5] = S6;
    out[6] = Vd;
    out[7] = Vq;
    out[8] = Id;
    out[9] = Iq;
    out[10] = angulo;
}

/////////////////////////////////////////////////////////////////////
// FUNCTION: SimulationBegin
//   Initialization function. This function runs once at the beginning of simulation
//   For parameter sweep or AC sweep simulation, this function runs at the beginning of each simulation cycle.
//   Use this function to initialize static or global variables.
// const char *szId: (read only) Name of the C-block
// int nInputCount: (read only) Number of input nodes
// int nOutputCount: (read only) Number of output nodes
// int nParameterCount: (read only) Number of parameters is always zero for C-Blocks.  Ignore nParameterCount and pszParameters
// int *pnError: (write only)  assign  *pnError = 1;  if there is an error and set the error message in szErrorMsg
//    strcpy(szErrorMsg, "Error message here...");
// DO NOT CHANGE THE NAME OR PARAMETERS OF THIS FUNCTION
void SimulationBegin(
    const char *szId, int nInputCount, int nOutputCount,
    int nParameterCount, const char **pszParameters,
    int *pnError, char *szErrorMsg,
    void **reserved_UserData, int reserved_ThreadIndex, void *reserved_AppPtr)
{
    g_nInputNodes = nInputCount;
    g_nOutputNodes = nOutputCount;

    // In case of error, uncomment next two lines. Set *pnError to 1 and copy Error message to szErrorMsg
    //*pnError=1;
    // strcpy(szErrorMsg, "Place Error description here.");
}

/////////////////////////////////////////////////////////////////////
// FUNCTION: SimulationEnd
//   Termination function. This function runs once at the end of simulation
//   For parameter sweep or AC sweep simulation, this function runs at the end of each simulation cycle.
//   Use this function to de-allocate any allocated memory or to save the result of simulation in an alternate file.
// Ignore all parameters for C-block
// DO NOT CHANGE THE NAME OR PARAMETERS OF THIS FUNCTION
void SimulationEnd(const char szId, void *reserved_UserData, int reserved_ThreadIndex, void *reserved_AppPtr)
{
}