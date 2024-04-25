

/**
 * main.c
 */
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "MyLaunchpad.h"

//#define PROJECT_DEBUG

#define SQRT_3_2    0.866025403784439
#define SQRT_3      1.732050807568877
#define PI_DIV_2    1.570796326794897
#define PI_DIV_3    1.047197551196598
#define PIx2_DIV_3  2.094395102393195
#define SQRT_2      1.414213562373095
#define Wn          314.1592653589793

#define GAINV 0.042

#define GAINI 0.044

//#define M           0.8
//#define R_Rate      0.031415926535898

#define I_LIMIT     10
#define I_OK        7
#define VRMS        14.1421356237
#define VMAX_REF    20
#define P           50
//#define IRMS        P/(VRMS*3)
#define VDC         150
#define VACMAX      43


//#define DSOGI
#define SRF

// PLL variable
volatile float VA=0,VB=0,VC=0;
volatile float Val=0,Vbe=0;
volatile float Vd=0,Vq=0;          // Before LPF
volatile float V_d_=0,V_q_=0;      // After  LPF
volatile float Vmax = 0;
volatile float E_PD = 0;
volatile float V_LF = 0;
volatile float W_EST = 0;          // Before LPF
volatile float W_EST_ = 0;         // After LPF
volatile float THETA = 0;
volatile float F_EST=0;

const float BP_coefficient[] = {0.007790869964250,0,-0.007790869964250,-1.983439230477709,0.984418260071499};
volatile SECOND_ORDER_FILTER BPF_A={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER BPF_B={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER BPF_C={{0,0,0},{0,0,0}};

const float DQ_LP_COEFFICIENT[] = {0.015465039,0.015465039,-0.969069922};
FIRST_ORDER_FILTER D_LP_FILTER = {{0,0},{0,0}};
FIRST_ORDER_FILTER Q_LP_FILTER = {{0,0},{0,0}};

const float TEST_LP_COEFFICIENT[] = {0,-0.995012479192682,-0.995012479192682};
FIRST_ORDER_FILTER TEST_FILTER = {{0,0},{0,0}};

const float PI_COEFFICENT[] = {333.8,-332.8,-1};
FIRST_ORDER_FILTER PI_CONTROLLER = {{0,0},{0,0}};


const float IN_MOD_2PI_COEFFICIENT[] = {0.00005,0.00005,-1};
FIRST_ORDER_FILTER IN_MOD_2PI = {{0,0},{0,0}};


const float W0_2LP_COEFFICIENT[] = {0.0002413724953441133,0.0004827449906882266,0.0002413724953441133,-1.956009009166930,0.956974499148306};
SECOND_ORDER_FILTER W0_2ND_LPF = {{0,0,0},{0,0,0}};


// PLL variable

// Current controller variable
// num 10.628073085496045 -20.651262799921845 10.033383219283222
// den 1 -1.998613658894070 0.999600178616249
//10.628073085496045 -20.651262799921845 10.033383219283222
//1 -1.998613658894070 0.999600178616249

// ideal
// num 10.480462957566244 -20.655392033634250 10.185124619123492
// dem 1 -1.999013283022547 1.000000000000000
//const float PR_COEFF[] = {10.628073085496045,-20.651262799921845,10.033383219283222,-1.998613658894070,0.999600178616249};
//const float PR_COEFF[] = {10.923497604157761,-20.661422833964867,9.737957684719689,-1.999596939336042,0.999600080298037}; // ss
//const float PR_COEFF[] = {20.611402463707830,-37.562807499364354,16.969946112994638,-1.998613658894070,0.999600178616249};
//const float PR_COEFF[] = {10.480462957566244,-20.655392033634250,10.185124619123492,-1.999013283022547,1.000000000000000};

// WC = 10 rad/s
//const float PR_COEFF[] = {13.284194918573936,-20.644908519501435,7.360746035234540,-1.997998696421135,0.998001835388940};
const float PR_COEFF[] = {13.283469792268214,-20.634758991836420,7.361476234184991,-1.997016432778511,0.998002326315675};
// WC = 5rad/s
//const float PR_COEFF[] = {11.809231895053212,-20.655226655235420,8.846027210699615,-1.998997277825674,0.999000418362306};

//const float PR_COEFF[] = {10.150350206872798,-19.104001529736756,8.963081096446146,-1.998613658894070,0.999600178616249};
//volatile SECOND_ORDER_FILTER PR_A={{0,0,0},{0,0,0}};
//volatile SECOND_ORDER_FILTER PR_B={{0,0,0},{0,0,0}};
//volatile SECOND_ORDER_FILTER PR_C={{0,0,0},{0,0,0}};z

volatile float Ia_ref=0,Ib_ref=0,Ic_ref=0;
volatile float Ia_adc = 0, Ib_adc = 0, Ic_adc = 0;
volatile float Iam = 0, Ibm = 0, Icm = 0;
volatile float Err_a = 0, Err_b = 0, Err_c = 0;
volatile float Va_l=0,Vb_l=0,Vc_l=0;
volatile float Da = 0, Db = 0, Dc = 0;
volatile float Imax = 0;
volatile float Ic_test1 = 0;

// Alpha beta current vars
volatile float Ialpha_ref=0,Ibeta_ref=0;
volatile float Ialpha_measurement=0,Ibeta_measurement=0;
volatile float Err_alpha=0,Err_beta=0;
volatile float Valpha_l=0,Vbeta_l=0;

volatile SECOND_ORDER_FILTER PR_ALPHA = {{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER PR_BETA  = {{0,0,0},{0,0,0}};

// Current controller variable


volatile uint16_t TEST_THETA=0,TEST_EPD=0;
volatile float Real_Current = 0,volt = 0,Voltage_Sen=0;
volatile float AVG_F = 0;
volatile uint16_t Ecount = 0, Inphase = 0;
volatile float C_Vmax = 0;
volatile float Ia_test = 0, Ib_test = 0, Ic_test = 0;

volatile uint8_t s_count = 0,t_count = 0;
volatile float s_theta = 0;

// Enable Current controller variable
volatile uint8_t Ena_Current_count = 0;
volatile bool  OK2RUN = 0;
volatile bool OVER_CURRENT = 0;
// Enable Current controller variable

volatile uint16_t daint=0;
volatile uint16_t dbint=0;
volatile uint16_t dcint=0;

volatile int64_t avg[6]={0,0,0,0,0,0};
volatile float off[6] = {0,0,0,0,0,0};
volatile uint16_t nsample = 0;

// SPACE VECTOR
volatile float dal, dbe;
volatile float dTheta;
volatile uint8_t sector;
volatile float t1,t2,t0;
volatile float psa,psb,psc;
volatile float dMax;

volatile bool CONTROL = 1;
volatile float Id_ref = 1;
volatile float Iq_ref = 0;

//uint8_t i__;

#ifdef DSOGI

// SOGI variables
volatile float _Val=0,_qVal=0,_Vbe=0,_qVbe=0;
volatile float Val_pos=0,Vbe_pos=0;

const float Qz_coefficient[] = {0.0001725129025599240,0.0003450258051198481,0.0001725129025599240,-1.977059094702427,0.978034975048366};
const float Dz_coefficient[] = {0.010982512475817,0,-0.010982512475817,-1.977059094702427,0.978034975048366};
volatile SECOND_ORDER_FILTER Dz_ALPHA ={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER Qz_ALPHA ={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER Dz_BETA  ={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER Qz_BETA  ={{0,0,0},{0,0,0}};

#pragma FUNC_ALWAYS_INLINE(PLL)
inline void PLL(){
//    VA = AdcaResultRegs.ADCRESULT1;
//    VB = AdccResultRegs.ADCRESULT1;
//    VC = AdcbResultRegs.ADCRESULT1;
//
//    VA = (VA-2358.3)/60.216;   // y = 60.216x + 2358.3
//    VB = (VB-2236)/56.369;     // y = 56.369x + 2236
//    VC = (VC-2237.2)/53.47;    // y = 53.47x + 2237.2

//    Compute_2nd_Filter(&BPF_A, VA, BP_coefficient);
//    Compute_2nd_Filter(&BPF_B, VB, BP_coefficient);
//    Compute_2nd_Filter(&BPF_C, VC, BP_coefficient);
//    VA = BPF_A.y[0];
//    VB = BPF_B.y[0];
//    VC = BPF_C.y[0];

    // abc to alpha-beta
    Val = (2*VA - VB - VC)/3;
    Vbe = (SQRT_3*VB-SQRT_3*VC)/3;


    // SOGI
    Compute_2nd_Filter(&Dz_ALPHA, Val, Dz_coefficient);
    _Val = Dz_ALPHA.y[0];
    Compute_2nd_Filter(&Qz_ALPHA, Val, Qz_coefficient);
    _qVal = Qz_ALPHA.y[0];


    Compute_2nd_Filter(&Dz_BETA, Vbe, Dz_coefficient);
    _Vbe = Dz_BETA.y[0];
    Compute_2nd_Filter(&Qz_BETA, Vbe, Qz_coefficient);
    _qVbe = Qz_BETA.y[0];

    // SOGI


    // Instantaneous Symmetrical components calculation (ISC)
    Val_pos = (_Val-_qVbe)/2;
    Vbe_pos = (_qVal+_Vbe)/2;

    /// alpha-beta to dq
    Vd =  cos(THETA)*Val_pos+sin(THETA)*Vbe_pos;
    Vq = -sin(THETA)*Val_pos+cos(THETA)*Vbe_pos;

    // DQ_LPF
    Compute_1st_Filter(&D_LP_FILTER, Vd, DQ_LP_COEFFICIENT);    // d LPF
    Compute_1st_Filter(&Q_LP_FILTER, Vq, DQ_LP_COEFFICIENT);    // q LPF
    V_d_ = D_LP_FILTER.y[0];
    V_q_ = Q_LP_FILTER.y[0];

    // E_PD
    Vmax = sqrt(V_d_*V_d_ + V_q_*V_q_);
    E_PD = V_q_/(Vmax+0.00000001);   // Make sure not divide for 0

    // PI controller
    Compute_1st_Filter(&PI_CONTROLLER, E_PD, PI_COEFFICENT);
    V_LF = PI_CONTROLLER.y[0];

    // W0
    W_EST = Wn + V_LF;
    Compute_2nd_Filter(&W0_2ND_LPF, W_EST, W0_2LP_COEFFICIENT);
    W_EST_ = W0_2ND_LPF.y[0];

    // THETA
    Integrator_mod_2_pi(&IN_MOD_2PI, W_EST, IN_MOD_2PI_COEFFICIENT);
    THETA = IN_MOD_2PI.y[0];

    // F0
    F_EST = W_EST_/PIx2;
}
#endif

#ifdef SRF
#pragma FUNC_ALWAYS_INLINE(PLL)
inline void PLL(){

//    VA = AdcaResultRegs.ADCRESULT0;
//    VB = AdccResultRegs.ADCRESULT0;
//    VC = AdcbResultRegs.ADCRESULT0;
//
//    VA = VA - 2331; // Remove DC offset
//    VB = VB - 2372;
//    VC = VC - 2266;
//
//    VA = VA*GAINV;
//    VB = VB*GAINV;
//    VC = VC*GAINV;


//    VA = AdcaResultRegs.ADCRESULT1;
//    VB = AdccResultRegs.ADCRESULT1;
//    VC = AdcbResultRegs.ADCRESULT1;
//
//    VA = (VA-2358.3)/60.216;   // y = 60.216x + 2358.3
//    VB = (VB-2236)/56.369;     // y = 56.369x + 2236
//    VC = (VC-2237.2)/53.47;    // y = 53.47x + 2237.2

//    Compute_2nd_Filter(&BPF_A, VA, BP_coefficient);
//    Compute_2nd_Filter(&BPF_B, VB, BP_coefficient);
//    Compute_2nd_Filter(&BPF_C, VC, BP_coefficient);
//    VA = BPF_A.y[0];
//    VB = BPF_B.y[0];
//    VC = BPF_C.y[0];

//    VA = Iam;
//    VB = Ibm;
//    VC = Icm;

    // abc to alpha-beta
    Val = (2*VA - VB - VC)/3;
    Vbe = (VB-VC)/SQRT_3;

    // alpha-beta to dq
    Vd =  cos(THETA)*Val+sin(THETA)*Vbe;
    Vq = -sin(THETA)*Val+cos(THETA)*Vbe;

    // DQ_LPF
    Compute_1st_Filter(&D_LP_FILTER, Vd, DQ_LP_COEFFICIENT);    // d LPF
    Compute_1st_Filter(&Q_LP_FILTER, Vq, DQ_LP_COEFFICIENT);    // q LPF
    V_d_ = D_LP_FILTER.y[0];
    V_q_ = Q_LP_FILTER.y[0];

    // E_PD
    Vmax = sqrt(V_d_*V_d_ + V_q_*V_q_);
    E_PD = V_q_/(Vmax+0.00000001);   // Make sure not divide for 0

    // PI controller
    Compute_1st_Filter(&PI_CONTROLLER, E_PD, PI_COEFFICENT);
    V_LF = PI_CONTROLLER.y[0];

    // W0
    W_EST = Wn + V_LF;
    Compute_2nd_Filter(&W0_2ND_LPF, W_EST, W0_2LP_COEFFICIENT);
    W_EST_ = W0_2ND_LPF.y[0];

    // THETA
    Integrator_mod_2_pi(&IN_MOD_2PI, W_EST, IN_MOD_2PI_COEFFICIENT);
    THETA = IN_MOD_2PI.y[0];

    // F0
    F_EST = W_EST_/PIx2;
}

#ifdef PROJECT_DEBUG
SECOND_ORDER_FILTER W0_2ND_LPF_I = {{0,0,0},{0,0,0}};
FIRST_ORDER_FILTER IN_MOD_2PI_I = {{0,0},{0,0}};
FIRST_ORDER_FILTER PI_CONTROLLER_I = {{0,0},{0,0}};
FIRST_ORDER_FILTER D_LP_FILTER_I = {{0,0},{0,0}};
FIRST_ORDER_FILTER Q_LP_FILTER_I = {{0,0},{0,0}};

// PLL variable
//volatile float VA=0,VB=0,VC=0;
volatile float Ial=0,Ibe=0;
volatile float Id=0,Iq=0;          // Before LPF
volatile float I_d_=0,I_q_=0;      // After  LPF
volatile float Imax_I = 0;
volatile float E_PD_I = 0;
volatile float I_LF = 0;
volatile float W_EST_I = 0;          // Before LPF
volatile float W_EST__I = 0;         // After LPF
volatile float THETA_I = 0;

#pragma FUNC_ALWAYS_INLINE(PLL_I)
void PLL_I(){

//    // abc to alpha-beta
//    Ial = (2*Iam - Ibm - Icm)/3;
//    Ibe = (SQRT_3*Ibm-SQRT_3*Icm)/3;
//
//    // alpha-beta to dq
//    Id =  cos(THETA_I)*Ial+sin(THETA_I)*Ibe;
//    Iq = -sin(THETA_I)*Ial+cos(THETA_I)*Ibe;
//
//    // DQ_LPF
//    Compute_1st_Filter(&D_LP_FILTER_I, Id, DQ_LP_COEFFICIENT);    // d LPF
//    Compute_1st_Filter(&Q_LP_FILTER_I, Iq, DQ_LP_COEFFICIENT);    // q LPF
//    I_d_ = D_LP_FILTER_I.y[0];
//    I_q_ = Q_LP_FILTER_I.y[0];
//
//    // E_PD
//    Imax_I = sqrt(I_d_*I_d_ + I_q_*I_q_);
//    E_PD_I = I_q_/(Imax_I+0.00000001);   // Make sure not divide for 0
//
//    // PI controller
//    Compute_1st_Filter(&PI_CONTROLLER_I, E_PD_I, PI_COEFFICENT);
//    I_LF = PI_CONTROLLER_I.y[0];
//
//    // W0
//    W_EST_I = Wn + I_LF;
//    Compute_2nd_Filter(&W0_2ND_LPF_I, W_EST, W0_2LP_COEFFICIENT);
//    W_EST__I = W0_2ND_LPF_I.y[0];
//
//    // THETA
//    Integrator_mod_2_pi(&IN_MOD_2PI_I, W_EST, IN_MOD_2PI_COEFFICIENT);
//    THETA_I = IN_MOD_2PI_I.y[0];

}
#endif

#endif

inline bool Is_In_phase();

interrupt void adcb1_isr();

int main(void)

{
    InitSysCtrl();
    InitGpio();

    DINT;   //Disable Globe INT

    InitPieCtrl();
    // Disable and clear all INT CPU flag
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCB1_INT = &adcb1_isr; //function for ADCB interrupt 1
    EDIS;

    Init_ePWM();
    Init_ADC();
    Init_DAC();
    Init_RELAY();

//    ON_RELAY();
    OFF_RELAY();

    IER |= M_INT1;  //Enable group 1 interrupts
    EINT;           // Enable Global interrupt INTM

    PieCtrlRegs.PIEIER1.bit.INTx2 = 1; // Enable PIE interrupt ADCB1
    Dis_ePWM();
//    Imax = 1.0;
    Iq_ref = 0;
    Id_ref = 1;
    CONTROL = 1;

    while(1){
//        Dis_ePWM();
//        DELAY_US(500);
//        ReEna_ePWM();
//        DELAY_US(500);

        if((fabs(Iam) >= I_LIMIT) || (fabs(Ibm) >= I_LIMIT) || (fabs(Icm) >= I_LIMIT)){
            if(!OVER_CURRENT){
                OVER_CURRENT = 1;
                Dis_ePWM();
            }
        }
    }

}

//&& (fabs(Vmax-VMAX_REF) <= 2.5)

inline bool Is_In_phase(){
    if(Ecount == 10){
        Ecount = 0;
        if( (fabs(E_PD) <= 0.1) && (fabs(Vmax-VACMAX) <= 12))            Inphase = (Inphase<<1)|1;
        else                                                          Inphase = (Inphase<<1)|0;
    }else Ecount++;
    if(Inphase == 0xffff) return 1;

    return 0;
}

interrupt void adcb1_isr(){
    GpioDataRegs.GPBTOGGLE.bit.GPIO63 = 1;
    // Measurement
    Ia_adc = AdcaResultRegs.ADCRESULT1;
    Ib_adc = AdccResultRegs.ADCRESULT1;
    Ic_adc = AdcbResultRegs.ADCRESULT1;

    Ia_adc = Ia_adc - 2196.0 - off[3];
    Ib_adc = Ib_adc - 2275.0 - off[4];
    Ic_adc = Ic_adc - 2279.0 - off[5];

    VA = AdcaResultRegs.ADCRESULT0;
    VB = AdccResultRegs.ADCRESULT0;
    VC = AdcbResultRegs.ADCRESULT0;

    VA = VA - 2266 - off[0]; // Remove DC offset
    VB = VB - 2394 - off[1];
    VC = VC - 2351 - off[2];

    // Reset offset
    if(nsample<0xFFFF){
        nsample++;
        avg[0] += VA;
        avg[1] += VB;
        avg[2] += VC;
        avg[3] += Ia_adc;
        avg[4] += Ib_adc;
        avg[5] += Ic_adc;
        off[0] = off[1] = off[2] = off[3] = off[4] = off[5] = 0;
        if(nsample==0xFFFF){
            off[0] = avg[0]/0xFFFF;
            off[1] = avg[1]/0xFFFF;
            off[2] = avg[2]/0xFFFF;
            off[3] = avg[3]/0xFFFF;
            off[4] = avg[4]/0xFFFF;
            off[5] = avg[5]/0xFFFF;
        }
    }

    VA = VA*GAINV;
    VB = VB*GAINV;
    VC = VC*GAINV;

    Iam = (Ia_adc/60.216);   // y = 60.216x + 2358.3
    Ibm = (Ib_adc/56.369);     // y = 56.369x + 2236
    Icm = (Ic_adc/53.47);    // y = 53.47x + 2237.2

    // Measurement

    PLL();

//    Ia_ref = sin(THETA+PI_DIV_2);
//    Ib_ref = sin(THETA+PI_DIV_2-PIx2_DIV_3);
//    Ic_ref = sin(THETA+PI_DIV_2+PIx2_DIV_3);
//
//    ReEna_ePWM();
//    Set_CompareA(1, 0);
//    Set_CompareA(2, 0);
//    Set_CompareA(3, 2500);


    if(Ena_Current_count==10){
        Ena_Current_count=0;
        if ( (fabs(THETA-PI_DIV_2) <= 0.1)  && Is_In_phase() && (!OK2RUN)) OK2RUN = 1;
        else OK2RUN = Is_In_phase();
    }else Ena_Current_count++;

#ifdef  PROJECT_DEBUG
    DacbRegs.DACVALS.all = AdcaResultRegs.ADCRESULT0;
    DacaRegs.DACVALS.all = THETA*200;
//    PLL_I();
//    ReEna_ePWM();
//    EALLOW;
//    EPwm1Regs.TZCLR.bit.OST = 1;
//    EPwm2Regs.TZCLR.bit.OST = 1;
//    EPwm3Regs.TZCLR.bit.OST = 1;
//    EDIS;
//    Dis_ePWM();
//    // TESTING CURRENT SENSOR
//    Real_Current = AdcaResultRegs.ADCRESULT1;
//    Compute_1st_Filter(&TEST_LP_A, AdcaResultRegs.ADCRESULT1, TEST_LP_COEFF);
//    Compute_1st_Filter(&TEST_LP_B, AdccResultRegs.ADCRESULT1, TEST_LP_COEFF);
//    Compute_1st_Filter(&TEST_LP_C, AdcbResultRegs.ADCRESULT1, TEST_LP_COEFF);
//    // TESTING CURRENT SENSOR

//    s_count++; s_count%=200;
//    s_theta = M_PI*1.8*s_count/180;
//
//    Va_l = 0.9*sin(s_theta);
//    Vb_l = 0.9*sin(s_theta-PIx2_DIV_3);
//    Vc_l = 0.9*sin(s_theta+PIx2_DIV_3);

//    // Current controller
//    Imax = 0.7;
//    // Reference current
//    Ia_ref = sin(THETA+PI_DIV_2);
//    Ib_ref = sin(THETA+PI_DIV_2-PIx2_DIV_3);
//    Ic_ref = sin(THETA+PI_DIV_2+PIx2_DIV_3);

//     Duty cycle
//    Da = 2500+Va_l*2500;
//    Db = 2500+Vc_l*2500;
//    Dc = 2500+Vb_l*2500;
//
//    // Load duty cycle
//    Set_CompareA(1, 0);
//    Set_CompareA(2, 0);
//    Set_CompareA(3, 0);

//    TEST_THETA = THETA*300;
//    VA = AdcaResultRegs.ADCRESULT0;
//    VA = (VA-2270)*0.041736;
//    VA = (VA-2270)*0.039;
//    t_count++; t_count %=200;

//    Ia_test = Iam/cos(THETA);
//    Ib_test = Iam/sin(THETA+PI_DIV_2-PIx2_DIV_3);
//    Ic_test = Iam/sin(THETA+PI_DIV_2+PIx2_DIV_3);

    // DAC
//    DacaRegs.DACVALS.all = AdcbResultRegs.ADCRESULT0;               // Load output B value
//    DacbRegs.DACVALS.all = AdcbResultRegs.ADCRESULT1;               // Load output B value
//    DacaRegs.DACVALS.all = THETA*200;               // Load output B value
//    DacbRegs.DACVALS.all = THETA_I*200;               // Load output B value

//    DacaRegs.DACVALS.all = THETA*200;              // Load output B value
//    DacbRegs.DACVALS.all = AdccResultRegs.ADCRESULT0;               // Load output B value

#endif
//    ON_RELAY();
#ifndef PROJECT_DEBUG

    if(OK2RUN&CONTROL){
        ON_RELAY();
        ReEna_ePWM();

//        if(Imax > 1.6) Imax = 1.6;
//        if(Imax <-1.6) Imax = -1.6;
//        // Current controller
//        // Reference current
//        Ia_ref = Imax*sin(THETA+PI_DIV_2);
//        Ic_ref = Imax*sin(THETA+PI_DIV_2+PIx2_DIV_3);
//        Ib_ref = Imax*sin(THETA+PI_DIV_2-PIx2_DIV_3);
//
//        // abc to alpha-beta
//        Ialpha_ref = (2*Ia_ref - Ib_ref - Ic_ref)/3;
//        Ibeta_ref = (Ib_ref-Ic_ref)/SQRT_3;
        // Current controller
        if(Id_ref > 2.5) Id_ref = 2.5;
        if(Id_ref < -2.5) Id_ref = -2.5;
        if(Iq_ref > 2.5) Iq_ref = 2.5;
        if(Iq_ref < -2.5) Iq_ref = -2.5;

        Ialpha_ref = cos(THETA)*Id_ref - sin(THETA)*Iq_ref;
        Ibeta_ref  = sin(THETA)*Id_ref + cos(THETA)*Iq_ref;

//        Ia_ref = Imax*sin(THETA+PI_DIV_2);
//        Ib_ref = Imax*sin(THETA+PI_DIV_2-PIx2_DIV_3);
//        Ic_ref = Imax*sin(THETA+PI_DIV_2+PIx2_DIV_3);

        Ialpha_measurement = (2*Iam - Ibm - Icm)/3;
        Ibeta_measurement = (Ibm-Icm)/SQRT_3;

        // Error
        Err_alpha = Ialpha_ref - Ialpha_measurement;
        Err_beta = Ibeta_ref - Ibeta_measurement;

//        // PR controller
        Compute_2nd_Filter(&PR_ALPHA, Err_alpha, PR_COEFF);
        Compute_2nd_Filter(&PR_BETA, Err_beta, PR_COEFF);
//
        // Alpha beta to ABC
        Va_l = PR_ALPHA.y[0];
        Vb_l = (-PR_ALPHA.y[0]/2+SQRT_3/2*PR_BETA.y[0]);
        Vc_l = (-PR_ALPHA.y[0]/2-SQRT_3/2*PR_BETA.y[0]);

////        // Control signal
        Va_l = Va_l/VDC;
        Vb_l = Vb_l/VDC;
        Vc_l = Vc_l/VDC;

        // Load duty cycle
//        EPwm1Regs.CMPA.bit.CMPA =  (uint16_t)((Va_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
//        EPwm2Regs.CMPA.bit.CMPA =  (uint16_t)((Vb_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
//        EPwm3Regs.CMPA.bit.CMPA =  (uint16_t)((Vc_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));

        // SPACE VECTOR //
        dal = (2*Va_l - Vb_l - Vc_l)/3;
        dbe = (Vb_l - Vc_l)/SQRT_3;

        dTheta = atan2(dbe,dal);
        if(dTheta < 0) dTheta = dTheta + PIx2;

        dMax = sqrt(dal*dal+dbe*dbe);

        sector = 1+ (uint8_t)(dTheta/PI_DIV_3);
        dTheta = dTheta - (sector-1)*PI_DIV_3;

        t1 = SQRT_3*dMax*sin(PI_DIV_3-dTheta);
        t2 = SQRT_3*dMax*sin(dTheta);
        t0 = 1-t1-t2;

        if (sector == 1){
            psa = t1 +t2 +(t0/2);
            psb = t2 + (t0/2);
            psc = t0/2;
        }else if (sector == 2){
            psa = t1 + (t0/2);
            psb = t2 + t1 + (t0/2);
            psc = t0/2;
        }else if (sector == 3){
            psa = t0/2;
            psb = t1+ t2 + (t0/2);
            psc = t2 + (t0/2);
        }else if (sector == 4){
            psa = t0/2;
            psb = t1 + (t0/2);
            psc = t1 + t2 +(t0/2);
        }else if (sector == 5){
            psa = t2+(t0/2);
            psb = t0/2;
            psc = t1+t2+(t0/2);
        }else if (sector == 6){
            psa = t1 +t2 +(t0/2);
            psb = t0/2;
            psc = t1+(t0/2);
        }

        psa = psa - 0.5;
        psb = psb - 0.5;
        psc = psc - 0.5;

        // Load duty cycle
        EPwm1Regs.CMPA.bit.CMPA =  (uint16_t)((psa+1.0)*0.5*((float)EPwm1Regs.TBPRD));
        EPwm2Regs.CMPA.bit.CMPA =  (uint16_t)((psb+1.0)*0.5*((float)EPwm1Regs.TBPRD));
        EPwm3Regs.CMPA.bit.CMPA =  (uint16_t)((psc+1.0)*0.5*((float)EPwm1Regs.TBPRD));


        Ic_test =  1000.0*Vc_l;
        Ia_test =  1000.0*Va_l;
        DacbRegs.DACVALS.all = 2048.0+Ialpha_ref*500;                // Load output B value CHAN 2
        DacaRegs.DACVALS.all = 2048.0+Ialpha_measurement*500;                   // Load output B value CHAN 3
     }else{
        Dis_ePWM();
//        OFF_RELAY();
    }

#endif

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

    if(AdcbRegs.ADCINTOVF.bit.ADCINT1 == 1)
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPCTOGGLE.bit.GPIO64 = 1;
}
