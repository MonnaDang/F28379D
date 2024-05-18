
/**
 * main.c
 */
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "MyLaunchpad.h"
#include "Filter.h"
#include "Power.h"

#define PROJECT_DEBUG
#define RUN_MODE
//#define USE_SINPWM
#define USE_SVPWM

#define DSOGI
//#define SRF

#define SQRT_3_2    0.866025403784439
#define SQRT_3      1.732050807568877
#define PI_DIV_2    1.570796326794897
#define PI_DIV_3    1.047197551196598
#define PIx2_DIV_3  2.094395102393195
#define SQRT_2      1.414213562373095
#define Wn          314.1592653589793

//#define GAINV 0.114

#define GAINV 0.1155
#define GAINI 0.0163*1.2205*0.8555

#define I_LIMIT     10
#define I_OK        7
#define VRMS        14.1421356237
#define VDC         100
#define VACMAX      30

//


// Power Analyzer
POWER_MEAS_3PHASE_ANALYZER  VABC;
ST_STATUS  MY_ST = ST_STATUS_DEFAULT;
// Power Analyzer

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
// WC = 10 rad/s
//const float PR_COEFF[] = {13.283469792268214,-20.634758991836420,7.361476234184991,-1.997016432778511,0.998002326315675};

// WC = 8, non BW = 1000Hz
//const float PR_COEFF[] = {19.590455971151480,-37.031956204908640,17.459779281828540,-1.997415744603669,0.998401673032946};
// WC = 4, non BW = 1000Hz L = 3mH
//const float PR_COEFF[] = {19.598461598007330,-37.047092304192134,17.466917225465235,-1.998214194511929,0.999200517057686};
// Ideal BW = 1000Hz L = 3mH
const float PR_COEFF[] = {19.606473627784805,-37.062240509368046,17.474060878048217,-1.999013283022547,1.000000000000000};

// WC = 4, non BW = 1000Hz L = 10mH
//const float PR_COEFF[] = {65.964878880679550,-124.7686883193680,58.865395514211200,-1.998214194511929,0.999200517057686};

// WC = 4, non BW = 550Hz L = 10mH
//const float PR_COEFF[] = {35.394677007803120,-68.571778992935790,33.210949153173590,-1.998214194511929,0.999200517057686};

// WC = 8, non BW = 1000Hz L = 10mH
//const float PR_COEFF[] = {65.938098122403020,-124.7180250927389,58.841488038388090,-1.997415744603669,0.998401673032946};

// WC = 8, non BW = 550Hz L = 10mH
//const float PR_COEFF[] = {35.379776589959440,-68.542901447566850,33.196957771633860,-1.997415744603669,0.998401673032946};

// WC = 8, non BW = 200Hz L = 10mH
// NonIdeal
//const float PR_COEFF[] = {12.577596391245224,-24.846538758731300,12.281206668995427,-1.997415744603669,0.998401673032946};
// Ideal
//const float PR_COEFF[] = {12.591996258927120,-24.875014409801466,12.295296508022348,-1.999013283022547,1.000000000000000};
// NonIdeal WC = 2
//const float PR_COEFF[] = {12.588394132991013,-24.867891227593596,12.291771935733516,-1.998613658894070,0.999600178616249};

// WC = 5rad/s
//const float PR_COEFF[] = {11.809231895053212,-20.655226655235420,8.846027210699615,-1.998997277825674,0.999000418362306};

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

volatile SECOND_ORDER_FILTER PR_A = {{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER PR_B  = {{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER PR_C  = {{0,0,0},{0,0,0}};

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

volatile bool CONTROL = 0;
volatile float Id_ref = 0.8;
volatile float Iq_ref = 0;
volatile uint8_t Test_mode = 0;

float OUT_DATA = 0;
volatile uint16_t index = 0;
volatile float plot[400];
volatile float *Data = &OUT_DATA;
volatile float sqr_Vmax = 0;

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

// Delta Operator
#define Delta 1/32
volatile DELTA_SECOND_ORDER Delta_alpha,Delta_beta;

#pragma FUNC_ALWAYS_INLINE(PLL)
inline void PLL(){

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

    IER |= M_INT1;  //Enable group 1 interrupts
    EINT;           // Enable Global interrupt INTM

    PieCtrlRegs.PIEIER1.bit.INTx2 = 1; // Enable PIE interrupt ADCB1

    // System init
    Iq_ref = 0;
    Id_ref = 0.8;
    CONTROL = 0;
    Dis_ePWM();
    OFF_RELAY();

    MY_ST.CONTROL   = 0;
    MY_ST.Pref      = 40;
    MY_ST.Qref      = 0;
    MY_ST.VDCI      = VDC;

    POWER_MEAS_3PHASE_ANALYZER_reset(&VABC);

    // System init

//    Init_Delta_Filter(&Delta_alpha, PR_COEFF, Delta);
//    Init_Delta_Filter(&Delta_beta, PR_COEFF, Delta);


    while(1){
        if((fabs(Iam) >= I_LIMIT) || (fabs(Ibm) >= I_LIMIT) || (fabs(Icm) >= I_LIMIT)){
            if(!OVER_CURRENT){
                OVER_CURRENT = 1;
                Dis_ePWM();
            }
        }
        if(MY_ST.CONTROL) ON_RELAY();
        else OFF_RELAY();
    }
}

inline bool Is_In_phase(){
    if(Ecount == 10){
        Ecount = 0;
        if( (fabs(E_PD) <= 0.1) && (fabs(Vmax-VACMAX) <= 8))            Inphase = (Inphase<<1)|1;
        else                                                            Inphase = (Inphase<<1)|0;
    }else Ecount++;
    if(Inphase == 0xffff) return 1;

    return 0;
}

interrupt void adcb1_isr(){
    GpioDataRegs.GPBTOGGLE.bit.GPIO63 = 1;
    // Measurement
    Iam = AdcaResultRegs.ADCRESULT1;
    Ibm = AdccResultRegs.ADCRESULT1;
    Icm = AdcbResultRegs.ADCRESULT1;

    Iam = Iam - 2048.0 - off[3];
    Ibm = Ibm - 2048.0 - off[4];
    Icm = Icm - 2048.0 - off[5];

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
        avg[3] += Iam;
        avg[4] += Ibm;
        avg[5] += Icm;
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

//    Iam = Iam*GAINI*1.01;
//    Ibm = Ibm*GAINI*0.99;
//    Icm = Icm*GAINI*1.07;

    Iam = Iam*GAINI*1.055153787705000;
    Ibm = Ibm*GAINI*1.072804740182010;
    Icm = Icm*GAINI*1.044952460687850;

    // Measurement

    PLL();

    if(Ena_Current_count==10){
        Ena_Current_count=0;
        if ( (fabs(THETA-PI_DIV_2) <= 0.1)  && Is_In_phase() && (!MY_ST.CANRUN)) MY_ST.CANRUN = 1;
        else MY_ST.CANRUN = Is_In_phase();
    }else Ena_Current_count++;

//         RUN          //
#ifdef RUN_MODE
    if(MY_ST.CANRUN&MY_ST.CONTROL){
        ON_RELAY();
        ReEna_ePWM();

        // Current controller

        if(MY_ST.Pref > 300) MY_ST.Pref = 300;
        if(MY_ST.Pref < -300) MY_ST.Pref = -300;
        if(MY_ST.Qref > 300) MY_ST.Qref = 300;
        if(MY_ST.Qref < -300) MY_ST.Qref = -300;

        sqr_Vmax = Val_pos*Val_pos + Vbe_pos*Vbe_pos+0.00001;
        Ialpha_ref = 0.666666666666667*(MY_ST.Pref*Val_pos+MY_ST.Qref*Vbe_pos)/sqr_Vmax;
        Ibeta_ref  = 0.666666666666667*(MY_ST.Pref*Vbe_pos-MY_ST.Qref*Val_pos)/sqr_Vmax;

        // Alpha Beta to ABC
        Ia_ref = Ialpha_ref;
        Ib_ref = -0.5*Ialpha_ref + 0.866025403784439*Ibeta_ref;
        Ic_ref = -0.5*Ialpha_ref - 0.866025403784439*Ibeta_ref;

        // Error
        Err_a = Ia_ref - Iam;
        Err_b = Ib_ref - Ibm;
        Err_c = Ic_ref - Icm;

        // PR controller
        Compute_2nd_Filter(&PR_A, Err_a, PR_COEFF);
        Compute_2nd_Filter(&PR_B, Err_b, PR_COEFF);
        Compute_2nd_Filter(&PR_C, Err_c, PR_COEFF);

        Va_l = PR_A.y[0];
        Vb_l = PR_B.y[0];
        Vc_l = PR_C.y[0];

        // Control signal
        Va_l = Va_l/MY_ST.VDCI;
        Vb_l = Vb_l/MY_ST.VDCI;
        Vc_l = Vc_l/MY_ST.VDCI;


#ifdef USE_SINPWM
        // Load duty cycle
        EPwm1Regs.CMPA.bit.CMPA =  (uint16_t)((Va_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
        EPwm2Regs.CMPA.bit.CMPA =  (uint16_t)((Vb_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
        EPwm3Regs.CMPA.bit.CMPA =  (uint16_t)((Vc_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
#endif

#ifdef   USE_SVPWM
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
#endif
     }else{
        Dis_ePWM();
    }
#endif
#ifdef  PROJECT_DEBUG

    VABC.va = VA;
    VABC.vb = VB;
    VABC.vc = VC;
    VABC.ia = Iam;
    VABC.ib = Ibm;
    VABC.ic = Icm;

    POWER_MEAS_3PHASE_ANALYZER_run(&VABC);
    switch (Test_mode) {
        case 0:
            OUT_DATA = THETA;
            DacbRegs.DACVALS.all = THETA*50;                                       // Load output A value
            DacaRegs.DACVALS.all = AdcaResultRegs.ADCRESULT0;                       // Load output B value
            break;
        case 1:
            OUT_DATA = Iam;
            DacbRegs.DACVALS.all = Ia_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Iam*100+2048;                      // Load output B value
            break;
        case 2:
            OUT_DATA = Ibm;
            DacbRegs.DACVALS.all = Ib_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibm*100+2048;                      // Load output B value
            break;
        case 3:
            OUT_DATA = Icm;
            DacbRegs.DACVALS.all = Ic_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Icm*100+2048;                      // Load output B value
            break;
        case 4:
            OUT_DATA = VA;
            DacbRegs.DACVALS.all = THETA*200;                                       // Load output A value
            DacaRegs.DACVALS.all = AdcaResultRegs.ADCRESULT0;                       // Load output B value
            break;
        case 5:
            OUT_DATA = VB;
            DacbRegs.DACVALS.all = THETA*200;                                       // Load output A value
            DacaRegs.DACVALS.all = AdccResultRegs.ADCRESULT0;                       // Load output B value
            break;
        case 6:
            OUT_DATA = VC;
            DacbRegs.DACVALS.all = THETA*200;                                       // Load output A value
            DacaRegs.DACVALS.all = AdcbResultRegs.ADCRESULT0;                       // Load output B value
            break;
        case 7:
            OUT_DATA = VABC.P_m;
            DacbRegs.DACVALS.all = MY_ST.Pref*10+500;                              // Load output A value
            DacaRegs.DACVALS.all = VABC.P_m*10+500;                                  // Load output B value
            break;
        case 8:
            OUT_DATA = VABC.Q_m;
            DacbRegs.DACVALS.all = MY_ST.Qref*10+500;                              // Load output A value
            DacaRegs.DACVALS.all = VABC.Q_m*10+500;                                  // Load output B value
            break;
        case 9:
            OUT_DATA = Ialpha_ref;
            DacbRegs.DACVALS.all = Ialpha_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ialpha_measurement*100+2048;                      // Load output B value
            break;
        case 10:
            OUT_DATA = Ibeta_ref;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
        case 11:
            OUT_DATA = Iam;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
        case 12:
            OUT_DATA = EPwm2Regs.CMPA.bit.CMPA;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
        case 13:
            OUT_DATA = EPwm3Regs.CMPA.bit.CMPA;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
        case 14:
            OUT_DATA = Va_l;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
        case 15:
            OUT_DATA = Vb_l;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
        case 16:
            OUT_DATA = Vc_l;
            DacbRegs.DACVALS.all = Ibeta_ref*100+2048;                              // Load output A value
            DacaRegs.DACVALS.all = Ibeta_measurement*100+2048;                      // Load output B value
            break;
    }

    index++; index%=400;
    plot[index] = *Data;
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
