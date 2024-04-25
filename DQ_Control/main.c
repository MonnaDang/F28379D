
/**
 * main.c
 */
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "MyLaunchpad.h"
#include "PI.h"

#define PROJECT_DEBUG

#define SQRT_3_2    0.866025403784439
#define SQRT_3      1.732050807568877
#define PI_DIV_2    1.570796326794897
#define PIx2_DIV_3  2.094395102393195
#define SQRT_2      1.414213562373095
#define Wn          314.1592653589793

#define GAINV 0.044

#define GAINI 0.044

//#define M           0.8
//#define R_Rate      0.031415926535898

#define I_LIMIT     10
#define I_OK        7
#define VRMS        14.1421356237
#define VMAX_REF    20
#define P           50
//#define IRMS        P/(VRMS*3)
#define VDC         120


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
FIRST_ORDER_FILTER PM_FILTER = {{0,0},{0,0}};
FIRST_ORDER_FILTER QM_FILTER = {{0,0},{0,0}};

const float TEST_LP_COEFFICIENT[] = {0,-0.995012479192682,-0.995012479192682};
FIRST_ORDER_FILTER TEST_FILTER = {{0,0},{0,0}};

const float PI_COEFFICENT[] = {333.8,-332.8,-1};
FIRST_ORDER_FILTER PI_CONTROLLER = {{0,0},{0,0}};


const float IN_MOD_2PI_COEFFICIENT[] = {0.00005,0.00005,-1};
FIRST_ORDER_FILTER IN_MOD_2PI = {{0,0},{0,0}};


const float W0_2LP_COEFFICIENT[] = {0.0002413724953441133,0.0004827449906882266,0.0002413724953441133,-1.956009009166930,0.956974499148306};
SECOND_ORDER_FILTER W0_2ND_LPF = {{0,0,0},{0,0,0}};

volatile PI_VAL ID_PI = PI_DEFAULT;
volatile PI_VAL IQ_PI = PI_DEFAULT;


// PLL variable

const float PR_COEFF[] = {10.628073085496045,-20.651262799921845,10.033383219283222,-1.998613658894070,0.999600178616249};
//const float PR_COEFF[] = {20.611402463707830,-37.562807499364354,16.969946112994638,-1.998613658894070,0.999600178616249};
//const float PR_COEFF[] = {10.480462957566244,-20.655392033634250,10.185124619123492,-1.999013283022547,1.000000000000000};

//const float PR_COEFF[] = {10.150350206872798,-19.104001529736756,8.963081096446146,-1.998613658894070,0.999600178616249};
volatile SECOND_ORDER_FILTER PR_A={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER PR_B={{0,0,0},{0,0,0}};
volatile SECOND_ORDER_FILTER PR_C={{0,0,0},{0,0,0}};

volatile float Ia_ref=0,Ib_ref=0,Ic_ref=0;
volatile float Ia_adc = 0, Ib_adc = 0, Ic_adc = 0;
volatile float Iam = 0, Ibm = 0, Icm = 0;
volatile float Err_a = 0, Err_b = 0, Err_c = 0;
volatile float Va_l=0,Vb_l=0,Vc_l=0;
volatile float Da = 0, Db = 0, Dc = 0;
volatile float Imax = 0;
volatile float Ic_test1 = 0;
volatile float p_m = 0, q_m = 0;

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
volatile float Id_ref = 0, Iq_ref = 0;
volatile float Ial_m = 0, Ibe_m = 0;
volatile float Id_m = 0, Iq_m = 0;

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

    // alpha-beta to dq
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

bool Is_In_phase();

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

    ID_PI.Kp = 10.3327;
    ID_PI.Ki = 2954.1121;
    ID_PI.Ts = 0.0001;

    IQ_PI.Kp = 10.3327;
    IQ_PI.Ki = 2954.1121;
    ID_PI.Ts = 0.0001;
    Id_ref = 1;
    Iq_ref = 0;

    while(1){
        if((fabs(Iam) >= I_LIMIT) || (fabs(Ibm) >= I_LIMIT) || (fabs(Icm) >= I_LIMIT)){
            if(!OVER_CURRENT){
                OVER_CURRENT = 1;
                Dis_ePWM();
            }
        }
    }

}
//&& (fabs(Vmax-VMAX_REF) <= 2.5)

bool Is_In_phase(){
    if(Ecount == 10){
        Ecount = 0;
        if( (fabs(E_PD) <= 0.1) && (fabs(Vmax-33) <= 12))                Inphase = (Inphase<<1)|1;
        else                                                             Inphase = (Inphase<<1)|0;
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
    Icm = (Ic_adc/53.470);    // y = 53.47x + 2237.2

    PLL();


    if(Ena_Current_count==10){
        Ena_Current_count=0;
        if ( (fabs(THETA-PI_DIV_2) <= 0.1)  && Is_In_phase() && (!OK2RUN)) OK2RUN = 1;
        else OK2RUN = Is_In_phase();
    }else Ena_Current_count++;

#ifdef  PROJECT_DEBUG
    DacaRegs.DACVALS.all = THETA*200;                               // Load output B value
    DacbRegs.DACVALS.all = AdcaResultRegs.ADCRESULT0;               // Load output B value

#endif

#ifndef PROJECT_DEBUG
    if(OK2RUN){
        ON_RELAY();
        ReEna_ePWM();

        // ABC to alpha-beta
        Ial_m = (2*Iam-Ibm-Icm)/3;
        Ibe_m = (Ibm-Icm)/SQRT_3;

        // Alpha-beta to DQ
        Id_m = cos(THETA)*Ial_m + sin(THETA)*Ibe_m;
        Iq_m =-sin(THETA)*Ial_m + cos(THETA)*Ibe_m;

        p_m = 1.5*(Vd*Id_m + Vq*Iq_m);
        q_m = 1.5*(Vq*Id_m - Vd*Iq_m);

        Compute_1st_Filter(&PM_FILTER, p_m, DQ_LP_COEFFICIENT);
        Compute_1st_Filter(&QM_FILTER, q_m, DQ_LP_COEFFICIENT);

        // Current controller
        if(Id_ref > 2.5) Id_ref = 2.5;
        if(Id_ref < -2.5) Id_ref = -2.5;
        if(Iq_ref > 2.5) Iq_ref = 2.5;
        if(Iq_ref < -2.5) Iq_ref = -2.5;

        ID_PI.Err[0] = Id_ref-Id_m;
        Compute_PI(&ID_PI);

        IQ_PI.Err[0] = Iq_ref-Iq_m;
        Compute_PI(&IQ_PI);

        // DQ to ABC
        Va_l = (cos(THETA)*ID_PI.Out - sin(THETA)*IQ_PI.Out);
        Vb_l = (cos(THETA-PIx2_DIV_3)*ID_PI.Out - sin(THETA-PIx2_DIV_3)*IQ_PI.Out);
        Vc_l = (cos(THETA+PIx2_DIV_3)*ID_PI.Out - sin(THETA+PIx2_DIV_3)*IQ_PI.Out);

        // Control signal
        Va_l = Va_l/VDC;
        Vb_l = Vb_l/VDC;
        Vc_l = Vc_l/VDC;

        // Load duty cycle
        EPwm1Regs.CMPA.bit.CMPA =  (uint16_t)((Va_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
        EPwm2Regs.CMPA.bit.CMPA =  (uint16_t)((Vb_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));
        EPwm3Regs.CMPA.bit.CMPA =  (uint16_t)((Vc_l+1.0)*0.5*((float)EPwm1Regs.TBPRD));

        Ia_ref = Id_ref*sin(THETA+PI_DIV_2);
        Ic_ref = Id_ref*sin(THETA+PI_DIV_2+PIx2_DIV_3);
        Ib_ref = Id_ref*sin(THETA+PI_DIV_2-PIx2_DIV_3);
        DacbRegs.DACVALS.all = 2048.0+Ia_ref*500;                // Load output B value CHAN 2
        DacaRegs.DACVALS.all = 2048.0+Iam*500;                   // Load output B value CHAN 3
    }else{
        Dis_ePWM();
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
