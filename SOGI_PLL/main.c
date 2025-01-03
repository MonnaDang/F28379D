//
// Included Files
//

#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "Filter.h"
#include <math.h>

#define SQRT_3_2    0.866025403784439
#define SQRT_3      1.732050807568877
#define PI_DIV_2    1.570796326794897
#define PI_DIV_3    1.047197551196598
#define PIx2_DIV_3  2.094395102393195
#define SQRT_2      1.414213562373095
#define Wn          314.1592653589793

// SOGI variables
const double Qz_coefficient[] = { 0.0001725129025599240, 0.0003450258051198481,0.0001725129025599240,
                                  -1.977059094702427, 0.978034975048366 };
const double Dz_coefficient[] = { 0.010982512475817, 0, -0.010982512475817,
                                  -1.977059094702427, 0.978034975048366 };
volatile SECOND_ORDER_FILTER Dz_ = { { 0, 0, 0 }, { 0, 0, 0 } };
volatile SECOND_ORDER_FILTER Qz_ = { { 0, 0, 0 }, { 0, 0, 0 } };

const double PI_COEFFICENT[] = { 50.1, -49.9, -1 };
FIRST_ORDER_FILTER PI_CONTROLLER = { { 0, 0 }, { 0, 0 } };

volatile double Val = 0;
volatile double Vbe = 0;
volatile double Vd = 0;
volatile double Vq = 0;
volatile double V_MAX = 0;
volatile double E_PD = 0;
volatile double V_LF = 0;
volatile double W_EST = 0;
volatile double F_EST = 0;
volatile double THETA = 0;
volatile double Vin = 0;
const double IN_MOD_2PI_COEFFICIENT[] = { 0.00005, 0.00005, -1 };
FIRST_ORDER_FILTER IN_MOD_2PI = { { 0, 0 }, { 0, 0 } };

volatile int testMode = 0;

volatile bool isSystemEnable = false;

#define GAINV 0.23

void PLL()
{
    Vin = ((double) AdcaResultRegs.ADCRESULT0 - 1950) * GAINV;

    // SOGI
    Compute_2nd_Filter(&Dz_, Vin, Dz_coefficient);
    Val = Dz_.y[0];
    Compute_2nd_Filter(&Qz_, Vin, Qz_coefficient);
    Vbe = Qz_.y[0];

    // PD - Park transform
    Vd = cos(THETA) * Val + sin(THETA) * Vbe;
    Vq = -sin(THETA) * Val + cos(THETA) * Vbe;

    // LF
    Compute_1st_Filter(&PI_CONTROLLER, Vq, PI_COEFFICENT);
    V_LF = PI_CONTROLLER.y[0];

    // VCO
    W_EST = Wn + V_LF;
    Integrator_mod_2_pi(&IN_MOD_2PI, W_EST, IN_MOD_2PI_COEFFICIENT);
    THETA = IN_MOD_2PI.y[0];

    F_EST = W_EST / PIx2;
}

uint16_t adc14_value = 0, count = 0, Ccount = 0;
double Sincount = 0;

void Init_ADC();
void Init_DAC();
void Init_epwm1_gpio();
void Init_ePWM();
interrupt void adca1_isr();

//
// Main
//
void main(void)
{
    InitSysCtrl();
    InitGpio();

    DINT;
    //Disable Globle INT

    InitPieCtrl();
    // Disable and clear all INT CPU flag
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;

    Init_ADC();
    Init_DAC();
    Init_ePWM();

    IER |= M_INT1; //Enable group 1 interrupts
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    while (1)
    {
    }

}

void Init_ADC()
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;    // ADCCLK = Input Clock / 4
    EDIS;

    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    EALLOW;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // POWER UP ADC module
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Generate after conversion
    EDIS;

    DELAY_US(1000);                          // Wait for ADCmodule is powered up

    /*
     * Configuration Acquisition window = (ACQPS + 1)(System Clock (SYSCLK) cycle time)
     * The selected acquisition window duration must be at least as long as one ADCCLK cycle.
     * FOR(See the DATASHEET):
     *  + 12 Bit interface(Table 5-43): The minimum ACQPS = 14 (15 System CLK cycle [75ns])
     *  + 16 Bit interface(Table 5-41): The minimum ACQPS = 63 (54 System CLK cycle [320ns])
     */
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;    // Sample and hold window
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14;    // SELECT Channel ADCIN14
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05; // Trigger source ADCTRIG5 - ePWM1, ADCSOCA
    EDIS;

    EALLOW;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // EOC0 is trigger for ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable Interrupt ADCINT1
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear the flag
    EDIS;
}
void Init_DAC()
{
    EALLOW;

    DacaRegs.DACCTL.bit.DACREFSEL = 1; // VREFHI/VSSA are the reference voltages
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // DAC output is enabled
    DacaRegs.DACVALS.all = 0;           // set DAC value
    DELAY_US(10);                       // Wait for buffered DAC power up

    DacbRegs.DACCTL.bit.DACREFSEL = 1; // VREFHI/VSSA are the reference voltages
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // DAC output is enabled
    DacbRegs.DACVALS.all = 0;           // set DAC value
    DELAY_US(10);                       // Wait for buffered DAC power up

    EDIS;
}

void Init_epwm1_gpio()
{
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);

    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
}

void Init_ePWM()
{

    Init_epwm1_gpio();
    /*
     * ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV] = 01; CLOCKSYS divide by 2 => CLOCK = 200MHz/2 = 100Mhz; (datasheet)
     *                 CLKDIV*HSPCLKDIV
     * Tpwm = 2xTBPRDx---------------- = 100us (10kHz)
     *                      100MHz
     *  TBPRD = 5000; CLKDIV=1; HSPCLKDIV=1
     */
    EPwm1Regs.TBPRD = 5000;             // Set Period
    EPwm1Regs.TBPHS.bit.TBPHS = 0;      // Set Phase shift
    EPwm1Regs.TBCTR = 0;                 // Clear counter

    EPwm1Regs.CMPA.bit.CMPA = 0;      // Set Compare A
    EPwm1Regs.CMPB.bit.CMPB = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // Set count direction
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;           // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x0;                // divide by 1
    EPwm1Regs.TBCTL.bit.CLKDIV = 0x0;                // divide by 1

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // Counter shadow mode
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   //
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;   //

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set when counter = 0
    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;          // Clear when counter = PRD
    EPwm1Regs.AQCTLB.bit.CBU = AQ_NO_ACTION;      // Do no thing
    EPwm1Regs.AQCTLB.bit.CBD = AQ_NO_ACTION;      // Do no thing

    EALLOW;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Generate EPWM1SOCA to trigger ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                // Enable EPWM1SOCA pulse
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable interrupt
    EPwm1Regs.ETPS.bit.SOCAPRD = 1; // Generate the EPWMxSOCA pulse on the first event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate an interrupt on the first event
    EDIS;
}

interrupt void adca1_isr()
{
    if (isSystemEnable)
    {
        PLL();
        switch (testMode)
        {
        case 0:
            DacbRegs.DACVALS.all = THETA * 200 + 2048;
            DacaRegs.DACVALS.all = Val * 2 + 2048;
            break;
        case 1:
            DacbRegs.DACVALS.all = Vd * 2 + 2048;
            DacaRegs.DACVALS.all = Val * 2 + 2048;
            break;
        case 2:
            DacbRegs.DACVALS.all = Vbe * 2 + 2048;
            DacaRegs.DACVALS.all = Val * 2 + 2048;
            break;
        default:
            break;
        }
    }
    else
    {
        DacbRegs.DACVALS.all = 0;
        DacaRegs.DACVALS.all = 0;

        // Reset all controller, filter
        int i = 0;
        for(i = 0; i< 2; i++){
            IN_MOD_2PI.x[i] = 0;
            IN_MOD_2PI.y[i] = 0;
            PI_CONTROLLER.x[i] = 0;
            PI_CONTROLLER.y[i] = 0;
        }
        for(i=0;i<3;i++){
            Dz_.x[i] = 0;
            Dz_.y[i] = 0;
            Qz_.x[i] = 0;
            Qz_.y[i] = 0;
        }
    }



AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

if (1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
{
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
}

PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
//
// End of File
//
