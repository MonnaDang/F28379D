

/**
 * main.c
 */
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

// Formula: sin(2*pi*t/T)
const uint16_t PHASE_A[200] = {
 1500, 1547, 1594, 1641, 1688, 1735, 1781, 1827,
 1873, 1918, 1964, 2008, 2052, 2096, 2139, 2181,
 2223, 2264, 2304, 2343, 2382, 2419, 2456, 2492,
 2527, 2561, 2593, 2625, 2656, 2685, 2714, 2741,
 2766, 2791, 2814, 2837, 2857, 2877, 2895, 2911,
 2927, 2940, 2953, 2964, 2973, 2982, 2988, 2993,
 2997, 2999, 3000, 2999, 2997, 2993, 2988, 2982,
 2973, 2964, 2953, 2940, 2927, 2911, 2895, 2877,
 2857, 2837, 2814, 2791, 2766, 2741, 2714, 2685,
 2656, 2625, 2593, 2561, 2527, 2492, 2456, 2419,
 2382, 2343, 2304, 2264, 2223, 2181, 2139, 2096,
 2052, 2008, 1964, 1918, 1873, 1827, 1781, 1735,
 1688, 1641, 1594, 1547, 1500, 1453, 1406, 1359,
 1312, 1265, 1219, 1173, 1127, 1082, 1036,  992,
  948,  904,  861,  819,  777,  736,  696,  657,
  618,  581,  544,  508,  473,  439,  407,  375,
  344,  315,  286,  259,  234,  209,  186,  163,
  143,  123,  105,   89,   73,   60,   47,   36,
   27,   18,   12,    7,    3,    1,    0,    1,
    3,    7,   12,   18,   27,   36,   47,   60,
   73,   89,  105,  123,  143,  163,  186,  209,
  234,  259,  286,  315,  344,  375,  407,  439,
  473,  508,  544,  581,  618,  657,  696,  736,
  777,  819,  861,  904,  948,  992, 1036, 1082,
 1127, 1173, 1219, 1265, 1312, 1359, 1406, 1453 };


#define CSA(x)      (x ? (GpioDataRegs.GPBSET.bit.GPIO63 = 1): (GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1))
#define CSB(x)      (x ? (GpioDataRegs.GPCSET.bit.GPIO64 = 1): (GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1))
#define CSC(x)      (x ? (GpioDataRegs.GPASET.bit.GPIO26 = 1): (GpioDataRegs.GPACLEAR.bit.GPIO26 = 1))
#define CLK(x)      (x ? (GpioDataRegs.GPASET.bit.GPIO27 = 1): (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1))
#define DATA(x)     (x ? (GpioDataRegs.GPASET.bit.GPIO25 = 1): (GpioDataRegs.GPACLEAR.bit.GPIO25 = 1))

uint16_t count = 0;

void Init_ePWM();
void SEND_DAC(uint16_t value);
void Init_GPIO();

interrupt void ePWM1_ISR();

int main(void)
{
    InitSysCtrl();
//    InitGpio();

    DINT;   //Disable Globle INT

    InitPieCtrl();
    // Disable and clear all INT CPU flag
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.EPWM1_INT = &ePWM1_ISR; //function for ePWM1 ISR
    EDIS;

    // ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Disable the time-base clock (Synchronization)
    EDIS;
    Init_ePWM();
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;       // Enable the time-base clock (Synchronization)
    EDIS;
    // ePWM
    Init_GPIO();

    IER |= M_INT3; //Enable group 3 interrupts
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;          // Enable ePWM1 interrupt on Group 3

    EINT;  // Enable Global interrupt INTM

    while(1){

    }

}

void Init_ePWM(){
//    // GPIO0
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0x0;     // ePWM
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0x1;      // ePWM
    GpioCtrlRegs.GPADIR.bit.GPIO0  = 1;        // OUTPUT

    // GPIO1
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0x0;     // ePWM
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0x1;      // ePWM
    GpioCtrlRegs.GPADIR.bit.GPIO1  = 1;        // OUTPUT
    EDIS;
    /*
     * ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV] = 01; CLOCKSYS divide by 2 => CLOCK to ePWM = 200MHz/2 = 100Mhz; (datasheet)
     *                 CLKDIV*HSPCLKDIV                                  CLKDIV*HSPCLKDIV
     * Tpwm = 2xTBPRDx---------------- = 100us (10kHz)      // TBCLK = --------------------- = 10ns
     *                      100MHz                                             100MHz
     *  TBPRD = 5000; CLKDIV=1; HSPCLKDIV=1
     */
    // TB block
    EPwm1Regs.TBPRD = 5000;                             // Set Period
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                      // Set Phase shift
    EPwm1Regs.TBCTR = 0;                                // Clear counter
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // Set count direction
    EPwm1Regs.TBCTL.bit.PHSEN   = TB_DISABLE;           // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x0;                // divide by 1
    EPwm1Regs.TBCTL.bit.CLKDIV    = 0x0;                // divide by 1

    // CC block
    EPwm1Regs.CMPA.bit.CMPA =   2500;                   // Set Compare A
    EPwm1Regs.CMPB.bit.CMPB =   0;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // Counter shadow mode
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;       // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;       //

    // AQ block
    EPwm1Regs.AQCTLA.bit.CAU   = AQ_SET;                // SET   when counter = CMPA UP   direction (i.e when Vref >=  Vtri => S = 1)
    EPwm1Regs.AQCTLA.bit.CAD   = AQ_CLEAR;              // CLEAR when counter = CMPA DOWN direction (i.e when Vref <=  Vtri => S = 0)
    EPwm1Regs.AQCTLB.bit.CBU   = AQ_NO_ACTION;          // Do no thing
    EPwm1Regs.AQCTLB.bit.CBD   = AQ_NO_ACTION;          // Do no thing

    // DB block
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;              // S4, S5 = 0
    EPwm1Regs.DBCTL.bit.DEDB_MODE = 0;                  // S8 = 0
    EPwm1Regs.DBCTL.bit.POLSEL = 0x0;                   // S3 =0, S2=0 (0x0)
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // S1 = 1, S0 = 1;
    EPwm1Regs.DBCTL.bit.OUTSWAP  = 0;                   // S6,S7 =0

    EPwm1Regs.DBFED.all = 500;                          // Falling edge delay       500*10ns = 5000ns = 5us
    EPwm1Regs.DBRED.all = 200;                          // Rising edge delay        200*10ns = 2000ns = 2us

    // ET block
    EPwm1Regs.ETSEL.bit.INTSEL  = ET_CTR_PRD;         // Enable Interrupt on Period event
    EPwm1Regs.ETSEL.bit.INTEN   = 1;                  // Enable interrupt
    EPwm1Regs.ETPS.bit.INTPRD   = ET_1ST;             // Generate an interrupt on the first event
}

void SEND_DAC(uint16_t value){
    value = (value&0x0fff)|0x7000;
    uint16_t __ibit;
    for(__ibit = 0x8000 ; __ibit > 0; __ibit>>=1){
        DATA(__ibit&value);
        CLK(1);CLK(0);
    }
}

void Init_GPIO(){
    EALLOW;
    // CSC
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO26  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO26 = 0;       // CPU1
    // CLK
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO27 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO27  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO27 = 0;       // CPU1
    // DATA
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO25  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO25 = 0;       // CPU1

    // CSA
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0x0;      // GPIO
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0x0;     // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO63  = 1;        // OUTPUT
    GpioCtrlRegs.GPBCSEL4.bit.GPIO63 = 0;       // CPU1
    // CSB
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0x0;      // GPIO
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0x0;     // GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO64  = 1;        // OUTPUT
    GpioCtrlRegs.GPCCSEL1.bit.GPIO64 = 0;       // CPU1
    EDIS;
}

interrupt void ePWM1_ISR(){
    count+=1; count%=200; // 0 -> 199
    CSA(0);
    SEND_DAC(PHASE_A[count]);
    CSA(1);


    EPwm1Regs.ETCLR.bit.INT = 1;                // Clear interrupt Flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;     // Clear PIE flag on group 3
}
