

/**
 * main.c
 */
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

void Init_ePWM();

int main(void)
{
    InitSysCtrl();
    InitGpio();

    Init_ePWM();

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
    EPwm1Regs.DBCTL.bit.POLSEL = 0x0;                   // S3 =1, S2=0 (0x2)
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // S1 = 1, S0 = 1;
    EPwm1Regs.DBCTL.bit.OUTSWAP  = 0;                   // S6,S7 =0

    EPwm1Regs.DBFED.all = 500;                          // Falling edge delay       500*10ns = 5000ns = 5us
    EPwm1Regs.DBRED.all = 200;                          // Rising edge delay        200*10ns = 2000ns = 2us

}
