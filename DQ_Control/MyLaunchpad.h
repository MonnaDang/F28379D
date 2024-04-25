/*
 * MyLaunchpad.h
 *
 *  Created on: Feb 27, 2024
 *      Author: Chuong
 *
 */

#ifndef MYLAUNCHPAD_H_
#define MYLAUNCHPAD_H_


#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "Filter.h"

volatile struct EPWM_REGS* EPWM_PTR[4] = {0x0,&EPwm1Regs,&EPwm2Regs,&EPwm3Regs};

// ePWM module
void Setup_ePWM();
// Time-base
inline void Setup_TB_Block(uint8_t chan){

 /*
 * ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV] = 01; CLOCKSYS divide by 2 => CLOCK to ePWM = 200MHz/2 = 100Mhz; (datasheet)
 *                 CLKDIV*HSPCLKDIV                                  CLKDIV*HSPCLKDIV
 * Tpwm = 2xTBPRDx---------------- = 100us (10kHz)      // TBCLK = --------------------- = 10ns
 *                      100MHz                                             100MHz
 *  TBPRD = 5000; CLKDIV=1; HSPCLKDIV=1
 */
    EPWM_PTR[chan]->TBPRD                   = 5000;                 // Set Period
    EPWM_PTR[chan]->TBPHS.bit.TBPHS         = 0;                    // Set Phase shift
    EPWM_PTR[chan]->TBCTR                   = 5000;                    // Clear counter
    EPWM_PTR[chan]->TBCTL.bit.CTRMODE       = TB_COUNT_UPDOWN;      // Set count mode
    EPWM_PTR[chan]->TBCTL.bit.PHSEN         = TB_DISABLE;           // Set Phase shift
    EPWM_PTR[chan]->TBCTL.bit.HSPCLKDIV     = 0;                    // Set high speed divider
    EPWM_PTR[chan]->TBCTL.bit.CLKDIV        = 0;                    // Set clock divider
}
// Counter-compare
inline void Set_CompareA(uint8_t chan,uint16_t CompareA){
    EPWM_PTR[chan]->CMPA.bit.CMPA           = CompareA;
}
inline void Set_CompareB(uint8_t chan,uint16_t CompareB){
    EPWM_PTR[chan]->CMPB.bit.CMPB           = CompareB;
}
inline void Setup_CC_Block(uint8_t chan,uint16_t CompareA,uint16_t CompareB){
    Set_CompareA(chan, CompareA);
    Set_CompareB(chan, CompareB);
    EPWM_PTR[chan]->CMPCTL.bit.SHDWAMODE    = CC_SHADOW;    // Use shadow register
    EPWM_PTR[chan]->CMPCTL.bit.SHDWBMODE    = CC_SHADOW;
    EPWM_PTR[chan]->CMPCTL.bit.LOADAMODE    = CC_CTR_ZERO_PRD;  // Load on zero
    EPWM_PTR[chan]->CMPCTL.bit.LOADBMODE    = CC_CTR_ZERO_PRD;
}

// Action-Qualifier
inline void Setup_AQ_Block(uint8_t chan){
    EPWM_PTR[chan]->AQCTLA.bit.PRD   = AQ_NO_ACTION;
    EPWM_PTR[chan]->AQCTLA.bit.ZRO   = AQ_NO_ACTION;
    EPWM_PTR[chan]->AQCTLA.bit.CAU   = AQ_CLEAR;              // CLEAR when counter = CMPA UP   direction (i.e when Vref <=  Vtri => S = 0)
    EPWM_PTR[chan]->AQCTLA.bit.CAD   = AQ_SET;                // SET   when counter = CMPA DOWN direction (i.e when Vref >=  Vtri => S = 1)
    EPWM_PTR[chan]->AQCTLB.bit.CBU   = AQ_CLEAR;                // CLEAR
    EPWM_PTR[chan]->AQCTLB.bit.CBD   = AQ_CLEAR;                // CLEAR
}
// Dead-band
inline void Setup_DB_Block(uint8_t chan,uint16_t RisingEdgeDelay,uint16_t FallingEdgeDelay){
    // Default is Active High Complementary(AHC)
    // See Tech-man Table 15-8 in page 1933 for more details.

    EPWM_PTR[chan]->DBCTL.bit.IN_MODE   = DBA_ALL;              // S4 = 0,S5 = 0
    EPWM_PTR[chan]->DBCTL.bit.DEDB_MODE = 0;                    // S8 = 0
    EPWM_PTR[chan]->DBCTL.bit.OUT_MODE  = DB_FULL_ENABLE;       // S1 = 1, S0 = 1;
    EPWM_PTR[chan]->DBCTL.bit.OUTSWAP   = 0;                    // S6,S7 =0
    EPWM_PTR[chan]->DBCTL.bit.POLSEL    = DB_ACTV_HIC;          // S2=0,S3=1

    EPWM_PTR[chan]->DBFED.all = FallingEdgeDelay;
    EPWM_PTR[chan]->DBRED.all = RisingEdgeDelay;
}

//inline void Dis_ePWM(){
//    EALLOW;
//    EPwm1Regs.TZFRC.bit.OST = 1;
//    EPwm2Regs.TZFRC.bit.OST = 1;
//    EPwm3Regs.TZFRC.bit.OST = 1;
//    EDIS;
//}
//
//inline void ReEna_ePWM(){
//    EALLOW;
//    EPwm1Regs.TZCLR.bit.OST = 1;
//    EPwm2Regs.TZCLR.bit.OST = 1;
//    EPwm3Regs.TZCLR.bit.OST = 1;
//    EDIS;
//}

inline void Dis_ePWM(){
    int i__;
    for(i__=1;i__<4;i__++){
        EPWM_PTR[i__]->DBCTL.bit.OUTSWAP   = 1;                     // S6,S7 =0
        EPWM_PTR[i__]->DBCTL.bit.IN_MODE   = DBB_ALL;              // S4 = 1,S5 = 1
    }
}

inline void ReEna_ePWM(){
    int i__;
    for(i__=1;i__<4;i__++){
        EPWM_PTR[i__]->DBCTL.bit.OUTSWAP   = 0;                    // S6,S7 =0
        EPWM_PTR[i__]->DBCTL.bit.IN_MODE   = DBA_ALL;              // S4 = 1,S5 = 1
    }
}

inline void Init_ePWM(){
//    // GPIO0
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.all   &= ~((1<<0)|(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10));     // ePWM GPIO 0-5
    GpioCtrlRegs.GPAMUX1.all    |=   (1<<0)|(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10);      // ePWM GPIO 0-5
    GpioCtrlRegs.GPADIR.all     |= 0x0000003F;                                        // OUTPUT GPIO 0-5
    EDIS;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Disable the time-base clock (Synchronization)
    EDIS;

    uint8_t ePWM_Chan;
    for(ePWM_Chan = 1; ePWM_Chan < 4; ePWM_Chan++){
//        EALLOW;
//        EPWM_PTR[ePWM_Chan]->TZFRC.bit.OST = 1;
//        EPWM_PTR[ePWM_Chan]->TZCTL.bit.TZA = TZ_FORCE_LO;
//        EPWM_PTR[ePWM_Chan]->TZCTL.bit.TZB = TZ_FORCE_LO;
//        EDIS;

        Setup_TB_Block(ePWM_Chan);
        Setup_CC_Block(ePWM_Chan, 2500, 0);
        Setup_AQ_Block(ePWM_Chan);
        Setup_DB_Block(ePWM_Chan, 50, 50);
    }

    // ET block
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;       // Generate EPWM1SOCA to trigger ADC
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;                // Enable EPWM1SOCA pulse
//    EPwm1Regs.ETSEL.bit.INTEN   = 1;                // Enable interrupt
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;                // Generate the EPWMxSOCA pulse on the first event
//    EPwm1Regs.ETPS.bit.INTPRD   = 1;                // Generate an interrupt on the first event

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;           // Enable the time-base clock (Synchronization)
    EDIS;
}
// ePWM module

// ADC mudule
inline void Init_ADC(){
    EALLOW;
    AdcbRegs.ADCCTL2.bit.PRESCALE   =   6;    // ADCCLK = Input Clock / 4
    AdccRegs.ADCCTL2.bit.PRESCALE   =   6;
    AdcaRegs.ADCCTL2.bit.PRESCALE   =   6;
    EDIS;

    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    EALLOW;
    // MODULE A
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ   =   1;      // POWER UP ADC module
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Generate after conversion
    DELAY_US(1000);
    // MODULE B
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ   =   1;      // POWER UP ADC module
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Generate after conversion
    DELAY_US(1000);
    // MODULE C
    AdccRegs.ADCCTL1.bit.ADCPWDNZ   =   1;      // POWER UP ADC module
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Generate after conversion
    DELAY_US(1000);                             // Wait for ADCmodule is powered up
    EDIS;

    /*
     * Configuration Acquisition window = (ACQPS + 1)(System Clock (SYSCLK) cycle time)
     * The selected acquisition window duration must be at least as long as one ADCCLK cycle.
     * FOR(See the DATASHEET):
     *  + 12 Bit interface(Table 5-43): The minimum ACQPS = 14 (15 System CLK cycle [75ns])
     *  + 16 Bit interface(Table 5-41): The minimum ACQPS = 63 (64 System CLK cycle [320ns])
     */

    EALLOW;
    // -------------------------- Voltage Sensor ------------------------//
    // Channel ADCIN14
    AdcaRegs.ADCSOC0CTL.bit.ACQPS   = 14;    // Sample and hold window  // Phase A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 14;    // SELECT Channel ADCIN14
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  // Trigger source ADCTRIG5 - ePWM1, ADCSOCA

    // Channel ADCINC3
    AdccRegs.ADCSOC0CTL.bit.ACQPS   = 14;    // Sample and hold window // Phase B
    AdccRegs.ADCSOC0CTL.bit.CHSEL   = 3 ;    // SELECT Channel ADCINC3
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  // Trigger source ADCTRIG5 - ePWM1, ADCSOCA

    // Channel ADCINB3
    AdcbRegs.ADCSOC0CTL.bit.ACQPS   = 14;    // Sample and hold window // Phase C
    AdcbRegs.ADCSOC0CTL.bit.CHSEL   = 3 ;    // SELECT Channel ADCINB3
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  // Trigger source ADCTRIG5 - ePWM1, ADCSOCA
    // -------------------------- Voltage Sensor ------------------------//

    // -------------------------- Current Sensor ------------------------//
    // Channel ADCINA3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = 14;    // Sample and hold window
    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = 15;     // SELECT Channel ADCIN15
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;  // Trigger source ADCTRIG5 - ePWM1, ADCSOCA

    // Channel ADCINC2
    AdccRegs.ADCSOC1CTL.bit.ACQPS   = 14;    // Sample and hold window // Phase B
    AdccRegs.ADCSOC1CTL.bit.CHSEL   = 5 ;    // SELECT Channel ADCINC5
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;  // Trigger source ADCTRIG5 - ePWM1, ADCSOCA

    // Channel ADCINB2
    AdcbRegs.ADCSOC1CTL.bit.ACQPS   = 14;    // Sample and hold window // Phase C
    AdcbRegs.ADCSOC1CTL.bit.CHSEL   = 5 ;    // SELECT Channel ADCINB5
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;  // Trigger source ADCTRIG5 - ePWM1, ADCSOCA
    // -------------------------- Current Sensor ------------------------//
    EDIS;

    EALLOW;
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;      // EOC1 is trigger for ADCINT1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E   = 1;      // Enable Interrupt ADCINT1
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear the flag
    EDIS;
}
// ADC module

// DAC module
inline void Init_DAC(){
    EALLOW;

    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // VREFHI/VSSA are the reference voltages
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // DAC output is enabled
    DacaRegs.DACVALS.all = 0;           // set DAC value
    DELAY_US(1000);                       // Wait for buffered DAC power up

    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // VREFHI/VSSA are the reference voltages
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // DAC output is enabled
    DacbRegs.DACVALS.all = 0;           // set DAC value
    DELAY_US(1000);                       // Wait for buffered DAC power up

    EDIS;
}

// DAC module

inline void ON_RELAY(){
    // ON all relays
    GpioDataRegs.GPACLEAR.bit.GPIO26=1;
    GpioDataRegs.GPACLEAR.bit.GPIO27=1;
    GpioDataRegs.GPACLEAR.bit.GPIO25=1;
}

inline void OFF_RELAY(){
    // OFF all relays
    GpioDataRegs.GPASET.bit.GPIO26=1;
    GpioDataRegs.GPASET.bit.GPIO27=1;
    GpioDataRegs.GPASET.bit.GPIO25=1;
}

inline void Init_RELAY(){
    // BLUE LED
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO31  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = 0;       // CPU1

    // RED LED
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0x0;      // GPIO
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0x0;     // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO34  = 1;        // OUTPUT
    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = 0;       // CPU1


    // RELAY B
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO26  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO26 = 0;       // CPU1

    // RELAY B
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO27 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO27  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO27 = 0;       // CPU1

    // RELAY C
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO25  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL4.bit.GPIO25 = 0;       // CPU1

    // RELAY A
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0x0;      // GPIO
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0x0;     // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO63  = 1;        // OUTPUT
    GpioCtrlRegs.GPBCSEL4.bit.GPIO63 = 0;       // CPU1

    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0x0;      // GPIO
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0x0;     // GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO64  = 1;        // OUTPUT
    GpioCtrlRegs.GPCCSEL1.bit.GPIO64 = 0;       // CPU1

    OFF_RELAY();

    EDIS;
}





#endif /* MYLAUNCHPAD_H_ */
