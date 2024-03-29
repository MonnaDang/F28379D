
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


//#define GPIO
#define GPIO_BTN // Connect GPIO0 - > GPIO1
bool GPIO1_STATE = 0;

//#define MCP4921

#define CSA(x)      (x ? (GpioDataRegs.GPBSET.bit.GPIO63 = 1): (GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1))
#define CSB(x)      (x ? (GpioDataRegs.GPCSET.bit.GPIO64 = 1): (GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1))
#define CSC(x)      (x ? (GpioDataRegs.GPASET.bit.GPIO26 = 1): (GpioDataRegs.GPACLEAR.bit.GPIO26 = 1))
#define CLK(x)      (x ? (GpioDataRegs.GPASET.bit.GPIO27 = 1): (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1))
#define DATA(x)     (x ? (GpioDataRegs.GPASET.bit.GPIO25 = 1): (GpioDataRegs.GPACLEAR.bit.GPIO25 = 1))

uint16_t count = 0;

void SEND_DAC(uint16_t value);
void Init_GPIO();

int main(void)
{
    InitSysCtrl();  // CLOCK
    InitGpio();     //

    Init_GPIO();
//    CSA(1); CSB(1); CSC(1);
//    DATA(0);CLK(0);
    while(1){
#ifdef GPIO
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Toggle BLUE LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Toggle RED LED
        DELAY_US(500000);
#endif
#ifdef GPIO_BTN
        GPIO1_STATE = GpioDataRegs.GPADAT.bit.GPIO1; //
        // Press - > 0
        if(!GPIO1_STATE){
            GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;       // Turn on led
            GpioDataRegs.GPASET.bit.GPIO0 = 1;          // Release btn
            DELAY_US(500000);
        }else{
            GpioDataRegs.GPASET.bit.GPIO31 = 1;         // Turn off led
            GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;          // Release btn
            DELAY_US(500000);
        }
#endif
#ifdef MCP4921
        count+=1; count%=200; // 0 -> 199
        CSA(0);
        SEND_DAC(PHASE_A[count]);
        CSA(1);
        DELAY_US(100);
#endif
    }
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

    // GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO0  = 1;        // OUTPUT
    GpioCtrlRegs.GPACSEL1.bit.GPIO0 = 0;       // CPU1

    // GPIO1
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0x0;      // GPIO
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0x0;     // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO1  = 0;        // INPUT
    GpioCtrlRegs.GPACSEL1.bit.GPIO1 = 0;       // CPU1

    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xff; // Period 255
    GpioCtrlRegs.GPAQSEL1.bit.GPIO1   = 0x2;    // 6 sample
    GpioCtrlRegs.GPAPUD.bit.GPIO1     = 0;      //Pull up resister

    /*
     * Sampling period
     * Tsp = 2*QUALPRD0/F_sys = 2*255/200Mhz = 2.55us
     * Sampling window
     * Tsw = 5*Tsp = 12.75us
     */
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


