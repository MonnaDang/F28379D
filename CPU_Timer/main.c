//
// Included Files
//
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

//
// Function Prototypes
//
interrupt void cpu_timer0_isr(void);
interrupt void cpu_timer1_isr(void);
interrupt void cpu_timer2_isr(void);

void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio();
//    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL);
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

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

//
// Step 4. Initialize the Device Peripheral. This function can be
//         found in F2837xD_CpuTimers.c
//
    InitCpuTimers();   // For this example, only initialize the Cpu Timers

//
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 200MHz CPU Freq, 1 second Period (in uSeconds)
//
    ConfigCpuTimer(&CpuTimer0, 200, 1000000);
    ConfigCpuTimer(&CpuTimer1, 200, 1000000);
    ConfigCpuTimer(&CpuTimer2, 200, 1000000);

//
// To ensure precise timing, use write-only instructions to write to the
// entire register. Therefore, if any of the configuration bits are changed in
// ConfigCpuTimer and InitCpuTimers (in F2837xD_cputimervars.h), the below
// settings must also be updated.
//
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

//
// Step 5. User specific code, enable interrupts:
//

//
// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:
//
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;

//
// Enable TINT0 in the PIE: Group 1 interrupt 7
//
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Step 6. IDLE loop. Just sit and loop forever (optional):
//
    while(1)
    {
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Toggle BLUE LED
        DELAY_US(1000000);
    }
}

//
// cpu_timer0_isr - CPU Timer0 ISR with interrupt counter
//
interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Toggle RED LED

   //
   // Acknowledge this interrupt to receive more interrupts from group 1
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr - CPU Timer1 ISR
//
interrupt void cpu_timer1_isr(void)
{

   CpuTimer1.InterruptCount++;
}

//
// cpu_timer2_isr CPU Timer2 ISR
//
interrupt void cpu_timer2_isr(void)
{

   CpuTimer2.InterruptCount++;
}

//
// End of file
//
