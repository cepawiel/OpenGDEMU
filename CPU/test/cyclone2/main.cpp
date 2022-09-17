
#include <argon/argon.h>

#include <drivers/pmc.h>
#include <drivers/gpio.h>
#include <drivers/wdt.h>
#include <drivers/cyclone_ps.h>

#include <smc.h>

#include <OpenGDEMUFPGA.h>

extern "C" {
extern const uint8_t _binary_OpenGDEMU_rbf_start[];
extern const uint8_t _binary_OpenGDEMU_rbf_end;
extern const uint8_t _binary_OpenGDEMU_rbf_size;
}

using namespace ATSAM3U;

void FPGA_IRQ(void);
// PIO Handlers
extern "C" {
    void PIOA_Handler(void) {
        uint32_t status = 0;
        status  = PIOA->PIO_ISR;
        status &= PIOA->PIO_IMR;
        if (status != 0)
            FPGA_IRQ();
    }
}

void blink_thread(void * params) {
    GPIO led(PIOB, PIO_PB18);
    led.SetOutput(LOW, DISABLE, DISABLE);
    while(1) {
        led.Set();
        Ar::Thread::sleep(1000);
        led.Clear();
        Ar::Thread::sleep(1000);
    }
}

Ar::ThreadWithStack<2048> blinkThread("my", blink_thread, 0, 100, kArSuspendThread);

bool InitFPGA() {
    GPIO confDone(PIOB, PIO_PB1);
    GPIO nConfig(PIOA, PIO_PA25);
    GPIO clk(PIOA, PIO_PA24);
    GPIO data(PIOA, PIO_PA23);
    GPIO nStatus(PIOB, PIO_PB2);

    AlteraCyclonePS fpgaSetup(nConfig, nStatus, clk, data, confDone);
    return fpgaSetup.InitFPGA(&_binary_OpenGDEMU_rbf_start[0], (size_t) &_binary_OpenGDEMU_rbf_size);
}


void FPGA_IRQ(void) {
    // uint16_t val[2] = {};
    // val[0] = slaveMem[0];
    // val[1] = slaveMem[1];

    __BKPT();
}

// void FPGA_Thread(void * params) {
//     while(1) {

//         uint16_t status = slaveMem[0];
//         uint16_t cmd = slaveMem[1];

//         if(status != 0)
//             __BKPT();

//         if(cmd != 0)
//             __BKPT();

//     }
// }
// Ar::ThreadWithStack<2048> fpgaThread("fpga", FPGA_Thread, 0, 100, kArSuspendThread);

void InitEBI() {
    // GPIO a0(PIOB, PIO_PB7);
    // a0.SetPeripheralB();
    GPIO a1(PIOB, PIO_PB8);
    a1.SetPeripheralB();
    GPIO a2(PIOC, PIO_PC0);
    a2.SetPeripheralA();
    GPIO a3(PIOC, PIO_PC1);
    a3.SetPeripheralA();
    GPIO a4(PIOC, PIO_PC2);
    a4.SetPeripheralA();
    GPIO a5(PIOC, PIO_PC3);
    a5.SetPeripheralA();
    GPIO a6(PIOC, PIO_PC4);
    a6.SetPeripheralA();
    GPIO a7(PIOC, PIO_PC5);
    a7.SetPeripheralA();
    GPIO a8(PIOC, PIO_PC6);
    a8.SetPeripheralA();

    GPIO d0(PIOB, PIO_PB9);
    d0.SetPeripheralA();
    GPIO d1(PIOB, PIO_PB10);
    d1.SetPeripheralA();
    GPIO d2(PIOB, PIO_PB11);
    d2.SetPeripheralA();
    GPIO d3(PIOB, PIO_PB12);
    d3.SetPeripheralA();
    GPIO d4(PIOB, PIO_PB13);
    d4.SetPeripheralA();
    GPIO d5(PIOB, PIO_PB14);
    d5.SetPeripheralA();
    GPIO d6(PIOB, PIO_PB15);
    d6.SetPeripheralA();
    GPIO d7(PIOB, PIO_PB16);
    d7.SetPeripheralA();
    GPIO d8(PIOB, PIO_PB25);
    d8.SetPeripheralA();
    GPIO d9(PIOB, PIO_PB26);
    d9.SetPeripheralA();
    GPIO d10(PIOB, PIO_PB27);
    d10.SetPeripheralA();
    GPIO d11(PIOB, PIO_PB28);
    d11.SetPeripheralA();
    GPIO d12(PIOB, PIO_PB29);
    d12.SetPeripheralA();
    GPIO d13(PIOB, PIO_PB30);
    d13.SetPeripheralA();
    GPIO d14(PIOB, PIO_PB31);
    d14.SetPeripheralA();
    GPIO d15(PIOB, PIO_PB6);
    d15.SetPeripheralB();

    GPIO cs(PIOC, PIO_PC17);
    cs.SetPeripheralA();
    GPIO rd_en(PIOB, PIO_PB19);
    rd_en.SetPeripheralA();
    GPIO wr_en(PIOB, PIO_PB23);
    wr_en.SetPeripheralA();

    GPIO bs_en0(PIOB, PIO_PB7);
    bs_en0.SetPeripheralB();
    GPIO bs_en1(PIOC, PIO_PC15);
    bs_en1.SetPeripheralA();

    GPIO pwmh0(PIOA, PIO_PA28);
    pwmh0.SetPeripheralB();
}

#include <pwm/pwm.h>

struct FIFOAccess {
        //     fifo1.io.data(15 downto 7) := 0
        // fifo1.io.data(6 downto 5) := io.DC.CSn
        // fifo1.io.data(4) := io.DC.WR
        // fifo1.io.data(3) := io.DC.RD
        // fifo1.io.data(2 downto 0) := io.DC.ADDR.asBits
    bool CS0n;
    bool CS1n;
    bool WR;
    bool RD;
    uint8_t ADDR;
};

uint16_t val[50] = {};

int __attribute__((optimize("Og"))) main() {

    SystemInit();
    WatchdogTimer::Disable();

    PowerManagementController::EnablePeripheralClock(ID_PIOA);
    PowerManagementController::EnablePeripheralClock(ID_PIOB);

    PowerManagementController::EnablePeripheralClock(ID_PWM);
    pwm_channel_disable(PWM, PWM_CHANNEL_0);
    PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_MCK;
    // select the period
    PWM->PWM_CH_NUM[0].PWM_CPRD = 2;
    // select the duty cycle
    PWM->PWM_CH_NUM[0].PWM_CDTY = 1;
    pwm_channel_enable(PWM, PWM_CHANNEL_0);

    PowerManagementController::EnablePeripheralClock(ID_SMC);
    int cs = 3;
    smc_set_setup_timing(SMC, cs, SMC_SETUP_NWE_SETUP(0)
			| SMC_SETUP_NCS_WR_SETUP(0)
			| SMC_SETUP_NRD_SETUP(0)
			| SMC_SETUP_NCS_RD_SETUP(0));
	smc_set_pulse_timing(SMC, cs, SMC_PULSE_NWE_PULSE(2)
			| SMC_PULSE_NCS_WR_PULSE(2)
			| SMC_PULSE_NRD_PULSE(2)
			| SMC_PULSE_NCS_RD_PULSE(2));
	smc_set_cycle_timing(SMC, cs, SMC_CYCLE_NWE_CYCLE(2)
			| SMC_CYCLE_NRD_CYCLE(2));
	smc_set_mode(SMC, cs, SMC_MODE_READ_MODE | SMC_MODE_WRITE_MODE | SMC_MODE_DBW_BIT_16);

    GPIO fpgaReset(PIOA, PIO_PA30);
    fpgaReset.SetOutput(LOW, DISABLE, DISABLE);

    // Reset and program FPGA before enabling IRQ
    InitEBI();

    bool configured = InitFPGA();
    if (configured == false) 
        __BKPT();

    GPIO irq(PIOA, PIO_PA22);
    irq.SetInput(PIO_PULLUP);
    // High Level
    irq.ConfigureInterrupt(PIO_IT_AIME);

    // Bring FPGA Logic out of Reset
    fpgaReset.Set();

    irq.EnableInterrupt();
    NVIC_EnableIRQ(PIOA_IRQn); 

    uint64_t fpgaEpoch = OpenGDEMU_FPGA::GetFPGABuildEpoch();

    // Test Read/Write Reg
    for(uint32_t i = 0; i < 0xFFFF; i++) {
        OpenGDEMU_FPGA::WriteTest(i);
        uint16_t a = OpenGDEMU_FPGA::ReadTest();
        if (a != i) {
            __BKPT();
        }
    }

    // Test FIFOs
    // uint16_t countRD[10] = {};
    // uint16_t countWR[10] = {};

    // for(int i = 0; i < 10; i++){
    //     countRD[i] = slaveMem[6];
    //     countWR[i] = slaveMem[7];
    //     slaveMem[5] = 0xA5A5;
    // }

    // uint32_t count = 0;
    // // While FIFO not full
    // while((slaveMem[6] & 0x800) == 0) {
    //     slaveMem[5] = 0xA5A5;
    //     count++;
    // }
    
    __BKPT();
    
    blinkThread.resume();
    // fpgaThread.resume();
    ar_kernel_run();

    while(1) {};


    return 0;
}