#include <log.h>
#define TRICE_FILE Id(46100)

#include <argon/argon.h>

#include <drivers/pmc.h>
#include <drivers/gpio.h>
#include <drivers/wdt.h>

#include <string.h>
#include <string>

#include "cyclone2.h"

using namespace ATSAM3U;

#include "pata/HDD_LBA48.h"
HDD_LBA48 hdd(1);

GPIO irq(PIOA, PIO_PA22);

void FPGA_IRQ(void);
// PIO Handlers
extern "C" {
    void PIOA_Handler(void) {
        uint32_t status = 0;
        status  = PIOA->PIO_ISR;
        status &= PIOA->PIO_IMR;
        if (status != 0) {
            if(irq.Get()) {
                // __BKPT(); // IRQ is HIGH
            } else {
                FPGA_IRQ(); // IRQ is LOW
            }
        }
    }
}

void blink_thread(void * params) {
    TRICE( Id(46152), "info:Starting Blink Thread\n" );
    GPIO led(PIOB, PIO_PB18);
    led.SetOutput(LOW, DISABLE, DISABLE);
    while(1) {
        led.Set();
        // TRICE( Id(33172), "trace:LED On\n" );
        Ar::Thread::sleep(500);
        led.Clear();
        // TRICE( Id(64479), "trace:LED Off\n" );
        Ar::Thread::sleep(500);
    }
}
Ar::ThreadWithStack<512> blinkThread("my", blink_thread, 0, 100, kArStartThread);



// int puts(const char *str) {
//     size_t pos = 0;
//     while(str[pos]) {
//         debugOutput.Write(str[pos++]);
//         while( !debugOutput.IsTXReady() ) {};
//     };

//     debugOutput.Write('\n');
//     while( !debugOutput.IsTXEmpty() ) {};
//     return pos;
// }

void FPGA_IRQ(void) {
    // uint16_t val[4] = {};
    // val[2] = masterFPGA[8];
    // val[3] = slaveFPGA[8];

    // val[0] = slaveFPGA[0];
    // val[1] = slaveFPGA[1];

    // __BKPT();
    hdd.IRQ();
}

// size_t strlen(const char *str) {
//     size_t len = 0;
//     while (str[len] != '\0') {
//         len++;
//     }
//     return len;
// }

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

#include <type_traits>
#include "pata/cmds/IdentifyDrive.h"

#include <efc/efc.h>
void setupFlashBooting() {
    uint32_t bitIndex = 1;

    if (EFC_RC_OK != efc_perform_command(EFC0, EFC_FCMD_GGPB, 0)) {
        __BKPT();
		return; // couldnt perform command
	}

	uint32_t bits = efc_get_result(EFC0);
	if (bits & (1 << bitIndex)) {
		return; // already set
	}

    if (EFC_RC_OK == efc_perform_command(EFC0, EFC_FCMD_SGPB, bitIndex)) {
        return; // set bit
    }

    __BKPT();
}

int __attribute__((optimize("O0"))) main() {

    setupFlashBooting();

    SystemInit();
    WatchdogTimer::Disable();

    GPIO rxd(PIOA, PIO_PA11);
    rxd.SetPeripheralA();
    GPIO txd(PIOA, PIO_PA12);
    txd.SetPeripheralA();

    PowerManagementController::EnablePeripheralClock(ID_UART);
    loggingInit();
    TRICE( Id(34206), "info:Booting openGDEMU\n" );  
    TRICE_S( Id(39562), "info:Build Info: %s", __DATE__ );
    TRICE_S( Id(54236), "info: at %s\n",__TIME__);
    
    TRICE( Id(46126), "info:Enabling PeripheralClocks\n" );
    PowerManagementController::EnablePeripheralClock(ID_PIOA);
    PowerManagementController::EnablePeripheralClock(ID_PIOB);
    PowerManagementController::EnablePeripheralClock(ID_PWM);
    PowerManagementController::EnablePeripheralClock(ID_SMC);
    
    GPIO fpgaReset(PIOA, PIO_PA30);
    fpgaReset.SetOutput(LOW, DISABLE, DISABLE);

    TRICE( Id(49512), "info:Initializing Cyclone2...\n");
    bool configured = Cyclone2::Init();
    if (configured == false) {
        TRICE( Id(34703), "err:Failed to Configure Cyclone2\n");
        __BKPT();
    }

    
    irq.SetInput(PIO_PULLUP);

    // Bring FPGA Logic out of Reset
    fpgaReset.Set();

    while(!irq.Get());

    // High Level
    TRICE( Id(46082), "info:Setting Up FPGA IRQ\n");
    irq.ConfigureInterrupt(PIO_IT_AIME | PIO_IT_EDGE);
    irq.EnableInterrupt();
    NVIC_EnableIRQ(PIOA_IRQn); 

    uint64_t fpgaEpoch = OpenGDEMU_FPGA::GetFPGABuildEpoch();
    TRICE( Id(48362), "info:FPGA Build Epoch %lld\n", fpgaEpoch);

    for(int i = 0; i < 10; i++) asm("nop");

    // Test Read/Write Reg
    TRICE( Id(45036), "info:Testing FPGA RW Register\n");
    for(uint32_t i = 0; i < 0xFFFF; i++) {
        OpenGDEMU_FPGA::WriteTest(i);
        uint16_t a = OpenGDEMU_FPGA::ReadTest();
        if (a != i) {
            TRICE( Id(39660), "err:FPGA read 0x%X when 0x%X was expected\n", a, i);
            __BKPT();
        }
    }
    
    TRICE( Id(59212), "info:Starting Scheduler\n");
    ar_kernel_run();

    while(1) {};

    return 0;
}