

#include <pio/pio.h>
#include <pmc/pmc.h>
#include <wdt/wdt.h>

#include <string.h>

// #include <FreeRTOS.h>
// #include <task.h>

#include <argon/argon.h>

#include <ff.h>

#include <hsmci.h>
#include <sysclk.h>

const char * COMPILE_DATE = __DATE__;
const char * COMPILE_TIME = __TIME__;

void ledOn() {
    pio_clear(PIOB, PIO_PB18);
}

void ledOff() {
    pio_set(PIOB, PIO_PB18);
}

static void sdcard_test(void * params) {
    FATFS FatFS;

    FIL fp;

    memset(&FatFS, 0, sizeof(FatFS));
    

    while(f_mount(&FatFS, "", 1) != FR_OK) {
    
    }

    FRESULT ret = f_open(&fp, "test.txt", FA_READ);
    if(ret) {
        asm("bkpt");
        return;
    }

    UINT bytesRead;
    char buffer[1024];
    // memset(&buffer, 0, sizeof(buffer));
    ret = f_read(&fp, &buffer[0],  sizeof(buffer), &bytesRead);
    if(ret == 0) {
        asm("bkpt");
    }

    f_close(&fp);

    f_unmount("0:");


    while(1) {
        ledOn();
        Ar::Thread::sleep(1000);
        ledOff();
        Ar::Thread::sleep(1000);
    }
}

 Ar::ThreadWithStack<4096> blinkThread("my", sdcard_test, 0, 100, kArSuspendThread);

int __attribute__((optimize("O0"))) main() {
    uint32_t resetReason = (RSTC->RSTC_SR >> 8) & 0xF;

    SystemInit();

    uint32_t mainClk = sysclk_get_peripheral_hz();

    // TODO: move watchdog reset into rtos, prob should use it if we have it
    wdt_disable(WDT);

    // pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOB);

    // led
    pio_set_output(PIOB, PIO_PB18, HIGH, DISABLE, DISABLE);

    // // sdcard inserted, active low, needs internal pullup
    // pio_set_input(PIOA, PIO_PA1, PIO_PULLUP);


    blinkThread.resume();
    ar_kernel_run();
    for(;;) {};
}
