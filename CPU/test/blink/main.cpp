
#include <argon/argon.h>

#include <drivers/common.h>
#include <drivers/pmc.h>
#include <drivers/gpio.h>
#include <drivers/wdt.h>

using namespace ATSAM3U;

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

int __attribute__((optimize("O0"))) main() {

    SystemInit();
    WatchdogTimer::Disable();

    PowerManagementController::EnablePeripheralClock(ID_PIOB);

    blinkThread.resume();
    ar_kernel_run();

    while(1) {};


    return 0;
}


