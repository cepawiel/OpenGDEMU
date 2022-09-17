#include "log.h"

#include <drivers/uart.h>
#include <argon/argon.h>

ATSAM3U::SAM_UART debugOutput(UART);

void loggingInit() {
    debugOutput.Init(115200, UART_MR_PAR_NO, SystemCoreClock);
    debugOutput.EnableTX();
}

uint32_t loggingUARTTXEmpty(void) {
    return debugOutput.IsTXEmpty();
}

void loggingUARTWrite(uint8_t v) {
    debugOutput.Write(v);
}

void loggingUARTEnableTxEmptyInterrupt(void) {
    asm("bkpt");
}

void loggingUARTDisableTxEmptyInterrupt(void) {
    asm("bkpt");
}
