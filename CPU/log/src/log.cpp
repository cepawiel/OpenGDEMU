#define NANOPRINTF_IMPLEMENTATION
#include "nanoprintf.h"

#include "log.h"

#include <drivers/uart.h>
#include <argon/argon.h>

ATSAM3U::SAM_UART debugOutput(UART);

// char buffer[256];

extern "C" {
    void loggingInit() {
    debugOutput.Init(115200, UART_MR_PAR_NO, SystemCoreClock);
    debugOutput.EnableTX();
}


    void putchar_(int c, void *ctx) {
        if(c == '\n') {
            putchar_('\r', ctx);
        }

        while(!debugOutput.IsTXReady()) {}
        debugOutput.Write(c);
        // ATSAM3U::SAM_UART * output = (ATSAM3U::SAM_UART *) ctx;
        // output->Write(c);
    }

    int DEBUG(char const *fmt, ...) {
        va_list val;
        va_start(val, fmt);
        int const rv = npf_vpprintf(&putchar_, &debugOutput, fmt, val);
        va_end(val);
        return rv;
    }

    int INFO(char const *fmt, ...) {
        va_list val;
        va_start(val, fmt);
        int const rv = npf_vpprintf(&putchar_, &debugOutput, fmt, val);
        va_end(val);
        return rv;
    }

    int WARN(char const *fmt, ...) {
        va_list val;
        va_start(val, fmt);
        int const rv = npf_vpprintf(&putchar_, &debugOutput, fmt, val);
        va_end(val);
        return rv;
    }

    int ERROR(char const *fmt, ...) {
        va_list val;
        va_start(val, fmt);
        int const rv = npf_vpprintf(&putchar_, &debugOutput, fmt, val);
        va_end(val);
        return rv;
    }
}