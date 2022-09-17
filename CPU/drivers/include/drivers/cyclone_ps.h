#pragma once

#include <stdint.h>
#include <stddef.h>

#include "drivers/gpio.h"

class AlteraCyclonePS {
protected:
    ATSAM3U::GPIO& gpio_nConfig;
    ATSAM3U::GPIO& gpio_nStatus;
    ATSAM3U::GPIO& gpio_clk;
    ATSAM3U::GPIO& gpio_data;
    ATSAM3U::GPIO& gpio_configDone;

    void clkDelay();
    void sendBit(bool bit);

public:
    AlteraCyclonePS(ATSAM3U::GPIO& nConfig, ATSAM3U::GPIO& nStatus, 
        ATSAM3U::GPIO& clk, ATSAM3U::GPIO& data, ATSAM3U::GPIO& configDone);
    ~AlteraCyclonePS();

    bool InitFPGA(const uint8_t * data, size_t len);
};