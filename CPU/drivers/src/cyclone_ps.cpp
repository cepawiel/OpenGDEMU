#include "drivers/cyclone_ps.h"

using namespace ATSAM3U;

AlteraCyclonePS::AlteraCyclonePS(GPIO& nConfig, GPIO& nStatus, GPIO& clk, GPIO& data, GPIO& configDone)
    : gpio_nConfig(nConfig), gpio_nStatus(nStatus), 
      gpio_clk(clk), gpio_data(data), gpio_configDone(configDone) {

}

AlteraCyclonePS::~AlteraCyclonePS() {

}

void __attribute__((optimize("O0"))) AlteraCyclonePS::clkDelay() {
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
    // asm("nop");
}

void AlteraCyclonePS::sendBit(bool bit) {
    // Clock Low
    gpio_clk.Clear();
    clkDelay();

    // Set Data
    if(bit) {
        gpio_data.Set();
    } else {
        gpio_data.Clear();
    }
    clkDelay();

    // Clock High
    gpio_clk.Set();
    clkDelay();

    // Here to make clock more uniform, probably unnecessary
    clkDelay();
}

bool AlteraCyclonePS::InitFPGA(const uint8_t * data, size_t len) {
    // set config low to keep on reset
    gpio_nConfig.SetOutput(LOW, DISABLE, DISABLE);



    // pull up config done, will be held low until configured
    gpio_configDone.SetInput(PIO_PULLUP);  

    // POR lasts 100MS, nStatus will be held low,
    // released when done
    gpio_nStatus.SetInput(PIO_PULLUP);

    // Setup data and clock pins
    gpio_clk.SetOutput(LOW, DISABLE, DISABLE);
    gpio_data.SetOutput(LOW, DISABLE, DISABLE);

    for(int i = 0; i < 10000; i++){
        asm("nop");
    }

    // Bring HIGH to take out of reset
    gpio_nConfig.Set();
    
    // Wait until ready for config
    while(gpio_nStatus.Get() == 0);

    uint32_t bytePos = 0;
    uint8_t bitPos = 0;

    // Until CONF_DONE high keep clocking data
    while( (gpio_configDone.Get() == 0) && (bytePos < len) ) {
        //  
        sendBit(data[bytePos] & (1 << bitPos));

        // if error occured
        if(gpio_nStatus.Get() == 0)
            __BKPT();

        bitPos++;
        if(bitPos >= 8) {
            bytePos++;
            bitPos = 0;
        }
    }
  
    for(int i = 0; i < 64; i++){
        sendBit(1);
    }
  
    bool done = gpio_configDone.Get();
    if (!done) __BKPT();
    return done;
}