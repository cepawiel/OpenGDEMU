#pragma once

#include <stdint.h>

#include "OpenGDEMURegs.h"

class OpenGDEMU_FPGA {
protected: 


public:
    static uint64_t GetFPGABuildEpoch();
    static void WriteTest(uint16_t v);
    static uint16_t ReadTest();
};
