#include "Global.h"

static volatile uint16_t * globalFPGA = (volatile uint16_t *) (0x63000000 + (0 << 7));

uint64_t OpenGDEMU_FPGA::GetFPGABuildEpoch() {
    union {
        uint64_t a;
        uint16_t b[4];
    } u;

    u.b[0] = globalFPGA[BUILD_TS_WORD_0];
    u.b[1] = globalFPGA[BUILD_TS_WORD_1];
    u.b[2] = globalFPGA[BUILD_TS_WORD_2];
    u.b[3] = globalFPGA[BUILD_TS_WORD_3];
    return u.a;
}

void OpenGDEMU_FPGA::WriteTest(uint16_t v) {
    globalFPGA[TEST_REG] = v;
}

uint16_t OpenGDEMU_FPGA::ReadTest() {
    return globalFPGA[TEST_REG];
}