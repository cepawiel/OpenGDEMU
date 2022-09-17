#include "IDERegs.h"

bool __attribute__((optimize("O0"))) IDERegs::GetCommand(CommandRegs &regs) {
    uint16_t testVal[2] = {
        m_FPGARegs[DISK_TEST_REG],
        m_FPGARegs[DISK_TEST_REG]
    };
    if(testVal[1] != 0xCAFE) asm("bkpt");

    uint16_t a, b[10];   
    a = m_FPGARegs[COMMAND_FEATURES_REG];
    for(unsigned int i = 0; i < (sizeof(b) / sizeof(b[0])); i++) {
        b[i] = m_FPGARegs[COMMAND_FEATURES_REG];
    }
    for(unsigned int i = 0; i < (sizeof(b) / sizeof(b[0])); i++) {
        if(a != b[i]) asm("bkpt");
    }

    regs.Commnand = a & 0xFF;
    regs.Features = (a >> 8) & 0xFF;

    regs.SectorCount = m_FPGARegs[SECTOR_COUNT_REG];
    regs.Device = m_FPGARegs[DEVICE_REG];

    a = m_FPGARegs[LBA_LOW_MID_REG];
    regs.LBALow = a & 0xFF;
    regs.LBAMid = (a >> 8) & 0xFF;

    a = m_FPGARegs[LBA_HIGH_LOWP_REG];
    regs.LBAHigh = a & 0xFF;
    regs.LBALowPrev = (a >> 8) & 0xFF;

    a = m_FPGARegs[LBA_MIDP_HIGHP_REG];
    regs.LBAMidPrev = a & 0xFF;
    regs.LBAHighPrev = (a >> 8) & 0xFF;

    uint16_t hwInfo = m_FPGARegs[HW_INFO_REG];
    if (hwInfo & HW_INFO_COMMAND_PEND) {
        m_FPGARegs[HW_INFO_REG] = hwInfo & ~HW_INFO_COMMAND_PEND;
        return true;
    } else {
        return false;
    }
}

    uint16_t IDERegs::GetStatusReg() { return m_FPGARegs[STATUS_REG]; }

    void IDERegs::ClearBSYFlag() { 
        uint16_t status = m_FPGARegs[STATUS_REG];
        status &= ~STATUS_COMMAND_BSY; // clear bsy
        m_FPGARegs[STATUS_REG] = status;
    }

    void IDERegs::SetDRDYFlag() {
        uint16_t a = m_FPGARegs[STATUS_REG];
        a |= STATUS_COMMAND_DRDY; // set drdy
        m_FPGARegs[STATUS_REG] = a;
    }

    void IDERegs::SetDRQFlag() {
        uint16_t a = m_FPGARegs[STATUS_REG];
        a |= STATUS_COMMAND_DRQ; // set drq
        m_FPGARegs[STATUS_REG] = a;
    }

    void IDERegs::SetABRTFlag() {
        m_FPGARegs[ERROR_REG] |= ERROR_ABRT;
        m_FPGARegs[STATUS_REG] |= STATUS_COMMAND_ERR;
    }