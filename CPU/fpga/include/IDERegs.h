#pragma once

#include <stdint.h>

#include "OpenGDEMURegs.h"

class IDERegs {
protected:

public:
    struct CommandRegs {
        uint8_t Commnand;
        uint8_t Features;
        uint8_t Device;
        uint16_t SectorCount;
        uint8_t LBALow;
        uint8_t LBAMid;
        uint8_t LBAHigh;
        uint8_t LBALowPrev;
        uint8_t LBAMidPrev;
        uint8_t LBAHighPrev;
    };

protected:
    volatile uint16_t * m_FPGARegs;
public:
    IDERegs(volatile uint16_t * baseRegs) :m_FPGARegs(baseRegs) {

    }

    bool GetCommand(CommandRegs &regs);

    uint16_t GetStatusReg();
    void ClearBSYFlag();
    void SetDRDYFlag();
    void SetDRQFlag();
    void SetABRTFlag();


    // TODO: DMA
    inline uint16_t ReadFIFO() { return false; }
    inline void WriteFIFO(uint16_t data) { m_FPGARegs[DATA_FIFO_REG] = data; }

    // temp??
    inline uint16_t GetReadFIFOCount() { return m_FPGARegs[READ_FIFO_INFO_REG]; }
    inline uint16_t GetWriteFIFOCount() { return m_FPGARegs[WRITE_FIFO_INFO_REG]; }
};