#include <log.h>

#include "HDD_LBA48.h"

#include "IDERegs.h"

HDD_LBA48::HDD_LBA48(bool slaveDevice) : PATADevice("HDD_LBA48", slaveDevice) {

}

HDD_LBA48::~HDD_LBA48() {

}

void HDD_LBA48::OnCommand() {
    DEBUG("HDD_LBA48 CMD: %X\n", m_Regs.Commnand ); 
    switch (m_Regs.Commnand) {
        case ATACommand::ATA_IdentifyDrive:
            OnIdentifyDrive();
            break;
        case ATACommand::ATA_SetFeatures:
            OnSetFeatures();
            break;
        case ATACommand::ATA_ReadSectors:
        case ATACommand::ATA_ReadSectors_Ext:
            OnReadSectors();
            break;
        default:
            OnUnknownCommand();
            break;
    }
    
    m_CommandPending = false;
    m_ideRegs.ClearBSYFlag();

    if (1) {
        uint16_t fifoCount = m_ideRegs.GetReadFIFOCount();
        while(fifoCount > 0) {
            DEBUG("fifo count 0x%X\n", fifoCount ); 
            fifoCount = m_ideRegs.GetReadFIFOCount();
        }
    }
}

void __attribute__((optimize("O0"))) HDD_LBA48::OnIdentifyDrive() {
    uint16_t identifyResp[256] = {};

    identifyResp[49] = (1 << 9);    // LBA Supported
    identifyResp[63] = 0x7;         // Multiword DMA 0-2 Supported
    identifyResp[83] = (1 << 10);   // LBA48 Supported
    uint64_t maxLBA = 0x10000000;
    memcpy(&identifyResp[100], &maxLBA, sizeof(maxLBA));

    // uint16_t fifoCountBefore = m_ideRegs.GetReadFIFOCount();
    for(int i = 0; i < 256; i++) {
        m_ideRegs.WriteFIFO(identifyResp[i]);
        // m_ideRegs.WriteFIFO(i);
    }
    // uint16_t fifoCountAfter = m_ideRegs.GetReadFIFOCount();

    m_ideRegs.SetDRQFlag();
    m_ideRegs.SetDRDYFlag();
    return;
}

void HDD_LBA48::OnSetFeatures() {
    // See Table 44
    if (m_Regs.Features == ATASetFeatures::ATA_SET_TRANSFER_MODE) {
        m_TransferMode = m_Regs.SectorCount;
        
        // Set Table 45
        switch (m_TransferMode & 0xF8) {
            case ATATransferMode::ATA_PIO_DEFAULT:
                break;
            case ATATransferMode::ATA_PIO_FLOW_CONTROL:
            case ATATransferMode::ATA_MULTIWORD_DMA:
            case ATATransferMode::ATA_ULTRA_DMA:
            default:
                // m_ideRegs.SetABRTFlag();
                // __BKPT();
                break;
        }

    }
    m_ideRegs.SetDRDYFlag();
    m_ideRegs.SetDRQFlag();
}

void __attribute__((optimize("O0"))) HDD_LBA48::OnReadSectors() {
    bool lba48 = (m_Regs.Commnand == ATACommand::ATA_ReadSectors_Ext);
    
    if ((m_Regs.Device & 0x40) == 0) {
        __BKPT();   // LBA Bit not Set
    }

    uint64_t LBA = 0;
    LBA |= (uint64_t)m_Regs.LBALow;
    LBA |= (uint64_t)m_Regs.LBAMid << 8;
    LBA |= (uint64_t)m_Regs.LBAHigh << 16;
    if(lba48) {
        LBA |= (uint64_t)m_Regs.LBALowPrev << 24;
        LBA |= (uint64_t)m_Regs.LBAMidPrev << 32;
        LBA |= (uint64_t)m_Regs.LBAHighPrev << 40;
    } else { // lba 28
        LBA |= (m_Regs.Device & 0xF) << 24;
    }

    uint16_t fifoCountBefore = m_ideRegs.GetReadFIFOCount();
    DEBUG("fifo count before 0x%X\n", fifoCountBefore ); 
    for(int i = 0; i < 256; i++) {
        m_ideRegs.WriteFIFO(i);
        // m_ideRegs.WriteFIFO(i);
    }
    uint16_t fifoCountAfter = m_ideRegs.GetReadFIFOCount();
    DEBUG("fifo count after 0x%X\n", fifoCountAfter ); 
    
    // for(int i = 0; i < 100000; i++) {
    //     asm("nop");
    // }

    m_ideRegs.SetDRDYFlag();
    m_ideRegs.SetDRQFlag();
}

void __attribute__((optimize("O0"))) HDD_LBA48::OnUnknownCommand() {
    uint8_t cmd = m_Regs.Commnand;
    __BKPT();
}