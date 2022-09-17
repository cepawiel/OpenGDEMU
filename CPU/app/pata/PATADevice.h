#pragma once

#include <argon/argon.h>

#include "IDERegs.h"

class PATADevice : public Ar::ThreadWithStack<2048> {
protected:
    bool m_isSlave;
    IDERegs m_ideRegs;

    bool m_CommandPending;
    IDERegs::CommandRegs m_Regs;
    uint8_t m_TransferMode;

    enum ATACommand : uint8_t {
        ATA_ReadSectors     = 0x20,
        ATA_ReadSectors_Ext = 0x24,
        ATA_IdentifyDrive   = 0xEC,
        ATA_SetFeatures     = 0xEF
    };

    enum ATASetFeatures : uint8_t {
        ATA_SET_TRANSFER_MODE           = 0x03,
    };

    enum ATATransferMode : uint8_t {
        ATA_PIO_DEFAULT                 = 0x00,
        ATA_PIO_FLOW_CONTROL            = 0x01,
        ATA_MULTIWORD_DMA               = 0x04,
        ATA_ULTRA_DMA                   = 0x08
    };

public:
    PATADevice(const char * DeviceName, bool slaveDevice);
    virtual ~PATADevice() = 0;

    static void startThread(void *pataDeviceObj);

    void IRQ();

    void ThreadLoop();

    virtual void OnCommand() = 0;
};