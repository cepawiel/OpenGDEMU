#pragma once

#include "PATADevice.h"

class HDD_LBA48 : public PATADevice {
protected:

public:
    HDD_LBA48(bool slaveDevice);
    virtual ~HDD_LBA48();

    virtual void OnCommand() override;

    void OnIdentifyDrive();
    void OnSetFeatures();
    void OnReadSectors();

    void OnUnknownCommand();
};