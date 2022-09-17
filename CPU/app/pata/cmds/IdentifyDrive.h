#pragma once

#include <stdint.h>

// https://users.utcluj.ro/~baruch/sie/labor/ATA-ATAPI/d1410r3b-ATA-ATAPI-6.pdf
// Page 115

namespace ATA_Commands {
    class IdentifyDrive {
    public:
        struct __attribute__ ((packed)) IdentifyDriveResponse {
            // 0
            uint16_t GeneralConfiguration;
            uint16_t Obsolete1;  // Obsolete
            uint16_t SpecificConfiguration;  // Specific Configuration
            uint16_t Obsolete3;  // Obsolete
            uint16_t Retired4_5[2];  // Retired
            uint16_t Obsolete6;  // Obsolete
            uint16_t ReservedCompactFlash[2];  // Reserved for CompactFlash
            uint16_t Retired9;

            // 10
            const char SerialNumber[20];
            uint16_t Retired20_21[2];  // Retired
            uint16_t Obsolete22;  // Obsolete
            const char FirmwareVersion[8];
            const char ModelNumber[40];
            uint16_t MaxSectorsPerMultipleCommand;
            uint16_t Reserved48;  // Reserved
            uint16_t Capabilities;

            // 50
            uint16_t Capabilities2;
            uint16_t Obsolete51_52[2];  // Obsolete
            uint16_t FieldValidity;
            uint16_t Obsolete54_58[5];  // Obsolete
            uint16_t MultipleSectorPerInt;

            // 60
            uint32_t TotalNumberOfSectors;
            uint16_t Obsolete62;  // Obsolete
            uint16_t SupportedDMAModes;
            uint16_t SupportedPIOModes;
            uint16_t MinDMACycleTime;
            uint16_t RecDMACycleTime;
            uint16_t MinPIOCycleTime;
            uint16_t MinPIOCycleTimeFlowControl;
            uint16_t Reserved69_70[2];  // Reserved

            // 71
            uint16_t Reserved71_74[4];  // Reserved
            uint16_t QueueDepth;
            uint16_t Reserved76_79[4];  // Reserved

            // 80
            uint16_t MajorVersion;
            uint16_t MinorVersion;
            uint16_t CommandSetSupported;
            uint16_t CommandSetsSupported;
            uint16_t CommandFeatureSupportedExtension;
            uint16_t CommandSetFeatureEnabled;
            uint16_t CommandSetFeatureEnabled2;
            uint16_t CommandSetFeatureDefault;
            uint16_t UltraDMAMode;
            uint16_t SecureEraseTimeRequired;
            uint16_t EnhancedSecureReaseTimeRequired;

            // 91
            uint16_t CurrentAdvancedPowerManagementValue;
            uint16_t MasterPasswordRevisionCode;
            uint16_t HardwareResetResult;
            uint16_t AcousticValues;
            uint16_t Reserved95_99[5];  // Reserved
            uint64_t MaxLBAforLBA48;
            uint16_t Reserved104_126[23];  // Reserved
            uint16_t RemovableMediaNotification;
            uint16_t SecurityStatus;
            uint16_t VendorSpecific[31];
            uint16_t CFAPowerMode;
            uint16_t CompactFlashAssoc[15];
            uint16_t CurrentMediaSerialNumber[30];
            uint16_t Reserved206_254[49];
            uint16_t IntegrityWord;
        };

    };
};