#pragma once

#include <stdint.h>

//================================================
// Global Regs
//================================================
#define BUILD_TS_WORD_0         0
#define BUILD_TS_WORD_1         1
#define BUILD_TS_WORD_2         2
#define BUILD_TS_WORD_3         3

#define TEST_REG                4


//================================================
// Master ATA Regs
//================================================

#define HW_INFO_REG             0
#define HW_INFO_IS_SLAVE        0x0001
#define HW_INFO_COMMAND_PEND    0x0002

#define STATUS_REG              1
#define STATUS_COMMAND_ERR      0x0001
#define STATUS_COMMAND_DRQ      0x0008
#define STATUS_COMMAND_DF       0x0020
#define STATUS_COMMAND_DRDY     0x0040
#define STATUS_COMMAND_BSY      0x0080

#define DEVICE_REG              2

#define ERROR_REG               3
#define ERROR_ABRT              0x0004


#define COMMAND_FEATURES_REG    4
#define SECTOR_COUNT_REG        5
#define LBA_LOW_MID_REG         6
#define LBA_HIGH_LOWP_REG       7
#define LBA_MIDP_HIGHP_REG      8
#define DATA_FIFO_REG           9
#define READ_FIFO_INFO_REG      10
#define WRITE_FIFO_INFO_REG     11
#define DISK_TEST_REG           12


//================================================
// Slave ATA Regs
//================================================