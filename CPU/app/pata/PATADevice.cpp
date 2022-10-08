#include <log.h>

#include "PATADevice.h"

static volatile uint16_t * masterFPGA = (volatile uint16_t *) (0x63000000 + (2 << 7));
static volatile uint16_t * slaveFPGA =  (volatile uint16_t *) (0x63000000 + (3 << 7));

PATADevice::PATADevice(const char * DeviceName, bool slaveDevice) 
    : Ar::ThreadWithStack<2048>(DeviceName, startThread, this, 100, kArStartThread),
      m_isSlave(slaveDevice),
      m_ideRegs((m_isSlave ? slaveFPGA : masterFPGA)),
      m_CommandPending(false) {

}

PATADevice::~PATADevice() {

}

void PATADevice::ThreadLoop() {
    INFO("Starting Thread for PATADevice\n"); 
    while(true) {
        // m_CommandPending = m_ideRegs.GetCommand(m_Regs);
        NVIC_DisableIRQ(PIOA_IRQn);
        if (m_CommandPending) {
            OnCommand();
        }
        NVIC_EnableIRQ(PIOA_IRQn);
    }
}

void PATADevice::startThread(void * pataDeviceObj) {
    ((PATADevice *) pataDeviceObj)->ThreadLoop();
}

void PATADevice::IRQ() {
    if(m_CommandPending) {
        __BKPT();
    } else {
        m_CommandPending = m_ideRegs.GetCommand(m_Regs);
    }
}