#include "drivers/gpio.h"

using namespace ATSAM3U;

GPIO::GPIO(Pio * port, uint32_t pin)
            : p_pio(port), ul_mask(pin), b_output(false) {
            
};

GPIO::~GPIO() {

};

void GPIO::EnableInterrupt() {
    p_pio->PIO_IER = ul_mask;
}

void GPIO::DisableInterrupt() {
    p_pio->PIO_IDR = ul_mask;
}

void GPIO::SetPullUp(const uint32_t ul_pull_up_enable) {
    /* Enable the pull-up(s) if necessary */
    if (ul_pull_up_enable) {
        p_pio->PIO_PUER = ul_mask;
    } else {
        p_pio->PIO_PUDR = ul_mask;
    }
}

void GPIO::SetOutput(const uint32_t ul_default_level, 
    const uint32_t ul_multidrive_enable, 
    const uint32_t ul_pull_up_enable) {
    DisableInterrupt();
    SetPullUp(ul_pull_up_enable);

    /* Enable multi-drive if necessary */
    if (ul_multidrive_enable) {
        p_pio->PIO_MDER = ul_mask;
    } else {
        p_pio->PIO_MDDR = ul_mask;
    }

    /* Set default value */
    if (ul_default_level) {
        p_pio->PIO_SODR = ul_mask;
    } else {
        p_pio->PIO_CODR = ul_mask;
    }

    /* Configure pin(s) as output(s) */
    p_pio->PIO_OER = ul_mask;
    p_pio->PIO_PER = ul_mask;

    b_output = true;
}

void GPIO::SetInput(const uint32_t ul_attribute) {
    DisableInterrupt();
    SetPullUp(ul_attribute & PIO_PULLUP);

    /* Enable Input Filter if necessary */
    if (ul_attribute & (PIO_DEGLITCH | PIO_DEBOUNCE)) {
        p_pio->PIO_IFER = ul_mask;
    } else {
        p_pio->PIO_IFDR = ul_mask;
    }

    /* Enable de-glitch or de-bounce if necessary */
    if (ul_attribute & PIO_DEGLITCH) {
        p_pio->PIO_SCIFSR = ul_mask;
    } else {
        if (ul_attribute & PIO_DEBOUNCE) {
            p_pio->PIO_DIFSR = ul_mask;
        }
    }
}

void GPIO::SetPeripheralA() {
	uint32_t ul_sr;

	/* Disable interrupts on the pin(s) */
	p_pio->PIO_IDR = ul_mask;

    ul_sr = p_pio->PIO_ABSR;
    p_pio->PIO_ABSR &= (~ul_mask & ul_sr);

    /* Remove the pins from under the control of PIO */
	p_pio->PIO_PDR = ul_mask;
};

void GPIO::SetPeripheralB() {
	uint32_t ul_sr;

	/* Disable interrupts on the pin(s) */
	p_pio->PIO_IDR = ul_mask;

    ul_sr = p_pio->PIO_ABSR;
    p_pio->PIO_ABSR = (ul_mask | ul_sr);
    

    /* Remove the pins from under the control of PIO */
	p_pio->PIO_PDR = ul_mask;
};

void GPIO::ConfigureInterrupt(uint32_t ul_attr) {
    /* Configure additional interrupt mode registers. */
	if (ul_attr & PIO_IT_AIME) {
		/* Enable additional interrupt mode. */
		p_pio->PIO_AIMER = ul_mask;

		/* If bit field of the selected pin is 1, set as
		   Rising Edge/High level detection event. */
		if (ul_attr & PIO_IT_RE_OR_HL) {
			/* Rising Edge or High Level */
			p_pio->PIO_REHLSR = ul_mask;
		} else {
			/* Falling Edge or Low Level */
			p_pio->PIO_FELLSR = ul_mask;
		}

		/* If bit field of the selected pin is 1, set as
		   edge detection source. */
		if (ul_attr & PIO_IT_EDGE) {
			/* Edge select */
			p_pio->PIO_ESR = ul_mask;
		} else {
			/* Level select */
			p_pio->PIO_LSR = ul_mask;
		}
	} else {
		/* Disable additional interrupt mode. */
		p_pio->PIO_AIMDR = ul_mask;
	}
}

void GPIO::Set() {
    p_pio->PIO_SODR = ul_mask;
}

void GPIO::Clear() {
    p_pio->PIO_CODR = ul_mask;
}

bool GPIO::Get() {
    uint32_t ul_reg;

    if (b_output) { 
        ul_reg = p_pio->PIO_ODSR;
    } else {
        ul_reg = p_pio->PIO_PDSR;
    }

    return (ul_reg & ul_mask);
}
