#[doc = "Register `IMR` reader"]
pub type R = crate::R<IMR_SPEC>;
#[doc = "Field `RXRDY` reader - Mask RXRDY Interrupt"]
pub type RXRDY_R = crate::BitReader;
#[doc = "Field `TXRDY` reader - Disable TXRDY Interrupt"]
pub type TXRDY_R = crate::BitReader;
#[doc = "Field `ENDRX` reader - Mask End of Receive Transfer Interrupt"]
pub type ENDRX_R = crate::BitReader;
#[doc = "Field `ENDTX` reader - Mask End of Transmit Interrupt"]
pub type ENDTX_R = crate::BitReader;
#[doc = "Field `OVRE` reader - Mask Overrun Error Interrupt"]
pub type OVRE_R = crate::BitReader;
#[doc = "Field `FRAME` reader - Mask Framing Error Interrupt"]
pub type FRAME_R = crate::BitReader;
#[doc = "Field `PARE` reader - Mask Parity Error Interrupt"]
pub type PARE_R = crate::BitReader;
#[doc = "Field `TXEMPTY` reader - Mask TXEMPTY Interrupt"]
pub type TXEMPTY_R = crate::BitReader;
#[doc = "Field `TXBUFE` reader - Mask TXBUFE Interrupt"]
pub type TXBUFE_R = crate::BitReader;
#[doc = "Field `RXBUFF` reader - Mask RXBUFF Interrupt"]
pub type RXBUFF_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Mask RXRDY Interrupt"]
    #[inline(always)]
    pub fn rxrdy(&self) -> RXRDY_R {
        RXRDY_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Disable TXRDY Interrupt"]
    #[inline(always)]
    pub fn txrdy(&self) -> TXRDY_R {
        TXRDY_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 3 - Mask End of Receive Transfer Interrupt"]
    #[inline(always)]
    pub fn endrx(&self) -> ENDRX_R {
        ENDRX_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Mask End of Transmit Interrupt"]
    #[inline(always)]
    pub fn endtx(&self) -> ENDTX_R {
        ENDTX_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Mask Overrun Error Interrupt"]
    #[inline(always)]
    pub fn ovre(&self) -> OVRE_R {
        OVRE_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Mask Framing Error Interrupt"]
    #[inline(always)]
    pub fn frame(&self) -> FRAME_R {
        FRAME_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - Mask Parity Error Interrupt"]
    #[inline(always)]
    pub fn pare(&self) -> PARE_R {
        PARE_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 9 - Mask TXEMPTY Interrupt"]
    #[inline(always)]
    pub fn txempty(&self) -> TXEMPTY_R {
        TXEMPTY_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 11 - Mask TXBUFE Interrupt"]
    #[inline(always)]
    pub fn txbufe(&self) -> TXBUFE_R {
        TXBUFE_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - Mask RXBUFF Interrupt"]
    #[inline(always)]
    pub fn rxbuff(&self) -> RXBUFF_R {
        RXBUFF_R::new(((self.bits >> 12) & 1) != 0)
    }
}
#[doc = "Interrupt Mask Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`imr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IMR_SPEC;
impl crate::RegisterSpec for IMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`imr::R`](R) reader structure"]
impl crate::Readable for IMR_SPEC {}
#[doc = "`reset()` method sets IMR to value 0"]
impl crate::Resettable for IMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
