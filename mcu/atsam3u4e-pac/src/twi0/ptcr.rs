#[doc = "Register `PTCR` writer"]
pub type W = crate::W<PTCR_SPEC>;
#[doc = "Field `RXTEN` writer - Receiver Transfer Enable"]
pub type RXTEN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RXTDIS` writer - Receiver Transfer Disable"]
pub type RXTDIS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TXTEN` writer - Transmitter Transfer Enable"]
pub type TXTEN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TXTDIS` writer - Transmitter Transfer Disable"]
pub type TXTDIS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 0 - Receiver Transfer Enable"]
    #[inline(always)]
    #[must_use]
    pub fn rxten(&mut self) -> RXTEN_W<PTCR_SPEC, 0> {
        RXTEN_W::new(self)
    }
    #[doc = "Bit 1 - Receiver Transfer Disable"]
    #[inline(always)]
    #[must_use]
    pub fn rxtdis(&mut self) -> RXTDIS_W<PTCR_SPEC, 1> {
        RXTDIS_W::new(self)
    }
    #[doc = "Bit 8 - Transmitter Transfer Enable"]
    #[inline(always)]
    #[must_use]
    pub fn txten(&mut self) -> TXTEN_W<PTCR_SPEC, 8> {
        TXTEN_W::new(self)
    }
    #[doc = "Bit 9 - Transmitter Transfer Disable"]
    #[inline(always)]
    #[must_use]
    pub fn txtdis(&mut self) -> TXTDIS_W<PTCR_SPEC, 9> {
        TXTDIS_W::new(self)
    }
    #[doc = r" Writes raw bits to the register."]
    #[doc = r""]
    #[doc = r" # Safety"]
    #[doc = r""]
    #[doc = r" Passing incorrect value can cause undefined behaviour. See reference manual"]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.bits = bits;
        self
    }
}
#[doc = "Transfer Control Register\n\nYou can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ptcr::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PTCR_SPEC;
impl crate::RegisterSpec for PTCR_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`ptcr::W`](W) writer structure"]
impl crate::Writable for PTCR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets PTCR to value 0"]
impl crate::Resettable for PTCR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
