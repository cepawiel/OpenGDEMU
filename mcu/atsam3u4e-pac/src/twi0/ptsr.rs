#[doc = "Register `PTSR` reader"]
pub type R = crate::R<PTSR_SPEC>;
#[doc = "Field `RXTEN` reader - Receiver Transfer Enable"]
pub type RXTEN_R = crate::BitReader;
#[doc = "Field `TXTEN` reader - Transmitter Transfer Enable"]
pub type TXTEN_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Receiver Transfer Enable"]
    #[inline(always)]
    pub fn rxten(&self) -> RXTEN_R {
        RXTEN_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 8 - Transmitter Transfer Enable"]
    #[inline(always)]
    pub fn txten(&self) -> TXTEN_R {
        TXTEN_R::new(((self.bits >> 8) & 1) != 0)
    }
}
#[doc = "Transfer Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ptsr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PTSR_SPEC;
impl crate::RegisterSpec for PTSR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ptsr::R`](R) reader structure"]
impl crate::Readable for PTSR_SPEC {}
#[doc = "`reset()` method sets PTSR to value 0"]
impl crate::Resettable for PTSR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
