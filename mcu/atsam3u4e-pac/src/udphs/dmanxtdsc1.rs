#[doc = "Register `DMANXTDSC1` reader"]
pub type R = crate::R<DMANXTDSC1_SPEC>;
#[doc = "Register `DMANXTDSC1` writer"]
pub type W = crate::W<DMANXTDSC1_SPEC>;
#[doc = "Field `NXT_DSC_ADD` reader - "]
pub type NXT_DSC_ADD_R = crate::FieldReader<u32>;
#[doc = "Field `NXT_DSC_ADD` writer - "]
pub type NXT_DSC_ADD_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 32, O, u32>;
impl R {
    #[doc = "Bits 0:31"]
    #[inline(always)]
    pub fn nxt_dsc_add(&self) -> NXT_DSC_ADD_R {
        NXT_DSC_ADD_R::new(self.bits)
    }
}
impl W {
    #[doc = "Bits 0:31"]
    #[inline(always)]
    #[must_use]
    pub fn nxt_dsc_add(&mut self) -> NXT_DSC_ADD_W<DMANXTDSC1_SPEC, 0> {
        NXT_DSC_ADD_W::new(self)
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
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc1::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc1::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct DMANXTDSC1_SPEC;
impl crate::RegisterSpec for DMANXTDSC1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`dmanxtdsc1::R`](R) reader structure"]
impl crate::Readable for DMANXTDSC1_SPEC {}
#[doc = "`write(|w| ..)` method takes [`dmanxtdsc1::W`](W) writer structure"]
impl crate::Writable for DMANXTDSC1_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets DMANXTDSC1 to value 0"]
impl crate::Resettable for DMANXTDSC1_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
