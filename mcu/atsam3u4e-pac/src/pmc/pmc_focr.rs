#[doc = "Register `PMC_FOCR` writer"]
pub type W = crate::W<PMC_FOCR_SPEC>;
#[doc = "Field `FOCLR` writer - Fault Output Clear"]
pub type FOCLR_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 0 - Fault Output Clear"]
    #[inline(always)]
    #[must_use]
    pub fn foclr(&mut self) -> FOCLR_W<PMC_FOCR_SPEC, 0> {
        FOCLR_W::new(self)
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
#[doc = "Fault Output Clear Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`pmc_focr::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_FOCR_SPEC;
impl crate::RegisterSpec for PMC_FOCR_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`pmc_focr::W`](W) writer structure"]
impl crate::Writable for PMC_FOCR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
