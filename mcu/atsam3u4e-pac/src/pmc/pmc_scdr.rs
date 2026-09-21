#[doc = "Register `PMC_SCDR` writer"]
pub type W = crate::W<PMC_SCDR_SPEC>;
#[doc = "Field `PCK0` writer - Programmable Clock 0 Output Disable"]
pub type PCK0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `PCK1` writer - Programmable Clock 1 Output Disable"]
pub type PCK1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `PCK2` writer - Programmable Clock 2 Output Disable"]
pub type PCK2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 8 - Programmable Clock 0 Output Disable"]
    #[inline(always)]
    #[must_use]
    pub fn pck0(&mut self) -> PCK0_W<PMC_SCDR_SPEC, 8> {
        PCK0_W::new(self)
    }
    #[doc = "Bit 9 - Programmable Clock 1 Output Disable"]
    #[inline(always)]
    #[must_use]
    pub fn pck1(&mut self) -> PCK1_W<PMC_SCDR_SPEC, 9> {
        PCK1_W::new(self)
    }
    #[doc = "Bit 10 - Programmable Clock 2 Output Disable"]
    #[inline(always)]
    #[must_use]
    pub fn pck2(&mut self) -> PCK2_W<PMC_SCDR_SPEC, 10> {
        PCK2_W::new(self)
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
#[doc = "System Clock Disable Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`pmc_scdr::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_SCDR_SPEC;
impl crate::RegisterSpec for PMC_SCDR_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`pmc_scdr::W`](W) writer structure"]
impl crate::Writable for PMC_SCDR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
