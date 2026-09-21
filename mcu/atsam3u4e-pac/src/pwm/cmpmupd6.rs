#[doc = "Register `CMPMUPD6` writer"]
pub type W = crate::W<CMPMUPD6_SPEC>;
#[doc = "Field `CENUPD` writer - Comparison x Enable Update"]
pub type CENUPD_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CTRUPD` writer - Comparison x Trigger Update"]
pub type CTRUPD_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `CPRUPD` writer - Comparison x Period Update"]
pub type CPRUPD_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `CUPRUPD` writer - Comparison x Update Period Update"]
pub type CUPRUPD_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
impl W {
    #[doc = "Bit 0 - Comparison x Enable Update"]
    #[inline(always)]
    #[must_use]
    pub fn cenupd(&mut self) -> CENUPD_W<CMPMUPD6_SPEC, 0> {
        CENUPD_W::new(self)
    }
    #[doc = "Bits 4:7 - Comparison x Trigger Update"]
    #[inline(always)]
    #[must_use]
    pub fn ctrupd(&mut self) -> CTRUPD_W<CMPMUPD6_SPEC, 4> {
        CTRUPD_W::new(self)
    }
    #[doc = "Bits 8:11 - Comparison x Period Update"]
    #[inline(always)]
    #[must_use]
    pub fn cprupd(&mut self) -> CPRUPD_W<CMPMUPD6_SPEC, 8> {
        CPRUPD_W::new(self)
    }
    #[doc = "Bits 16:19 - Comparison x Update Period Update"]
    #[inline(always)]
    #[must_use]
    pub fn cuprupd(&mut self) -> CUPRUPD_W<CMPMUPD6_SPEC, 16> {
        CUPRUPD_W::new(self)
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
#[doc = "PWM Comparison 6 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd6::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CMPMUPD6_SPEC;
impl crate::RegisterSpec for CMPMUPD6_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`cmpmupd6::W`](W) writer structure"]
impl crate::Writable for CMPMUPD6_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
