#[doc = "Register `EMR` reader"]
pub type R = crate::R<EMR_SPEC>;
#[doc = "Register `EMR` writer"]
pub type W = crate::W<EMR_SPEC>;
#[doc = "Field `OFFMODES` reader - Off Mode if Sleep Bit (ADC12B_MR) = 1"]
pub type OFFMODES_R = crate::BitReader;
#[doc = "Field `OFFMODES` writer - Off Mode if Sleep Bit (ADC12B_MR) = 1"]
pub type OFFMODES_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `OFF_MODE_STARTUP_TIME` reader - Startup Time"]
pub type OFF_MODE_STARTUP_TIME_R = crate::FieldReader;
#[doc = "Field `OFF_MODE_STARTUP_TIME` writer - Startup Time"]
pub type OFF_MODE_STARTUP_TIME_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
impl R {
    #[doc = "Bit 0 - Off Mode if Sleep Bit (ADC12B_MR) = 1"]
    #[inline(always)]
    pub fn offmodes(&self) -> OFFMODES_R {
        OFFMODES_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 16:23 - Startup Time"]
    #[inline(always)]
    pub fn off_mode_startup_time(&self) -> OFF_MODE_STARTUP_TIME_R {
        OFF_MODE_STARTUP_TIME_R::new(((self.bits >> 16) & 0xff) as u8)
    }
}
impl W {
    #[doc = "Bit 0 - Off Mode if Sleep Bit (ADC12B_MR) = 1"]
    #[inline(always)]
    #[must_use]
    pub fn offmodes(&mut self) -> OFFMODES_W<EMR_SPEC, 0> {
        OFFMODES_W::new(self)
    }
    #[doc = "Bits 16:23 - Startup Time"]
    #[inline(always)]
    #[must_use]
    pub fn off_mode_startup_time(&mut self) -> OFF_MODE_STARTUP_TIME_W<EMR_SPEC, 16> {
        OFF_MODE_STARTUP_TIME_W::new(self)
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
#[doc = "Extended Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`emr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`emr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EMR_SPEC;
impl crate::RegisterSpec for EMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`emr::R`](R) reader structure"]
impl crate::Readable for EMR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`emr::W`](W) writer structure"]
impl crate::Writable for EMR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets EMR to value 0"]
impl crate::Resettable for EMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
