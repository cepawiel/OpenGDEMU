#[doc = "Register `PMC_WPMR` reader"]
pub type R = crate::R<PMC_WPMR_SPEC>;
#[doc = "Register `PMC_WPMR` writer"]
pub type W = crate::W<PMC_WPMR_SPEC>;
#[doc = "Field `WPEN` reader - Write Protect Enable"]
pub type WPEN_R = crate::BitReader;
#[doc = "Field `WPEN` writer - Write Protect Enable"]
pub type WPEN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `WPKEY` reader - Write Protect KEY"]
pub type WPKEY_R = crate::FieldReader<u32>;
#[doc = "Field `WPKEY` writer - Write Protect KEY"]
pub type WPKEY_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 24, O, u32>;
impl R {
    #[doc = "Bit 0 - Write Protect Enable"]
    #[inline(always)]
    pub fn wpen(&self) -> WPEN_R {
        WPEN_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 8:31 - Write Protect KEY"]
    #[inline(always)]
    pub fn wpkey(&self) -> WPKEY_R {
        WPKEY_R::new((self.bits >> 8) & 0x00ff_ffff)
    }
}
impl W {
    #[doc = "Bit 0 - Write Protect Enable"]
    #[inline(always)]
    #[must_use]
    pub fn wpen(&mut self) -> WPEN_W<PMC_WPMR_SPEC, 0> {
        WPEN_W::new(self)
    }
    #[doc = "Bits 8:31 - Write Protect KEY"]
    #[inline(always)]
    #[must_use]
    pub fn wpkey(&mut self) -> WPKEY_W<PMC_WPMR_SPEC, 8> {
        WPKEY_W::new(self)
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
#[doc = "Write Protect Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`pmc_wpmr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`pmc_wpmr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_WPMR_SPEC;
impl crate::RegisterSpec for PMC_WPMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`pmc_wpmr::R`](R) reader structure"]
impl crate::Readable for PMC_WPMR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`pmc_wpmr::W`](W) writer structure"]
impl crate::Writable for PMC_WPMR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets PMC_WPMR to value 0"]
impl crate::Resettable for PMC_WPMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
