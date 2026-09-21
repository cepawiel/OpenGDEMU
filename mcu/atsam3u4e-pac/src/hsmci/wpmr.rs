#[doc = "Register `WPMR` reader"]
pub type R = crate::R<WPMR_SPEC>;
#[doc = "Register `WPMR` writer"]
pub type W = crate::W<WPMR_SPEC>;
#[doc = "Field `WP_EN` reader - Write Protection Enable"]
pub type WP_EN_R = crate::BitReader;
#[doc = "Field `WP_EN` writer - Write Protection Enable"]
pub type WP_EN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `WP_KEY` reader - Write Protection Key password"]
pub type WP_KEY_R = crate::FieldReader<u32>;
#[doc = "Field `WP_KEY` writer - Write Protection Key password"]
pub type WP_KEY_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 24, O, u32>;
impl R {
    #[doc = "Bit 0 - Write Protection Enable"]
    #[inline(always)]
    pub fn wp_en(&self) -> WP_EN_R {
        WP_EN_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 8:31 - Write Protection Key password"]
    #[inline(always)]
    pub fn wp_key(&self) -> WP_KEY_R {
        WP_KEY_R::new((self.bits >> 8) & 0x00ff_ffff)
    }
}
impl W {
    #[doc = "Bit 0 - Write Protection Enable"]
    #[inline(always)]
    #[must_use]
    pub fn wp_en(&mut self) -> WP_EN_W<WPMR_SPEC, 0> {
        WP_EN_W::new(self)
    }
    #[doc = "Bits 8:31 - Write Protection Key password"]
    #[inline(always)]
    #[must_use]
    pub fn wp_key(&mut self) -> WP_KEY_W<WPMR_SPEC, 8> {
        WP_KEY_W::new(self)
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
#[doc = "Write Protection Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wpmr::R`](R).  You can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`wpmr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct WPMR_SPEC;
impl crate::RegisterSpec for WPMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`wpmr::R`](R) reader structure"]
impl crate::Readable for WPMR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`wpmr::W`](W) writer structure"]
impl crate::Writable for WPMR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
