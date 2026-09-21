#[doc = "Register `FIDI` reader"]
pub type R = crate::R<FIDI_SPEC>;
#[doc = "Register `FIDI` writer"]
pub type W = crate::W<FIDI_SPEC>;
#[doc = "Field `FI_DI_RATIO` reader - FI Over DI Ratio Value"]
pub type FI_DI_RATIO_R = crate::FieldReader<u16>;
#[doc = "Field `FI_DI_RATIO` writer - FI Over DI Ratio Value"]
pub type FI_DI_RATIO_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 11, O, u16>;
impl R {
    #[doc = "Bits 0:10 - FI Over DI Ratio Value"]
    #[inline(always)]
    pub fn fi_di_ratio(&self) -> FI_DI_RATIO_R {
        FI_DI_RATIO_R::new((self.bits & 0x07ff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:10 - FI Over DI Ratio Value"]
    #[inline(always)]
    #[must_use]
    pub fn fi_di_ratio(&mut self) -> FI_DI_RATIO_W<FIDI_SPEC, 0> {
        FI_DI_RATIO_W::new(self)
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
#[doc = "FI DI Ratio Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fidi::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`fidi::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct FIDI_SPEC;
impl crate::RegisterSpec for FIDI_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`fidi::R`](R) reader structure"]
impl crate::Readable for FIDI_SPEC {}
#[doc = "`write(|w| ..)` method takes [`fidi::W`](W) writer structure"]
impl crate::Writable for FIDI_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets FIDI to value 0x0174"]
impl crate::Resettable for FIDI_SPEC {
    const RESET_VALUE: Self::Ux = 0x0174;
}
