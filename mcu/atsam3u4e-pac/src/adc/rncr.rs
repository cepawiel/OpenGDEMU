#[doc = "Register `RNCR` reader"]
pub type R = crate::R<RNCR_SPEC>;
#[doc = "Register `RNCR` writer"]
pub type W = crate::W<RNCR_SPEC>;
#[doc = "Field `RXNCTR` reader - Receive Next Counter"]
pub type RXNCTR_R = crate::FieldReader<u16>;
#[doc = "Field `RXNCTR` writer - Receive Next Counter"]
pub type RXNCTR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 16, O, u16>;
impl R {
    #[doc = "Bits 0:15 - Receive Next Counter"]
    #[inline(always)]
    pub fn rxnctr(&self) -> RXNCTR_R {
        RXNCTR_R::new((self.bits & 0xffff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:15 - Receive Next Counter"]
    #[inline(always)]
    #[must_use]
    pub fn rxnctr(&mut self) -> RXNCTR_W<RNCR_SPEC, 0> {
        RXNCTR_W::new(self)
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
#[doc = "Receive Next Counter Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`rncr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`rncr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct RNCR_SPEC;
impl crate::RegisterSpec for RNCR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`rncr::R`](R) reader structure"]
impl crate::Readable for RNCR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`rncr::W`](W) writer structure"]
impl crate::Writable for RNCR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets RNCR to value 0"]
impl crate::Resettable for RNCR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
