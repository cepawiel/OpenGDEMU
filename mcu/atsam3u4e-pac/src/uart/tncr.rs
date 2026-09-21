#[doc = "Register `TNCR` reader"]
pub type R = crate::R<TNCR_SPEC>;
#[doc = "Register `TNCR` writer"]
pub type W = crate::W<TNCR_SPEC>;
#[doc = "Field `TXNCTR` reader - Transmit Counter Next"]
pub type TXNCTR_R = crate::FieldReader<u16>;
#[doc = "Field `TXNCTR` writer - Transmit Counter Next"]
pub type TXNCTR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 16, O, u16>;
impl R {
    #[doc = "Bits 0:15 - Transmit Counter Next"]
    #[inline(always)]
    pub fn txnctr(&self) -> TXNCTR_R {
        TXNCTR_R::new((self.bits & 0xffff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:15 - Transmit Counter Next"]
    #[inline(always)]
    #[must_use]
    pub fn txnctr(&mut self) -> TXNCTR_W<TNCR_SPEC, 0> {
        TXNCTR_W::new(self)
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
#[doc = "Transmit Next Counter Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tncr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tncr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct TNCR_SPEC;
impl crate::RegisterSpec for TNCR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`tncr::R`](R) reader structure"]
impl crate::Readable for TNCR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`tncr::W`](W) writer structure"]
impl crate::Writable for TNCR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets TNCR to value 0"]
impl crate::Resettable for TNCR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
