#[doc = "Register `TNPR` reader"]
pub type R = crate::R<TNPR_SPEC>;
#[doc = "Register `TNPR` writer"]
pub type W = crate::W<TNPR_SPEC>;
#[doc = "Field `TXNPTR` reader - Transmit Next Pointer"]
pub type TXNPTR_R = crate::FieldReader<u32>;
#[doc = "Field `TXNPTR` writer - Transmit Next Pointer"]
pub type TXNPTR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 32, O, u32>;
impl R {
    #[doc = "Bits 0:31 - Transmit Next Pointer"]
    #[inline(always)]
    pub fn txnptr(&self) -> TXNPTR_R {
        TXNPTR_R::new(self.bits)
    }
}
impl W {
    #[doc = "Bits 0:31 - Transmit Next Pointer"]
    #[inline(always)]
    #[must_use]
    pub fn txnptr(&mut self) -> TXNPTR_W<TNPR_SPEC, 0> {
        TXNPTR_W::new(self)
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
#[doc = "Transmit Next Pointer Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tnpr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tnpr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct TNPR_SPEC;
impl crate::RegisterSpec for TNPR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`tnpr::R`](R) reader structure"]
impl crate::Readable for TNPR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`tnpr::W`](W) writer structure"]
impl crate::Writable for TNPR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets TNPR to value 0"]
impl crate::Resettable for TNPR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
