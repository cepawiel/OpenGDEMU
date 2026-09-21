#[doc = "Register `BANK` reader"]
pub type R = crate::R<BANK_SPEC>;
#[doc = "Register `BANK` writer"]
pub type W = crate::W<BANK_SPEC>;
#[doc = "Field `BANK` reader - Bank Identifier"]
pub type BANK_R = crate::FieldReader;
#[doc = "Field `BANK` writer - Bank Identifier"]
pub type BANK_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 3, O>;
impl R {
    #[doc = "Bits 0:2 - Bank Identifier"]
    #[inline(always)]
    pub fn bank(&self) -> BANK_R {
        BANK_R::new((self.bits & 7) as u8)
    }
}
impl W {
    #[doc = "Bits 0:2 - Bank Identifier"]
    #[inline(always)]
    #[must_use]
    pub fn bank(&mut self) -> BANK_W<BANK_SPEC, 0> {
        BANK_W::new(self)
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
#[doc = "SMC Bank Address Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`bank::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`bank::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct BANK_SPEC;
impl crate::RegisterSpec for BANK_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`bank::R`](R) reader structure"]
impl crate::Readable for BANK_SPEC {}
#[doc = "`write(|w| ..)` method takes [`bank::W`](W) writer structure"]
impl crate::Writable for BANK_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets BANK to value 0"]
impl crate::Resettable for BANK_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
