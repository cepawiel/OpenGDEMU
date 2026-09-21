#[doc = "Register `DSCR2` reader"]
pub type R = crate::R<DSCR2_SPEC>;
#[doc = "Register `DSCR2` writer"]
pub type W = crate::W<DSCR2_SPEC>;
#[doc = "Field `DSCR` reader - Buffer Transfer Descriptor Address"]
pub type DSCR_R = crate::FieldReader<u32>;
#[doc = "Field `DSCR` writer - Buffer Transfer Descriptor Address"]
pub type DSCR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 30, O, u32>;
impl R {
    #[doc = "Bits 2:31 - Buffer Transfer Descriptor Address"]
    #[inline(always)]
    pub fn dscr(&self) -> DSCR_R {
        DSCR_R::new((self.bits >> 2) & 0x3fff_ffff)
    }
}
impl W {
    #[doc = "Bits 2:31 - Buffer Transfer Descriptor Address"]
    #[inline(always)]
    #[must_use]
    pub fn dscr(&mut self) -> DSCR_W<DSCR2_SPEC, 2> {
        DSCR_W::new(self)
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
#[doc = "DMAC Channel Descriptor Address Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dscr2::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dscr2::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct DSCR2_SPEC;
impl crate::RegisterSpec for DSCR2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`dscr2::R`](R) reader structure"]
impl crate::Readable for DSCR2_SPEC {}
#[doc = "`write(|w| ..)` method takes [`dscr2::W`](W) writer structure"]
impl crate::Writable for DSCR2_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets DSCR2 to value 0"]
impl crate::Resettable for DSCR2_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
