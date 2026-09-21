#[doc = "Register `TIMINGS2` reader"]
pub type R = crate::R<TIMINGS2_SPEC>;
#[doc = "Register `TIMINGS2` writer"]
pub type W = crate::W<TIMINGS2_SPEC>;
#[doc = "Field `TCLR` reader - CLE to REN Low Delay"]
pub type TCLR_R = crate::FieldReader;
#[doc = "Field `TCLR` writer - CLE to REN Low Delay"]
pub type TCLR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `TADL` reader - ALE to Data Start"]
pub type TADL_R = crate::FieldReader;
#[doc = "Field `TADL` writer - ALE to Data Start"]
pub type TADL_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `TAR` reader - ALE to REN Low Delay"]
pub type TAR_R = crate::FieldReader;
#[doc = "Field `TAR` writer - ALE to REN Low Delay"]
pub type TAR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `OCMS` reader - Off Chip Memory Scrambling Enable"]
pub type OCMS_R = crate::BitReader;
#[doc = "Field `OCMS` writer - Off Chip Memory Scrambling Enable"]
pub type OCMS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TRR` reader - Ready to REN Low Delay"]
pub type TRR_R = crate::FieldReader;
#[doc = "Field `TRR` writer - Ready to REN Low Delay"]
pub type TRR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `TWB` reader - WEN High to REN to Busy"]
pub type TWB_R = crate::FieldReader;
#[doc = "Field `TWB` writer - WEN High to REN to Busy"]
pub type TWB_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `RBNSEL` reader - Ready/Busy Line Selection"]
pub type RBNSEL_R = crate::FieldReader;
#[doc = "Field `RBNSEL` writer - Ready/Busy Line Selection"]
pub type RBNSEL_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 3, O>;
#[doc = "Field `NFSEL` reader - NAND Flash Selection"]
pub type NFSEL_R = crate::BitReader;
#[doc = "Field `NFSEL` writer - NAND Flash Selection"]
pub type NFSEL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bits 0:3 - CLE to REN Low Delay"]
    #[inline(always)]
    pub fn tclr(&self) -> TCLR_R {
        TCLR_R::new((self.bits & 0x0f) as u8)
    }
    #[doc = "Bits 4:7 - ALE to Data Start"]
    #[inline(always)]
    pub fn tadl(&self) -> TADL_R {
        TADL_R::new(((self.bits >> 4) & 0x0f) as u8)
    }
    #[doc = "Bits 8:11 - ALE to REN Low Delay"]
    #[inline(always)]
    pub fn tar(&self) -> TAR_R {
        TAR_R::new(((self.bits >> 8) & 0x0f) as u8)
    }
    #[doc = "Bit 12 - Off Chip Memory Scrambling Enable"]
    #[inline(always)]
    pub fn ocms(&self) -> OCMS_R {
        OCMS_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bits 16:19 - Ready to REN Low Delay"]
    #[inline(always)]
    pub fn trr(&self) -> TRR_R {
        TRR_R::new(((self.bits >> 16) & 0x0f) as u8)
    }
    #[doc = "Bits 24:27 - WEN High to REN to Busy"]
    #[inline(always)]
    pub fn twb(&self) -> TWB_R {
        TWB_R::new(((self.bits >> 24) & 0x0f) as u8)
    }
    #[doc = "Bits 28:30 - Ready/Busy Line Selection"]
    #[inline(always)]
    pub fn rbnsel(&self) -> RBNSEL_R {
        RBNSEL_R::new(((self.bits >> 28) & 7) as u8)
    }
    #[doc = "Bit 31 - NAND Flash Selection"]
    #[inline(always)]
    pub fn nfsel(&self) -> NFSEL_R {
        NFSEL_R::new(((self.bits >> 31) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:3 - CLE to REN Low Delay"]
    #[inline(always)]
    #[must_use]
    pub fn tclr(&mut self) -> TCLR_W<TIMINGS2_SPEC, 0> {
        TCLR_W::new(self)
    }
    #[doc = "Bits 4:7 - ALE to Data Start"]
    #[inline(always)]
    #[must_use]
    pub fn tadl(&mut self) -> TADL_W<TIMINGS2_SPEC, 4> {
        TADL_W::new(self)
    }
    #[doc = "Bits 8:11 - ALE to REN Low Delay"]
    #[inline(always)]
    #[must_use]
    pub fn tar(&mut self) -> TAR_W<TIMINGS2_SPEC, 8> {
        TAR_W::new(self)
    }
    #[doc = "Bit 12 - Off Chip Memory Scrambling Enable"]
    #[inline(always)]
    #[must_use]
    pub fn ocms(&mut self) -> OCMS_W<TIMINGS2_SPEC, 12> {
        OCMS_W::new(self)
    }
    #[doc = "Bits 16:19 - Ready to REN Low Delay"]
    #[inline(always)]
    #[must_use]
    pub fn trr(&mut self) -> TRR_W<TIMINGS2_SPEC, 16> {
        TRR_W::new(self)
    }
    #[doc = "Bits 24:27 - WEN High to REN to Busy"]
    #[inline(always)]
    #[must_use]
    pub fn twb(&mut self) -> TWB_W<TIMINGS2_SPEC, 24> {
        TWB_W::new(self)
    }
    #[doc = "Bits 28:30 - Ready/Busy Line Selection"]
    #[inline(always)]
    #[must_use]
    pub fn rbnsel(&mut self) -> RBNSEL_W<TIMINGS2_SPEC, 28> {
        RBNSEL_W::new(self)
    }
    #[doc = "Bit 31 - NAND Flash Selection"]
    #[inline(always)]
    #[must_use]
    pub fn nfsel(&mut self) -> NFSEL_W<TIMINGS2_SPEC, 31> {
        NFSEL_W::new(self)
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
#[doc = "SMC Timings Register (CS_number = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`timings2::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`timings2::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct TIMINGS2_SPEC;
impl crate::RegisterSpec for TIMINGS2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`timings2::R`](R) reader structure"]
impl crate::Readable for TIMINGS2_SPEC {}
#[doc = "`write(|w| ..)` method takes [`timings2::W`](W) writer structure"]
impl crate::Writable for TIMINGS2_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets TIMINGS2 to value 0"]
impl crate::Resettable for TIMINGS2_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
