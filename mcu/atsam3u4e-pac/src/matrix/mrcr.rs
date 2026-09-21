#[doc = "Register `MRCR` reader"]
pub type R = crate::R<MRCR_SPEC>;
#[doc = "Register `MRCR` writer"]
pub type W = crate::W<MRCR_SPEC>;
#[doc = "Field `RCB0` reader - Remap Command Bit for AHB Master 0"]
pub type RCB0_R = crate::BitReader;
#[doc = "Field `RCB0` writer - Remap Command Bit for AHB Master 0"]
pub type RCB0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RCB1` reader - Remap Command Bit for AHB Master 1"]
pub type RCB1_R = crate::BitReader;
#[doc = "Field `RCB1` writer - Remap Command Bit for AHB Master 1"]
pub type RCB1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RCB2` reader - Remap Command Bit for AHB Master 2"]
pub type RCB2_R = crate::BitReader;
#[doc = "Field `RCB2` writer - Remap Command Bit for AHB Master 2"]
pub type RCB2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RCB3` reader - Remap Command Bit for AHB Master 3"]
pub type RCB3_R = crate::BitReader;
#[doc = "Field `RCB3` writer - Remap Command Bit for AHB Master 3"]
pub type RCB3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RCB4` reader - Remap Command Bit for AHB Master 4"]
pub type RCB4_R = crate::BitReader;
#[doc = "Field `RCB4` writer - Remap Command Bit for AHB Master 4"]
pub type RCB4_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bit 0 - Remap Command Bit for AHB Master 0"]
    #[inline(always)]
    pub fn rcb0(&self) -> RCB0_R {
        RCB0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Remap Command Bit for AHB Master 1"]
    #[inline(always)]
    pub fn rcb1(&self) -> RCB1_R {
        RCB1_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Remap Command Bit for AHB Master 2"]
    #[inline(always)]
    pub fn rcb2(&self) -> RCB2_R {
        RCB2_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Remap Command Bit for AHB Master 3"]
    #[inline(always)]
    pub fn rcb3(&self) -> RCB3_R {
        RCB3_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Remap Command Bit for AHB Master 4"]
    #[inline(always)]
    pub fn rcb4(&self) -> RCB4_R {
        RCB4_R::new(((self.bits >> 4) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Remap Command Bit for AHB Master 0"]
    #[inline(always)]
    #[must_use]
    pub fn rcb0(&mut self) -> RCB0_W<MRCR_SPEC, 0> {
        RCB0_W::new(self)
    }
    #[doc = "Bit 1 - Remap Command Bit for AHB Master 1"]
    #[inline(always)]
    #[must_use]
    pub fn rcb1(&mut self) -> RCB1_W<MRCR_SPEC, 1> {
        RCB1_W::new(self)
    }
    #[doc = "Bit 2 - Remap Command Bit for AHB Master 2"]
    #[inline(always)]
    #[must_use]
    pub fn rcb2(&mut self) -> RCB2_W<MRCR_SPEC, 2> {
        RCB2_W::new(self)
    }
    #[doc = "Bit 3 - Remap Command Bit for AHB Master 3"]
    #[inline(always)]
    #[must_use]
    pub fn rcb3(&mut self) -> RCB3_W<MRCR_SPEC, 3> {
        RCB3_W::new(self)
    }
    #[doc = "Bit 4 - Remap Command Bit for AHB Master 4"]
    #[inline(always)]
    #[must_use]
    pub fn rcb4(&mut self) -> RCB4_W<MRCR_SPEC, 4> {
        RCB4_W::new(self)
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
#[doc = "Master Remap Control Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`mrcr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`mrcr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct MRCR_SPEC;
impl crate::RegisterSpec for MRCR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`mrcr::R`](R) reader structure"]
impl crate::Readable for MRCR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`mrcr::W`](W) writer structure"]
impl crate::Writable for MRCR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets MRCR to value 0"]
impl crate::Resettable for MRCR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
