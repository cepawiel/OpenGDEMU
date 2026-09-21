#[doc = "Register `LAST` reader"]
pub type R = crate::R<LAST_SPEC>;
#[doc = "Register `LAST` writer"]
pub type W = crate::W<LAST_SPEC>;
#[doc = "Field `SLAST0` reader - Source Last"]
pub type SLAST0_R = crate::BitReader;
#[doc = "Field `SLAST0` writer - Source Last"]
pub type SLAST0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DLAST0` reader - Destination Last"]
pub type DLAST0_R = crate::BitReader;
#[doc = "Field `DLAST0` writer - Destination Last"]
pub type DLAST0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SLAST1` reader - Source Last"]
pub type SLAST1_R = crate::BitReader;
#[doc = "Field `SLAST1` writer - Source Last"]
pub type SLAST1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DLAST1` reader - Destination Last"]
pub type DLAST1_R = crate::BitReader;
#[doc = "Field `DLAST1` writer - Destination Last"]
pub type DLAST1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SLAST2` reader - Source Last"]
pub type SLAST2_R = crate::BitReader;
#[doc = "Field `SLAST2` writer - Source Last"]
pub type SLAST2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DLAST2` reader - Destination Last"]
pub type DLAST2_R = crate::BitReader;
#[doc = "Field `DLAST2` writer - Destination Last"]
pub type DLAST2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SLAST3` reader - Source Last"]
pub type SLAST3_R = crate::BitReader;
#[doc = "Field `SLAST3` writer - Source Last"]
pub type SLAST3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DLAST3` reader - Destination Last"]
pub type DLAST3_R = crate::BitReader;
#[doc = "Field `DLAST3` writer - Destination Last"]
pub type DLAST3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bit 0 - Source Last"]
    #[inline(always)]
    pub fn slast0(&self) -> SLAST0_R {
        SLAST0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Destination Last"]
    #[inline(always)]
    pub fn dlast0(&self) -> DLAST0_R {
        DLAST0_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Source Last"]
    #[inline(always)]
    pub fn slast1(&self) -> SLAST1_R {
        SLAST1_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Destination Last"]
    #[inline(always)]
    pub fn dlast1(&self) -> DLAST1_R {
        DLAST1_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Source Last"]
    #[inline(always)]
    pub fn slast2(&self) -> SLAST2_R {
        SLAST2_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Destination Last"]
    #[inline(always)]
    pub fn dlast2(&self) -> DLAST2_R {
        DLAST2_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Source Last"]
    #[inline(always)]
    pub fn slast3(&self) -> SLAST3_R {
        SLAST3_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - Destination Last"]
    #[inline(always)]
    pub fn dlast3(&self) -> DLAST3_R {
        DLAST3_R::new(((self.bits >> 7) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Source Last"]
    #[inline(always)]
    #[must_use]
    pub fn slast0(&mut self) -> SLAST0_W<LAST_SPEC, 0> {
        SLAST0_W::new(self)
    }
    #[doc = "Bit 1 - Destination Last"]
    #[inline(always)]
    #[must_use]
    pub fn dlast0(&mut self) -> DLAST0_W<LAST_SPEC, 1> {
        DLAST0_W::new(self)
    }
    #[doc = "Bit 2 - Source Last"]
    #[inline(always)]
    #[must_use]
    pub fn slast1(&mut self) -> SLAST1_W<LAST_SPEC, 2> {
        SLAST1_W::new(self)
    }
    #[doc = "Bit 3 - Destination Last"]
    #[inline(always)]
    #[must_use]
    pub fn dlast1(&mut self) -> DLAST1_W<LAST_SPEC, 3> {
        DLAST1_W::new(self)
    }
    #[doc = "Bit 4 - Source Last"]
    #[inline(always)]
    #[must_use]
    pub fn slast2(&mut self) -> SLAST2_W<LAST_SPEC, 4> {
        SLAST2_W::new(self)
    }
    #[doc = "Bit 5 - Destination Last"]
    #[inline(always)]
    #[must_use]
    pub fn dlast2(&mut self) -> DLAST2_W<LAST_SPEC, 5> {
        DLAST2_W::new(self)
    }
    #[doc = "Bit 6 - Source Last"]
    #[inline(always)]
    #[must_use]
    pub fn slast3(&mut self) -> SLAST3_W<LAST_SPEC, 6> {
        SLAST3_W::new(self)
    }
    #[doc = "Bit 7 - Destination Last"]
    #[inline(always)]
    #[must_use]
    pub fn dlast3(&mut self) -> DLAST3_W<LAST_SPEC, 7> {
        DLAST3_W::new(self)
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
#[doc = "DMAC Software Last Transfer Flag Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`last::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`last::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct LAST_SPEC;
impl crate::RegisterSpec for LAST_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`last::R`](R) reader structure"]
impl crate::Readable for LAST_SPEC {}
#[doc = "`write(|w| ..)` method takes [`last::W`](W) writer structure"]
impl crate::Writable for LAST_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets LAST to value 0"]
impl crate::Resettable for LAST_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
