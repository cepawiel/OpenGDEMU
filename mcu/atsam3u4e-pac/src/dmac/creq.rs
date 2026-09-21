#[doc = "Register `CREQ` reader"]
pub type R = crate::R<CREQ_SPEC>;
#[doc = "Register `CREQ` writer"]
pub type W = crate::W<CREQ_SPEC>;
#[doc = "Field `SCREQ0` reader - Source Chunk Request"]
pub type SCREQ0_R = crate::BitReader;
#[doc = "Field `SCREQ0` writer - Source Chunk Request"]
pub type SCREQ0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DCREQ0` reader - Destination Chunk Request"]
pub type DCREQ0_R = crate::BitReader;
#[doc = "Field `DCREQ0` writer - Destination Chunk Request"]
pub type DCREQ0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SCREQ1` reader - Source Chunk Request"]
pub type SCREQ1_R = crate::BitReader;
#[doc = "Field `SCREQ1` writer - Source Chunk Request"]
pub type SCREQ1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DCREQ1` reader - Destination Chunk Request"]
pub type DCREQ1_R = crate::BitReader;
#[doc = "Field `DCREQ1` writer - Destination Chunk Request"]
pub type DCREQ1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SCREQ2` reader - Source Chunk Request"]
pub type SCREQ2_R = crate::BitReader;
#[doc = "Field `SCREQ2` writer - Source Chunk Request"]
pub type SCREQ2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DCREQ2` reader - Destination Chunk Request"]
pub type DCREQ2_R = crate::BitReader;
#[doc = "Field `DCREQ2` writer - Destination Chunk Request"]
pub type DCREQ2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SCREQ3` reader - Source Chunk Request"]
pub type SCREQ3_R = crate::BitReader;
#[doc = "Field `SCREQ3` writer - Source Chunk Request"]
pub type SCREQ3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DCREQ3` reader - Destination Chunk Request"]
pub type DCREQ3_R = crate::BitReader;
#[doc = "Field `DCREQ3` writer - Destination Chunk Request"]
pub type DCREQ3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bit 0 - Source Chunk Request"]
    #[inline(always)]
    pub fn screq0(&self) -> SCREQ0_R {
        SCREQ0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Destination Chunk Request"]
    #[inline(always)]
    pub fn dcreq0(&self) -> DCREQ0_R {
        DCREQ0_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Source Chunk Request"]
    #[inline(always)]
    pub fn screq1(&self) -> SCREQ1_R {
        SCREQ1_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Destination Chunk Request"]
    #[inline(always)]
    pub fn dcreq1(&self) -> DCREQ1_R {
        DCREQ1_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Source Chunk Request"]
    #[inline(always)]
    pub fn screq2(&self) -> SCREQ2_R {
        SCREQ2_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Destination Chunk Request"]
    #[inline(always)]
    pub fn dcreq2(&self) -> DCREQ2_R {
        DCREQ2_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Source Chunk Request"]
    #[inline(always)]
    pub fn screq3(&self) -> SCREQ3_R {
        SCREQ3_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - Destination Chunk Request"]
    #[inline(always)]
    pub fn dcreq3(&self) -> DCREQ3_R {
        DCREQ3_R::new(((self.bits >> 7) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Source Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn screq0(&mut self) -> SCREQ0_W<CREQ_SPEC, 0> {
        SCREQ0_W::new(self)
    }
    #[doc = "Bit 1 - Destination Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn dcreq0(&mut self) -> DCREQ0_W<CREQ_SPEC, 1> {
        DCREQ0_W::new(self)
    }
    #[doc = "Bit 2 - Source Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn screq1(&mut self) -> SCREQ1_W<CREQ_SPEC, 2> {
        SCREQ1_W::new(self)
    }
    #[doc = "Bit 3 - Destination Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn dcreq1(&mut self) -> DCREQ1_W<CREQ_SPEC, 3> {
        DCREQ1_W::new(self)
    }
    #[doc = "Bit 4 - Source Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn screq2(&mut self) -> SCREQ2_W<CREQ_SPEC, 4> {
        SCREQ2_W::new(self)
    }
    #[doc = "Bit 5 - Destination Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn dcreq2(&mut self) -> DCREQ2_W<CREQ_SPEC, 5> {
        DCREQ2_W::new(self)
    }
    #[doc = "Bit 6 - Source Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn screq3(&mut self) -> SCREQ3_W<CREQ_SPEC, 6> {
        SCREQ3_W::new(self)
    }
    #[doc = "Bit 7 - Destination Chunk Request"]
    #[inline(always)]
    #[must_use]
    pub fn dcreq3(&mut self) -> DCREQ3_W<CREQ_SPEC, 7> {
        DCREQ3_W::new(self)
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
#[doc = "DMAC Software Chunk Transfer Request Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`creq::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`creq::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CREQ_SPEC;
impl crate::RegisterSpec for CREQ_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`creq::R`](R) reader structure"]
impl crate::Readable for CREQ_SPEC {}
#[doc = "`write(|w| ..)` method takes [`creq::W`](W) writer structure"]
impl crate::Writable for CREQ_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CREQ to value 0"]
impl crate::Resettable for CREQ_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
