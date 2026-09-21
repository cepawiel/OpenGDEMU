#[doc = "Register `CHER` writer"]
pub type W = crate::W<CHER_SPEC>;
#[doc = "Field `ENA0` writer - Enable \\[3:0\\]"]
pub type ENA0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ENA1` writer - Enable \\[3:0\\]"]
pub type ENA1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ENA2` writer - Enable \\[3:0\\]"]
pub type ENA2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ENA3` writer - Enable \\[3:0\\]"]
pub type ENA3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SUSP0` writer - Suspend \\[3:0\\]"]
pub type SUSP0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SUSP1` writer - Suspend \\[3:0\\]"]
pub type SUSP1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SUSP2` writer - Suspend \\[3:0\\]"]
pub type SUSP2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SUSP3` writer - Suspend \\[3:0\\]"]
pub type SUSP3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `KEEP0` writer - Keep on \\[3:0\\]"]
pub type KEEP0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `KEEP1` writer - Keep on \\[3:0\\]"]
pub type KEEP1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `KEEP2` writer - Keep on \\[3:0\\]"]
pub type KEEP2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `KEEP3` writer - Keep on \\[3:0\\]"]
pub type KEEP3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 0 - Enable \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn ena0(&mut self) -> ENA0_W<CHER_SPEC, 0> {
        ENA0_W::new(self)
    }
    #[doc = "Bit 1 - Enable \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn ena1(&mut self) -> ENA1_W<CHER_SPEC, 1> {
        ENA1_W::new(self)
    }
    #[doc = "Bit 2 - Enable \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn ena2(&mut self) -> ENA2_W<CHER_SPEC, 2> {
        ENA2_W::new(self)
    }
    #[doc = "Bit 3 - Enable \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn ena3(&mut self) -> ENA3_W<CHER_SPEC, 3> {
        ENA3_W::new(self)
    }
    #[doc = "Bit 8 - Suspend \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn susp0(&mut self) -> SUSP0_W<CHER_SPEC, 8> {
        SUSP0_W::new(self)
    }
    #[doc = "Bit 9 - Suspend \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn susp1(&mut self) -> SUSP1_W<CHER_SPEC, 9> {
        SUSP1_W::new(self)
    }
    #[doc = "Bit 10 - Suspend \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn susp2(&mut self) -> SUSP2_W<CHER_SPEC, 10> {
        SUSP2_W::new(self)
    }
    #[doc = "Bit 11 - Suspend \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn susp3(&mut self) -> SUSP3_W<CHER_SPEC, 11> {
        SUSP3_W::new(self)
    }
    #[doc = "Bit 24 - Keep on \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn keep0(&mut self) -> KEEP0_W<CHER_SPEC, 24> {
        KEEP0_W::new(self)
    }
    #[doc = "Bit 25 - Keep on \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn keep1(&mut self) -> KEEP1_W<CHER_SPEC, 25> {
        KEEP1_W::new(self)
    }
    #[doc = "Bit 26 - Keep on \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn keep2(&mut self) -> KEEP2_W<CHER_SPEC, 26> {
        KEEP2_W::new(self)
    }
    #[doc = "Bit 27 - Keep on \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn keep3(&mut self) -> KEEP3_W<CHER_SPEC, 27> {
        KEEP3_W::new(self)
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
#[doc = "DMAC Channel Handler Enable Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cher::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CHER_SPEC;
impl crate::RegisterSpec for CHER_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`cher::W`](W) writer structure"]
impl crate::Writable for CHER_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
