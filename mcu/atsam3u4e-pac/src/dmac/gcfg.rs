#[doc = "Register `GCFG` reader"]
pub type R = crate::R<GCFG_SPEC>;
#[doc = "Register `GCFG` writer"]
pub type W = crate::W<GCFG_SPEC>;
#[doc = "Field `ARB_CFG` reader - Arbiter Configuration"]
pub type ARB_CFG_R = crate::BitReader<ARB_CFG_A>;
#[doc = "Arbiter Configuration\n\nValue on reset: 1"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ARB_CFG_A {
    #[doc = "0: Fixed priority arbiter."]
    FIXED = 0,
    #[doc = "1: Modified round robin arbiter."]
    ROUND_ROBIN = 1,
}
impl From<ARB_CFG_A> for bool {
    #[inline(always)]
    fn from(variant: ARB_CFG_A) -> Self {
        variant as u8 != 0
    }
}
impl ARB_CFG_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ARB_CFG_A {
        match self.bits {
            false => ARB_CFG_A::FIXED,
            true => ARB_CFG_A::ROUND_ROBIN,
        }
    }
    #[doc = "Fixed priority arbiter."]
    #[inline(always)]
    pub fn is_fixed(&self) -> bool {
        *self == ARB_CFG_A::FIXED
    }
    #[doc = "Modified round robin arbiter."]
    #[inline(always)]
    pub fn is_round_robin(&self) -> bool {
        *self == ARB_CFG_A::ROUND_ROBIN
    }
}
#[doc = "Field `ARB_CFG` writer - Arbiter Configuration"]
pub type ARB_CFG_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, ARB_CFG_A>;
impl<'a, REG, const O: u8> ARB_CFG_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "Fixed priority arbiter."]
    #[inline(always)]
    pub fn fixed(self) -> &'a mut crate::W<REG> {
        self.variant(ARB_CFG_A::FIXED)
    }
    #[doc = "Modified round robin arbiter."]
    #[inline(always)]
    pub fn round_robin(self) -> &'a mut crate::W<REG> {
        self.variant(ARB_CFG_A::ROUND_ROBIN)
    }
}
impl R {
    #[doc = "Bit 4 - Arbiter Configuration"]
    #[inline(always)]
    pub fn arb_cfg(&self) -> ARB_CFG_R {
        ARB_CFG_R::new(((self.bits >> 4) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 4 - Arbiter Configuration"]
    #[inline(always)]
    #[must_use]
    pub fn arb_cfg(&mut self) -> ARB_CFG_W<GCFG_SPEC, 4> {
        ARB_CFG_W::new(self)
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
#[doc = "DMAC Global Configuration Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`gcfg::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`gcfg::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct GCFG_SPEC;
impl crate::RegisterSpec for GCFG_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`gcfg::R`](R) reader structure"]
impl crate::Readable for GCFG_SPEC {}
#[doc = "`write(|w| ..)` method takes [`gcfg::W`](W) writer structure"]
impl crate::Writable for GCFG_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets GCFG to value 0x10"]
impl crate::Resettable for GCFG_SPEC {
    const RESET_VALUE: Self::Ux = 0x10;
}
