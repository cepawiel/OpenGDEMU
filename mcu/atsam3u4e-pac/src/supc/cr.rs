#[doc = "Register `CR` writer"]
pub type W = crate::W<CR_SPEC>;
#[doc = "Voltage Regulator Off"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VROFF_AW {
    #[doc = "0: no effect."]
    NO_EFFECT = 0,
    #[doc = "1: if KEY is correct, asserts vddcore_nreset and stops the voltage regulator."]
    STOP_VREG = 1,
}
impl From<VROFF_AW> for bool {
    #[inline(always)]
    fn from(variant: VROFF_AW) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `VROFF` writer - Voltage Regulator Off"]
pub type VROFF_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, VROFF_AW>;
impl<'a, REG, const O: u8> VROFF_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "no effect."]
    #[inline(always)]
    pub fn no_effect(self) -> &'a mut crate::W<REG> {
        self.variant(VROFF_AW::NO_EFFECT)
    }
    #[doc = "if KEY is correct, asserts vddcore_nreset and stops the voltage regulator."]
    #[inline(always)]
    pub fn stop_vreg(self) -> &'a mut crate::W<REG> {
        self.variant(VROFF_AW::STOP_VREG)
    }
}
#[doc = "Crystal Oscillator Select"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum XTALSEL_AW {
    #[doc = "0: no effect."]
    NO_EFFECT = 0,
    #[doc = "1: if KEY is correct, switches the slow clock on the crystal oscillator output."]
    CRYSTAL_SEL = 1,
}
impl From<XTALSEL_AW> for bool {
    #[inline(always)]
    fn from(variant: XTALSEL_AW) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `XTALSEL` writer - Crystal Oscillator Select"]
pub type XTALSEL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, XTALSEL_AW>;
impl<'a, REG, const O: u8> XTALSEL_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "no effect."]
    #[inline(always)]
    pub fn no_effect(self) -> &'a mut crate::W<REG> {
        self.variant(XTALSEL_AW::NO_EFFECT)
    }
    #[doc = "if KEY is correct, switches the slow clock on the crystal oscillator output."]
    #[inline(always)]
    pub fn crystal_sel(self) -> &'a mut crate::W<REG> {
        self.variant(XTALSEL_AW::CRYSTAL_SEL)
    }
}
#[doc = "Field `KEY` writer - Password"]
pub type KEY_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
impl W {
    #[doc = "Bit 2 - Voltage Regulator Off"]
    #[inline(always)]
    #[must_use]
    pub fn vroff(&mut self) -> VROFF_W<CR_SPEC, 2> {
        VROFF_W::new(self)
    }
    #[doc = "Bit 3 - Crystal Oscillator Select"]
    #[inline(always)]
    #[must_use]
    pub fn xtalsel(&mut self) -> XTALSEL_W<CR_SPEC, 3> {
        XTALSEL_W::new(self)
    }
    #[doc = "Bits 24:31 - Password"]
    #[inline(always)]
    #[must_use]
    pub fn key(&mut self) -> KEY_W<CR_SPEC, 24> {
        KEY_W::new(self)
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
#[doc = "Supply Controller Control Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cr::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CR_SPEC;
impl crate::RegisterSpec for CR_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`cr::W`](W) writer structure"]
impl crate::Writable for CR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
