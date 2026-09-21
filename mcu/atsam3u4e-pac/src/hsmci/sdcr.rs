#[doc = "Register `SDCR` reader"]
pub type R = crate::R<SDCR_SPEC>;
#[doc = "Register `SDCR` writer"]
pub type W = crate::W<SDCR_SPEC>;
#[doc = "Field `SDCSEL` reader - SDCard/SDIO Slot"]
pub type SDCSEL_R = crate::FieldReader<SDCSEL_A>;
#[doc = "SDCard/SDIO Slot\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SDCSEL_A {
    #[doc = "0: Slot A is selected."]
    SLOTA = 0,
    #[doc = "1: -"]
    SLOTB = 1,
    #[doc = "2: -"]
    SLOTC = 2,
    #[doc = "3: -"]
    SLOTD = 3,
}
impl From<SDCSEL_A> for u8 {
    #[inline(always)]
    fn from(variant: SDCSEL_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for SDCSEL_A {
    type Ux = u8;
}
impl SDCSEL_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> SDCSEL_A {
        match self.bits {
            0 => SDCSEL_A::SLOTA,
            1 => SDCSEL_A::SLOTB,
            2 => SDCSEL_A::SLOTC,
            3 => SDCSEL_A::SLOTD,
            _ => unreachable!(),
        }
    }
    #[doc = "Slot A is selected."]
    #[inline(always)]
    pub fn is_slota(&self) -> bool {
        *self == SDCSEL_A::SLOTA
    }
    #[doc = "-"]
    #[inline(always)]
    pub fn is_slotb(&self) -> bool {
        *self == SDCSEL_A::SLOTB
    }
    #[doc = "-"]
    #[inline(always)]
    pub fn is_slotc(&self) -> bool {
        *self == SDCSEL_A::SLOTC
    }
    #[doc = "-"]
    #[inline(always)]
    pub fn is_slotd(&self) -> bool {
        *self == SDCSEL_A::SLOTD
    }
}
#[doc = "Field `SDCSEL` writer - SDCard/SDIO Slot"]
pub type SDCSEL_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, SDCSEL_A>;
impl<'a, REG, const O: u8> SDCSEL_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Slot A is selected."]
    #[inline(always)]
    pub fn slota(self) -> &'a mut crate::W<REG> {
        self.variant(SDCSEL_A::SLOTA)
    }
    #[doc = "-"]
    #[inline(always)]
    pub fn slotb(self) -> &'a mut crate::W<REG> {
        self.variant(SDCSEL_A::SLOTB)
    }
    #[doc = "-"]
    #[inline(always)]
    pub fn slotc(self) -> &'a mut crate::W<REG> {
        self.variant(SDCSEL_A::SLOTC)
    }
    #[doc = "-"]
    #[inline(always)]
    pub fn slotd(self) -> &'a mut crate::W<REG> {
        self.variant(SDCSEL_A::SLOTD)
    }
}
#[doc = "Field `SDCBUS` reader - SDCard/SDIO Bus Width"]
pub type SDCBUS_R = crate::FieldReader<SDCBUS_A>;
#[doc = "SDCard/SDIO Bus Width\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SDCBUS_A {
    #[doc = "0: 1 bit"]
    _1 = 0,
    #[doc = "2: 4 bit"]
    _4 = 2,
    #[doc = "3: 8 bit"]
    _8 = 3,
}
impl From<SDCBUS_A> for u8 {
    #[inline(always)]
    fn from(variant: SDCBUS_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for SDCBUS_A {
    type Ux = u8;
}
impl SDCBUS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<SDCBUS_A> {
        match self.bits {
            0 => Some(SDCBUS_A::_1),
            2 => Some(SDCBUS_A::_4),
            3 => Some(SDCBUS_A::_8),
            _ => None,
        }
    }
    #[doc = "1 bit"]
    #[inline(always)]
    pub fn is_1(&self) -> bool {
        *self == SDCBUS_A::_1
    }
    #[doc = "4 bit"]
    #[inline(always)]
    pub fn is_4(&self) -> bool {
        *self == SDCBUS_A::_4
    }
    #[doc = "8 bit"]
    #[inline(always)]
    pub fn is_8(&self) -> bool {
        *self == SDCBUS_A::_8
    }
}
#[doc = "Field `SDCBUS` writer - SDCard/SDIO Bus Width"]
pub type SDCBUS_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 2, O, SDCBUS_A>;
impl<'a, REG, const O: u8> SDCBUS_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "1 bit"]
    #[inline(always)]
    pub fn _1(self) -> &'a mut crate::W<REG> {
        self.variant(SDCBUS_A::_1)
    }
    #[doc = "4 bit"]
    #[inline(always)]
    pub fn _4(self) -> &'a mut crate::W<REG> {
        self.variant(SDCBUS_A::_4)
    }
    #[doc = "8 bit"]
    #[inline(always)]
    pub fn _8(self) -> &'a mut crate::W<REG> {
        self.variant(SDCBUS_A::_8)
    }
}
impl R {
    #[doc = "Bits 0:1 - SDCard/SDIO Slot"]
    #[inline(always)]
    pub fn sdcsel(&self) -> SDCSEL_R {
        SDCSEL_R::new((self.bits & 3) as u8)
    }
    #[doc = "Bits 6:7 - SDCard/SDIO Bus Width"]
    #[inline(always)]
    pub fn sdcbus(&self) -> SDCBUS_R {
        SDCBUS_R::new(((self.bits >> 6) & 3) as u8)
    }
}
impl W {
    #[doc = "Bits 0:1 - SDCard/SDIO Slot"]
    #[inline(always)]
    #[must_use]
    pub fn sdcsel(&mut self) -> SDCSEL_W<SDCR_SPEC, 0> {
        SDCSEL_W::new(self)
    }
    #[doc = "Bits 6:7 - SDCard/SDIO Bus Width"]
    #[inline(always)]
    #[must_use]
    pub fn sdcbus(&mut self) -> SDCBUS_W<SDCR_SPEC, 6> {
        SDCBUS_W::new(self)
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
#[doc = "SD/SDIO Card Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sdcr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`sdcr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct SDCR_SPEC;
impl crate::RegisterSpec for SDCR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`sdcr::R`](R) reader structure"]
impl crate::Readable for SDCR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`sdcr::W`](W) writer structure"]
impl crate::Writable for SDCR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets SDCR to value 0"]
impl crate::Resettable for SDCR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
