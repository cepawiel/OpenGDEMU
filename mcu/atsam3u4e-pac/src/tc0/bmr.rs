#[doc = "Register `BMR` reader"]
pub type R = crate::R<BMR_SPEC>;
#[doc = "Register `BMR` writer"]
pub type W = crate::W<BMR_SPEC>;
#[doc = "Field `TC0XC0S` reader - External Clock Signal 0 Selection"]
pub type TC0XC0S_R = crate::FieldReader<TC0XC0S_A>;
#[doc = "External Clock Signal 0 Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TC0XC0S_A {
    #[doc = "0: Signal connected to XC0: TCLK0"]
    TCLK0 = 0,
    #[doc = "2: Signal connected to XC0: TIOA1"]
    TIOA1 = 2,
    #[doc = "3: Signal connected to XC0: TIOA2"]
    TIOA2 = 3,
}
impl From<TC0XC0S_A> for u8 {
    #[inline(always)]
    fn from(variant: TC0XC0S_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for TC0XC0S_A {
    type Ux = u8;
}
impl TC0XC0S_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<TC0XC0S_A> {
        match self.bits {
            0 => Some(TC0XC0S_A::TCLK0),
            2 => Some(TC0XC0S_A::TIOA1),
            3 => Some(TC0XC0S_A::TIOA2),
            _ => None,
        }
    }
    #[doc = "Signal connected to XC0: TCLK0"]
    #[inline(always)]
    pub fn is_tclk0(&self) -> bool {
        *self == TC0XC0S_A::TCLK0
    }
    #[doc = "Signal connected to XC0: TIOA1"]
    #[inline(always)]
    pub fn is_tioa1(&self) -> bool {
        *self == TC0XC0S_A::TIOA1
    }
    #[doc = "Signal connected to XC0: TIOA2"]
    #[inline(always)]
    pub fn is_tioa2(&self) -> bool {
        *self == TC0XC0S_A::TIOA2
    }
}
#[doc = "Field `TC0XC0S` writer - External Clock Signal 0 Selection"]
pub type TC0XC0S_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 2, O, TC0XC0S_A>;
impl<'a, REG, const O: u8> TC0XC0S_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Signal connected to XC0: TCLK0"]
    #[inline(always)]
    pub fn tclk0(self) -> &'a mut crate::W<REG> {
        self.variant(TC0XC0S_A::TCLK0)
    }
    #[doc = "Signal connected to XC0: TIOA1"]
    #[inline(always)]
    pub fn tioa1(self) -> &'a mut crate::W<REG> {
        self.variant(TC0XC0S_A::TIOA1)
    }
    #[doc = "Signal connected to XC0: TIOA2"]
    #[inline(always)]
    pub fn tioa2(self) -> &'a mut crate::W<REG> {
        self.variant(TC0XC0S_A::TIOA2)
    }
}
#[doc = "Field `TC1XC1S` reader - External Clock Signal 1 Selection"]
pub type TC1XC1S_R = crate::FieldReader<TC1XC1S_A>;
#[doc = "External Clock Signal 1 Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TC1XC1S_A {
    #[doc = "0: Signal connected to XC1: TCLK1"]
    TCLK1 = 0,
    #[doc = "2: Signal connected to XC1: TIOA0"]
    TIOA0 = 2,
    #[doc = "3: Signal connected to XC1: TIOA2"]
    TIOA2 = 3,
}
impl From<TC1XC1S_A> for u8 {
    #[inline(always)]
    fn from(variant: TC1XC1S_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for TC1XC1S_A {
    type Ux = u8;
}
impl TC1XC1S_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<TC1XC1S_A> {
        match self.bits {
            0 => Some(TC1XC1S_A::TCLK1),
            2 => Some(TC1XC1S_A::TIOA0),
            3 => Some(TC1XC1S_A::TIOA2),
            _ => None,
        }
    }
    #[doc = "Signal connected to XC1: TCLK1"]
    #[inline(always)]
    pub fn is_tclk1(&self) -> bool {
        *self == TC1XC1S_A::TCLK1
    }
    #[doc = "Signal connected to XC1: TIOA0"]
    #[inline(always)]
    pub fn is_tioa0(&self) -> bool {
        *self == TC1XC1S_A::TIOA0
    }
    #[doc = "Signal connected to XC1: TIOA2"]
    #[inline(always)]
    pub fn is_tioa2(&self) -> bool {
        *self == TC1XC1S_A::TIOA2
    }
}
#[doc = "Field `TC1XC1S` writer - External Clock Signal 1 Selection"]
pub type TC1XC1S_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 2, O, TC1XC1S_A>;
impl<'a, REG, const O: u8> TC1XC1S_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Signal connected to XC1: TCLK1"]
    #[inline(always)]
    pub fn tclk1(self) -> &'a mut crate::W<REG> {
        self.variant(TC1XC1S_A::TCLK1)
    }
    #[doc = "Signal connected to XC1: TIOA0"]
    #[inline(always)]
    pub fn tioa0(self) -> &'a mut crate::W<REG> {
        self.variant(TC1XC1S_A::TIOA0)
    }
    #[doc = "Signal connected to XC1: TIOA2"]
    #[inline(always)]
    pub fn tioa2(self) -> &'a mut crate::W<REG> {
        self.variant(TC1XC1S_A::TIOA2)
    }
}
#[doc = "Field `TC2XC2S` reader - External Clock Signal 2 Selection"]
pub type TC2XC2S_R = crate::FieldReader<TC2XC2S_A>;
#[doc = "External Clock Signal 2 Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TC2XC2S_A {
    #[doc = "0: Signal connected to XC2: TCLK2"]
    TCLK2 = 0,
    #[doc = "2: Signal connected to XC2: TIOA1"]
    TIOA1 = 2,
    #[doc = "3: Signal connected to XC2: TIOA2"]
    TIOA2 = 3,
}
impl From<TC2XC2S_A> for u8 {
    #[inline(always)]
    fn from(variant: TC2XC2S_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for TC2XC2S_A {
    type Ux = u8;
}
impl TC2XC2S_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<TC2XC2S_A> {
        match self.bits {
            0 => Some(TC2XC2S_A::TCLK2),
            2 => Some(TC2XC2S_A::TIOA1),
            3 => Some(TC2XC2S_A::TIOA2),
            _ => None,
        }
    }
    #[doc = "Signal connected to XC2: TCLK2"]
    #[inline(always)]
    pub fn is_tclk2(&self) -> bool {
        *self == TC2XC2S_A::TCLK2
    }
    #[doc = "Signal connected to XC2: TIOA1"]
    #[inline(always)]
    pub fn is_tioa1(&self) -> bool {
        *self == TC2XC2S_A::TIOA1
    }
    #[doc = "Signal connected to XC2: TIOA2"]
    #[inline(always)]
    pub fn is_tioa2(&self) -> bool {
        *self == TC2XC2S_A::TIOA2
    }
}
#[doc = "Field `TC2XC2S` writer - External Clock Signal 2 Selection"]
pub type TC2XC2S_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 2, O, TC2XC2S_A>;
impl<'a, REG, const O: u8> TC2XC2S_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Signal connected to XC2: TCLK2"]
    #[inline(always)]
    pub fn tclk2(self) -> &'a mut crate::W<REG> {
        self.variant(TC2XC2S_A::TCLK2)
    }
    #[doc = "Signal connected to XC2: TIOA1"]
    #[inline(always)]
    pub fn tioa1(self) -> &'a mut crate::W<REG> {
        self.variant(TC2XC2S_A::TIOA1)
    }
    #[doc = "Signal connected to XC2: TIOA2"]
    #[inline(always)]
    pub fn tioa2(self) -> &'a mut crate::W<REG> {
        self.variant(TC2XC2S_A::TIOA2)
    }
}
#[doc = "Field `QDEN` reader - Quadrature Decoder ENabled"]
pub type QDEN_R = crate::BitReader;
#[doc = "Field `QDEN` writer - Quadrature Decoder ENabled"]
pub type QDEN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `POSEN` reader - POSition ENabled"]
pub type POSEN_R = crate::BitReader;
#[doc = "Field `POSEN` writer - POSition ENabled"]
pub type POSEN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SPEEDEN` reader - SPEED ENabled"]
pub type SPEEDEN_R = crate::BitReader;
#[doc = "Field `SPEEDEN` writer - SPEED ENabled"]
pub type SPEEDEN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `QDTRANS` reader - Quadrature Decoding TRANSparent"]
pub type QDTRANS_R = crate::BitReader;
#[doc = "Field `QDTRANS` writer - Quadrature Decoding TRANSparent"]
pub type QDTRANS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `EDGPHA` reader - EDGe on PHA count mode"]
pub type EDGPHA_R = crate::BitReader;
#[doc = "Field `EDGPHA` writer - EDGe on PHA count mode"]
pub type EDGPHA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `INVA` reader - INVerted phA"]
pub type INVA_R = crate::BitReader;
#[doc = "Field `INVA` writer - INVerted phA"]
pub type INVA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `INVB` reader - INVerted phB"]
pub type INVB_R = crate::BitReader;
#[doc = "Field `INVB` writer - INVerted phB"]
pub type INVB_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `INVIDX` reader - INVerted InDeX"]
pub type INVIDX_R = crate::BitReader;
#[doc = "Field `INVIDX` writer - INVerted InDeX"]
pub type INVIDX_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SWAP` reader - SWAP PHA and PHB"]
pub type SWAP_R = crate::BitReader;
#[doc = "Field `SWAP` writer - SWAP PHA and PHB"]
pub type SWAP_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `IDXPHB` reader - InDeX pin is PHB pin"]
pub type IDXPHB_R = crate::BitReader;
#[doc = "Field `IDXPHB` writer - InDeX pin is PHB pin"]
pub type IDXPHB_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `FILTER` reader - "]
pub type FILTER_R = crate::BitReader;
#[doc = "Field `FILTER` writer - "]
pub type FILTER_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MAXFILT` reader - MAXimum FILTer"]
pub type MAXFILT_R = crate::FieldReader;
#[doc = "Field `MAXFILT` writer - MAXimum FILTer"]
pub type MAXFILT_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 6, O>;
impl R {
    #[doc = "Bits 0:1 - External Clock Signal 0 Selection"]
    #[inline(always)]
    pub fn tc0xc0s(&self) -> TC0XC0S_R {
        TC0XC0S_R::new((self.bits & 3) as u8)
    }
    #[doc = "Bits 2:3 - External Clock Signal 1 Selection"]
    #[inline(always)]
    pub fn tc1xc1s(&self) -> TC1XC1S_R {
        TC1XC1S_R::new(((self.bits >> 2) & 3) as u8)
    }
    #[doc = "Bits 4:5 - External Clock Signal 2 Selection"]
    #[inline(always)]
    pub fn tc2xc2s(&self) -> TC2XC2S_R {
        TC2XC2S_R::new(((self.bits >> 4) & 3) as u8)
    }
    #[doc = "Bit 8 - Quadrature Decoder ENabled"]
    #[inline(always)]
    pub fn qden(&self) -> QDEN_R {
        QDEN_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - POSition ENabled"]
    #[inline(always)]
    pub fn posen(&self) -> POSEN_R {
        POSEN_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - SPEED ENabled"]
    #[inline(always)]
    pub fn speeden(&self) -> SPEEDEN_R {
        SPEEDEN_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Quadrature Decoding TRANSparent"]
    #[inline(always)]
    pub fn qdtrans(&self) -> QDTRANS_R {
        QDTRANS_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - EDGe on PHA count mode"]
    #[inline(always)]
    pub fn edgpha(&self) -> EDGPHA_R {
        EDGPHA_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - INVerted phA"]
    #[inline(always)]
    pub fn inva(&self) -> INVA_R {
        INVA_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - INVerted phB"]
    #[inline(always)]
    pub fn invb(&self) -> INVB_R {
        INVB_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 15 - INVerted InDeX"]
    #[inline(always)]
    pub fn invidx(&self) -> INVIDX_R {
        INVIDX_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bit 16 - SWAP PHA and PHB"]
    #[inline(always)]
    pub fn swap(&self) -> SWAP_R {
        SWAP_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - InDeX pin is PHB pin"]
    #[inline(always)]
    pub fn idxphb(&self) -> IDXPHB_R {
        IDXPHB_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 19"]
    #[inline(always)]
    pub fn filter(&self) -> FILTER_R {
        FILTER_R::new(((self.bits >> 19) & 1) != 0)
    }
    #[doc = "Bits 20:25 - MAXimum FILTer"]
    #[inline(always)]
    pub fn maxfilt(&self) -> MAXFILT_R {
        MAXFILT_R::new(((self.bits >> 20) & 0x3f) as u8)
    }
}
impl W {
    #[doc = "Bits 0:1 - External Clock Signal 0 Selection"]
    #[inline(always)]
    #[must_use]
    pub fn tc0xc0s(&mut self) -> TC0XC0S_W<BMR_SPEC, 0> {
        TC0XC0S_W::new(self)
    }
    #[doc = "Bits 2:3 - External Clock Signal 1 Selection"]
    #[inline(always)]
    #[must_use]
    pub fn tc1xc1s(&mut self) -> TC1XC1S_W<BMR_SPEC, 2> {
        TC1XC1S_W::new(self)
    }
    #[doc = "Bits 4:5 - External Clock Signal 2 Selection"]
    #[inline(always)]
    #[must_use]
    pub fn tc2xc2s(&mut self) -> TC2XC2S_W<BMR_SPEC, 4> {
        TC2XC2S_W::new(self)
    }
    #[doc = "Bit 8 - Quadrature Decoder ENabled"]
    #[inline(always)]
    #[must_use]
    pub fn qden(&mut self) -> QDEN_W<BMR_SPEC, 8> {
        QDEN_W::new(self)
    }
    #[doc = "Bit 9 - POSition ENabled"]
    #[inline(always)]
    #[must_use]
    pub fn posen(&mut self) -> POSEN_W<BMR_SPEC, 9> {
        POSEN_W::new(self)
    }
    #[doc = "Bit 10 - SPEED ENabled"]
    #[inline(always)]
    #[must_use]
    pub fn speeden(&mut self) -> SPEEDEN_W<BMR_SPEC, 10> {
        SPEEDEN_W::new(self)
    }
    #[doc = "Bit 11 - Quadrature Decoding TRANSparent"]
    #[inline(always)]
    #[must_use]
    pub fn qdtrans(&mut self) -> QDTRANS_W<BMR_SPEC, 11> {
        QDTRANS_W::new(self)
    }
    #[doc = "Bit 12 - EDGe on PHA count mode"]
    #[inline(always)]
    #[must_use]
    pub fn edgpha(&mut self) -> EDGPHA_W<BMR_SPEC, 12> {
        EDGPHA_W::new(self)
    }
    #[doc = "Bit 13 - INVerted phA"]
    #[inline(always)]
    #[must_use]
    pub fn inva(&mut self) -> INVA_W<BMR_SPEC, 13> {
        INVA_W::new(self)
    }
    #[doc = "Bit 14 - INVerted phB"]
    #[inline(always)]
    #[must_use]
    pub fn invb(&mut self) -> INVB_W<BMR_SPEC, 14> {
        INVB_W::new(self)
    }
    #[doc = "Bit 15 - INVerted InDeX"]
    #[inline(always)]
    #[must_use]
    pub fn invidx(&mut self) -> INVIDX_W<BMR_SPEC, 15> {
        INVIDX_W::new(self)
    }
    #[doc = "Bit 16 - SWAP PHA and PHB"]
    #[inline(always)]
    #[must_use]
    pub fn swap(&mut self) -> SWAP_W<BMR_SPEC, 16> {
        SWAP_W::new(self)
    }
    #[doc = "Bit 17 - InDeX pin is PHB pin"]
    #[inline(always)]
    #[must_use]
    pub fn idxphb(&mut self) -> IDXPHB_W<BMR_SPEC, 17> {
        IDXPHB_W::new(self)
    }
    #[doc = "Bit 19"]
    #[inline(always)]
    #[must_use]
    pub fn filter(&mut self) -> FILTER_W<BMR_SPEC, 19> {
        FILTER_W::new(self)
    }
    #[doc = "Bits 20:25 - MAXimum FILTer"]
    #[inline(always)]
    #[must_use]
    pub fn maxfilt(&mut self) -> MAXFILT_W<BMR_SPEC, 20> {
        MAXFILT_W::new(self)
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
#[doc = "Block Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`bmr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`bmr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct BMR_SPEC;
impl crate::RegisterSpec for BMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`bmr::R`](R) reader structure"]
impl crate::Readable for BMR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`bmr::W`](W) writer structure"]
impl crate::Writable for BMR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets BMR to value 0"]
impl crate::Resettable for BMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
