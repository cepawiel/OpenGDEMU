#[doc = "Register `CSR%s` reader"]
pub type R = crate::R<CSR_SPEC>;
#[doc = "Register `CSR%s` writer"]
pub type W = crate::W<CSR_SPEC>;
#[doc = "Field `CPOL` reader - Clock Polarity"]
pub type CPOL_R = crate::BitReader;
#[doc = "Field `CPOL` writer - Clock Polarity"]
pub type CPOL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `NCPHA` reader - Clock Phase"]
pub type NCPHA_R = crate::BitReader;
#[doc = "Field `NCPHA` writer - Clock Phase"]
pub type NCPHA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CSNAAT` reader - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
pub type CSNAAT_R = crate::BitReader;
#[doc = "Field `CSNAAT` writer - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
pub type CSNAAT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CSAAT` reader - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
pub type CSAAT_R = crate::BitReader;
#[doc = "Field `CSAAT` writer - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
pub type CSAAT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `BITS` reader - Bits Per Transfer"]
pub type BITS_R = crate::FieldReader<BITS_A>;
#[doc = "Bits Per Transfer"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BITS_A {
    #[doc = "0: 8 bits for transfer"]
    _8_BIT = 0,
    #[doc = "1: 9 bits for transfer"]
    _9_BIT = 1,
    #[doc = "2: 10 bits for transfer"]
    _10_BIT = 2,
    #[doc = "3: 11 bits for transfer"]
    _11_BIT = 3,
    #[doc = "4: 12 bits for transfer"]
    _12_BIT = 4,
    #[doc = "5: 13 bits for transfer"]
    _13_BIT = 5,
    #[doc = "6: 14 bits for transfer"]
    _14_BIT = 6,
    #[doc = "7: 15 bits for transfer"]
    _15_BIT = 7,
    #[doc = "8: 16 bits for transfer"]
    _16_BIT = 8,
}
impl From<BITS_A> for u8 {
    #[inline(always)]
    fn from(variant: BITS_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for BITS_A {
    type Ux = u8;
}
impl BITS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<BITS_A> {
        match self.bits {
            0 => Some(BITS_A::_8_BIT),
            1 => Some(BITS_A::_9_BIT),
            2 => Some(BITS_A::_10_BIT),
            3 => Some(BITS_A::_11_BIT),
            4 => Some(BITS_A::_12_BIT),
            5 => Some(BITS_A::_13_BIT),
            6 => Some(BITS_A::_14_BIT),
            7 => Some(BITS_A::_15_BIT),
            8 => Some(BITS_A::_16_BIT),
            _ => None,
        }
    }
    #[doc = "8 bits for transfer"]
    #[inline(always)]
    pub fn is_8_bit(&self) -> bool {
        *self == BITS_A::_8_BIT
    }
    #[doc = "9 bits for transfer"]
    #[inline(always)]
    pub fn is_9_bit(&self) -> bool {
        *self == BITS_A::_9_BIT
    }
    #[doc = "10 bits for transfer"]
    #[inline(always)]
    pub fn is_10_bit(&self) -> bool {
        *self == BITS_A::_10_BIT
    }
    #[doc = "11 bits for transfer"]
    #[inline(always)]
    pub fn is_11_bit(&self) -> bool {
        *self == BITS_A::_11_BIT
    }
    #[doc = "12 bits for transfer"]
    #[inline(always)]
    pub fn is_12_bit(&self) -> bool {
        *self == BITS_A::_12_BIT
    }
    #[doc = "13 bits for transfer"]
    #[inline(always)]
    pub fn is_13_bit(&self) -> bool {
        *self == BITS_A::_13_BIT
    }
    #[doc = "14 bits for transfer"]
    #[inline(always)]
    pub fn is_14_bit(&self) -> bool {
        *self == BITS_A::_14_BIT
    }
    #[doc = "15 bits for transfer"]
    #[inline(always)]
    pub fn is_15_bit(&self) -> bool {
        *self == BITS_A::_15_BIT
    }
    #[doc = "16 bits for transfer"]
    #[inline(always)]
    pub fn is_16_bit(&self) -> bool {
        *self == BITS_A::_16_BIT
    }
}
#[doc = "Field `BITS` writer - Bits Per Transfer"]
pub type BITS_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O, BITS_A>;
impl<'a, REG, const O: u8> BITS_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "8 bits for transfer"]
    #[inline(always)]
    pub fn _8_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_8_BIT)
    }
    #[doc = "9 bits for transfer"]
    #[inline(always)]
    pub fn _9_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_9_BIT)
    }
    #[doc = "10 bits for transfer"]
    #[inline(always)]
    pub fn _10_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_10_BIT)
    }
    #[doc = "11 bits for transfer"]
    #[inline(always)]
    pub fn _11_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_11_BIT)
    }
    #[doc = "12 bits for transfer"]
    #[inline(always)]
    pub fn _12_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_12_BIT)
    }
    #[doc = "13 bits for transfer"]
    #[inline(always)]
    pub fn _13_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_13_BIT)
    }
    #[doc = "14 bits for transfer"]
    #[inline(always)]
    pub fn _14_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_14_BIT)
    }
    #[doc = "15 bits for transfer"]
    #[inline(always)]
    pub fn _15_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_15_BIT)
    }
    #[doc = "16 bits for transfer"]
    #[inline(always)]
    pub fn _16_bit(self) -> &'a mut crate::W<REG> {
        self.variant(BITS_A::_16_BIT)
    }
}
#[doc = "Field `SCBR` reader - Serial Clock Baud Rate"]
pub type SCBR_R = crate::FieldReader;
#[doc = "Field `SCBR` writer - Serial Clock Baud Rate"]
pub type SCBR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
#[doc = "Field `DLYBS` reader - Delay Before SPCK"]
pub type DLYBS_R = crate::FieldReader;
#[doc = "Field `DLYBS` writer - Delay Before SPCK"]
pub type DLYBS_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
#[doc = "Field `DLYBCT` reader - Delay Between Consecutive Transfers"]
pub type DLYBCT_R = crate::FieldReader;
#[doc = "Field `DLYBCT` writer - Delay Between Consecutive Transfers"]
pub type DLYBCT_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
impl R {
    #[doc = "Bit 0 - Clock Polarity"]
    #[inline(always)]
    pub fn cpol(&self) -> CPOL_R {
        CPOL_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Clock Phase"]
    #[inline(always)]
    pub fn ncpha(&self) -> NCPHA_R {
        NCPHA_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
    #[inline(always)]
    pub fn csnaat(&self) -> CSNAAT_R {
        CSNAAT_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
    #[inline(always)]
    pub fn csaat(&self) -> CSAAT_R {
        CSAAT_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bits 4:7 - Bits Per Transfer"]
    #[inline(always)]
    pub fn bits_(&self) -> BITS_R {
        BITS_R::new(((self.bits >> 4) & 0x0f) as u8)
    }
    #[doc = "Bits 8:15 - Serial Clock Baud Rate"]
    #[inline(always)]
    pub fn scbr(&self) -> SCBR_R {
        SCBR_R::new(((self.bits >> 8) & 0xff) as u8)
    }
    #[doc = "Bits 16:23 - Delay Before SPCK"]
    #[inline(always)]
    pub fn dlybs(&self) -> DLYBS_R {
        DLYBS_R::new(((self.bits >> 16) & 0xff) as u8)
    }
    #[doc = "Bits 24:31 - Delay Between Consecutive Transfers"]
    #[inline(always)]
    pub fn dlybct(&self) -> DLYBCT_R {
        DLYBCT_R::new(((self.bits >> 24) & 0xff) as u8)
    }
}
impl W {
    #[doc = "Bit 0 - Clock Polarity"]
    #[inline(always)]
    #[must_use]
    pub fn cpol(&mut self) -> CPOL_W<CSR_SPEC, 0> {
        CPOL_W::new(self)
    }
    #[doc = "Bit 1 - Clock Phase"]
    #[inline(always)]
    #[must_use]
    pub fn ncpha(&mut self) -> NCPHA_W<CSR_SPEC, 1> {
        NCPHA_W::new(self)
    }
    #[doc = "Bit 2 - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
    #[inline(always)]
    #[must_use]
    pub fn csnaat(&mut self) -> CSNAAT_W<CSR_SPEC, 2> {
        CSNAAT_W::new(self)
    }
    #[doc = "Bit 3 - Chip Select Not Active After Transfer (Ignored if CSAAT = 1)"]
    #[inline(always)]
    #[must_use]
    pub fn csaat(&mut self) -> CSAAT_W<CSR_SPEC, 3> {
        CSAAT_W::new(self)
    }
    #[doc = "Bits 4:7 - Bits Per Transfer"]
    #[inline(always)]
    #[must_use]
    pub fn bits_(&mut self) -> BITS_W<CSR_SPEC, 4> {
        BITS_W::new(self)
    }
    #[doc = "Bits 8:15 - Serial Clock Baud Rate"]
    #[inline(always)]
    #[must_use]
    pub fn scbr(&mut self) -> SCBR_W<CSR_SPEC, 8> {
        SCBR_W::new(self)
    }
    #[doc = "Bits 16:23 - Delay Before SPCK"]
    #[inline(always)]
    #[must_use]
    pub fn dlybs(&mut self) -> DLYBS_W<CSR_SPEC, 16> {
        DLYBS_W::new(self)
    }
    #[doc = "Bits 24:31 - Delay Between Consecutive Transfers"]
    #[inline(always)]
    #[must_use]
    pub fn dlybct(&mut self) -> DLYBCT_W<CSR_SPEC, 24> {
        DLYBCT_W::new(self)
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
#[doc = "Chip Select Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`csr::R`](R).  You can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`csr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CSR_SPEC;
impl crate::RegisterSpec for CSR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`csr::R`](R) reader structure"]
impl crate::Readable for CSR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`csr::W`](W) writer structure"]
impl crate::Writable for CSR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
