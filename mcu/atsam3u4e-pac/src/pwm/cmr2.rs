#[doc = "Register `CMR2` reader"]
pub type R = crate::R<CMR2_SPEC>;
#[doc = "Register `CMR2` writer"]
pub type W = crate::W<CMR2_SPEC>;
#[doc = "Field `CPRE` reader - Channel Pre-scaler"]
pub type CPRE_R = crate::FieldReader<CPRE_A>;
#[doc = "Channel Pre-scaler\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CPRE_A {
    #[doc = "0: Master clock"]
    MCK = 0,
    #[doc = "1: Master clock/2"]
    MCK_DIV_2 = 1,
    #[doc = "2: Master clock/4"]
    MCK_DIV_4 = 2,
    #[doc = "3: Master clock/8"]
    MCK_DIV_8 = 3,
    #[doc = "4: Master clock/16"]
    MCK_DIV_16 = 4,
    #[doc = "5: Master clock/32"]
    MCK_DIV_32 = 5,
    #[doc = "6: Master clock/64"]
    MCK_DIV_64 = 6,
    #[doc = "7: Master clock/128"]
    MCK_DIV_128 = 7,
    #[doc = "8: Master clock/256"]
    MCK_DIV_256 = 8,
    #[doc = "9: Master clock/512"]
    MCK_DIV_512 = 9,
    #[doc = "10: Master clock/1024"]
    MCK_DIV_1024 = 10,
    #[doc = "11: Clock A"]
    CLKA = 11,
    #[doc = "12: Clock B"]
    CLKB = 12,
}
impl From<CPRE_A> for u8 {
    #[inline(always)]
    fn from(variant: CPRE_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for CPRE_A {
    type Ux = u8;
}
impl CPRE_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<CPRE_A> {
        match self.bits {
            0 => Some(CPRE_A::MCK),
            1 => Some(CPRE_A::MCK_DIV_2),
            2 => Some(CPRE_A::MCK_DIV_4),
            3 => Some(CPRE_A::MCK_DIV_8),
            4 => Some(CPRE_A::MCK_DIV_16),
            5 => Some(CPRE_A::MCK_DIV_32),
            6 => Some(CPRE_A::MCK_DIV_64),
            7 => Some(CPRE_A::MCK_DIV_128),
            8 => Some(CPRE_A::MCK_DIV_256),
            9 => Some(CPRE_A::MCK_DIV_512),
            10 => Some(CPRE_A::MCK_DIV_1024),
            11 => Some(CPRE_A::CLKA),
            12 => Some(CPRE_A::CLKB),
            _ => None,
        }
    }
    #[doc = "Master clock"]
    #[inline(always)]
    pub fn is_mck(&self) -> bool {
        *self == CPRE_A::MCK
    }
    #[doc = "Master clock/2"]
    #[inline(always)]
    pub fn is_mck_div_2(&self) -> bool {
        *self == CPRE_A::MCK_DIV_2
    }
    #[doc = "Master clock/4"]
    #[inline(always)]
    pub fn is_mck_div_4(&self) -> bool {
        *self == CPRE_A::MCK_DIV_4
    }
    #[doc = "Master clock/8"]
    #[inline(always)]
    pub fn is_mck_div_8(&self) -> bool {
        *self == CPRE_A::MCK_DIV_8
    }
    #[doc = "Master clock/16"]
    #[inline(always)]
    pub fn is_mck_div_16(&self) -> bool {
        *self == CPRE_A::MCK_DIV_16
    }
    #[doc = "Master clock/32"]
    #[inline(always)]
    pub fn is_mck_div_32(&self) -> bool {
        *self == CPRE_A::MCK_DIV_32
    }
    #[doc = "Master clock/64"]
    #[inline(always)]
    pub fn is_mck_div_64(&self) -> bool {
        *self == CPRE_A::MCK_DIV_64
    }
    #[doc = "Master clock/128"]
    #[inline(always)]
    pub fn is_mck_div_128(&self) -> bool {
        *self == CPRE_A::MCK_DIV_128
    }
    #[doc = "Master clock/256"]
    #[inline(always)]
    pub fn is_mck_div_256(&self) -> bool {
        *self == CPRE_A::MCK_DIV_256
    }
    #[doc = "Master clock/512"]
    #[inline(always)]
    pub fn is_mck_div_512(&self) -> bool {
        *self == CPRE_A::MCK_DIV_512
    }
    #[doc = "Master clock/1024"]
    #[inline(always)]
    pub fn is_mck_div_1024(&self) -> bool {
        *self == CPRE_A::MCK_DIV_1024
    }
    #[doc = "Clock A"]
    #[inline(always)]
    pub fn is_clka(&self) -> bool {
        *self == CPRE_A::CLKA
    }
    #[doc = "Clock B"]
    #[inline(always)]
    pub fn is_clkb(&self) -> bool {
        *self == CPRE_A::CLKB
    }
}
#[doc = "Field `CPRE` writer - Channel Pre-scaler"]
pub type CPRE_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O, CPRE_A>;
impl<'a, REG, const O: u8> CPRE_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Master clock"]
    #[inline(always)]
    pub fn mck(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK)
    }
    #[doc = "Master clock/2"]
    #[inline(always)]
    pub fn mck_div_2(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_2)
    }
    #[doc = "Master clock/4"]
    #[inline(always)]
    pub fn mck_div_4(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_4)
    }
    #[doc = "Master clock/8"]
    #[inline(always)]
    pub fn mck_div_8(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_8)
    }
    #[doc = "Master clock/16"]
    #[inline(always)]
    pub fn mck_div_16(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_16)
    }
    #[doc = "Master clock/32"]
    #[inline(always)]
    pub fn mck_div_32(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_32)
    }
    #[doc = "Master clock/64"]
    #[inline(always)]
    pub fn mck_div_64(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_64)
    }
    #[doc = "Master clock/128"]
    #[inline(always)]
    pub fn mck_div_128(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_128)
    }
    #[doc = "Master clock/256"]
    #[inline(always)]
    pub fn mck_div_256(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_256)
    }
    #[doc = "Master clock/512"]
    #[inline(always)]
    pub fn mck_div_512(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_512)
    }
    #[doc = "Master clock/1024"]
    #[inline(always)]
    pub fn mck_div_1024(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::MCK_DIV_1024)
    }
    #[doc = "Clock A"]
    #[inline(always)]
    pub fn clka(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::CLKA)
    }
    #[doc = "Clock B"]
    #[inline(always)]
    pub fn clkb(self) -> &'a mut crate::W<REG> {
        self.variant(CPRE_A::CLKB)
    }
}
#[doc = "Field `CALG` reader - Channel Alignment"]
pub type CALG_R = crate::BitReader;
#[doc = "Field `CALG` writer - Channel Alignment"]
pub type CALG_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CPOL` reader - Channel Polarity"]
pub type CPOL_R = crate::BitReader;
#[doc = "Field `CPOL` writer - Channel Polarity"]
pub type CPOL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CES` reader - Counter Event Selection"]
pub type CES_R = crate::BitReader;
#[doc = "Field `CES` writer - Counter Event Selection"]
pub type CES_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DTE` reader - Dead-Time Generator Enable"]
pub type DTE_R = crate::BitReader;
#[doc = "Field `DTE` writer - Dead-Time Generator Enable"]
pub type DTE_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DTHI` reader - Dead-Time PWMHx Output Inverted"]
pub type DTHI_R = crate::BitReader;
#[doc = "Field `DTHI` writer - Dead-Time PWMHx Output Inverted"]
pub type DTHI_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DTLI` reader - Dead-Time PWMLx Output Inverted"]
pub type DTLI_R = crate::BitReader;
#[doc = "Field `DTLI` writer - Dead-Time PWMLx Output Inverted"]
pub type DTLI_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bits 0:3 - Channel Pre-scaler"]
    #[inline(always)]
    pub fn cpre(&self) -> CPRE_R {
        CPRE_R::new((self.bits & 0x0f) as u8)
    }
    #[doc = "Bit 8 - Channel Alignment"]
    #[inline(always)]
    pub fn calg(&self) -> CALG_R {
        CALG_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Channel Polarity"]
    #[inline(always)]
    pub fn cpol(&self) -> CPOL_R {
        CPOL_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Counter Event Selection"]
    #[inline(always)]
    pub fn ces(&self) -> CES_R {
        CES_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 16 - Dead-Time Generator Enable"]
    #[inline(always)]
    pub fn dte(&self) -> DTE_R {
        DTE_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Dead-Time PWMHx Output Inverted"]
    #[inline(always)]
    pub fn dthi(&self) -> DTHI_R {
        DTHI_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - Dead-Time PWMLx Output Inverted"]
    #[inline(always)]
    pub fn dtli(&self) -> DTLI_R {
        DTLI_R::new(((self.bits >> 18) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:3 - Channel Pre-scaler"]
    #[inline(always)]
    #[must_use]
    pub fn cpre(&mut self) -> CPRE_W<CMR2_SPEC, 0> {
        CPRE_W::new(self)
    }
    #[doc = "Bit 8 - Channel Alignment"]
    #[inline(always)]
    #[must_use]
    pub fn calg(&mut self) -> CALG_W<CMR2_SPEC, 8> {
        CALG_W::new(self)
    }
    #[doc = "Bit 9 - Channel Polarity"]
    #[inline(always)]
    #[must_use]
    pub fn cpol(&mut self) -> CPOL_W<CMR2_SPEC, 9> {
        CPOL_W::new(self)
    }
    #[doc = "Bit 10 - Counter Event Selection"]
    #[inline(always)]
    #[must_use]
    pub fn ces(&mut self) -> CES_W<CMR2_SPEC, 10> {
        CES_W::new(self)
    }
    #[doc = "Bit 16 - Dead-Time Generator Enable"]
    #[inline(always)]
    #[must_use]
    pub fn dte(&mut self) -> DTE_W<CMR2_SPEC, 16> {
        DTE_W::new(self)
    }
    #[doc = "Bit 17 - Dead-Time PWMHx Output Inverted"]
    #[inline(always)]
    #[must_use]
    pub fn dthi(&mut self) -> DTHI_W<CMR2_SPEC, 17> {
        DTHI_W::new(self)
    }
    #[doc = "Bit 18 - Dead-Time PWMLx Output Inverted"]
    #[inline(always)]
    #[must_use]
    pub fn dtli(&mut self) -> DTLI_W<CMR2_SPEC, 18> {
        DTLI_W::new(self)
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
#[doc = "PWM Channel Mode Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmr2::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmr2::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CMR2_SPEC;
impl crate::RegisterSpec for CMR2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`cmr2::R`](R) reader structure"]
impl crate::Readable for CMR2_SPEC {}
#[doc = "`write(|w| ..)` method takes [`cmr2::W`](W) writer structure"]
impl crate::Writable for CMR2_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CMR2 to value 0"]
impl crate::Resettable for CMR2_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
