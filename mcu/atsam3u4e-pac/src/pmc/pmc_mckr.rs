#[doc = "Register `PMC_MCKR` reader"]
pub type R = crate::R<PMC_MCKR_SPEC>;
#[doc = "Register `PMC_MCKR` writer"]
pub type W = crate::W<PMC_MCKR_SPEC>;
#[doc = "Field `CSS` reader - Master Clock Source Selection"]
pub type CSS_R = crate::FieldReader<CSS_A>;
#[doc = "Master Clock Source Selection\n\nValue on reset: 1"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CSS_A {
    #[doc = "0: Slow Clock is selected"]
    SLOW_CLK = 0,
    #[doc = "1: Main Clock is selected"]
    MAIN_CLK = 1,
    #[doc = "2: PLLA Clock is selected"]
    PLLA_CLK = 2,
    #[doc = "3: UPLLClock is selected"]
    UPLL_CLK = 3,
}
impl From<CSS_A> for u8 {
    #[inline(always)]
    fn from(variant: CSS_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for CSS_A {
    type Ux = u8;
}
impl CSS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> CSS_A {
        match self.bits {
            0 => CSS_A::SLOW_CLK,
            1 => CSS_A::MAIN_CLK,
            2 => CSS_A::PLLA_CLK,
            3 => CSS_A::UPLL_CLK,
            _ => unreachable!(),
        }
    }
    #[doc = "Slow Clock is selected"]
    #[inline(always)]
    pub fn is_slow_clk(&self) -> bool {
        *self == CSS_A::SLOW_CLK
    }
    #[doc = "Main Clock is selected"]
    #[inline(always)]
    pub fn is_main_clk(&self) -> bool {
        *self == CSS_A::MAIN_CLK
    }
    #[doc = "PLLA Clock is selected"]
    #[inline(always)]
    pub fn is_plla_clk(&self) -> bool {
        *self == CSS_A::PLLA_CLK
    }
    #[doc = "UPLLClock is selected"]
    #[inline(always)]
    pub fn is_upll_clk(&self) -> bool {
        *self == CSS_A::UPLL_CLK
    }
}
#[doc = "Field `CSS` writer - Master Clock Source Selection"]
pub type CSS_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, CSS_A>;
impl<'a, REG, const O: u8> CSS_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Slow Clock is selected"]
    #[inline(always)]
    pub fn slow_clk(self) -> &'a mut crate::W<REG> {
        self.variant(CSS_A::SLOW_CLK)
    }
    #[doc = "Main Clock is selected"]
    #[inline(always)]
    pub fn main_clk(self) -> &'a mut crate::W<REG> {
        self.variant(CSS_A::MAIN_CLK)
    }
    #[doc = "PLLA Clock is selected"]
    #[inline(always)]
    pub fn plla_clk(self) -> &'a mut crate::W<REG> {
        self.variant(CSS_A::PLLA_CLK)
    }
    #[doc = "UPLLClock is selected"]
    #[inline(always)]
    pub fn upll_clk(self) -> &'a mut crate::W<REG> {
        self.variant(CSS_A::UPLL_CLK)
    }
}
#[doc = "Field `PRES` reader - Processor Clock Prescaler"]
pub type PRES_R = crate::FieldReader<PRES_A>;
#[doc = "Processor Clock Prescaler\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PRES_A {
    #[doc = "0: Selected clock"]
    CLK_1 = 0,
    #[doc = "1: Selected clock divided by 2"]
    CLK_2 = 1,
    #[doc = "2: Selected clock divided by 4"]
    CLK_4 = 2,
    #[doc = "3: Selected clock divided by 8"]
    CLK_8 = 3,
    #[doc = "4: Selected clock divided by 16"]
    CLK_16 = 4,
    #[doc = "5: Selected clock divided by 32"]
    CLK_32 = 5,
    #[doc = "6: Selected clock divided by 64"]
    CLK_64 = 6,
    #[doc = "7: Selected clock divided by 3"]
    CLK_3 = 7,
}
impl From<PRES_A> for u8 {
    #[inline(always)]
    fn from(variant: PRES_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for PRES_A {
    type Ux = u8;
}
impl PRES_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> PRES_A {
        match self.bits {
            0 => PRES_A::CLK_1,
            1 => PRES_A::CLK_2,
            2 => PRES_A::CLK_4,
            3 => PRES_A::CLK_8,
            4 => PRES_A::CLK_16,
            5 => PRES_A::CLK_32,
            6 => PRES_A::CLK_64,
            7 => PRES_A::CLK_3,
            _ => unreachable!(),
        }
    }
    #[doc = "Selected clock"]
    #[inline(always)]
    pub fn is_clk_1(&self) -> bool {
        *self == PRES_A::CLK_1
    }
    #[doc = "Selected clock divided by 2"]
    #[inline(always)]
    pub fn is_clk_2(&self) -> bool {
        *self == PRES_A::CLK_2
    }
    #[doc = "Selected clock divided by 4"]
    #[inline(always)]
    pub fn is_clk_4(&self) -> bool {
        *self == PRES_A::CLK_4
    }
    #[doc = "Selected clock divided by 8"]
    #[inline(always)]
    pub fn is_clk_8(&self) -> bool {
        *self == PRES_A::CLK_8
    }
    #[doc = "Selected clock divided by 16"]
    #[inline(always)]
    pub fn is_clk_16(&self) -> bool {
        *self == PRES_A::CLK_16
    }
    #[doc = "Selected clock divided by 32"]
    #[inline(always)]
    pub fn is_clk_32(&self) -> bool {
        *self == PRES_A::CLK_32
    }
    #[doc = "Selected clock divided by 64"]
    #[inline(always)]
    pub fn is_clk_64(&self) -> bool {
        *self == PRES_A::CLK_64
    }
    #[doc = "Selected clock divided by 3"]
    #[inline(always)]
    pub fn is_clk_3(&self) -> bool {
        *self == PRES_A::CLK_3
    }
}
#[doc = "Field `PRES` writer - Processor Clock Prescaler"]
pub type PRES_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 3, O, PRES_A>;
impl<'a, REG, const O: u8> PRES_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Selected clock"]
    #[inline(always)]
    pub fn clk_1(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_1)
    }
    #[doc = "Selected clock divided by 2"]
    #[inline(always)]
    pub fn clk_2(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_2)
    }
    #[doc = "Selected clock divided by 4"]
    #[inline(always)]
    pub fn clk_4(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_4)
    }
    #[doc = "Selected clock divided by 8"]
    #[inline(always)]
    pub fn clk_8(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_8)
    }
    #[doc = "Selected clock divided by 16"]
    #[inline(always)]
    pub fn clk_16(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_16)
    }
    #[doc = "Selected clock divided by 32"]
    #[inline(always)]
    pub fn clk_32(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_32)
    }
    #[doc = "Selected clock divided by 64"]
    #[inline(always)]
    pub fn clk_64(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_64)
    }
    #[doc = "Selected clock divided by 3"]
    #[inline(always)]
    pub fn clk_3(self) -> &'a mut crate::W<REG> {
        self.variant(PRES_A::CLK_3)
    }
}
#[doc = "Field `PLLADIV2` reader - PLLA Divisor by 2"]
pub type PLLADIV2_R = crate::BitReader;
#[doc = "Field `PLLADIV2` writer - PLLA Divisor by 2"]
pub type PLLADIV2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `UPLLDIV2` reader - UPLL Divisor by 2"]
pub type UPLLDIV2_R = crate::BitReader;
#[doc = "Field `UPLLDIV2` writer - UPLL Divisor by 2"]
pub type UPLLDIV2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bits 0:1 - Master Clock Source Selection"]
    #[inline(always)]
    pub fn css(&self) -> CSS_R {
        CSS_R::new((self.bits & 3) as u8)
    }
    #[doc = "Bits 4:6 - Processor Clock Prescaler"]
    #[inline(always)]
    pub fn pres(&self) -> PRES_R {
        PRES_R::new(((self.bits >> 4) & 7) as u8)
    }
    #[doc = "Bit 12 - PLLA Divisor by 2"]
    #[inline(always)]
    pub fn plladiv2(&self) -> PLLADIV2_R {
        PLLADIV2_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - UPLL Divisor by 2"]
    #[inline(always)]
    pub fn uplldiv2(&self) -> UPLLDIV2_R {
        UPLLDIV2_R::new(((self.bits >> 13) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:1 - Master Clock Source Selection"]
    #[inline(always)]
    #[must_use]
    pub fn css(&mut self) -> CSS_W<PMC_MCKR_SPEC, 0> {
        CSS_W::new(self)
    }
    #[doc = "Bits 4:6 - Processor Clock Prescaler"]
    #[inline(always)]
    #[must_use]
    pub fn pres(&mut self) -> PRES_W<PMC_MCKR_SPEC, 4> {
        PRES_W::new(self)
    }
    #[doc = "Bit 12 - PLLA Divisor by 2"]
    #[inline(always)]
    #[must_use]
    pub fn plladiv2(&mut self) -> PLLADIV2_W<PMC_MCKR_SPEC, 12> {
        PLLADIV2_W::new(self)
    }
    #[doc = "Bit 13 - UPLL Divisor by 2"]
    #[inline(always)]
    #[must_use]
    pub fn uplldiv2(&mut self) -> UPLLDIV2_W<PMC_MCKR_SPEC, 13> {
        UPLLDIV2_W::new(self)
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
#[doc = "Master Clock Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`pmc_mckr::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`pmc_mckr::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_MCKR_SPEC;
impl crate::RegisterSpec for PMC_MCKR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`pmc_mckr::R`](R) reader structure"]
impl crate::Readable for PMC_MCKR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`pmc_mckr::W`](W) writer structure"]
impl crate::Writable for PMC_MCKR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets PMC_MCKR to value 0x01"]
impl crate::Resettable for PMC_MCKR_SPEC {
    const RESET_VALUE: Self::Ux = 0x01;
}
