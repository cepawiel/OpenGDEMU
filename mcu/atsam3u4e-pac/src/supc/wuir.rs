#[doc = "Register `WUIR` reader"]
pub type R = crate::R<WUIR_SPEC>;
#[doc = "Register `WUIR` writer"]
pub type W = crate::W<WUIR_SPEC>;
#[doc = "Field `WKUPEN0` reader - Wake Up Input Enable 0"]
pub type WKUPEN0_R = crate::BitReader<WKUPEN0_A>;
#[doc = "Wake Up Input Enable 0\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN0_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN0_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN0_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN0_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN0_A {
        match self.bits {
            false => WKUPEN0_A::NOT_ENABLE,
            true => WKUPEN0_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN0_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN0_A::ENABLE
    }
}
#[doc = "Field `WKUPEN0` writer - Wake Up Input Enable 0"]
pub type WKUPEN0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN0_A>;
impl<'a, REG, const O: u8> WKUPEN0_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN0_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN0_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN1` reader - Wake Up Input Enable 1"]
pub type WKUPEN1_R = crate::BitReader<WKUPEN1_A>;
#[doc = "Wake Up Input Enable 1\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN1_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN1_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN1_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN1_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN1_A {
        match self.bits {
            false => WKUPEN1_A::NOT_ENABLE,
            true => WKUPEN1_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN1_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN1_A::ENABLE
    }
}
#[doc = "Field `WKUPEN1` writer - Wake Up Input Enable 1"]
pub type WKUPEN1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN1_A>;
impl<'a, REG, const O: u8> WKUPEN1_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN1_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN1_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN2` reader - Wake Up Input Enable 2"]
pub type WKUPEN2_R = crate::BitReader<WKUPEN2_A>;
#[doc = "Wake Up Input Enable 2\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN2_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN2_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN2_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN2_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN2_A {
        match self.bits {
            false => WKUPEN2_A::NOT_ENABLE,
            true => WKUPEN2_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN2_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN2_A::ENABLE
    }
}
#[doc = "Field `WKUPEN2` writer - Wake Up Input Enable 2"]
pub type WKUPEN2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN2_A>;
impl<'a, REG, const O: u8> WKUPEN2_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN2_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN2_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN3` reader - Wake Up Input Enable 3"]
pub type WKUPEN3_R = crate::BitReader<WKUPEN3_A>;
#[doc = "Wake Up Input Enable 3\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN3_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN3_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN3_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN3_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN3_A {
        match self.bits {
            false => WKUPEN3_A::NOT_ENABLE,
            true => WKUPEN3_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN3_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN3_A::ENABLE
    }
}
#[doc = "Field `WKUPEN3` writer - Wake Up Input Enable 3"]
pub type WKUPEN3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN3_A>;
impl<'a, REG, const O: u8> WKUPEN3_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN3_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN3_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN4` reader - Wake Up Input Enable 4"]
pub type WKUPEN4_R = crate::BitReader<WKUPEN4_A>;
#[doc = "Wake Up Input Enable 4\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN4_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN4_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN4_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN4_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN4_A {
        match self.bits {
            false => WKUPEN4_A::NOT_ENABLE,
            true => WKUPEN4_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN4_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN4_A::ENABLE
    }
}
#[doc = "Field `WKUPEN4` writer - Wake Up Input Enable 4"]
pub type WKUPEN4_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN4_A>;
impl<'a, REG, const O: u8> WKUPEN4_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN4_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN4_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN5` reader - Wake Up Input Enable 5"]
pub type WKUPEN5_R = crate::BitReader<WKUPEN5_A>;
#[doc = "Wake Up Input Enable 5\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN5_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN5_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN5_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN5_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN5_A {
        match self.bits {
            false => WKUPEN5_A::NOT_ENABLE,
            true => WKUPEN5_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN5_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN5_A::ENABLE
    }
}
#[doc = "Field `WKUPEN5` writer - Wake Up Input Enable 5"]
pub type WKUPEN5_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN5_A>;
impl<'a, REG, const O: u8> WKUPEN5_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN5_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN5_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN6` reader - Wake Up Input Enable 6"]
pub type WKUPEN6_R = crate::BitReader<WKUPEN6_A>;
#[doc = "Wake Up Input Enable 6\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN6_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN6_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN6_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN6_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN6_A {
        match self.bits {
            false => WKUPEN6_A::NOT_ENABLE,
            true => WKUPEN6_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN6_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN6_A::ENABLE
    }
}
#[doc = "Field `WKUPEN6` writer - Wake Up Input Enable 6"]
pub type WKUPEN6_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN6_A>;
impl<'a, REG, const O: u8> WKUPEN6_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN6_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN6_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN7` reader - Wake Up Input Enable 7"]
pub type WKUPEN7_R = crate::BitReader<WKUPEN7_A>;
#[doc = "Wake Up Input Enable 7\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN7_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN7_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN7_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN7_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN7_A {
        match self.bits {
            false => WKUPEN7_A::NOT_ENABLE,
            true => WKUPEN7_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN7_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN7_A::ENABLE
    }
}
#[doc = "Field `WKUPEN7` writer - Wake Up Input Enable 7"]
pub type WKUPEN7_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN7_A>;
impl<'a, REG, const O: u8> WKUPEN7_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN7_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN7_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN8` reader - Wake Up Input Enable 8"]
pub type WKUPEN8_R = crate::BitReader<WKUPEN8_A>;
#[doc = "Wake Up Input Enable 8\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN8_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN8_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN8_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN8_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN8_A {
        match self.bits {
            false => WKUPEN8_A::NOT_ENABLE,
            true => WKUPEN8_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN8_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN8_A::ENABLE
    }
}
#[doc = "Field `WKUPEN8` writer - Wake Up Input Enable 8"]
pub type WKUPEN8_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN8_A>;
impl<'a, REG, const O: u8> WKUPEN8_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN8_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN8_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN9` reader - Wake Up Input Enable 9"]
pub type WKUPEN9_R = crate::BitReader<WKUPEN9_A>;
#[doc = "Wake Up Input Enable 9\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN9_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN9_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN9_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN9_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN9_A {
        match self.bits {
            false => WKUPEN9_A::NOT_ENABLE,
            true => WKUPEN9_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN9_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN9_A::ENABLE
    }
}
#[doc = "Field `WKUPEN9` writer - Wake Up Input Enable 9"]
pub type WKUPEN9_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN9_A>;
impl<'a, REG, const O: u8> WKUPEN9_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN9_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN9_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN10` reader - Wake Up Input Enable 10"]
pub type WKUPEN10_R = crate::BitReader<WKUPEN10_A>;
#[doc = "Wake Up Input Enable 10\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN10_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN10_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN10_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN10_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN10_A {
        match self.bits {
            false => WKUPEN10_A::NOT_ENABLE,
            true => WKUPEN10_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN10_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN10_A::ENABLE
    }
}
#[doc = "Field `WKUPEN10` writer - Wake Up Input Enable 10"]
pub type WKUPEN10_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN10_A>;
impl<'a, REG, const O: u8> WKUPEN10_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN10_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN10_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN11` reader - Wake Up Input Enable 11"]
pub type WKUPEN11_R = crate::BitReader<WKUPEN11_A>;
#[doc = "Wake Up Input Enable 11\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN11_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN11_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN11_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN11_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN11_A {
        match self.bits {
            false => WKUPEN11_A::NOT_ENABLE,
            true => WKUPEN11_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN11_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN11_A::ENABLE
    }
}
#[doc = "Field `WKUPEN11` writer - Wake Up Input Enable 11"]
pub type WKUPEN11_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN11_A>;
impl<'a, REG, const O: u8> WKUPEN11_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN11_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN11_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN12` reader - Wake Up Input Enable 12"]
pub type WKUPEN12_R = crate::BitReader<WKUPEN12_A>;
#[doc = "Wake Up Input Enable 12\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN12_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN12_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN12_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN12_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN12_A {
        match self.bits {
            false => WKUPEN12_A::NOT_ENABLE,
            true => WKUPEN12_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN12_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN12_A::ENABLE
    }
}
#[doc = "Field `WKUPEN12` writer - Wake Up Input Enable 12"]
pub type WKUPEN12_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN12_A>;
impl<'a, REG, const O: u8> WKUPEN12_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN12_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN12_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN13` reader - Wake Up Input Enable 13"]
pub type WKUPEN13_R = crate::BitReader<WKUPEN13_A>;
#[doc = "Wake Up Input Enable 13\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN13_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN13_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN13_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN13_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN13_A {
        match self.bits {
            false => WKUPEN13_A::NOT_ENABLE,
            true => WKUPEN13_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN13_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN13_A::ENABLE
    }
}
#[doc = "Field `WKUPEN13` writer - Wake Up Input Enable 13"]
pub type WKUPEN13_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN13_A>;
impl<'a, REG, const O: u8> WKUPEN13_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN13_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN13_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN14` reader - Wake Up Input Enable 14"]
pub type WKUPEN14_R = crate::BitReader<WKUPEN14_A>;
#[doc = "Wake Up Input Enable 14\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN14_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN14_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN14_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN14_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN14_A {
        match self.bits {
            false => WKUPEN14_A::NOT_ENABLE,
            true => WKUPEN14_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN14_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN14_A::ENABLE
    }
}
#[doc = "Field `WKUPEN14` writer - Wake Up Input Enable 14"]
pub type WKUPEN14_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN14_A>;
impl<'a, REG, const O: u8> WKUPEN14_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN14_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN14_A::ENABLE)
    }
}
#[doc = "Field `WKUPEN15` reader - Wake Up Input Enable 15"]
pub type WKUPEN15_R = crate::BitReader<WKUPEN15_A>;
#[doc = "Wake Up Input Enable 15\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPEN15_A {
    #[doc = "0: the corresponding wake-up input has no wake up effect."]
    NOT_ENABLE = 0,
    #[doc = "1: the corresponding wake-up input forces the wake up of the core power supply."]
    ENABLE = 1,
}
impl From<WKUPEN15_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPEN15_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPEN15_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPEN15_A {
        match self.bits {
            false => WKUPEN15_A::NOT_ENABLE,
            true => WKUPEN15_A::ENABLE,
        }
    }
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn is_not_enable(&self) -> bool {
        *self == WKUPEN15_A::NOT_ENABLE
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_enable(&self) -> bool {
        *self == WKUPEN15_A::ENABLE
    }
}
#[doc = "Field `WKUPEN15` writer - Wake Up Input Enable 15"]
pub type WKUPEN15_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPEN15_A>;
impl<'a, REG, const O: u8> WKUPEN15_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "the corresponding wake-up input has no wake up effect."]
    #[inline(always)]
    pub fn not_enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN15_A::NOT_ENABLE)
    }
    #[doc = "the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn enable(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPEN15_A::ENABLE)
    }
}
#[doc = "Field `WKUPT0` reader - Wake Up Input Transition 0"]
pub type WKUPT0_R = crate::BitReader<WKUPT0_A>;
#[doc = "Wake Up Input Transition 0\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT0_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT0_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT0_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT0_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT0_A {
        match self.bits {
            false => WKUPT0_A::HIGH_TO_LOW,
            true => WKUPT0_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT0_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT0_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT0` writer - Wake Up Input Transition 0"]
pub type WKUPT0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT0_A>;
impl<'a, REG, const O: u8> WKUPT0_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT0_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT0_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT1` reader - Wake Up Input Transition 1"]
pub type WKUPT1_R = crate::BitReader<WKUPT1_A>;
#[doc = "Wake Up Input Transition 1\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT1_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT1_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT1_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT1_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT1_A {
        match self.bits {
            false => WKUPT1_A::HIGH_TO_LOW,
            true => WKUPT1_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT1_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT1_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT1` writer - Wake Up Input Transition 1"]
pub type WKUPT1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT1_A>;
impl<'a, REG, const O: u8> WKUPT1_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT1_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT1_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT2` reader - Wake Up Input Transition 2"]
pub type WKUPT2_R = crate::BitReader<WKUPT2_A>;
#[doc = "Wake Up Input Transition 2\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT2_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT2_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT2_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT2_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT2_A {
        match self.bits {
            false => WKUPT2_A::HIGH_TO_LOW,
            true => WKUPT2_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT2_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT2_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT2` writer - Wake Up Input Transition 2"]
pub type WKUPT2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT2_A>;
impl<'a, REG, const O: u8> WKUPT2_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT2_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT2_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT3` reader - Wake Up Input Transition 3"]
pub type WKUPT3_R = crate::BitReader<WKUPT3_A>;
#[doc = "Wake Up Input Transition 3\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT3_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT3_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT3_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT3_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT3_A {
        match self.bits {
            false => WKUPT3_A::HIGH_TO_LOW,
            true => WKUPT3_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT3_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT3_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT3` writer - Wake Up Input Transition 3"]
pub type WKUPT3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT3_A>;
impl<'a, REG, const O: u8> WKUPT3_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT3_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT3_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT4` reader - Wake Up Input Transition 4"]
pub type WKUPT4_R = crate::BitReader<WKUPT4_A>;
#[doc = "Wake Up Input Transition 4\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT4_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT4_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT4_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT4_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT4_A {
        match self.bits {
            false => WKUPT4_A::HIGH_TO_LOW,
            true => WKUPT4_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT4_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT4_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT4` writer - Wake Up Input Transition 4"]
pub type WKUPT4_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT4_A>;
impl<'a, REG, const O: u8> WKUPT4_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT4_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT4_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT5` reader - Wake Up Input Transition 5"]
pub type WKUPT5_R = crate::BitReader<WKUPT5_A>;
#[doc = "Wake Up Input Transition 5\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT5_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT5_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT5_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT5_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT5_A {
        match self.bits {
            false => WKUPT5_A::HIGH_TO_LOW,
            true => WKUPT5_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT5_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT5_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT5` writer - Wake Up Input Transition 5"]
pub type WKUPT5_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT5_A>;
impl<'a, REG, const O: u8> WKUPT5_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT5_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT5_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT6` reader - Wake Up Input Transition 6"]
pub type WKUPT6_R = crate::BitReader<WKUPT6_A>;
#[doc = "Wake Up Input Transition 6\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT6_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT6_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT6_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT6_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT6_A {
        match self.bits {
            false => WKUPT6_A::HIGH_TO_LOW,
            true => WKUPT6_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT6_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT6_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT6` writer - Wake Up Input Transition 6"]
pub type WKUPT6_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT6_A>;
impl<'a, REG, const O: u8> WKUPT6_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT6_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT6_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT7` reader - Wake Up Input Transition 7"]
pub type WKUPT7_R = crate::BitReader<WKUPT7_A>;
#[doc = "Wake Up Input Transition 7\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT7_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT7_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT7_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT7_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT7_A {
        match self.bits {
            false => WKUPT7_A::HIGH_TO_LOW,
            true => WKUPT7_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT7_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT7_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT7` writer - Wake Up Input Transition 7"]
pub type WKUPT7_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT7_A>;
impl<'a, REG, const O: u8> WKUPT7_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT7_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT7_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT8` reader - Wake Up Input Transition 8"]
pub type WKUPT8_R = crate::BitReader<WKUPT8_A>;
#[doc = "Wake Up Input Transition 8\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT8_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT8_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT8_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT8_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT8_A {
        match self.bits {
            false => WKUPT8_A::HIGH_TO_LOW,
            true => WKUPT8_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT8_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT8_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT8` writer - Wake Up Input Transition 8"]
pub type WKUPT8_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT8_A>;
impl<'a, REG, const O: u8> WKUPT8_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT8_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT8_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT9` reader - Wake Up Input Transition 9"]
pub type WKUPT9_R = crate::BitReader<WKUPT9_A>;
#[doc = "Wake Up Input Transition 9\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT9_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT9_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT9_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT9_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT9_A {
        match self.bits {
            false => WKUPT9_A::HIGH_TO_LOW,
            true => WKUPT9_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT9_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT9_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT9` writer - Wake Up Input Transition 9"]
pub type WKUPT9_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT9_A>;
impl<'a, REG, const O: u8> WKUPT9_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT9_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT9_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT10` reader - Wake Up Input Transition 10"]
pub type WKUPT10_R = crate::BitReader<WKUPT10_A>;
#[doc = "Wake Up Input Transition 10\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT10_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT10_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT10_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT10_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT10_A {
        match self.bits {
            false => WKUPT10_A::HIGH_TO_LOW,
            true => WKUPT10_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT10_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT10_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT10` writer - Wake Up Input Transition 10"]
pub type WKUPT10_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT10_A>;
impl<'a, REG, const O: u8> WKUPT10_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT10_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT10_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT11` reader - Wake Up Input Transition 11"]
pub type WKUPT11_R = crate::BitReader<WKUPT11_A>;
#[doc = "Wake Up Input Transition 11\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT11_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT11_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT11_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT11_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT11_A {
        match self.bits {
            false => WKUPT11_A::HIGH_TO_LOW,
            true => WKUPT11_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT11_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT11_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT11` writer - Wake Up Input Transition 11"]
pub type WKUPT11_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT11_A>;
impl<'a, REG, const O: u8> WKUPT11_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT11_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT11_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT12` reader - Wake Up Input Transition 12"]
pub type WKUPT12_R = crate::BitReader<WKUPT12_A>;
#[doc = "Wake Up Input Transition 12\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT12_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT12_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT12_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT12_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT12_A {
        match self.bits {
            false => WKUPT12_A::HIGH_TO_LOW,
            true => WKUPT12_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT12_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT12_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT12` writer - Wake Up Input Transition 12"]
pub type WKUPT12_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT12_A>;
impl<'a, REG, const O: u8> WKUPT12_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT12_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT12_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT13` reader - Wake Up Input Transition 13"]
pub type WKUPT13_R = crate::BitReader<WKUPT13_A>;
#[doc = "Wake Up Input Transition 13\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT13_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT13_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT13_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT13_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT13_A {
        match self.bits {
            false => WKUPT13_A::HIGH_TO_LOW,
            true => WKUPT13_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT13_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT13_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT13` writer - Wake Up Input Transition 13"]
pub type WKUPT13_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT13_A>;
impl<'a, REG, const O: u8> WKUPT13_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT13_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT13_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT14` reader - Wake Up Input Transition 14"]
pub type WKUPT14_R = crate::BitReader<WKUPT14_A>;
#[doc = "Wake Up Input Transition 14\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT14_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT14_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT14_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT14_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT14_A {
        match self.bits {
            false => WKUPT14_A::HIGH_TO_LOW,
            true => WKUPT14_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT14_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT14_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT14` writer - Wake Up Input Transition 14"]
pub type WKUPT14_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT14_A>;
impl<'a, REG, const O: u8> WKUPT14_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT14_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT14_A::LOW_TO_HIGH)
    }
}
#[doc = "Field `WKUPT15` reader - Wake Up Input Transition 15"]
pub type WKUPT15_R = crate::BitReader<WKUPT15_A>;
#[doc = "Wake Up Input Transition 15\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPT15_A {
    #[doc = "0: a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    HIGH_TO_LOW = 0,
    #[doc = "1: a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    LOW_TO_HIGH = 1,
}
impl From<WKUPT15_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPT15_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPT15_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPT15_A {
        match self.bits {
            false => WKUPT15_A::HIGH_TO_LOW,
            true => WKUPT15_A::LOW_TO_HIGH,
        }
    }
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_high_to_low(&self) -> bool {
        *self == WKUPT15_A::HIGH_TO_LOW
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn is_low_to_high(&self) -> bool {
        *self == WKUPT15_A::LOW_TO_HIGH
    }
}
#[doc = "Field `WKUPT15` writer - Wake Up Input Transition 15"]
pub type WKUPT15_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O, WKUPT15_A>;
impl<'a, REG, const O: u8> WKUPT15_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
{
    #[doc = "a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn high_to_low(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT15_A::HIGH_TO_LOW)
    }
    #[doc = "a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply."]
    #[inline(always)]
    pub fn low_to_high(self) -> &'a mut crate::W<REG> {
        self.variant(WKUPT15_A::LOW_TO_HIGH)
    }
}
impl R {
    #[doc = "Bit 0 - Wake Up Input Enable 0"]
    #[inline(always)]
    pub fn wkupen0(&self) -> WKUPEN0_R {
        WKUPEN0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Wake Up Input Enable 1"]
    #[inline(always)]
    pub fn wkupen1(&self) -> WKUPEN1_R {
        WKUPEN1_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Wake Up Input Enable 2"]
    #[inline(always)]
    pub fn wkupen2(&self) -> WKUPEN2_R {
        WKUPEN2_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Wake Up Input Enable 3"]
    #[inline(always)]
    pub fn wkupen3(&self) -> WKUPEN3_R {
        WKUPEN3_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Wake Up Input Enable 4"]
    #[inline(always)]
    pub fn wkupen4(&self) -> WKUPEN4_R {
        WKUPEN4_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Wake Up Input Enable 5"]
    #[inline(always)]
    pub fn wkupen5(&self) -> WKUPEN5_R {
        WKUPEN5_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Wake Up Input Enable 6"]
    #[inline(always)]
    pub fn wkupen6(&self) -> WKUPEN6_R {
        WKUPEN6_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - Wake Up Input Enable 7"]
    #[inline(always)]
    pub fn wkupen7(&self) -> WKUPEN7_R {
        WKUPEN7_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 8 - Wake Up Input Enable 8"]
    #[inline(always)]
    pub fn wkupen8(&self) -> WKUPEN8_R {
        WKUPEN8_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Wake Up Input Enable 9"]
    #[inline(always)]
    pub fn wkupen9(&self) -> WKUPEN9_R {
        WKUPEN9_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Wake Up Input Enable 10"]
    #[inline(always)]
    pub fn wkupen10(&self) -> WKUPEN10_R {
        WKUPEN10_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Wake Up Input Enable 11"]
    #[inline(always)]
    pub fn wkupen11(&self) -> WKUPEN11_R {
        WKUPEN11_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - Wake Up Input Enable 12"]
    #[inline(always)]
    pub fn wkupen12(&self) -> WKUPEN12_R {
        WKUPEN12_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - Wake Up Input Enable 13"]
    #[inline(always)]
    pub fn wkupen13(&self) -> WKUPEN13_R {
        WKUPEN13_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - Wake Up Input Enable 14"]
    #[inline(always)]
    pub fn wkupen14(&self) -> WKUPEN14_R {
        WKUPEN14_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 15 - Wake Up Input Enable 15"]
    #[inline(always)]
    pub fn wkupen15(&self) -> WKUPEN15_R {
        WKUPEN15_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bit 16 - Wake Up Input Transition 0"]
    #[inline(always)]
    pub fn wkupt0(&self) -> WKUPT0_R {
        WKUPT0_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Wake Up Input Transition 1"]
    #[inline(always)]
    pub fn wkupt1(&self) -> WKUPT1_R {
        WKUPT1_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - Wake Up Input Transition 2"]
    #[inline(always)]
    pub fn wkupt2(&self) -> WKUPT2_R {
        WKUPT2_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 19 - Wake Up Input Transition 3"]
    #[inline(always)]
    pub fn wkupt3(&self) -> WKUPT3_R {
        WKUPT3_R::new(((self.bits >> 19) & 1) != 0)
    }
    #[doc = "Bit 20 - Wake Up Input Transition 4"]
    #[inline(always)]
    pub fn wkupt4(&self) -> WKUPT4_R {
        WKUPT4_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bit 21 - Wake Up Input Transition 5"]
    #[inline(always)]
    pub fn wkupt5(&self) -> WKUPT5_R {
        WKUPT5_R::new(((self.bits >> 21) & 1) != 0)
    }
    #[doc = "Bit 22 - Wake Up Input Transition 6"]
    #[inline(always)]
    pub fn wkupt6(&self) -> WKUPT6_R {
        WKUPT6_R::new(((self.bits >> 22) & 1) != 0)
    }
    #[doc = "Bit 23 - Wake Up Input Transition 7"]
    #[inline(always)]
    pub fn wkupt7(&self) -> WKUPT7_R {
        WKUPT7_R::new(((self.bits >> 23) & 1) != 0)
    }
    #[doc = "Bit 24 - Wake Up Input Transition 8"]
    #[inline(always)]
    pub fn wkupt8(&self) -> WKUPT8_R {
        WKUPT8_R::new(((self.bits >> 24) & 1) != 0)
    }
    #[doc = "Bit 25 - Wake Up Input Transition 9"]
    #[inline(always)]
    pub fn wkupt9(&self) -> WKUPT9_R {
        WKUPT9_R::new(((self.bits >> 25) & 1) != 0)
    }
    #[doc = "Bit 26 - Wake Up Input Transition 10"]
    #[inline(always)]
    pub fn wkupt10(&self) -> WKUPT10_R {
        WKUPT10_R::new(((self.bits >> 26) & 1) != 0)
    }
    #[doc = "Bit 27 - Wake Up Input Transition 11"]
    #[inline(always)]
    pub fn wkupt11(&self) -> WKUPT11_R {
        WKUPT11_R::new(((self.bits >> 27) & 1) != 0)
    }
    #[doc = "Bit 28 - Wake Up Input Transition 12"]
    #[inline(always)]
    pub fn wkupt12(&self) -> WKUPT12_R {
        WKUPT12_R::new(((self.bits >> 28) & 1) != 0)
    }
    #[doc = "Bit 29 - Wake Up Input Transition 13"]
    #[inline(always)]
    pub fn wkupt13(&self) -> WKUPT13_R {
        WKUPT13_R::new(((self.bits >> 29) & 1) != 0)
    }
    #[doc = "Bit 30 - Wake Up Input Transition 14"]
    #[inline(always)]
    pub fn wkupt14(&self) -> WKUPT14_R {
        WKUPT14_R::new(((self.bits >> 30) & 1) != 0)
    }
    #[doc = "Bit 31 - Wake Up Input Transition 15"]
    #[inline(always)]
    pub fn wkupt15(&self) -> WKUPT15_R {
        WKUPT15_R::new(((self.bits >> 31) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Wake Up Input Enable 0"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen0(&mut self) -> WKUPEN0_W<WUIR_SPEC, 0> {
        WKUPEN0_W::new(self)
    }
    #[doc = "Bit 1 - Wake Up Input Enable 1"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen1(&mut self) -> WKUPEN1_W<WUIR_SPEC, 1> {
        WKUPEN1_W::new(self)
    }
    #[doc = "Bit 2 - Wake Up Input Enable 2"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen2(&mut self) -> WKUPEN2_W<WUIR_SPEC, 2> {
        WKUPEN2_W::new(self)
    }
    #[doc = "Bit 3 - Wake Up Input Enable 3"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen3(&mut self) -> WKUPEN3_W<WUIR_SPEC, 3> {
        WKUPEN3_W::new(self)
    }
    #[doc = "Bit 4 - Wake Up Input Enable 4"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen4(&mut self) -> WKUPEN4_W<WUIR_SPEC, 4> {
        WKUPEN4_W::new(self)
    }
    #[doc = "Bit 5 - Wake Up Input Enable 5"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen5(&mut self) -> WKUPEN5_W<WUIR_SPEC, 5> {
        WKUPEN5_W::new(self)
    }
    #[doc = "Bit 6 - Wake Up Input Enable 6"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen6(&mut self) -> WKUPEN6_W<WUIR_SPEC, 6> {
        WKUPEN6_W::new(self)
    }
    #[doc = "Bit 7 - Wake Up Input Enable 7"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen7(&mut self) -> WKUPEN7_W<WUIR_SPEC, 7> {
        WKUPEN7_W::new(self)
    }
    #[doc = "Bit 8 - Wake Up Input Enable 8"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen8(&mut self) -> WKUPEN8_W<WUIR_SPEC, 8> {
        WKUPEN8_W::new(self)
    }
    #[doc = "Bit 9 - Wake Up Input Enable 9"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen9(&mut self) -> WKUPEN9_W<WUIR_SPEC, 9> {
        WKUPEN9_W::new(self)
    }
    #[doc = "Bit 10 - Wake Up Input Enable 10"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen10(&mut self) -> WKUPEN10_W<WUIR_SPEC, 10> {
        WKUPEN10_W::new(self)
    }
    #[doc = "Bit 11 - Wake Up Input Enable 11"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen11(&mut self) -> WKUPEN11_W<WUIR_SPEC, 11> {
        WKUPEN11_W::new(self)
    }
    #[doc = "Bit 12 - Wake Up Input Enable 12"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen12(&mut self) -> WKUPEN12_W<WUIR_SPEC, 12> {
        WKUPEN12_W::new(self)
    }
    #[doc = "Bit 13 - Wake Up Input Enable 13"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen13(&mut self) -> WKUPEN13_W<WUIR_SPEC, 13> {
        WKUPEN13_W::new(self)
    }
    #[doc = "Bit 14 - Wake Up Input Enable 14"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen14(&mut self) -> WKUPEN14_W<WUIR_SPEC, 14> {
        WKUPEN14_W::new(self)
    }
    #[doc = "Bit 15 - Wake Up Input Enable 15"]
    #[inline(always)]
    #[must_use]
    pub fn wkupen15(&mut self) -> WKUPEN15_W<WUIR_SPEC, 15> {
        WKUPEN15_W::new(self)
    }
    #[doc = "Bit 16 - Wake Up Input Transition 0"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt0(&mut self) -> WKUPT0_W<WUIR_SPEC, 16> {
        WKUPT0_W::new(self)
    }
    #[doc = "Bit 17 - Wake Up Input Transition 1"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt1(&mut self) -> WKUPT1_W<WUIR_SPEC, 17> {
        WKUPT1_W::new(self)
    }
    #[doc = "Bit 18 - Wake Up Input Transition 2"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt2(&mut self) -> WKUPT2_W<WUIR_SPEC, 18> {
        WKUPT2_W::new(self)
    }
    #[doc = "Bit 19 - Wake Up Input Transition 3"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt3(&mut self) -> WKUPT3_W<WUIR_SPEC, 19> {
        WKUPT3_W::new(self)
    }
    #[doc = "Bit 20 - Wake Up Input Transition 4"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt4(&mut self) -> WKUPT4_W<WUIR_SPEC, 20> {
        WKUPT4_W::new(self)
    }
    #[doc = "Bit 21 - Wake Up Input Transition 5"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt5(&mut self) -> WKUPT5_W<WUIR_SPEC, 21> {
        WKUPT5_W::new(self)
    }
    #[doc = "Bit 22 - Wake Up Input Transition 6"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt6(&mut self) -> WKUPT6_W<WUIR_SPEC, 22> {
        WKUPT6_W::new(self)
    }
    #[doc = "Bit 23 - Wake Up Input Transition 7"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt7(&mut self) -> WKUPT7_W<WUIR_SPEC, 23> {
        WKUPT7_W::new(self)
    }
    #[doc = "Bit 24 - Wake Up Input Transition 8"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt8(&mut self) -> WKUPT8_W<WUIR_SPEC, 24> {
        WKUPT8_W::new(self)
    }
    #[doc = "Bit 25 - Wake Up Input Transition 9"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt9(&mut self) -> WKUPT9_W<WUIR_SPEC, 25> {
        WKUPT9_W::new(self)
    }
    #[doc = "Bit 26 - Wake Up Input Transition 10"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt10(&mut self) -> WKUPT10_W<WUIR_SPEC, 26> {
        WKUPT10_W::new(self)
    }
    #[doc = "Bit 27 - Wake Up Input Transition 11"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt11(&mut self) -> WKUPT11_W<WUIR_SPEC, 27> {
        WKUPT11_W::new(self)
    }
    #[doc = "Bit 28 - Wake Up Input Transition 12"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt12(&mut self) -> WKUPT12_W<WUIR_SPEC, 28> {
        WKUPT12_W::new(self)
    }
    #[doc = "Bit 29 - Wake Up Input Transition 13"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt13(&mut self) -> WKUPT13_W<WUIR_SPEC, 29> {
        WKUPT13_W::new(self)
    }
    #[doc = "Bit 30 - Wake Up Input Transition 14"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt14(&mut self) -> WKUPT14_W<WUIR_SPEC, 30> {
        WKUPT14_W::new(self)
    }
    #[doc = "Bit 31 - Wake Up Input Transition 15"]
    #[inline(always)]
    #[must_use]
    pub fn wkupt15(&mut self) -> WKUPT15_W<WUIR_SPEC, 31> {
        WKUPT15_W::new(self)
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
#[doc = "Supply Controller Wake Up Inputs Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wuir::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`wuir::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct WUIR_SPEC;
impl crate::RegisterSpec for WUIR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`wuir::R`](R) reader structure"]
impl crate::Readable for WUIR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`wuir::W`](W) writer structure"]
impl crate::Writable for WUIR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets WUIR to value 0"]
impl crate::Resettable for WUIR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
