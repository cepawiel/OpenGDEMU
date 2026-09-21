#[doc = "Register `CMR2_WAVE_EQ_1` reader"]
pub type R = crate::R<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC>;
#[doc = "Register `CMR2_WAVE_EQ_1` writer"]
pub type W = crate::W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC>;
#[doc = "Field `TCCLKS` reader - Clock Selection"]
pub type TCCLKS_R = crate::FieldReader<TCCLKS_A>;
#[doc = "Clock Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TCCLKS_A {
    #[doc = "0: Clock selected: TCLK1"]
    TIMER_CLOCK1 = 0,
    #[doc = "1: Clock selected: TCLK2"]
    TIMER_CLOCK2 = 1,
    #[doc = "2: Clock selected: TCLK3"]
    TIMER_CLOCK3 = 2,
    #[doc = "3: Clock selected: TCLK4"]
    TIMER_CLOCK4 = 3,
    #[doc = "4: Clock selected: TCLK5"]
    TIMER_CLOCK5 = 4,
    #[doc = "5: Clock selected: XC0"]
    XC0 = 5,
    #[doc = "6: Clock selected: XC1"]
    XC1 = 6,
    #[doc = "7: Clock selected: XC2"]
    XC2 = 7,
}
impl From<TCCLKS_A> for u8 {
    #[inline(always)]
    fn from(variant: TCCLKS_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for TCCLKS_A {
    type Ux = u8;
}
impl TCCLKS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TCCLKS_A {
        match self.bits {
            0 => TCCLKS_A::TIMER_CLOCK1,
            1 => TCCLKS_A::TIMER_CLOCK2,
            2 => TCCLKS_A::TIMER_CLOCK3,
            3 => TCCLKS_A::TIMER_CLOCK4,
            4 => TCCLKS_A::TIMER_CLOCK5,
            5 => TCCLKS_A::XC0,
            6 => TCCLKS_A::XC1,
            7 => TCCLKS_A::XC2,
            _ => unreachable!(),
        }
    }
    #[doc = "Clock selected: TCLK1"]
    #[inline(always)]
    pub fn is_timer_clock1(&self) -> bool {
        *self == TCCLKS_A::TIMER_CLOCK1
    }
    #[doc = "Clock selected: TCLK2"]
    #[inline(always)]
    pub fn is_timer_clock2(&self) -> bool {
        *self == TCCLKS_A::TIMER_CLOCK2
    }
    #[doc = "Clock selected: TCLK3"]
    #[inline(always)]
    pub fn is_timer_clock3(&self) -> bool {
        *self == TCCLKS_A::TIMER_CLOCK3
    }
    #[doc = "Clock selected: TCLK4"]
    #[inline(always)]
    pub fn is_timer_clock4(&self) -> bool {
        *self == TCCLKS_A::TIMER_CLOCK4
    }
    #[doc = "Clock selected: TCLK5"]
    #[inline(always)]
    pub fn is_timer_clock5(&self) -> bool {
        *self == TCCLKS_A::TIMER_CLOCK5
    }
    #[doc = "Clock selected: XC0"]
    #[inline(always)]
    pub fn is_xc0(&self) -> bool {
        *self == TCCLKS_A::XC0
    }
    #[doc = "Clock selected: XC1"]
    #[inline(always)]
    pub fn is_xc1(&self) -> bool {
        *self == TCCLKS_A::XC1
    }
    #[doc = "Clock selected: XC2"]
    #[inline(always)]
    pub fn is_xc2(&self) -> bool {
        *self == TCCLKS_A::XC2
    }
}
#[doc = "Field `TCCLKS` writer - Clock Selection"]
pub type TCCLKS_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 3, O, TCCLKS_A>;
impl<'a, REG, const O: u8> TCCLKS_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Clock selected: TCLK1"]
    #[inline(always)]
    pub fn timer_clock1(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::TIMER_CLOCK1)
    }
    #[doc = "Clock selected: TCLK2"]
    #[inline(always)]
    pub fn timer_clock2(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::TIMER_CLOCK2)
    }
    #[doc = "Clock selected: TCLK3"]
    #[inline(always)]
    pub fn timer_clock3(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::TIMER_CLOCK3)
    }
    #[doc = "Clock selected: TCLK4"]
    #[inline(always)]
    pub fn timer_clock4(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::TIMER_CLOCK4)
    }
    #[doc = "Clock selected: TCLK5"]
    #[inline(always)]
    pub fn timer_clock5(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::TIMER_CLOCK5)
    }
    #[doc = "Clock selected: XC0"]
    #[inline(always)]
    pub fn xc0(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::XC0)
    }
    #[doc = "Clock selected: XC1"]
    #[inline(always)]
    pub fn xc1(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::XC1)
    }
    #[doc = "Clock selected: XC2"]
    #[inline(always)]
    pub fn xc2(self) -> &'a mut crate::W<REG> {
        self.variant(TCCLKS_A::XC2)
    }
}
#[doc = "Field `CLKI` reader - Clock Invert"]
pub type CLKI_R = crate::BitReader;
#[doc = "Field `CLKI` writer - Clock Invert"]
pub type CLKI_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `BURST` reader - Burst Signal Selection"]
pub type BURST_R = crate::FieldReader<BURST_A>;
#[doc = "Burst Signal Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BURST_A {
    #[doc = "0: The clock is not gated by an external signal."]
    NONE = 0,
    #[doc = "1: XC0 is ANDed with the selected clock."]
    XC0 = 1,
    #[doc = "2: XC1 is ANDed with the selected clock."]
    XC1 = 2,
    #[doc = "3: XC2 is ANDed with the selected clock."]
    XC2 = 3,
}
impl From<BURST_A> for u8 {
    #[inline(always)]
    fn from(variant: BURST_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for BURST_A {
    type Ux = u8;
}
impl BURST_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> BURST_A {
        match self.bits {
            0 => BURST_A::NONE,
            1 => BURST_A::XC0,
            2 => BURST_A::XC1,
            3 => BURST_A::XC2,
            _ => unreachable!(),
        }
    }
    #[doc = "The clock is not gated by an external signal."]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == BURST_A::NONE
    }
    #[doc = "XC0 is ANDed with the selected clock."]
    #[inline(always)]
    pub fn is_xc0(&self) -> bool {
        *self == BURST_A::XC0
    }
    #[doc = "XC1 is ANDed with the selected clock."]
    #[inline(always)]
    pub fn is_xc1(&self) -> bool {
        *self == BURST_A::XC1
    }
    #[doc = "XC2 is ANDed with the selected clock."]
    #[inline(always)]
    pub fn is_xc2(&self) -> bool {
        *self == BURST_A::XC2
    }
}
#[doc = "Field `BURST` writer - Burst Signal Selection"]
pub type BURST_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, BURST_A>;
impl<'a, REG, const O: u8> BURST_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "The clock is not gated by an external signal."]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(BURST_A::NONE)
    }
    #[doc = "XC0 is ANDed with the selected clock."]
    #[inline(always)]
    pub fn xc0(self) -> &'a mut crate::W<REG> {
        self.variant(BURST_A::XC0)
    }
    #[doc = "XC1 is ANDed with the selected clock."]
    #[inline(always)]
    pub fn xc1(self) -> &'a mut crate::W<REG> {
        self.variant(BURST_A::XC1)
    }
    #[doc = "XC2 is ANDed with the selected clock."]
    #[inline(always)]
    pub fn xc2(self) -> &'a mut crate::W<REG> {
        self.variant(BURST_A::XC2)
    }
}
#[doc = "Field `CPCSTOP` reader - Counter Clock Stopped with RC Compare"]
pub type CPCSTOP_R = crate::BitReader;
#[doc = "Field `CPCSTOP` writer - Counter Clock Stopped with RC Compare"]
pub type CPCSTOP_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CPCDIS` reader - Counter Clock Disable with RC Compare"]
pub type CPCDIS_R = crate::BitReader;
#[doc = "Field `CPCDIS` writer - Counter Clock Disable with RC Compare"]
pub type CPCDIS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `EEVTEDG` reader - External Event Edge Selection"]
pub type EEVTEDG_R = crate::FieldReader<EEVTEDG_A>;
#[doc = "External Event Edge Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EEVTEDG_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Rising edge"]
    RISING = 1,
    #[doc = "2: Falling edge"]
    FALLING = 2,
    #[doc = "3: Each edge"]
    EDGE = 3,
}
impl From<EEVTEDG_A> for u8 {
    #[inline(always)]
    fn from(variant: EEVTEDG_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for EEVTEDG_A {
    type Ux = u8;
}
impl EEVTEDG_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> EEVTEDG_A {
        match self.bits {
            0 => EEVTEDG_A::NONE,
            1 => EEVTEDG_A::RISING,
            2 => EEVTEDG_A::FALLING,
            3 => EEVTEDG_A::EDGE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == EEVTEDG_A::NONE
    }
    #[doc = "Rising edge"]
    #[inline(always)]
    pub fn is_rising(&self) -> bool {
        *self == EEVTEDG_A::RISING
    }
    #[doc = "Falling edge"]
    #[inline(always)]
    pub fn is_falling(&self) -> bool {
        *self == EEVTEDG_A::FALLING
    }
    #[doc = "Each edge"]
    #[inline(always)]
    pub fn is_edge(&self) -> bool {
        *self == EEVTEDG_A::EDGE
    }
}
#[doc = "Field `EEVTEDG` writer - External Event Edge Selection"]
pub type EEVTEDG_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, EEVTEDG_A>;
impl<'a, REG, const O: u8> EEVTEDG_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(EEVTEDG_A::NONE)
    }
    #[doc = "Rising edge"]
    #[inline(always)]
    pub fn rising(self) -> &'a mut crate::W<REG> {
        self.variant(EEVTEDG_A::RISING)
    }
    #[doc = "Falling edge"]
    #[inline(always)]
    pub fn falling(self) -> &'a mut crate::W<REG> {
        self.variant(EEVTEDG_A::FALLING)
    }
    #[doc = "Each edge"]
    #[inline(always)]
    pub fn edge(self) -> &'a mut crate::W<REG> {
        self.variant(EEVTEDG_A::EDGE)
    }
}
#[doc = "Field `EEVT` reader - External Event Selection"]
pub type EEVT_R = crate::FieldReader<EEVT_A>;
#[doc = "External Event Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EEVT_A {
    #[doc = "0: TIOB"]
    TIOB = 0,
    #[doc = "1: XC0"]
    XC0 = 1,
    #[doc = "2: XC1"]
    XC1 = 2,
    #[doc = "3: XC2"]
    XC2 = 3,
}
impl From<EEVT_A> for u8 {
    #[inline(always)]
    fn from(variant: EEVT_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for EEVT_A {
    type Ux = u8;
}
impl EEVT_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> EEVT_A {
        match self.bits {
            0 => EEVT_A::TIOB,
            1 => EEVT_A::XC0,
            2 => EEVT_A::XC1,
            3 => EEVT_A::XC2,
            _ => unreachable!(),
        }
    }
    #[doc = "TIOB"]
    #[inline(always)]
    pub fn is_tiob(&self) -> bool {
        *self == EEVT_A::TIOB
    }
    #[doc = "XC0"]
    #[inline(always)]
    pub fn is_xc0(&self) -> bool {
        *self == EEVT_A::XC0
    }
    #[doc = "XC1"]
    #[inline(always)]
    pub fn is_xc1(&self) -> bool {
        *self == EEVT_A::XC1
    }
    #[doc = "XC2"]
    #[inline(always)]
    pub fn is_xc2(&self) -> bool {
        *self == EEVT_A::XC2
    }
}
#[doc = "Field `EEVT` writer - External Event Selection"]
pub type EEVT_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, EEVT_A>;
impl<'a, REG, const O: u8> EEVT_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "TIOB"]
    #[inline(always)]
    pub fn tiob(self) -> &'a mut crate::W<REG> {
        self.variant(EEVT_A::TIOB)
    }
    #[doc = "XC0"]
    #[inline(always)]
    pub fn xc0(self) -> &'a mut crate::W<REG> {
        self.variant(EEVT_A::XC0)
    }
    #[doc = "XC1"]
    #[inline(always)]
    pub fn xc1(self) -> &'a mut crate::W<REG> {
        self.variant(EEVT_A::XC1)
    }
    #[doc = "XC2"]
    #[inline(always)]
    pub fn xc2(self) -> &'a mut crate::W<REG> {
        self.variant(EEVT_A::XC2)
    }
}
#[doc = "Field `ENETRG` reader - External Event Trigger Enable"]
pub type ENETRG_R = crate::BitReader;
#[doc = "Field `ENETRG` writer - External Event Trigger Enable"]
pub type ENETRG_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `WAVSEL` reader - Waveform Selection"]
pub type WAVSEL_R = crate::FieldReader<WAVSEL_A>;
#[doc = "Waveform Selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WAVSEL_A {
    #[doc = "0: UP mode without automatic trigger on RC Compare"]
    UP = 0,
    #[doc = "1: UPDOWN mode without automatic trigger on RC Compare"]
    UPDOWN = 1,
    #[doc = "2: UP mode with automatic trigger on RC Compare"]
    UP_RC = 2,
    #[doc = "3: UPDOWN mode with automatic trigger on RC Compare"]
    UPDOWN_RC = 3,
}
impl From<WAVSEL_A> for u8 {
    #[inline(always)]
    fn from(variant: WAVSEL_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for WAVSEL_A {
    type Ux = u8;
}
impl WAVSEL_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WAVSEL_A {
        match self.bits {
            0 => WAVSEL_A::UP,
            1 => WAVSEL_A::UPDOWN,
            2 => WAVSEL_A::UP_RC,
            3 => WAVSEL_A::UPDOWN_RC,
            _ => unreachable!(),
        }
    }
    #[doc = "UP mode without automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn is_up(&self) -> bool {
        *self == WAVSEL_A::UP
    }
    #[doc = "UPDOWN mode without automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn is_updown(&self) -> bool {
        *self == WAVSEL_A::UPDOWN
    }
    #[doc = "UP mode with automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn is_up_rc(&self) -> bool {
        *self == WAVSEL_A::UP_RC
    }
    #[doc = "UPDOWN mode with automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn is_updown_rc(&self) -> bool {
        *self == WAVSEL_A::UPDOWN_RC
    }
}
#[doc = "Field `WAVSEL` writer - Waveform Selection"]
pub type WAVSEL_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, WAVSEL_A>;
impl<'a, REG, const O: u8> WAVSEL_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "UP mode without automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn up(self) -> &'a mut crate::W<REG> {
        self.variant(WAVSEL_A::UP)
    }
    #[doc = "UPDOWN mode without automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn updown(self) -> &'a mut crate::W<REG> {
        self.variant(WAVSEL_A::UPDOWN)
    }
    #[doc = "UP mode with automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn up_rc(self) -> &'a mut crate::W<REG> {
        self.variant(WAVSEL_A::UP_RC)
    }
    #[doc = "UPDOWN mode with automatic trigger on RC Compare"]
    #[inline(always)]
    pub fn updown_rc(self) -> &'a mut crate::W<REG> {
        self.variant(WAVSEL_A::UPDOWN_RC)
    }
}
#[doc = "Field `WAVE` reader - Waveform Mode"]
pub type WAVE_R = crate::BitReader;
#[doc = "Field `WAVE` writer - Waveform Mode"]
pub type WAVE_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ACPA` reader - RA Compare Effect on TIOA"]
pub type ACPA_R = crate::FieldReader<ACPA_A>;
#[doc = "RA Compare Effect on TIOA\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ACPA_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<ACPA_A> for u8 {
    #[inline(always)]
    fn from(variant: ACPA_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for ACPA_A {
    type Ux = u8;
}
impl ACPA_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ACPA_A {
        match self.bits {
            0 => ACPA_A::NONE,
            1 => ACPA_A::SET,
            2 => ACPA_A::CLEAR,
            3 => ACPA_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == ACPA_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == ACPA_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == ACPA_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == ACPA_A::TOGGLE
    }
}
#[doc = "Field `ACPA` writer - RA Compare Effect on TIOA"]
pub type ACPA_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, ACPA_A>;
impl<'a, REG, const O: u8> ACPA_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(ACPA_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(ACPA_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(ACPA_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(ACPA_A::TOGGLE)
    }
}
#[doc = "Field `ACPC` reader - RC Compare Effect on TIOA"]
pub type ACPC_R = crate::FieldReader<ACPC_A>;
#[doc = "RC Compare Effect on TIOA\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ACPC_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<ACPC_A> for u8 {
    #[inline(always)]
    fn from(variant: ACPC_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for ACPC_A {
    type Ux = u8;
}
impl ACPC_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ACPC_A {
        match self.bits {
            0 => ACPC_A::NONE,
            1 => ACPC_A::SET,
            2 => ACPC_A::CLEAR,
            3 => ACPC_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == ACPC_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == ACPC_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == ACPC_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == ACPC_A::TOGGLE
    }
}
#[doc = "Field `ACPC` writer - RC Compare Effect on TIOA"]
pub type ACPC_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, ACPC_A>;
impl<'a, REG, const O: u8> ACPC_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(ACPC_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(ACPC_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(ACPC_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(ACPC_A::TOGGLE)
    }
}
#[doc = "Field `AEEVT` reader - External Event Effect on TIOA"]
pub type AEEVT_R = crate::FieldReader<AEEVT_A>;
#[doc = "External Event Effect on TIOA\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AEEVT_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<AEEVT_A> for u8 {
    #[inline(always)]
    fn from(variant: AEEVT_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for AEEVT_A {
    type Ux = u8;
}
impl AEEVT_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> AEEVT_A {
        match self.bits {
            0 => AEEVT_A::NONE,
            1 => AEEVT_A::SET,
            2 => AEEVT_A::CLEAR,
            3 => AEEVT_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == AEEVT_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == AEEVT_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == AEEVT_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == AEEVT_A::TOGGLE
    }
}
#[doc = "Field `AEEVT` writer - External Event Effect on TIOA"]
pub type AEEVT_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, AEEVT_A>;
impl<'a, REG, const O: u8> AEEVT_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(AEEVT_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(AEEVT_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(AEEVT_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(AEEVT_A::TOGGLE)
    }
}
#[doc = "Field `ASWTRG` reader - Software Trigger Effect on TIOA"]
pub type ASWTRG_R = crate::FieldReader<ASWTRG_A>;
#[doc = "Software Trigger Effect on TIOA\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ASWTRG_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<ASWTRG_A> for u8 {
    #[inline(always)]
    fn from(variant: ASWTRG_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for ASWTRG_A {
    type Ux = u8;
}
impl ASWTRG_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ASWTRG_A {
        match self.bits {
            0 => ASWTRG_A::NONE,
            1 => ASWTRG_A::SET,
            2 => ASWTRG_A::CLEAR,
            3 => ASWTRG_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == ASWTRG_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == ASWTRG_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == ASWTRG_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == ASWTRG_A::TOGGLE
    }
}
#[doc = "Field `ASWTRG` writer - Software Trigger Effect on TIOA"]
pub type ASWTRG_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, ASWTRG_A>;
impl<'a, REG, const O: u8> ASWTRG_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(ASWTRG_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(ASWTRG_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(ASWTRG_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(ASWTRG_A::TOGGLE)
    }
}
#[doc = "Field `BCPB` reader - RB Compare Effect on TIOB"]
pub type BCPB_R = crate::FieldReader<BCPB_A>;
#[doc = "RB Compare Effect on TIOB\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BCPB_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<BCPB_A> for u8 {
    #[inline(always)]
    fn from(variant: BCPB_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for BCPB_A {
    type Ux = u8;
}
impl BCPB_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> BCPB_A {
        match self.bits {
            0 => BCPB_A::NONE,
            1 => BCPB_A::SET,
            2 => BCPB_A::CLEAR,
            3 => BCPB_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == BCPB_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == BCPB_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == BCPB_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == BCPB_A::TOGGLE
    }
}
#[doc = "Field `BCPB` writer - RB Compare Effect on TIOB"]
pub type BCPB_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, BCPB_A>;
impl<'a, REG, const O: u8> BCPB_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(BCPB_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(BCPB_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(BCPB_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(BCPB_A::TOGGLE)
    }
}
#[doc = "Field `BCPC` reader - RC Compare Effect on TIOB"]
pub type BCPC_R = crate::FieldReader<BCPC_A>;
#[doc = "RC Compare Effect on TIOB\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BCPC_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<BCPC_A> for u8 {
    #[inline(always)]
    fn from(variant: BCPC_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for BCPC_A {
    type Ux = u8;
}
impl BCPC_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> BCPC_A {
        match self.bits {
            0 => BCPC_A::NONE,
            1 => BCPC_A::SET,
            2 => BCPC_A::CLEAR,
            3 => BCPC_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == BCPC_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == BCPC_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == BCPC_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == BCPC_A::TOGGLE
    }
}
#[doc = "Field `BCPC` writer - RC Compare Effect on TIOB"]
pub type BCPC_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, BCPC_A>;
impl<'a, REG, const O: u8> BCPC_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(BCPC_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(BCPC_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(BCPC_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(BCPC_A::TOGGLE)
    }
}
#[doc = "Field `BEEVT` reader - External Event Effect on TIOB"]
pub type BEEVT_R = crate::FieldReader<BEEVT_A>;
#[doc = "External Event Effect on TIOB\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BEEVT_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<BEEVT_A> for u8 {
    #[inline(always)]
    fn from(variant: BEEVT_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for BEEVT_A {
    type Ux = u8;
}
impl BEEVT_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> BEEVT_A {
        match self.bits {
            0 => BEEVT_A::NONE,
            1 => BEEVT_A::SET,
            2 => BEEVT_A::CLEAR,
            3 => BEEVT_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == BEEVT_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == BEEVT_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == BEEVT_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == BEEVT_A::TOGGLE
    }
}
#[doc = "Field `BEEVT` writer - External Event Effect on TIOB"]
pub type BEEVT_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, BEEVT_A>;
impl<'a, REG, const O: u8> BEEVT_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(BEEVT_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(BEEVT_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(BEEVT_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(BEEVT_A::TOGGLE)
    }
}
#[doc = "Field `BSWTRG` reader - Software Trigger Effect on TIOB"]
pub type BSWTRG_R = crate::FieldReader<BSWTRG_A>;
#[doc = "Software Trigger Effect on TIOB\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BSWTRG_A {
    #[doc = "0: None"]
    NONE = 0,
    #[doc = "1: Set"]
    SET = 1,
    #[doc = "2: Clear"]
    CLEAR = 2,
    #[doc = "3: Toggle"]
    TOGGLE = 3,
}
impl From<BSWTRG_A> for u8 {
    #[inline(always)]
    fn from(variant: BSWTRG_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for BSWTRG_A {
    type Ux = u8;
}
impl BSWTRG_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> BSWTRG_A {
        match self.bits {
            0 => BSWTRG_A::NONE,
            1 => BSWTRG_A::SET,
            2 => BSWTRG_A::CLEAR,
            3 => BSWTRG_A::TOGGLE,
            _ => unreachable!(),
        }
    }
    #[doc = "None"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == BSWTRG_A::NONE
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        *self == BSWTRG_A::SET
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn is_clear(&self) -> bool {
        *self == BSWTRG_A::CLEAR
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn is_toggle(&self) -> bool {
        *self == BSWTRG_A::TOGGLE
    }
}
#[doc = "Field `BSWTRG` writer - Software Trigger Effect on TIOB"]
pub type BSWTRG_W<'a, REG, const O: u8> = crate::FieldWriterSafe<'a, REG, 2, O, BSWTRG_A>;
impl<'a, REG, const O: u8> BSWTRG_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "None"]
    #[inline(always)]
    pub fn none(self) -> &'a mut crate::W<REG> {
        self.variant(BSWTRG_A::NONE)
    }
    #[doc = "Set"]
    #[inline(always)]
    pub fn set(self) -> &'a mut crate::W<REG> {
        self.variant(BSWTRG_A::SET)
    }
    #[doc = "Clear"]
    #[inline(always)]
    pub fn clear(self) -> &'a mut crate::W<REG> {
        self.variant(BSWTRG_A::CLEAR)
    }
    #[doc = "Toggle"]
    #[inline(always)]
    pub fn toggle(self) -> &'a mut crate::W<REG> {
        self.variant(BSWTRG_A::TOGGLE)
    }
}
impl R {
    #[doc = "Bits 0:2 - Clock Selection"]
    #[inline(always)]
    pub fn tcclks(&self) -> TCCLKS_R {
        TCCLKS_R::new((self.bits & 7) as u8)
    }
    #[doc = "Bit 3 - Clock Invert"]
    #[inline(always)]
    pub fn clki(&self) -> CLKI_R {
        CLKI_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bits 4:5 - Burst Signal Selection"]
    #[inline(always)]
    pub fn burst(&self) -> BURST_R {
        BURST_R::new(((self.bits >> 4) & 3) as u8)
    }
    #[doc = "Bit 6 - Counter Clock Stopped with RC Compare"]
    #[inline(always)]
    pub fn cpcstop(&self) -> CPCSTOP_R {
        CPCSTOP_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - Counter Clock Disable with RC Compare"]
    #[inline(always)]
    pub fn cpcdis(&self) -> CPCDIS_R {
        CPCDIS_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bits 8:9 - External Event Edge Selection"]
    #[inline(always)]
    pub fn eevtedg(&self) -> EEVTEDG_R {
        EEVTEDG_R::new(((self.bits >> 8) & 3) as u8)
    }
    #[doc = "Bits 10:11 - External Event Selection"]
    #[inline(always)]
    pub fn eevt(&self) -> EEVT_R {
        EEVT_R::new(((self.bits >> 10) & 3) as u8)
    }
    #[doc = "Bit 12 - External Event Trigger Enable"]
    #[inline(always)]
    pub fn enetrg(&self) -> ENETRG_R {
        ENETRG_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bits 13:14 - Waveform Selection"]
    #[inline(always)]
    pub fn wavsel(&self) -> WAVSEL_R {
        WAVSEL_R::new(((self.bits >> 13) & 3) as u8)
    }
    #[doc = "Bit 15 - Waveform Mode"]
    #[inline(always)]
    pub fn wave(&self) -> WAVE_R {
        WAVE_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bits 16:17 - RA Compare Effect on TIOA"]
    #[inline(always)]
    pub fn acpa(&self) -> ACPA_R {
        ACPA_R::new(((self.bits >> 16) & 3) as u8)
    }
    #[doc = "Bits 18:19 - RC Compare Effect on TIOA"]
    #[inline(always)]
    pub fn acpc(&self) -> ACPC_R {
        ACPC_R::new(((self.bits >> 18) & 3) as u8)
    }
    #[doc = "Bits 20:21 - External Event Effect on TIOA"]
    #[inline(always)]
    pub fn aeevt(&self) -> AEEVT_R {
        AEEVT_R::new(((self.bits >> 20) & 3) as u8)
    }
    #[doc = "Bits 22:23 - Software Trigger Effect on TIOA"]
    #[inline(always)]
    pub fn aswtrg(&self) -> ASWTRG_R {
        ASWTRG_R::new(((self.bits >> 22) & 3) as u8)
    }
    #[doc = "Bits 24:25 - RB Compare Effect on TIOB"]
    #[inline(always)]
    pub fn bcpb(&self) -> BCPB_R {
        BCPB_R::new(((self.bits >> 24) & 3) as u8)
    }
    #[doc = "Bits 26:27 - RC Compare Effect on TIOB"]
    #[inline(always)]
    pub fn bcpc(&self) -> BCPC_R {
        BCPC_R::new(((self.bits >> 26) & 3) as u8)
    }
    #[doc = "Bits 28:29 - External Event Effect on TIOB"]
    #[inline(always)]
    pub fn beevt(&self) -> BEEVT_R {
        BEEVT_R::new(((self.bits >> 28) & 3) as u8)
    }
    #[doc = "Bits 30:31 - Software Trigger Effect on TIOB"]
    #[inline(always)]
    pub fn bswtrg(&self) -> BSWTRG_R {
        BSWTRG_R::new(((self.bits >> 30) & 3) as u8)
    }
}
impl W {
    #[doc = "Bits 0:2 - Clock Selection"]
    #[inline(always)]
    #[must_use]
    pub fn tcclks(&mut self) -> TCCLKS_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 0> {
        TCCLKS_W::new(self)
    }
    #[doc = "Bit 3 - Clock Invert"]
    #[inline(always)]
    #[must_use]
    pub fn clki(&mut self) -> CLKI_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 3> {
        CLKI_W::new(self)
    }
    #[doc = "Bits 4:5 - Burst Signal Selection"]
    #[inline(always)]
    #[must_use]
    pub fn burst(&mut self) -> BURST_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 4> {
        BURST_W::new(self)
    }
    #[doc = "Bit 6 - Counter Clock Stopped with RC Compare"]
    #[inline(always)]
    #[must_use]
    pub fn cpcstop(&mut self) -> CPCSTOP_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 6> {
        CPCSTOP_W::new(self)
    }
    #[doc = "Bit 7 - Counter Clock Disable with RC Compare"]
    #[inline(always)]
    #[must_use]
    pub fn cpcdis(&mut self) -> CPCDIS_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 7> {
        CPCDIS_W::new(self)
    }
    #[doc = "Bits 8:9 - External Event Edge Selection"]
    #[inline(always)]
    #[must_use]
    pub fn eevtedg(&mut self) -> EEVTEDG_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 8> {
        EEVTEDG_W::new(self)
    }
    #[doc = "Bits 10:11 - External Event Selection"]
    #[inline(always)]
    #[must_use]
    pub fn eevt(&mut self) -> EEVT_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 10> {
        EEVT_W::new(self)
    }
    #[doc = "Bit 12 - External Event Trigger Enable"]
    #[inline(always)]
    #[must_use]
    pub fn enetrg(&mut self) -> ENETRG_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 12> {
        ENETRG_W::new(self)
    }
    #[doc = "Bits 13:14 - Waveform Selection"]
    #[inline(always)]
    #[must_use]
    pub fn wavsel(&mut self) -> WAVSEL_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 13> {
        WAVSEL_W::new(self)
    }
    #[doc = "Bit 15 - Waveform Mode"]
    #[inline(always)]
    #[must_use]
    pub fn wave(&mut self) -> WAVE_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 15> {
        WAVE_W::new(self)
    }
    #[doc = "Bits 16:17 - RA Compare Effect on TIOA"]
    #[inline(always)]
    #[must_use]
    pub fn acpa(&mut self) -> ACPA_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 16> {
        ACPA_W::new(self)
    }
    #[doc = "Bits 18:19 - RC Compare Effect on TIOA"]
    #[inline(always)]
    #[must_use]
    pub fn acpc(&mut self) -> ACPC_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 18> {
        ACPC_W::new(self)
    }
    #[doc = "Bits 20:21 - External Event Effect on TIOA"]
    #[inline(always)]
    #[must_use]
    pub fn aeevt(&mut self) -> AEEVT_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 20> {
        AEEVT_W::new(self)
    }
    #[doc = "Bits 22:23 - Software Trigger Effect on TIOA"]
    #[inline(always)]
    #[must_use]
    pub fn aswtrg(&mut self) -> ASWTRG_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 22> {
        ASWTRG_W::new(self)
    }
    #[doc = "Bits 24:25 - RB Compare Effect on TIOB"]
    #[inline(always)]
    #[must_use]
    pub fn bcpb(&mut self) -> BCPB_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 24> {
        BCPB_W::new(self)
    }
    #[doc = "Bits 26:27 - RC Compare Effect on TIOB"]
    #[inline(always)]
    #[must_use]
    pub fn bcpc(&mut self) -> BCPC_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 26> {
        BCPC_W::new(self)
    }
    #[doc = "Bits 28:29 - External Event Effect on TIOB"]
    #[inline(always)]
    #[must_use]
    pub fn beevt(&mut self) -> BEEVT_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 28> {
        BEEVT_W::new(self)
    }
    #[doc = "Bits 30:31 - Software Trigger Effect on TIOB"]
    #[inline(always)]
    #[must_use]
    pub fn bswtrg(&mut self) -> BSWTRG_W<WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC, 30> {
        BSWTRG_W::new(self)
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
#[doc = "Channel Mode Register (channel = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wave_eq_1_cmr2_wave_eq_1::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`wave_eq_1_cmr2_wave_eq_1::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC;
impl crate::RegisterSpec for WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`wave_eq_1_cmr2_wave_eq_1::R`](R) reader structure"]
impl crate::Readable for WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC {}
#[doc = "`write(|w| ..)` method takes [`wave_eq_1_cmr2_wave_eq_1::W`](W) writer structure"]
impl crate::Writable for WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CMR2_WAVE_EQ_1 to value 0"]
impl crate::Resettable for WAVE_EQ_1_CMR2_WAVE_EQ_1_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
