#[doc = "Register `SR` reader"]
pub type R = crate::R<SR_SPEC>;
#[doc = "Field `FWUPS` reader - FWUP Wake Up Status"]
pub type FWUPS_R = crate::BitReader<FWUPS_A>;
#[doc = "FWUP Wake Up Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FWUPS_A {
    #[doc = "0: no wake up due to the assertion of the FWUP pin has occurred since the last read of SUPC_SR."]
    NO = 0,
    #[doc = "1: at least one wake up due to the assertion of the FWUP pin has occurred since the last read of SUPC_SR."]
    PRESENT = 1,
}
impl From<FWUPS_A> for bool {
    #[inline(always)]
    fn from(variant: FWUPS_A) -> Self {
        variant as u8 != 0
    }
}
impl FWUPS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> FWUPS_A {
        match self.bits {
            false => FWUPS_A::NO,
            true => FWUPS_A::PRESENT,
        }
    }
    #[doc = "no wake up due to the assertion of the FWUP pin has occurred since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_no(&self) -> bool {
        *self == FWUPS_A::NO
    }
    #[doc = "at least one wake up due to the assertion of the FWUP pin has occurred since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_present(&self) -> bool {
        *self == FWUPS_A::PRESENT
    }
}
#[doc = "Field `WKUPS` reader - WKUP Wake Up Status"]
pub type WKUPS_R = crate::BitReader<WKUPS_A>;
#[doc = "WKUP Wake Up Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPS_A {
    #[doc = "0: no wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR."]
    NO = 0,
    #[doc = "1: at least one wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR."]
    PRESENT = 1,
}
impl From<WKUPS_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPS_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPS_A {
        match self.bits {
            false => WKUPS_A::NO,
            true => WKUPS_A::PRESENT,
        }
    }
    #[doc = "no wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_no(&self) -> bool {
        *self == WKUPS_A::NO
    }
    #[doc = "at least one wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_present(&self) -> bool {
        *self == WKUPS_A::PRESENT
    }
}
#[doc = "Field `SMWS` reader - Supply Monitor Detection Wake Up Status"]
pub type SMWS_R = crate::BitReader<SMWS_A>;
#[doc = "Supply Monitor Detection Wake Up Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SMWS_A {
    #[doc = "0: no wake up due to a supply monitor detection has occurred since the last read of SUPC_SR."]
    NO = 0,
    #[doc = "1: at least one wake up due to a supply monitor detection has occurred since the last read of SUPC_SR."]
    PRESENT = 1,
}
impl From<SMWS_A> for bool {
    #[inline(always)]
    fn from(variant: SMWS_A) -> Self {
        variant as u8 != 0
    }
}
impl SMWS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> SMWS_A {
        match self.bits {
            false => SMWS_A::NO,
            true => SMWS_A::PRESENT,
        }
    }
    #[doc = "no wake up due to a supply monitor detection has occurred since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_no(&self) -> bool {
        *self == SMWS_A::NO
    }
    #[doc = "at least one wake up due to a supply monitor detection has occurred since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_present(&self) -> bool {
        *self == SMWS_A::PRESENT
    }
}
#[doc = "Field `BODRSTS` reader - Brownout Detector Reset Status"]
pub type BODRSTS_R = crate::BitReader<BODRSTS_A>;
#[doc = "Brownout Detector Reset Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BODRSTS_A {
    #[doc = "0: no core brownout rising edge event has been detected since the last read of the SUPC_SR."]
    NO = 0,
    #[doc = "1: at least one brownout output rising edge event has been detected since the last read of the SUPC_SR."]
    PRESENT = 1,
}
impl From<BODRSTS_A> for bool {
    #[inline(always)]
    fn from(variant: BODRSTS_A) -> Self {
        variant as u8 != 0
    }
}
impl BODRSTS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> BODRSTS_A {
        match self.bits {
            false => BODRSTS_A::NO,
            true => BODRSTS_A::PRESENT,
        }
    }
    #[doc = "no core brownout rising edge event has been detected since the last read of the SUPC_SR."]
    #[inline(always)]
    pub fn is_no(&self) -> bool {
        *self == BODRSTS_A::NO
    }
    #[doc = "at least one brownout output rising edge event has been detected since the last read of the SUPC_SR."]
    #[inline(always)]
    pub fn is_present(&self) -> bool {
        *self == BODRSTS_A::PRESENT
    }
}
#[doc = "Field `SMRSTS` reader - Supply Monitor Reset Status"]
pub type SMRSTS_R = crate::BitReader<SMRSTS_A>;
#[doc = "Supply Monitor Reset Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SMRSTS_A {
    #[doc = "0: no supply monitor detection has generated a core reset since the last read of the SUPC_SR."]
    NO = 0,
    #[doc = "1: at least one supply monitor detection has generated a core reset since the last read of the SUPC_SR."]
    PRESENT = 1,
}
impl From<SMRSTS_A> for bool {
    #[inline(always)]
    fn from(variant: SMRSTS_A) -> Self {
        variant as u8 != 0
    }
}
impl SMRSTS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> SMRSTS_A {
        match self.bits {
            false => SMRSTS_A::NO,
            true => SMRSTS_A::PRESENT,
        }
    }
    #[doc = "no supply monitor detection has generated a core reset since the last read of the SUPC_SR."]
    #[inline(always)]
    pub fn is_no(&self) -> bool {
        *self == SMRSTS_A::NO
    }
    #[doc = "at least one supply monitor detection has generated a core reset since the last read of the SUPC_SR."]
    #[inline(always)]
    pub fn is_present(&self) -> bool {
        *self == SMRSTS_A::PRESENT
    }
}
#[doc = "Field `SMS` reader - Supply Monitor Status"]
pub type SMS_R = crate::BitReader<SMS_A>;
#[doc = "Supply Monitor Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SMS_A {
    #[doc = "0: no supply monitor detection since the last read of SUPC_SR."]
    NO = 0,
    #[doc = "1: at least one supply monitor detection since the last read of SUPC_SR."]
    PRESENT = 1,
}
impl From<SMS_A> for bool {
    #[inline(always)]
    fn from(variant: SMS_A) -> Self {
        variant as u8 != 0
    }
}
impl SMS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> SMS_A {
        match self.bits {
            false => SMS_A::NO,
            true => SMS_A::PRESENT,
        }
    }
    #[doc = "no supply monitor detection since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_no(&self) -> bool {
        *self == SMS_A::NO
    }
    #[doc = "at least one supply monitor detection since the last read of SUPC_SR."]
    #[inline(always)]
    pub fn is_present(&self) -> bool {
        *self == SMS_A::PRESENT
    }
}
#[doc = "Field `SMOS` reader - Supply Monitor Output Status"]
pub type SMOS_R = crate::BitReader<SMOS_A>;
#[doc = "Supply Monitor Output Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SMOS_A {
    #[doc = "0: the supply monitor detected VDDUTMI higher than its threshold at its last measurement."]
    HIGH = 0,
    #[doc = "1: the supply monitor detected VDDUTMI lower than its threshold at its last measurement."]
    LOW = 1,
}
impl From<SMOS_A> for bool {
    #[inline(always)]
    fn from(variant: SMOS_A) -> Self {
        variant as u8 != 0
    }
}
impl SMOS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> SMOS_A {
        match self.bits {
            false => SMOS_A::HIGH,
            true => SMOS_A::LOW,
        }
    }
    #[doc = "the supply monitor detected VDDUTMI higher than its threshold at its last measurement."]
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        *self == SMOS_A::HIGH
    }
    #[doc = "the supply monitor detected VDDUTMI lower than its threshold at its last measurement."]
    #[inline(always)]
    pub fn is_low(&self) -> bool {
        *self == SMOS_A::LOW
    }
}
#[doc = "Field `OSCSEL` reader - 32-kHz Oscillator Selection Status"]
pub type OSCSEL_R = crate::BitReader<OSCSEL_A>;
#[doc = "32-kHz Oscillator Selection Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OSCSEL_A {
    #[doc = "0: the slow clock, SLCK is generated by the embedded 32-kHz RC oscillator."]
    RC = 0,
    #[doc = "1: the slow clock, SLCK is generated by the 32-kHz crystal oscillator."]
    CRYST = 1,
}
impl From<OSCSEL_A> for bool {
    #[inline(always)]
    fn from(variant: OSCSEL_A) -> Self {
        variant as u8 != 0
    }
}
impl OSCSEL_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> OSCSEL_A {
        match self.bits {
            false => OSCSEL_A::RC,
            true => OSCSEL_A::CRYST,
        }
    }
    #[doc = "the slow clock, SLCK is generated by the embedded 32-kHz RC oscillator."]
    #[inline(always)]
    pub fn is_rc(&self) -> bool {
        *self == OSCSEL_A::RC
    }
    #[doc = "the slow clock, SLCK is generated by the 32-kHz crystal oscillator."]
    #[inline(always)]
    pub fn is_cryst(&self) -> bool {
        *self == OSCSEL_A::CRYST
    }
}
#[doc = "Field `FWUPIS` reader - FWUP Input Status"]
pub type FWUPIS_R = crate::BitReader<FWUPIS_A>;
#[doc = "FWUP Input Status\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FWUPIS_A {
    #[doc = "0: FWUP input is tied low."]
    LOW = 0,
    #[doc = "1: FWUP input is tied high."]
    HIGH = 1,
}
impl From<FWUPIS_A> for bool {
    #[inline(always)]
    fn from(variant: FWUPIS_A) -> Self {
        variant as u8 != 0
    }
}
impl FWUPIS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> FWUPIS_A {
        match self.bits {
            false => FWUPIS_A::LOW,
            true => FWUPIS_A::HIGH,
        }
    }
    #[doc = "FWUP input is tied low."]
    #[inline(always)]
    pub fn is_low(&self) -> bool {
        *self == FWUPIS_A::LOW
    }
    #[doc = "FWUP input is tied high."]
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        *self == FWUPIS_A::HIGH
    }
}
#[doc = "Field `WKUPIS0` reader - WKUP Input Status 0"]
pub type WKUPIS0_R = crate::BitReader<WKUPIS0_A>;
#[doc = "WKUP Input Status 0\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS0_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS0_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS0_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS0_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS0_A {
        match self.bits {
            false => WKUPIS0_A::DIS,
            true => WKUPIS0_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS0_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS0_A::EN
    }
}
#[doc = "Field `WKUPIS1` reader - WKUP Input Status 1"]
pub type WKUPIS1_R = crate::BitReader<WKUPIS1_A>;
#[doc = "WKUP Input Status 1\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS1_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS1_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS1_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS1_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS1_A {
        match self.bits {
            false => WKUPIS1_A::DIS,
            true => WKUPIS1_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS1_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS1_A::EN
    }
}
#[doc = "Field `WKUPIS2` reader - WKUP Input Status 2"]
pub type WKUPIS2_R = crate::BitReader<WKUPIS2_A>;
#[doc = "WKUP Input Status 2\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS2_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS2_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS2_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS2_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS2_A {
        match self.bits {
            false => WKUPIS2_A::DIS,
            true => WKUPIS2_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS2_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS2_A::EN
    }
}
#[doc = "Field `WKUPIS3` reader - WKUP Input Status 3"]
pub type WKUPIS3_R = crate::BitReader<WKUPIS3_A>;
#[doc = "WKUP Input Status 3\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS3_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS3_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS3_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS3_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS3_A {
        match self.bits {
            false => WKUPIS3_A::DIS,
            true => WKUPIS3_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS3_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS3_A::EN
    }
}
#[doc = "Field `WKUPIS4` reader - WKUP Input Status 4"]
pub type WKUPIS4_R = crate::BitReader<WKUPIS4_A>;
#[doc = "WKUP Input Status 4\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS4_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS4_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS4_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS4_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS4_A {
        match self.bits {
            false => WKUPIS4_A::DIS,
            true => WKUPIS4_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS4_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS4_A::EN
    }
}
#[doc = "Field `WKUPIS5` reader - WKUP Input Status 5"]
pub type WKUPIS5_R = crate::BitReader<WKUPIS5_A>;
#[doc = "WKUP Input Status 5\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS5_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS5_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS5_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS5_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS5_A {
        match self.bits {
            false => WKUPIS5_A::DIS,
            true => WKUPIS5_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS5_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS5_A::EN
    }
}
#[doc = "Field `WKUPIS6` reader - WKUP Input Status 6"]
pub type WKUPIS6_R = crate::BitReader<WKUPIS6_A>;
#[doc = "WKUP Input Status 6\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS6_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS6_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS6_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS6_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS6_A {
        match self.bits {
            false => WKUPIS6_A::DIS,
            true => WKUPIS6_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS6_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS6_A::EN
    }
}
#[doc = "Field `WKUPIS7` reader - WKUP Input Status 7"]
pub type WKUPIS7_R = crate::BitReader<WKUPIS7_A>;
#[doc = "WKUP Input Status 7\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS7_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS7_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS7_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS7_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS7_A {
        match self.bits {
            false => WKUPIS7_A::DIS,
            true => WKUPIS7_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS7_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS7_A::EN
    }
}
#[doc = "Field `WKUPIS8` reader - WKUP Input Status 8"]
pub type WKUPIS8_R = crate::BitReader<WKUPIS8_A>;
#[doc = "WKUP Input Status 8\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS8_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS8_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS8_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS8_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS8_A {
        match self.bits {
            false => WKUPIS8_A::DIS,
            true => WKUPIS8_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS8_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS8_A::EN
    }
}
#[doc = "Field `WKUPIS9` reader - WKUP Input Status 9"]
pub type WKUPIS9_R = crate::BitReader<WKUPIS9_A>;
#[doc = "WKUP Input Status 9\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS9_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS9_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS9_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS9_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS9_A {
        match self.bits {
            false => WKUPIS9_A::DIS,
            true => WKUPIS9_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS9_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS9_A::EN
    }
}
#[doc = "Field `WKUPIS10` reader - WKUP Input Status 10"]
pub type WKUPIS10_R = crate::BitReader<WKUPIS10_A>;
#[doc = "WKUP Input Status 10\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS10_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS10_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS10_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS10_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS10_A {
        match self.bits {
            false => WKUPIS10_A::DIS,
            true => WKUPIS10_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS10_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS10_A::EN
    }
}
#[doc = "Field `WKUPIS11` reader - WKUP Input Status 11"]
pub type WKUPIS11_R = crate::BitReader<WKUPIS11_A>;
#[doc = "WKUP Input Status 11\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS11_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS11_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS11_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS11_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS11_A {
        match self.bits {
            false => WKUPIS11_A::DIS,
            true => WKUPIS11_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS11_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS11_A::EN
    }
}
#[doc = "Field `WKUPIS12` reader - WKUP Input Status 12"]
pub type WKUPIS12_R = crate::BitReader<WKUPIS12_A>;
#[doc = "WKUP Input Status 12\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS12_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS12_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS12_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS12_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS12_A {
        match self.bits {
            false => WKUPIS12_A::DIS,
            true => WKUPIS12_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS12_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS12_A::EN
    }
}
#[doc = "Field `WKUPIS13` reader - WKUP Input Status 13"]
pub type WKUPIS13_R = crate::BitReader<WKUPIS13_A>;
#[doc = "WKUP Input Status 13\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS13_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS13_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS13_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS13_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS13_A {
        match self.bits {
            false => WKUPIS13_A::DIS,
            true => WKUPIS13_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS13_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS13_A::EN
    }
}
#[doc = "Field `WKUPIS14` reader - WKUP Input Status 14"]
pub type WKUPIS14_R = crate::BitReader<WKUPIS14_A>;
#[doc = "WKUP Input Status 14\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS14_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS14_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS14_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS14_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS14_A {
        match self.bits {
            false => WKUPIS14_A::DIS,
            true => WKUPIS14_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS14_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS14_A::EN
    }
}
#[doc = "Field `WKUPIS15` reader - WKUP Input Status 15"]
pub type WKUPIS15_R = crate::BitReader<WKUPIS15_A>;
#[doc = "WKUP Input Status 15\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WKUPIS15_A {
    #[doc = "0: the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    DIS = 0,
    #[doc = "1: the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    EN = 1,
}
impl From<WKUPIS15_A> for bool {
    #[inline(always)]
    fn from(variant: WKUPIS15_A) -> Self {
        variant as u8 != 0
    }
}
impl WKUPIS15_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> WKUPIS15_A {
        match self.bits {
            false => WKUPIS15_A::DIS,
            true => WKUPIS15_A::EN,
        }
    }
    #[doc = "the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_dis(&self) -> bool {
        *self == WKUPIS15_A::DIS
    }
    #[doc = "the corresponding wake-up input was active at the time the debouncer triggered a wake up event."]
    #[inline(always)]
    pub fn is_en(&self) -> bool {
        *self == WKUPIS15_A::EN
    }
}
impl R {
    #[doc = "Bit 0 - FWUP Wake Up Status"]
    #[inline(always)]
    pub fn fwups(&self) -> FWUPS_R {
        FWUPS_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - WKUP Wake Up Status"]
    #[inline(always)]
    pub fn wkups(&self) -> WKUPS_R {
        WKUPS_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Supply Monitor Detection Wake Up Status"]
    #[inline(always)]
    pub fn smws(&self) -> SMWS_R {
        SMWS_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Brownout Detector Reset Status"]
    #[inline(always)]
    pub fn bodrsts(&self) -> BODRSTS_R {
        BODRSTS_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Supply Monitor Reset Status"]
    #[inline(always)]
    pub fn smrsts(&self) -> SMRSTS_R {
        SMRSTS_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Supply Monitor Status"]
    #[inline(always)]
    pub fn sms(&self) -> SMS_R {
        SMS_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Supply Monitor Output Status"]
    #[inline(always)]
    pub fn smos(&self) -> SMOS_R {
        SMOS_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - 32-kHz Oscillator Selection Status"]
    #[inline(always)]
    pub fn oscsel(&self) -> OSCSEL_R {
        OSCSEL_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 12 - FWUP Input Status"]
    #[inline(always)]
    pub fn fwupis(&self) -> FWUPIS_R {
        FWUPIS_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 16 - WKUP Input Status 0"]
    #[inline(always)]
    pub fn wkupis0(&self) -> WKUPIS0_R {
        WKUPIS0_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - WKUP Input Status 1"]
    #[inline(always)]
    pub fn wkupis1(&self) -> WKUPIS1_R {
        WKUPIS1_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - WKUP Input Status 2"]
    #[inline(always)]
    pub fn wkupis2(&self) -> WKUPIS2_R {
        WKUPIS2_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 19 - WKUP Input Status 3"]
    #[inline(always)]
    pub fn wkupis3(&self) -> WKUPIS3_R {
        WKUPIS3_R::new(((self.bits >> 19) & 1) != 0)
    }
    #[doc = "Bit 20 - WKUP Input Status 4"]
    #[inline(always)]
    pub fn wkupis4(&self) -> WKUPIS4_R {
        WKUPIS4_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bit 21 - WKUP Input Status 5"]
    #[inline(always)]
    pub fn wkupis5(&self) -> WKUPIS5_R {
        WKUPIS5_R::new(((self.bits >> 21) & 1) != 0)
    }
    #[doc = "Bit 22 - WKUP Input Status 6"]
    #[inline(always)]
    pub fn wkupis6(&self) -> WKUPIS6_R {
        WKUPIS6_R::new(((self.bits >> 22) & 1) != 0)
    }
    #[doc = "Bit 23 - WKUP Input Status 7"]
    #[inline(always)]
    pub fn wkupis7(&self) -> WKUPIS7_R {
        WKUPIS7_R::new(((self.bits >> 23) & 1) != 0)
    }
    #[doc = "Bit 24 - WKUP Input Status 8"]
    #[inline(always)]
    pub fn wkupis8(&self) -> WKUPIS8_R {
        WKUPIS8_R::new(((self.bits >> 24) & 1) != 0)
    }
    #[doc = "Bit 25 - WKUP Input Status 9"]
    #[inline(always)]
    pub fn wkupis9(&self) -> WKUPIS9_R {
        WKUPIS9_R::new(((self.bits >> 25) & 1) != 0)
    }
    #[doc = "Bit 26 - WKUP Input Status 10"]
    #[inline(always)]
    pub fn wkupis10(&self) -> WKUPIS10_R {
        WKUPIS10_R::new(((self.bits >> 26) & 1) != 0)
    }
    #[doc = "Bit 27 - WKUP Input Status 11"]
    #[inline(always)]
    pub fn wkupis11(&self) -> WKUPIS11_R {
        WKUPIS11_R::new(((self.bits >> 27) & 1) != 0)
    }
    #[doc = "Bit 28 - WKUP Input Status 12"]
    #[inline(always)]
    pub fn wkupis12(&self) -> WKUPIS12_R {
        WKUPIS12_R::new(((self.bits >> 28) & 1) != 0)
    }
    #[doc = "Bit 29 - WKUP Input Status 13"]
    #[inline(always)]
    pub fn wkupis13(&self) -> WKUPIS13_R {
        WKUPIS13_R::new(((self.bits >> 29) & 1) != 0)
    }
    #[doc = "Bit 30 - WKUP Input Status 14"]
    #[inline(always)]
    pub fn wkupis14(&self) -> WKUPIS14_R {
        WKUPIS14_R::new(((self.bits >> 30) & 1) != 0)
    }
    #[doc = "Bit 31 - WKUP Input Status 15"]
    #[inline(always)]
    pub fn wkupis15(&self) -> WKUPIS15_R {
        WKUPIS15_R::new(((self.bits >> 31) & 1) != 0)
    }
}
#[doc = "Supply Controller Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct SR_SPEC;
impl crate::RegisterSpec for SR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`sr::R`](R) reader structure"]
impl crate::Readable for SR_SPEC {}
#[doc = "`reset()` method sets SR to value 0x0800"]
impl crate::Resettable for SR_SPEC {
    const RESET_VALUE: Self::Ux = 0x0800;
}
