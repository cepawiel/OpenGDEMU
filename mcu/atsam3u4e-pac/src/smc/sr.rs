#[doc = "Register `SR` reader"]
pub type R = crate::R<SR_SPEC>;
#[doc = "Field `SMCSTS` reader - NAND Flash Controller status (this field cannot be reset)"]
pub type SMCSTS_R = crate::BitReader;
#[doc = "Field `RB_RISE` reader - Selected Ready Busy Rising Edge Detected"]
pub type RB_RISE_R = crate::BitReader;
#[doc = "Field `RB_FALL` reader - Selected Ready Busy Falling Edge Detected"]
pub type RB_FALL_R = crate::BitReader;
#[doc = "Field `NFCBUSY` reader - NFC Busy (this field cannot be reset)"]
pub type NFCBUSY_R = crate::BitReader;
#[doc = "Field `NFCWR` reader - NFC Write/Read Operation (this field cannot be reset)"]
pub type NFCWR_R = crate::BitReader;
#[doc = "Field `NFCSID` reader - NFC Chip Select ID (this field cannot be reset)"]
pub type NFCSID_R = crate::FieldReader;
#[doc = "Field `XFRDONE` reader - NFC Data Transfer Terminated"]
pub type XFRDONE_R = crate::BitReader;
#[doc = "Field `CMDDONE` reader - Command Done"]
pub type CMDDONE_R = crate::BitReader;
#[doc = "Field `DTOE` reader - Data Timeout Error"]
pub type DTOE_R = crate::BitReader;
#[doc = "Field `UNDEF` reader - Undefined Area Error"]
pub type UNDEF_R = crate::BitReader;
#[doc = "Field `AWB` reader - Accessing While Busy"]
pub type AWB_R = crate::BitReader;
#[doc = "Field `NFCASE` reader - NFC Access Size Error"]
pub type NFCASE_R = crate::BitReader;
#[doc = "Field `RB_EDGE0` reader - Ready/Busy Line 0 Edge Detected"]
pub type RB_EDGE0_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - NAND Flash Controller status (this field cannot be reset)"]
    #[inline(always)]
    pub fn smcsts(&self) -> SMCSTS_R {
        SMCSTS_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 4 - Selected Ready Busy Rising Edge Detected"]
    #[inline(always)]
    pub fn rb_rise(&self) -> RB_RISE_R {
        RB_RISE_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Selected Ready Busy Falling Edge Detected"]
    #[inline(always)]
    pub fn rb_fall(&self) -> RB_FALL_R {
        RB_FALL_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 8 - NFC Busy (this field cannot be reset)"]
    #[inline(always)]
    pub fn nfcbusy(&self) -> NFCBUSY_R {
        NFCBUSY_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 11 - NFC Write/Read Operation (this field cannot be reset)"]
    #[inline(always)]
    pub fn nfcwr(&self) -> NFCWR_R {
        NFCWR_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bits 12:14 - NFC Chip Select ID (this field cannot be reset)"]
    #[inline(always)]
    pub fn nfcsid(&self) -> NFCSID_R {
        NFCSID_R::new(((self.bits >> 12) & 7) as u8)
    }
    #[doc = "Bit 16 - NFC Data Transfer Terminated"]
    #[inline(always)]
    pub fn xfrdone(&self) -> XFRDONE_R {
        XFRDONE_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Command Done"]
    #[inline(always)]
    pub fn cmddone(&self) -> CMDDONE_R {
        CMDDONE_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 20 - Data Timeout Error"]
    #[inline(always)]
    pub fn dtoe(&self) -> DTOE_R {
        DTOE_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bit 21 - Undefined Area Error"]
    #[inline(always)]
    pub fn undef(&self) -> UNDEF_R {
        UNDEF_R::new(((self.bits >> 21) & 1) != 0)
    }
    #[doc = "Bit 22 - Accessing While Busy"]
    #[inline(always)]
    pub fn awb(&self) -> AWB_R {
        AWB_R::new(((self.bits >> 22) & 1) != 0)
    }
    #[doc = "Bit 23 - NFC Access Size Error"]
    #[inline(always)]
    pub fn nfcase(&self) -> NFCASE_R {
        NFCASE_R::new(((self.bits >> 23) & 1) != 0)
    }
    #[doc = "Bit 24 - Ready/Busy Line 0 Edge Detected"]
    #[inline(always)]
    pub fn rb_edge0(&self) -> RB_EDGE0_R {
        RB_EDGE0_R::new(((self.bits >> 24) & 1) != 0)
    }
}
#[doc = "SMC NFC Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct SR_SPEC;
impl crate::RegisterSpec for SR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`sr::R`](R) reader structure"]
impl crate::Readable for SR_SPEC {}
#[doc = "`reset()` method sets SR to value 0"]
impl crate::Resettable for SR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
