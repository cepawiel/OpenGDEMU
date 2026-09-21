#[doc = "Register `IMR` reader"]
pub type R = crate::R<IMR_SPEC>;
#[doc = "Field `RB_RISE` reader - Ready Busy Rising Edge Detection Interrupt Mask"]
pub type RB_RISE_R = crate::BitReader;
#[doc = "Field `RB_FALL` reader - Ready Busy Falling Edge Detection Interrupt Mask"]
pub type RB_FALL_R = crate::BitReader;
#[doc = "Field `XFRDONE` reader - Transfer Done Interrupt Mask"]
pub type XFRDONE_R = crate::BitReader;
#[doc = "Field `CMDDONE` reader - Command Done Interrupt Mask"]
pub type CMDDONE_R = crate::BitReader;
#[doc = "Field `DTOE` reader - Data Timeout Error Interrupt Mask"]
pub type DTOE_R = crate::BitReader;
#[doc = "Field `UNDEF` reader - Undefined Area Access Interrupt Mask5"]
pub type UNDEF_R = crate::BitReader;
#[doc = "Field `AWB` reader - Accessing While Busy Interrupt Mask"]
pub type AWB_R = crate::BitReader;
#[doc = "Field `NFCASE` reader - NFC Access Size Error Interrupt Mask"]
pub type NFCASE_R = crate::BitReader;
#[doc = "Field `RB_EDGE0` reader - Ready/Busy Line 0 Interrupt Mask"]
pub type RB_EDGE0_R = crate::BitReader;
impl R {
    #[doc = "Bit 4 - Ready Busy Rising Edge Detection Interrupt Mask"]
    #[inline(always)]
    pub fn rb_rise(&self) -> RB_RISE_R {
        RB_RISE_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Ready Busy Falling Edge Detection Interrupt Mask"]
    #[inline(always)]
    pub fn rb_fall(&self) -> RB_FALL_R {
        RB_FALL_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 16 - Transfer Done Interrupt Mask"]
    #[inline(always)]
    pub fn xfrdone(&self) -> XFRDONE_R {
        XFRDONE_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Command Done Interrupt Mask"]
    #[inline(always)]
    pub fn cmddone(&self) -> CMDDONE_R {
        CMDDONE_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 20 - Data Timeout Error Interrupt Mask"]
    #[inline(always)]
    pub fn dtoe(&self) -> DTOE_R {
        DTOE_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bit 21 - Undefined Area Access Interrupt Mask5"]
    #[inline(always)]
    pub fn undef(&self) -> UNDEF_R {
        UNDEF_R::new(((self.bits >> 21) & 1) != 0)
    }
    #[doc = "Bit 22 - Accessing While Busy Interrupt Mask"]
    #[inline(always)]
    pub fn awb(&self) -> AWB_R {
        AWB_R::new(((self.bits >> 22) & 1) != 0)
    }
    #[doc = "Bit 23 - NFC Access Size Error Interrupt Mask"]
    #[inline(always)]
    pub fn nfcase(&self) -> NFCASE_R {
        NFCASE_R::new(((self.bits >> 23) & 1) != 0)
    }
    #[doc = "Bit 24 - Ready/Busy Line 0 Interrupt Mask"]
    #[inline(always)]
    pub fn rb_edge0(&self) -> RB_EDGE0_R {
        RB_EDGE0_R::new(((self.bits >> 24) & 1) != 0)
    }
}
#[doc = "SMC NFC Interrupt Mask Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`imr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IMR_SPEC;
impl crate::RegisterSpec for IMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`imr::R`](R) reader structure"]
impl crate::Readable for IMR_SPEC {}
#[doc = "`reset()` method sets IMR to value 0"]
impl crate::Resettable for IMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
