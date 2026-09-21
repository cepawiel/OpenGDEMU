#[doc = "Register `ECC_SR2` reader"]
pub type R = crate::R<ECC_SR2_SPEC>;
#[doc = "Field `RECERR8` reader - Recoverable Error in the page between the 2048th and the 2303rd bytes"]
pub type RECERR8_R = crate::BitReader;
#[doc = "Field `ECCERR8` reader - ECC Error in the page between the 2048th and the 2303rd bytes"]
pub type ECCERR8_R = crate::FieldReader;
#[doc = "Field `RECERR9` reader - Recoverable Error in the page between the 2304th and the 2559th bytes"]
pub type RECERR9_R = crate::BitReader;
#[doc = "Field `ECCERR9` reader - ECC Error in the page between the 2304th and the 2559th bytes"]
pub type ECCERR9_R = crate::BitReader;
#[doc = "Field `MULERR9` reader - Multiple Error in the page between the 2304th and the 2559th bytes"]
pub type MULERR9_R = crate::BitReader;
#[doc = "Field `RECERR10` reader - Recoverable Error in the page between the 2560th and the 2815th bytes"]
pub type RECERR10_R = crate::BitReader;
#[doc = "Field `ECCERR10` reader - ECC Error in the page between the 2560th and the 2815th bytes"]
pub type ECCERR10_R = crate::BitReader;
#[doc = "Field `MULERR10` reader - Multiple Error in the page between the 2560th and the 2815th bytes"]
pub type MULERR10_R = crate::BitReader;
#[doc = "Field `RECERR11` reader - Recoverable Error in the page between the 2816th and the 3071st bytes"]
pub type RECERR11_R = crate::BitReader;
#[doc = "Field `ECCERR11` reader - ECC Error in the page between the 2816th and the 3071st bytes"]
pub type ECCERR11_R = crate::BitReader;
#[doc = "Field `MULERR11` reader - Multiple Error in the page between the 2816th and the 3071st bytes"]
pub type MULERR11_R = crate::BitReader;
#[doc = "Field `RECERR12` reader - Recoverable Error in the page between the 3072nd and the 3327th bytes"]
pub type RECERR12_R = crate::BitReader;
#[doc = "Field `ECCERR12` reader - ECC Error in the page between the 3072nd and the 3327th bytes"]
pub type ECCERR12_R = crate::FieldReader;
#[doc = "Field `RECERR13` reader - Recoverable Error in the page between the 3328th and the 3583rd bytes"]
pub type RECERR13_R = crate::BitReader;
#[doc = "Field `ECCERR13` reader - ECC Error in the page between the 3328th and the 3583rd bytes"]
pub type ECCERR13_R = crate::FieldReader;
#[doc = "Field `RECERR14` reader - Recoverable Error in the page between the 3584th and the 3839th bytes"]
pub type RECERR14_R = crate::BitReader;
#[doc = "Field `ECCERR14` reader - ECC Error in the page between the 3584th and the 3839th bytes"]
pub type ECCERR14_R = crate::FieldReader;
#[doc = "Field `RECERR15` reader - Recoverable Error in the page between the 3840th and the 4095th bytes"]
pub type RECERR15_R = crate::BitReader;
#[doc = "Field `ECCERR15` reader - ECC Error in the page between the 3840th and the 4095th bytes"]
pub type ECCERR15_R = crate::FieldReader;
impl R {
    #[doc = "Bit 0 - Recoverable Error in the page between the 2048th and the 2303rd bytes"]
    #[inline(always)]
    pub fn recerr8(&self) -> RECERR8_R {
        RECERR8_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 1:2 - ECC Error in the page between the 2048th and the 2303rd bytes"]
    #[inline(always)]
    pub fn eccerr8(&self) -> ECCERR8_R {
        ECCERR8_R::new(((self.bits >> 1) & 3) as u8)
    }
    #[doc = "Bit 4 - Recoverable Error in the page between the 2304th and the 2559th bytes"]
    #[inline(always)]
    pub fn recerr9(&self) -> RECERR9_R {
        RECERR9_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - ECC Error in the page between the 2304th and the 2559th bytes"]
    #[inline(always)]
    pub fn eccerr9(&self) -> ECCERR9_R {
        ECCERR9_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Multiple Error in the page between the 2304th and the 2559th bytes"]
    #[inline(always)]
    pub fn mulerr9(&self) -> MULERR9_R {
        MULERR9_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 8 - Recoverable Error in the page between the 2560th and the 2815th bytes"]
    #[inline(always)]
    pub fn recerr10(&self) -> RECERR10_R {
        RECERR10_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - ECC Error in the page between the 2560th and the 2815th bytes"]
    #[inline(always)]
    pub fn eccerr10(&self) -> ECCERR10_R {
        ECCERR10_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Multiple Error in the page between the 2560th and the 2815th bytes"]
    #[inline(always)]
    pub fn mulerr10(&self) -> MULERR10_R {
        MULERR10_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 12 - Recoverable Error in the page between the 2816th and the 3071st bytes"]
    #[inline(always)]
    pub fn recerr11(&self) -> RECERR11_R {
        RECERR11_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - ECC Error in the page between the 2816th and the 3071st bytes"]
    #[inline(always)]
    pub fn eccerr11(&self) -> ECCERR11_R {
        ECCERR11_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - Multiple Error in the page between the 2816th and the 3071st bytes"]
    #[inline(always)]
    pub fn mulerr11(&self) -> MULERR11_R {
        MULERR11_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 16 - Recoverable Error in the page between the 3072nd and the 3327th bytes"]
    #[inline(always)]
    pub fn recerr12(&self) -> RECERR12_R {
        RECERR12_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bits 17:18 - ECC Error in the page between the 3072nd and the 3327th bytes"]
    #[inline(always)]
    pub fn eccerr12(&self) -> ECCERR12_R {
        ECCERR12_R::new(((self.bits >> 17) & 3) as u8)
    }
    #[doc = "Bit 20 - Recoverable Error in the page between the 3328th and the 3583rd bytes"]
    #[inline(always)]
    pub fn recerr13(&self) -> RECERR13_R {
        RECERR13_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bits 21:22 - ECC Error in the page between the 3328th and the 3583rd bytes"]
    #[inline(always)]
    pub fn eccerr13(&self) -> ECCERR13_R {
        ECCERR13_R::new(((self.bits >> 21) & 3) as u8)
    }
    #[doc = "Bit 24 - Recoverable Error in the page between the 3584th and the 3839th bytes"]
    #[inline(always)]
    pub fn recerr14(&self) -> RECERR14_R {
        RECERR14_R::new(((self.bits >> 24) & 1) != 0)
    }
    #[doc = "Bits 25:26 - ECC Error in the page between the 3584th and the 3839th bytes"]
    #[inline(always)]
    pub fn eccerr14(&self) -> ECCERR14_R {
        ECCERR14_R::new(((self.bits >> 25) & 3) as u8)
    }
    #[doc = "Bit 28 - Recoverable Error in the page between the 3840th and the 4095th bytes"]
    #[inline(always)]
    pub fn recerr15(&self) -> RECERR15_R {
        RECERR15_R::new(((self.bits >> 28) & 1) != 0)
    }
    #[doc = "Bits 29:30 - ECC Error in the page between the 3840th and the 4095th bytes"]
    #[inline(always)]
    pub fn eccerr15(&self) -> ECCERR15_R {
        ECCERR15_R::new(((self.bits >> 29) & 3) as u8)
    }
}
#[doc = "SMC ECC status 2 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ecc_sr2::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct ECC_SR2_SPEC;
impl crate::RegisterSpec for ECC_SR2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ecc_sr2::R`](R) reader structure"]
impl crate::Readable for ECC_SR2_SPEC {}
#[doc = "`reset()` method sets ECC_SR2 to value 0"]
impl crate::Resettable for ECC_SR2_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
