#[doc = "Register `ECC_SR1` reader"]
pub type R = crate::R<ECC_SR1_SPEC>;
#[doc = "Field `RECERR0` reader - Recoverable Error"]
pub type RECERR0_R = crate::BitReader;
#[doc = "Field `ECCERR0` reader - ECC Error"]
pub type ECCERR0_R = crate::FieldReader;
#[doc = "Field `RECERR1` reader - Recoverable Error in the page between the 256th and the 511th bytes or the 512nd and the 1023rd bytes"]
pub type RECERR1_R = crate::BitReader;
#[doc = "Field `ECCERR1` reader - ECC Error in the page between the 256th and the 511th bytes or between the 512nd and the 1023rd bytes"]
pub type ECCERR1_R = crate::BitReader;
#[doc = "Field `MULERR1` reader - Multiple Error in the page between the 256th and the 511th bytes or between the 512nd and the 1023rd bytes"]
pub type MULERR1_R = crate::BitReader;
#[doc = "Field `RECERR2` reader - Recoverable Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes"]
pub type RECERR2_R = crate::BitReader;
#[doc = "Field `ECCERR2` reader - ECC Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes"]
pub type ECCERR2_R = crate::BitReader;
#[doc = "Field `MULERR2` reader - Multiple Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes"]
pub type MULERR2_R = crate::BitReader;
#[doc = "Field `RECERR3` reader - Recoverable Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes"]
pub type RECERR3_R = crate::BitReader;
#[doc = "Field `ECCERR3` reader - ECC Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes"]
pub type ECCERR3_R = crate::BitReader;
#[doc = "Field `MULERR3` reader - Multiple Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes"]
pub type MULERR3_R = crate::BitReader;
#[doc = "Field `RECERR4` reader - Recoverable Error in the page between the 1024th and the 1279th bytes or between the 2048th and the 2559th bytes"]
pub type RECERR4_R = crate::BitReader;
#[doc = "Field `ECCERR4` reader - ECC Error in the page between the 1024th and the 1279th bytes or between the 2048th and the 2559th bytes"]
pub type ECCERR4_R = crate::FieldReader;
#[doc = "Field `RECERR5` reader - Recoverable Error in the page between the 1280th and the 1535th bytes or between the 2560th and the 3071st bytes"]
pub type RECERR5_R = crate::BitReader;
#[doc = "Field `ECCERR5` reader - ECC Error in the page between the 1280th and the 1535th bytes or between the 2560th and the 3071st bytes"]
pub type ECCERR5_R = crate::FieldReader;
#[doc = "Field `RECERR6` reader - Recoverable Error in the page between the 1536th and the 1791st bytes or between the 3072nd and the 3583rd bytes"]
pub type RECERR6_R = crate::BitReader;
#[doc = "Field `ECCERR6` reader - ECC Error in the page between the 1536th and the 1791st bytes or between the 3072nd and the 3583rd bytes"]
pub type ECCERR6_R = crate::FieldReader;
#[doc = "Field `RECERR7` reader - Recoverable Error in the page between the 1792nd and the 2047th bytes or between the 3584th and the 4095th bytes"]
pub type RECERR7_R = crate::BitReader;
#[doc = "Field `ECCERR7` reader - ECC Error in the page between the 1792nd and the 2047th bytes or between the 3584th and the 4095th bytes"]
pub type ECCERR7_R = crate::FieldReader;
impl R {
    #[doc = "Bit 0 - Recoverable Error"]
    #[inline(always)]
    pub fn recerr0(&self) -> RECERR0_R {
        RECERR0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 1:2 - ECC Error"]
    #[inline(always)]
    pub fn eccerr0(&self) -> ECCERR0_R {
        ECCERR0_R::new(((self.bits >> 1) & 3) as u8)
    }
    #[doc = "Bit 4 - Recoverable Error in the page between the 256th and the 511th bytes or the 512nd and the 1023rd bytes"]
    #[inline(always)]
    pub fn recerr1(&self) -> RECERR1_R {
        RECERR1_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - ECC Error in the page between the 256th and the 511th bytes or between the 512nd and the 1023rd bytes"]
    #[inline(always)]
    pub fn eccerr1(&self) -> ECCERR1_R {
        ECCERR1_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Multiple Error in the page between the 256th and the 511th bytes or between the 512nd and the 1023rd bytes"]
    #[inline(always)]
    pub fn mulerr1(&self) -> MULERR1_R {
        MULERR1_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 8 - Recoverable Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes"]
    #[inline(always)]
    pub fn recerr2(&self) -> RECERR2_R {
        RECERR2_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - ECC Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes"]
    #[inline(always)]
    pub fn eccerr2(&self) -> ECCERR2_R {
        ECCERR2_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Multiple Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes"]
    #[inline(always)]
    pub fn mulerr2(&self) -> MULERR2_R {
        MULERR2_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 12 - Recoverable Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes"]
    #[inline(always)]
    pub fn recerr3(&self) -> RECERR3_R {
        RECERR3_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - ECC Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes"]
    #[inline(always)]
    pub fn eccerr3(&self) -> ECCERR3_R {
        ECCERR3_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - Multiple Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes"]
    #[inline(always)]
    pub fn mulerr3(&self) -> MULERR3_R {
        MULERR3_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 16 - Recoverable Error in the page between the 1024th and the 1279th bytes or between the 2048th and the 2559th bytes"]
    #[inline(always)]
    pub fn recerr4(&self) -> RECERR4_R {
        RECERR4_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bits 17:18 - ECC Error in the page between the 1024th and the 1279th bytes or between the 2048th and the 2559th bytes"]
    #[inline(always)]
    pub fn eccerr4(&self) -> ECCERR4_R {
        ECCERR4_R::new(((self.bits >> 17) & 3) as u8)
    }
    #[doc = "Bit 20 - Recoverable Error in the page between the 1280th and the 1535th bytes or between the 2560th and the 3071st bytes"]
    #[inline(always)]
    pub fn recerr5(&self) -> RECERR5_R {
        RECERR5_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bits 21:22 - ECC Error in the page between the 1280th and the 1535th bytes or between the 2560th and the 3071st bytes"]
    #[inline(always)]
    pub fn eccerr5(&self) -> ECCERR5_R {
        ECCERR5_R::new(((self.bits >> 21) & 3) as u8)
    }
    #[doc = "Bit 24 - Recoverable Error in the page between the 1536th and the 1791st bytes or between the 3072nd and the 3583rd bytes"]
    #[inline(always)]
    pub fn recerr6(&self) -> RECERR6_R {
        RECERR6_R::new(((self.bits >> 24) & 1) != 0)
    }
    #[doc = "Bits 25:26 - ECC Error in the page between the 1536th and the 1791st bytes or between the 3072nd and the 3583rd bytes"]
    #[inline(always)]
    pub fn eccerr6(&self) -> ECCERR6_R {
        ECCERR6_R::new(((self.bits >> 25) & 3) as u8)
    }
    #[doc = "Bit 28 - Recoverable Error in the page between the 1792nd and the 2047th bytes or between the 3584th and the 4095th bytes"]
    #[inline(always)]
    pub fn recerr7(&self) -> RECERR7_R {
        RECERR7_R::new(((self.bits >> 28) & 1) != 0)
    }
    #[doc = "Bits 29:30 - ECC Error in the page between the 1792nd and the 2047th bytes or between the 3584th and the 4095th bytes"]
    #[inline(always)]
    pub fn eccerr7(&self) -> ECCERR7_R {
        ECCERR7_R::new(((self.bits >> 29) & 3) as u8)
    }
}
#[doc = "SMC ECC Status 1 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ecc_sr1::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct ECC_SR1_SPEC;
impl crate::RegisterSpec for ECC_SR1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ecc_sr1::R`](R) reader structure"]
impl crate::Readable for ECC_SR1_SPEC {}
#[doc = "`reset()` method sets ECC_SR1 to value 0"]
impl crate::Resettable for ECC_SR1_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
