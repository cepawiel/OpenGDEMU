#[doc = "Register `EBCIMR` reader"]
pub type R = crate::R<EBCIMR_SPEC>;
#[doc = "Field `BTC0` reader - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC0_R = crate::BitReader;
#[doc = "Field `BTC1` reader - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC1_R = crate::BitReader;
#[doc = "Field `BTC2` reader - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC2_R = crate::BitReader;
#[doc = "Field `BTC3` reader - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC3_R = crate::BitReader;
#[doc = "Field `CBTC0` reader - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC0_R = crate::BitReader;
#[doc = "Field `CBTC1` reader - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC1_R = crate::BitReader;
#[doc = "Field `CBTC2` reader - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC2_R = crate::BitReader;
#[doc = "Field `CBTC3` reader - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC3_R = crate::BitReader;
#[doc = "Field `ERR0` reader - Access Error \\[3:0\\]"]
pub type ERR0_R = crate::BitReader;
#[doc = "Field `ERR1` reader - Access Error \\[3:0\\]"]
pub type ERR1_R = crate::BitReader;
#[doc = "Field `ERR2` reader - Access Error \\[3:0\\]"]
pub type ERR2_R = crate::BitReader;
#[doc = "Field `ERR3` reader - Access Error \\[3:0\\]"]
pub type ERR3_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn btc0(&self) -> BTC0_R {
        BTC0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn btc1(&self) -> BTC1_R {
        BTC1_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn btc2(&self) -> BTC2_R {
        BTC2_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn btc3(&self) -> BTC3_R {
        BTC3_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 8 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn cbtc0(&self) -> CBTC0_R {
        CBTC0_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn cbtc1(&self) -> CBTC1_R {
        CBTC1_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn cbtc2(&self) -> CBTC2_R {
        CBTC2_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    pub fn cbtc3(&self) -> CBTC3_R {
        CBTC3_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 16 - Access Error \\[3:0\\]"]
    #[inline(always)]
    pub fn err0(&self) -> ERR0_R {
        ERR0_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Access Error \\[3:0\\]"]
    #[inline(always)]
    pub fn err1(&self) -> ERR1_R {
        ERR1_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - Access Error \\[3:0\\]"]
    #[inline(always)]
    pub fn err2(&self) -> ERR2_R {
        ERR2_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 19 - Access Error \\[3:0\\]"]
    #[inline(always)]
    pub fn err3(&self) -> ERR3_R {
        ERR3_R::new(((self.bits >> 19) & 1) != 0)
    }
}
#[doc = "DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register.\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ebcimr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EBCIMR_SPEC;
impl crate::RegisterSpec for EBCIMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ebcimr::R`](R) reader structure"]
impl crate::Readable for EBCIMR_SPEC {}
#[doc = "`reset()` method sets EBCIMR to value 0"]
impl crate::Resettable for EBCIMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
