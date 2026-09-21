#[doc = "Register `EBCIER` writer"]
pub type W = crate::W<EBCIER_SPEC>;
#[doc = "Field `BTC0` writer - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `BTC1` writer - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `BTC2` writer - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `BTC3` writer - Buffer Transfer Completed \\[3:0\\]"]
pub type BTC3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CBTC0` writer - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CBTC1` writer - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CBTC2` writer - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CBTC3` writer - Chained Buffer Transfer Completed \\[3:0\\]"]
pub type CBTC3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR0` writer - Access Error \\[3:0\\]"]
pub type ERR0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR1` writer - Access Error \\[3:0\\]"]
pub type ERR1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR2` writer - Access Error \\[3:0\\]"]
pub type ERR2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR3` writer - Access Error \\[3:0\\]"]
pub type ERR3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 0 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn btc0(&mut self) -> BTC0_W<EBCIER_SPEC, 0> {
        BTC0_W::new(self)
    }
    #[doc = "Bit 1 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn btc1(&mut self) -> BTC1_W<EBCIER_SPEC, 1> {
        BTC1_W::new(self)
    }
    #[doc = "Bit 2 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn btc2(&mut self) -> BTC2_W<EBCIER_SPEC, 2> {
        BTC2_W::new(self)
    }
    #[doc = "Bit 3 - Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn btc3(&mut self) -> BTC3_W<EBCIER_SPEC, 3> {
        BTC3_W::new(self)
    }
    #[doc = "Bit 8 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn cbtc0(&mut self) -> CBTC0_W<EBCIER_SPEC, 8> {
        CBTC0_W::new(self)
    }
    #[doc = "Bit 9 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn cbtc1(&mut self) -> CBTC1_W<EBCIER_SPEC, 9> {
        CBTC1_W::new(self)
    }
    #[doc = "Bit 10 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn cbtc2(&mut self) -> CBTC2_W<EBCIER_SPEC, 10> {
        CBTC2_W::new(self)
    }
    #[doc = "Bit 11 - Chained Buffer Transfer Completed \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn cbtc3(&mut self) -> CBTC3_W<EBCIER_SPEC, 11> {
        CBTC3_W::new(self)
    }
    #[doc = "Bit 16 - Access Error \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn err0(&mut self) -> ERR0_W<EBCIER_SPEC, 16> {
        ERR0_W::new(self)
    }
    #[doc = "Bit 17 - Access Error \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn err1(&mut self) -> ERR1_W<EBCIER_SPEC, 17> {
        ERR1_W::new(self)
    }
    #[doc = "Bit 18 - Access Error \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn err2(&mut self) -> ERR2_W<EBCIER_SPEC, 18> {
        ERR2_W::new(self)
    }
    #[doc = "Bit 19 - Access Error \\[3:0\\]"]
    #[inline(always)]
    #[must_use]
    pub fn err3(&mut self) -> ERR3_W<EBCIER_SPEC, 19> {
        ERR3_W::new(self)
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
#[doc = "DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register.\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ebcier::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EBCIER_SPEC;
impl crate::RegisterSpec for EBCIER_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`ebcier::W`](W) writer structure"]
impl crate::Writable for EBCIER_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
