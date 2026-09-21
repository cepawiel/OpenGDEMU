#[doc = "Register `EPTCLRSTA4` writer"]
pub type W = crate::W<EPTCLRSTA4_SPEC>;
#[doc = "Field `FRCESTALL` writer - Stall Handshake Request Clear"]
pub type FRCESTALL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TOGGLESQ` writer - Data Toggle Clear"]
pub type TOGGLESQ_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RX_BK_RDY` writer - Received OUT Data Clear"]
pub type RX_BK_RDY_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TX_COMPLT` writer - Transmitted IN Data Complete Clear"]
pub type TX_COMPLT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RX_SETUP` writer - Received SETUP/Error Flow Clear"]
pub type RX_SETUP_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_FL_ISO` writer - Received SETUP/Error Flow Clear"]
pub type ERR_FL_ISO_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `STALL_SNT` writer - Stall Sent/Number of Transaction Error Clear"]
pub type STALL_SNT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_NBTRA` writer - Stall Sent/Number of Transaction Error Clear"]
pub type ERR_NBTRA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `NAK_IN` writer - NAKIN/Bank Flush Error Clear"]
pub type NAK_IN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_FLUSH` writer - NAKIN/Bank Flush Error Clear"]
pub type ERR_FLUSH_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `NAK_OUT` writer - NAKOUT Clear"]
pub type NAK_OUT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 5 - Stall Handshake Request Clear"]
    #[inline(always)]
    #[must_use]
    pub fn frcestall(&mut self) -> FRCESTALL_W<EPTCLRSTA4_SPEC, 5> {
        FRCESTALL_W::new(self)
    }
    #[doc = "Bit 6 - Data Toggle Clear"]
    #[inline(always)]
    #[must_use]
    pub fn togglesq(&mut self) -> TOGGLESQ_W<EPTCLRSTA4_SPEC, 6> {
        TOGGLESQ_W::new(self)
    }
    #[doc = "Bit 9 - Received OUT Data Clear"]
    #[inline(always)]
    #[must_use]
    pub fn rx_bk_rdy(&mut self) -> RX_BK_RDY_W<EPTCLRSTA4_SPEC, 9> {
        RX_BK_RDY_W::new(self)
    }
    #[doc = "Bit 10 - Transmitted IN Data Complete Clear"]
    #[inline(always)]
    #[must_use]
    pub fn tx_complt(&mut self) -> TX_COMPLT_W<EPTCLRSTA4_SPEC, 10> {
        TX_COMPLT_W::new(self)
    }
    #[doc = "Bit 12 - Received SETUP/Error Flow Clear"]
    #[inline(always)]
    #[must_use]
    pub fn rx_setup(&mut self) -> RX_SETUP_W<EPTCLRSTA4_SPEC, 12> {
        RX_SETUP_W::new(self)
    }
    #[doc = "Bit 12 - Received SETUP/Error Flow Clear"]
    #[inline(always)]
    #[must_use]
    pub fn err_fl_iso(&mut self) -> ERR_FL_ISO_W<EPTCLRSTA4_SPEC, 12> {
        ERR_FL_ISO_W::new(self)
    }
    #[doc = "Bit 13 - Stall Sent/Number of Transaction Error Clear"]
    #[inline(always)]
    #[must_use]
    pub fn stall_snt(&mut self) -> STALL_SNT_W<EPTCLRSTA4_SPEC, 13> {
        STALL_SNT_W::new(self)
    }
    #[doc = "Bit 13 - Stall Sent/Number of Transaction Error Clear"]
    #[inline(always)]
    #[must_use]
    pub fn err_nbtra(&mut self) -> ERR_NBTRA_W<EPTCLRSTA4_SPEC, 13> {
        ERR_NBTRA_W::new(self)
    }
    #[doc = "Bit 14 - NAKIN/Bank Flush Error Clear"]
    #[inline(always)]
    #[must_use]
    pub fn nak_in(&mut self) -> NAK_IN_W<EPTCLRSTA4_SPEC, 14> {
        NAK_IN_W::new(self)
    }
    #[doc = "Bit 14 - NAKIN/Bank Flush Error Clear"]
    #[inline(always)]
    #[must_use]
    pub fn err_flush(&mut self) -> ERR_FLUSH_W<EPTCLRSTA4_SPEC, 14> {
        ERR_FLUSH_W::new(self)
    }
    #[doc = "Bit 15 - NAKOUT Clear"]
    #[inline(always)]
    #[must_use]
    pub fn nak_out(&mut self) -> NAK_OUT_W<EPTCLRSTA4_SPEC, 15> {
        NAK_OUT_W::new(self)
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
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 4)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta4::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EPTCLRSTA4_SPEC;
impl crate::RegisterSpec for EPTCLRSTA4_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`eptclrsta4::W`](W) writer structure"]
impl crate::Writable for EPTCLRSTA4_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
