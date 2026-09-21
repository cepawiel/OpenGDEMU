#[doc = "Register `EPTCTLENB1` writer"]
pub type W = crate::W<EPTCTLENB1_SPEC>;
#[doc = "Field `EPT_ENABL` writer - Endpoint Enable"]
pub type EPT_ENABL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `AUTO_VALID` writer - Packet Auto-Valid Enable"]
pub type AUTO_VALID_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `INTDIS_DMA` writer - Interrupts Disable DMA"]
pub type INTDIS_DMA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `NYET_DIS` writer - NYET Disable (Only for High Speed Bulk OUT endpoints)"]
pub type NYET_DIS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DATAX_RX` writer - DATAx Interrupt Enable (Only for high bandwidth Isochronous OUT endpoints)"]
pub type DATAX_RX_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MDATA_RX` writer - MDATA Interrupt Enable (Only for high bandwidth Isochronous OUT endpoints)"]
pub type MDATA_RX_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_OVFLW` writer - Overflow Error Interrupt Enable"]
pub type ERR_OVFLW_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RX_BK_RDY` writer - Received OUT Data Interrupt Enable"]
pub type RX_BK_RDY_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TX_COMPLT` writer - Transmitted IN Data Complete Interrupt Enable"]
pub type TX_COMPLT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TX_PK_RDY` writer - TX Packet Ready/Transaction Error Interrupt Enable"]
pub type TX_PK_RDY_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_TRANS` writer - TX Packet Ready/Transaction Error Interrupt Enable"]
pub type ERR_TRANS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `RX_SETUP` writer - Received SETUP/Error Flow Interrupt Enable"]
pub type RX_SETUP_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_FL_ISO` writer - Received SETUP/Error Flow Interrupt Enable"]
pub type ERR_FL_ISO_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `STALL_SNT` writer - Stall Sent /ISO CRC Error/Number of Transaction Error Interrupt Enable"]
pub type STALL_SNT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_CRISO` writer - Stall Sent /ISO CRC Error/Number of Transaction Error Interrupt Enable"]
pub type ERR_CRISO_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_NBTRA` writer - Stall Sent /ISO CRC Error/Number of Transaction Error Interrupt Enable"]
pub type ERR_NBTRA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `NAK_IN` writer - NAKIN/Bank Flush Error Interrupt Enable"]
pub type NAK_IN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR_FLUSH` writer - NAKIN/Bank Flush Error Interrupt Enable"]
pub type ERR_FLUSH_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `NAK_OUT` writer - NAKOUT Interrupt Enable"]
pub type NAK_OUT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `BUSY_BANK` writer - Busy Bank Interrupt Enable"]
pub type BUSY_BANK_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `SHRT_PCKT` writer - Short Packet Send/Short Packet Interrupt Enable"]
pub type SHRT_PCKT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 0 - Endpoint Enable"]
    #[inline(always)]
    #[must_use]
    pub fn ept_enabl(&mut self) -> EPT_ENABL_W<EPTCTLENB1_SPEC, 0> {
        EPT_ENABL_W::new(self)
    }
    #[doc = "Bit 1 - Packet Auto-Valid Enable"]
    #[inline(always)]
    #[must_use]
    pub fn auto_valid(&mut self) -> AUTO_VALID_W<EPTCTLENB1_SPEC, 1> {
        AUTO_VALID_W::new(self)
    }
    #[doc = "Bit 3 - Interrupts Disable DMA"]
    #[inline(always)]
    #[must_use]
    pub fn intdis_dma(&mut self) -> INTDIS_DMA_W<EPTCTLENB1_SPEC, 3> {
        INTDIS_DMA_W::new(self)
    }
    #[doc = "Bit 4 - NYET Disable (Only for High Speed Bulk OUT endpoints)"]
    #[inline(always)]
    #[must_use]
    pub fn nyet_dis(&mut self) -> NYET_DIS_W<EPTCTLENB1_SPEC, 4> {
        NYET_DIS_W::new(self)
    }
    #[doc = "Bit 6 - DATAx Interrupt Enable (Only for high bandwidth Isochronous OUT endpoints)"]
    #[inline(always)]
    #[must_use]
    pub fn datax_rx(&mut self) -> DATAX_RX_W<EPTCTLENB1_SPEC, 6> {
        DATAX_RX_W::new(self)
    }
    #[doc = "Bit 7 - MDATA Interrupt Enable (Only for high bandwidth Isochronous OUT endpoints)"]
    #[inline(always)]
    #[must_use]
    pub fn mdata_rx(&mut self) -> MDATA_RX_W<EPTCTLENB1_SPEC, 7> {
        MDATA_RX_W::new(self)
    }
    #[doc = "Bit 8 - Overflow Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err_ovflw(&mut self) -> ERR_OVFLW_W<EPTCTLENB1_SPEC, 8> {
        ERR_OVFLW_W::new(self)
    }
    #[doc = "Bit 9 - Received OUT Data Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn rx_bk_rdy(&mut self) -> RX_BK_RDY_W<EPTCTLENB1_SPEC, 9> {
        RX_BK_RDY_W::new(self)
    }
    #[doc = "Bit 10 - Transmitted IN Data Complete Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn tx_complt(&mut self) -> TX_COMPLT_W<EPTCTLENB1_SPEC, 10> {
        TX_COMPLT_W::new(self)
    }
    #[doc = "Bit 11 - TX Packet Ready/Transaction Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn tx_pk_rdy(&mut self) -> TX_PK_RDY_W<EPTCTLENB1_SPEC, 11> {
        TX_PK_RDY_W::new(self)
    }
    #[doc = "Bit 11 - TX Packet Ready/Transaction Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err_trans(&mut self) -> ERR_TRANS_W<EPTCTLENB1_SPEC, 11> {
        ERR_TRANS_W::new(self)
    }
    #[doc = "Bit 12 - Received SETUP/Error Flow Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn rx_setup(&mut self) -> RX_SETUP_W<EPTCTLENB1_SPEC, 12> {
        RX_SETUP_W::new(self)
    }
    #[doc = "Bit 12 - Received SETUP/Error Flow Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err_fl_iso(&mut self) -> ERR_FL_ISO_W<EPTCTLENB1_SPEC, 12> {
        ERR_FL_ISO_W::new(self)
    }
    #[doc = "Bit 13 - Stall Sent /ISO CRC Error/Number of Transaction Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn stall_snt(&mut self) -> STALL_SNT_W<EPTCTLENB1_SPEC, 13> {
        STALL_SNT_W::new(self)
    }
    #[doc = "Bit 13 - Stall Sent /ISO CRC Error/Number of Transaction Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err_criso(&mut self) -> ERR_CRISO_W<EPTCTLENB1_SPEC, 13> {
        ERR_CRISO_W::new(self)
    }
    #[doc = "Bit 13 - Stall Sent /ISO CRC Error/Number of Transaction Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err_nbtra(&mut self) -> ERR_NBTRA_W<EPTCTLENB1_SPEC, 13> {
        ERR_NBTRA_W::new(self)
    }
    #[doc = "Bit 14 - NAKIN/Bank Flush Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn nak_in(&mut self) -> NAK_IN_W<EPTCTLENB1_SPEC, 14> {
        NAK_IN_W::new(self)
    }
    #[doc = "Bit 14 - NAKIN/Bank Flush Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err_flush(&mut self) -> ERR_FLUSH_W<EPTCTLENB1_SPEC, 14> {
        ERR_FLUSH_W::new(self)
    }
    #[doc = "Bit 15 - NAKOUT Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn nak_out(&mut self) -> NAK_OUT_W<EPTCTLENB1_SPEC, 15> {
        NAK_OUT_W::new(self)
    }
    #[doc = "Bit 18 - Busy Bank Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn busy_bank(&mut self) -> BUSY_BANK_W<EPTCTLENB1_SPEC, 18> {
        BUSY_BANK_W::new(self)
    }
    #[doc = "Bit 31 - Short Packet Send/Short Packet Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn shrt_pckt(&mut self) -> SHRT_PCKT_W<EPTCTLENB1_SPEC, 31> {
        SHRT_PCKT_W::new(self)
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
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb1::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EPTCTLENB1_SPEC;
impl crate::RegisterSpec for EPTCTLENB1_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`eptctlenb1::W`](W) writer structure"]
impl crate::Writable for EPTCTLENB1_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
