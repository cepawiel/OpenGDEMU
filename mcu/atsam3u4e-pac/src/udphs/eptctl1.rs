#[doc = "Register `EPTCTL1` reader"]
pub type R = crate::R<EPTCTL1_SPEC>;
#[doc = "Field `EPT_ENABL` reader - Endpoint Enable"]
pub type EPT_ENABL_R = crate::BitReader;
#[doc = "Field `AUTO_VALID` reader - Packet Auto-Valid Enabled (Not for CONTROL Endpoints)"]
pub type AUTO_VALID_R = crate::BitReader;
#[doc = "Field `INTDIS_DMA` reader - Interrupt Disables DMA"]
pub type INTDIS_DMA_R = crate::BitReader;
#[doc = "Field `NYET_DIS` reader - NYET Disable (Only for High Speed Bulk OUT endpoints)"]
pub type NYET_DIS_R = crate::BitReader;
#[doc = "Field `DATAX_RX` reader - DATAx Interrupt Enabled (Only for High Bandwidth Isochronous OUT endpoints)"]
pub type DATAX_RX_R = crate::BitReader;
#[doc = "Field `MDATA_RX` reader - MDATA Interrupt Enabled (Only for High Bandwidth Isochronous OUT endpoints)"]
pub type MDATA_RX_R = crate::BitReader;
#[doc = "Field `ERR_OVFLW` reader - Overflow Error Interrupt Enabled"]
pub type ERR_OVFLW_R = crate::BitReader;
#[doc = "Field `RX_BK_RDY` reader - Received OUT Data Interrupt Enabled"]
pub type RX_BK_RDY_R = crate::BitReader;
#[doc = "Field `TX_COMPLT` reader - Transmitted IN Data Complete Interrupt Enabled"]
pub type TX_COMPLT_R = crate::BitReader;
#[doc = "Field `TX_PK_RDY` reader - TX Packet Ready/Transaction Error Interrupt Enabled"]
pub type TX_PK_RDY_R = crate::BitReader;
#[doc = "Field `ERR_TRANS` reader - TX Packet Ready/Transaction Error Interrupt Enabled"]
pub type ERR_TRANS_R = crate::BitReader;
#[doc = "Field `RX_SETUP` reader - Received SETUP/Error Flow Interrupt Enabled"]
pub type RX_SETUP_R = crate::BitReader;
#[doc = "Field `ERR_FL_ISO` reader - Received SETUP/Error Flow Interrupt Enabled"]
pub type ERR_FL_ISO_R = crate::BitReader;
#[doc = "Field `STALL_SNT` reader - Stall Sent/ISO CRC Error/Number of Transaction Error Interrupt Enabled"]
pub type STALL_SNT_R = crate::BitReader;
#[doc = "Field `ERR_CRISO` reader - Stall Sent/ISO CRC Error/Number of Transaction Error Interrupt Enabled"]
pub type ERR_CRISO_R = crate::BitReader;
#[doc = "Field `ERR_NBTRA` reader - Stall Sent/ISO CRC Error/Number of Transaction Error Interrupt Enabled"]
pub type ERR_NBTRA_R = crate::BitReader;
#[doc = "Field `NAK_IN` reader - NAKIN/Bank Flush Error Interrupt Enabled"]
pub type NAK_IN_R = crate::BitReader;
#[doc = "Field `ERR_FLUSH` reader - NAKIN/Bank Flush Error Interrupt Enabled"]
pub type ERR_FLUSH_R = crate::BitReader;
#[doc = "Field `NAK_OUT` reader - NAKOUT Interrupt Enabled"]
pub type NAK_OUT_R = crate::BitReader;
#[doc = "Field `BUSY_BANK` reader - Busy Bank Interrupt Enabled"]
pub type BUSY_BANK_R = crate::BitReader;
#[doc = "Field `SHRT_PCKT` reader - Short Packet Interrupt Enabled"]
pub type SHRT_PCKT_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Endpoint Enable"]
    #[inline(always)]
    pub fn ept_enabl(&self) -> EPT_ENABL_R {
        EPT_ENABL_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Packet Auto-Valid Enabled (Not for CONTROL Endpoints)"]
    #[inline(always)]
    pub fn auto_valid(&self) -> AUTO_VALID_R {
        AUTO_VALID_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 3 - Interrupt Disables DMA"]
    #[inline(always)]
    pub fn intdis_dma(&self) -> INTDIS_DMA_R {
        INTDIS_DMA_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - NYET Disable (Only for High Speed Bulk OUT endpoints)"]
    #[inline(always)]
    pub fn nyet_dis(&self) -> NYET_DIS_R {
        NYET_DIS_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 6 - DATAx Interrupt Enabled (Only for High Bandwidth Isochronous OUT endpoints)"]
    #[inline(always)]
    pub fn datax_rx(&self) -> DATAX_RX_R {
        DATAX_RX_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - MDATA Interrupt Enabled (Only for High Bandwidth Isochronous OUT endpoints)"]
    #[inline(always)]
    pub fn mdata_rx(&self) -> MDATA_RX_R {
        MDATA_RX_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 8 - Overflow Error Interrupt Enabled"]
    #[inline(always)]
    pub fn err_ovflw(&self) -> ERR_OVFLW_R {
        ERR_OVFLW_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Received OUT Data Interrupt Enabled"]
    #[inline(always)]
    pub fn rx_bk_rdy(&self) -> RX_BK_RDY_R {
        RX_BK_RDY_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Transmitted IN Data Complete Interrupt Enabled"]
    #[inline(always)]
    pub fn tx_complt(&self) -> TX_COMPLT_R {
        TX_COMPLT_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - TX Packet Ready/Transaction Error Interrupt Enabled"]
    #[inline(always)]
    pub fn tx_pk_rdy(&self) -> TX_PK_RDY_R {
        TX_PK_RDY_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 11 - TX Packet Ready/Transaction Error Interrupt Enabled"]
    #[inline(always)]
    pub fn err_trans(&self) -> ERR_TRANS_R {
        ERR_TRANS_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - Received SETUP/Error Flow Interrupt Enabled"]
    #[inline(always)]
    pub fn rx_setup(&self) -> RX_SETUP_R {
        RX_SETUP_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 12 - Received SETUP/Error Flow Interrupt Enabled"]
    #[inline(always)]
    pub fn err_fl_iso(&self) -> ERR_FL_ISO_R {
        ERR_FL_ISO_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - Stall Sent/ISO CRC Error/Number of Transaction Error Interrupt Enabled"]
    #[inline(always)]
    pub fn stall_snt(&self) -> STALL_SNT_R {
        STALL_SNT_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 13 - Stall Sent/ISO CRC Error/Number of Transaction Error Interrupt Enabled"]
    #[inline(always)]
    pub fn err_criso(&self) -> ERR_CRISO_R {
        ERR_CRISO_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 13 - Stall Sent/ISO CRC Error/Number of Transaction Error Interrupt Enabled"]
    #[inline(always)]
    pub fn err_nbtra(&self) -> ERR_NBTRA_R {
        ERR_NBTRA_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - NAKIN/Bank Flush Error Interrupt Enabled"]
    #[inline(always)]
    pub fn nak_in(&self) -> NAK_IN_R {
        NAK_IN_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 14 - NAKIN/Bank Flush Error Interrupt Enabled"]
    #[inline(always)]
    pub fn err_flush(&self) -> ERR_FLUSH_R {
        ERR_FLUSH_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 15 - NAKOUT Interrupt Enabled"]
    #[inline(always)]
    pub fn nak_out(&self) -> NAK_OUT_R {
        NAK_OUT_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bit 18 - Busy Bank Interrupt Enabled"]
    #[inline(always)]
    pub fn busy_bank(&self) -> BUSY_BANK_R {
        BUSY_BANK_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 31 - Short Packet Interrupt Enabled"]
    #[inline(always)]
    pub fn shrt_pckt(&self) -> SHRT_PCKT_R {
        SHRT_PCKT_R::new(((self.bits >> 31) & 1) != 0)
    }
}
#[doc = "UDPHS Endpoint Control Register (endpoint = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl1::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EPTCTL1_SPEC;
impl crate::RegisterSpec for EPTCTL1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`eptctl1::R`](R) reader structure"]
impl crate::Readable for EPTCTL1_SPEC {}
#[doc = "`reset()` method sets EPTCTL1 to value 0"]
impl crate::Resettable for EPTCTL1_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
