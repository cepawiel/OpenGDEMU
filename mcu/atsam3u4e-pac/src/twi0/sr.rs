#[doc = "Register `SR` reader"]
pub type R = crate::R<SR_SPEC>;
#[doc = "Field `TXCOMP` reader - Transmission Completed (automatically set / reset)"]
pub type TXCOMP_R = crate::BitReader;
#[doc = "Field `RXRDY` reader - Receive Holding Register Ready (automatically set / reset)"]
pub type RXRDY_R = crate::BitReader;
#[doc = "Field `TXRDY` reader - Transmit Holding Register Ready (automatically set / reset)"]
pub type TXRDY_R = crate::BitReader;
#[doc = "Field `SVREAD` reader - Slave Read (automatically set / reset)"]
pub type SVREAD_R = crate::BitReader;
#[doc = "Field `SVACC` reader - Slave Access (automatically set / reset)"]
pub type SVACC_R = crate::BitReader;
#[doc = "Field `GACC` reader - General Call Access (clear on read)"]
pub type GACC_R = crate::BitReader;
#[doc = "Field `OVRE` reader - Overrun Error (clear on read)"]
pub type OVRE_R = crate::BitReader;
#[doc = "Field `NACK` reader - Not Acknowledged (clear on read)"]
pub type NACK_R = crate::BitReader;
#[doc = "Field `ARBLST` reader - Arbitration Lost (clear on read)"]
pub type ARBLST_R = crate::BitReader;
#[doc = "Field `SCLWS` reader - Clock Wait State (automatically set / reset)"]
pub type SCLWS_R = crate::BitReader;
#[doc = "Field `EOSACC` reader - End Of Slave Access (clear on read)"]
pub type EOSACC_R = crate::BitReader;
#[doc = "Field `ENDRX` reader - End of RX buffer"]
pub type ENDRX_R = crate::BitReader;
#[doc = "Field `ENDTX` reader - End of TX buffer"]
pub type ENDTX_R = crate::BitReader;
#[doc = "Field `RXBUFF` reader - RX Buffer Full"]
pub type RXBUFF_R = crate::BitReader;
#[doc = "Field `TXBUFE` reader - TX Buffer Empty"]
pub type TXBUFE_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Transmission Completed (automatically set / reset)"]
    #[inline(always)]
    pub fn txcomp(&self) -> TXCOMP_R {
        TXCOMP_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Receive Holding Register Ready (automatically set / reset)"]
    #[inline(always)]
    pub fn rxrdy(&self) -> RXRDY_R {
        RXRDY_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Transmit Holding Register Ready (automatically set / reset)"]
    #[inline(always)]
    pub fn txrdy(&self) -> TXRDY_R {
        TXRDY_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Slave Read (automatically set / reset)"]
    #[inline(always)]
    pub fn svread(&self) -> SVREAD_R {
        SVREAD_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Slave Access (automatically set / reset)"]
    #[inline(always)]
    pub fn svacc(&self) -> SVACC_R {
        SVACC_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - General Call Access (clear on read)"]
    #[inline(always)]
    pub fn gacc(&self) -> GACC_R {
        GACC_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - Overrun Error (clear on read)"]
    #[inline(always)]
    pub fn ovre(&self) -> OVRE_R {
        OVRE_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 8 - Not Acknowledged (clear on read)"]
    #[inline(always)]
    pub fn nack(&self) -> NACK_R {
        NACK_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Arbitration Lost (clear on read)"]
    #[inline(always)]
    pub fn arblst(&self) -> ARBLST_R {
        ARBLST_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Clock Wait State (automatically set / reset)"]
    #[inline(always)]
    pub fn sclws(&self) -> SCLWS_R {
        SCLWS_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - End Of Slave Access (clear on read)"]
    #[inline(always)]
    pub fn eosacc(&self) -> EOSACC_R {
        EOSACC_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - End of RX buffer"]
    #[inline(always)]
    pub fn endrx(&self) -> ENDRX_R {
        ENDRX_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - End of TX buffer"]
    #[inline(always)]
    pub fn endtx(&self) -> ENDTX_R {
        ENDTX_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - RX Buffer Full"]
    #[inline(always)]
    pub fn rxbuff(&self) -> RXBUFF_R {
        RXBUFF_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 15 - TX Buffer Empty"]
    #[inline(always)]
    pub fn txbufe(&self) -> TXBUFE_R {
        TXBUFE_R::new(((self.bits >> 15) & 1) != 0)
    }
}
#[doc = "Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct SR_SPEC;
impl crate::RegisterSpec for SR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`sr::R`](R) reader structure"]
impl crate::Readable for SR_SPEC {}
#[doc = "`reset()` method sets SR to value 0xf009"]
impl crate::Resettable for SR_SPEC {
    const RESET_VALUE: Self::Ux = 0xf009;
}
