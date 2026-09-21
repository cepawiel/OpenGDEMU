#[doc = "Register `IMR` reader"]
pub type R = crate::R<IMR_SPEC>;
#[doc = "Field `EOC0` reader - End of Conversion Interrupt Mask 0"]
pub type EOC0_R = crate::BitReader;
#[doc = "Field `EOC1` reader - End of Conversion Interrupt Mask 1"]
pub type EOC1_R = crate::BitReader;
#[doc = "Field `EOC2` reader - End of Conversion Interrupt Mask 2"]
pub type EOC2_R = crate::BitReader;
#[doc = "Field `EOC3` reader - End of Conversion Interrupt Mask 3"]
pub type EOC3_R = crate::BitReader;
#[doc = "Field `EOC4` reader - End of Conversion Interrupt Mask 4"]
pub type EOC4_R = crate::BitReader;
#[doc = "Field `EOC5` reader - End of Conversion Interrupt Mask 5"]
pub type EOC5_R = crate::BitReader;
#[doc = "Field `EOC6` reader - End of Conversion Interrupt Mask 6"]
pub type EOC6_R = crate::BitReader;
#[doc = "Field `EOC7` reader - End of Conversion Interrupt Mask 7"]
pub type EOC7_R = crate::BitReader;
#[doc = "Field `OVRE0` reader - Overrun Error Interrupt Mask 0"]
pub type OVRE0_R = crate::BitReader;
#[doc = "Field `OVRE1` reader - Overrun Error Interrupt Mask 1"]
pub type OVRE1_R = crate::BitReader;
#[doc = "Field `OVRE2` reader - Overrun Error Interrupt Mask 2"]
pub type OVRE2_R = crate::BitReader;
#[doc = "Field `OVRE3` reader - Overrun Error Interrupt Mask 3"]
pub type OVRE3_R = crate::BitReader;
#[doc = "Field `OVRE4` reader - Overrun Error Interrupt Mask 4"]
pub type OVRE4_R = crate::BitReader;
#[doc = "Field `OVRE5` reader - Overrun Error Interrupt Mask 5"]
pub type OVRE5_R = crate::BitReader;
#[doc = "Field `OVRE6` reader - Overrun Error Interrupt Mask 6"]
pub type OVRE6_R = crate::BitReader;
#[doc = "Field `OVRE7` reader - Overrun Error Interrupt Mask 7"]
pub type OVRE7_R = crate::BitReader;
#[doc = "Field `DRDY` reader - Data Ready Interrupt Mask"]
pub type DRDY_R = crate::BitReader;
#[doc = "Field `GOVRE` reader - General Overrun Error Interrupt Mask"]
pub type GOVRE_R = crate::BitReader;
#[doc = "Field `ENDRX` reader - End of Receive Buffer Interrupt Mask"]
pub type ENDRX_R = crate::BitReader;
#[doc = "Field `RXBUFF` reader - Receive Buffer Full Interrupt Mask"]
pub type RXBUFF_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - End of Conversion Interrupt Mask 0"]
    #[inline(always)]
    pub fn eoc0(&self) -> EOC0_R {
        EOC0_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - End of Conversion Interrupt Mask 1"]
    #[inline(always)]
    pub fn eoc1(&self) -> EOC1_R {
        EOC1_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - End of Conversion Interrupt Mask 2"]
    #[inline(always)]
    pub fn eoc2(&self) -> EOC2_R {
        EOC2_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - End of Conversion Interrupt Mask 3"]
    #[inline(always)]
    pub fn eoc3(&self) -> EOC3_R {
        EOC3_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - End of Conversion Interrupt Mask 4"]
    #[inline(always)]
    pub fn eoc4(&self) -> EOC4_R {
        EOC4_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - End of Conversion Interrupt Mask 5"]
    #[inline(always)]
    pub fn eoc5(&self) -> EOC5_R {
        EOC5_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - End of Conversion Interrupt Mask 6"]
    #[inline(always)]
    pub fn eoc6(&self) -> EOC6_R {
        EOC6_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - End of Conversion Interrupt Mask 7"]
    #[inline(always)]
    pub fn eoc7(&self) -> EOC7_R {
        EOC7_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 8 - Overrun Error Interrupt Mask 0"]
    #[inline(always)]
    pub fn ovre0(&self) -> OVRE0_R {
        OVRE0_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Overrun Error Interrupt Mask 1"]
    #[inline(always)]
    pub fn ovre1(&self) -> OVRE1_R {
        OVRE1_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Overrun Error Interrupt Mask 2"]
    #[inline(always)]
    pub fn ovre2(&self) -> OVRE2_R {
        OVRE2_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Overrun Error Interrupt Mask 3"]
    #[inline(always)]
    pub fn ovre3(&self) -> OVRE3_R {
        OVRE3_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - Overrun Error Interrupt Mask 4"]
    #[inline(always)]
    pub fn ovre4(&self) -> OVRE4_R {
        OVRE4_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - Overrun Error Interrupt Mask 5"]
    #[inline(always)]
    pub fn ovre5(&self) -> OVRE5_R {
        OVRE5_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - Overrun Error Interrupt Mask 6"]
    #[inline(always)]
    pub fn ovre6(&self) -> OVRE6_R {
        OVRE6_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 15 - Overrun Error Interrupt Mask 7"]
    #[inline(always)]
    pub fn ovre7(&self) -> OVRE7_R {
        OVRE7_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bit 16 - Data Ready Interrupt Mask"]
    #[inline(always)]
    pub fn drdy(&self) -> DRDY_R {
        DRDY_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - General Overrun Error Interrupt Mask"]
    #[inline(always)]
    pub fn govre(&self) -> GOVRE_R {
        GOVRE_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - End of Receive Buffer Interrupt Mask"]
    #[inline(always)]
    pub fn endrx(&self) -> ENDRX_R {
        ENDRX_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 19 - Receive Buffer Full Interrupt Mask"]
    #[inline(always)]
    pub fn rxbuff(&self) -> RXBUFF_R {
        RXBUFF_R::new(((self.bits >> 19) & 1) != 0)
    }
}
#[doc = "Interrupt Mask Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`imr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
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
