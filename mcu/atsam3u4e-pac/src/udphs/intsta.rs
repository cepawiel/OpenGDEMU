#[doc = "Register `INTSTA` reader"]
pub type R = crate::R<INTSTA_SPEC>;
#[doc = "Field `SPEED` reader - Speed Status"]
pub type SPEED_R = crate::BitReader;
#[doc = "Field `DET_SUSPD` reader - Suspend Interrupt"]
pub type DET_SUSPD_R = crate::BitReader;
#[doc = "Field `MICRO_SOF` reader - Micro Start Of Frame Interrupt"]
pub type MICRO_SOF_R = crate::BitReader;
#[doc = "Field `INT_SOF` reader - Start Of Frame Interrupt"]
pub type INT_SOF_R = crate::BitReader;
#[doc = "Field `ENDRESET` reader - End Of Reset Interrupt"]
pub type ENDRESET_R = crate::BitReader;
#[doc = "Field `WAKE_UP` reader - Wake Up CPU Interrupt"]
pub type WAKE_UP_R = crate::BitReader;
#[doc = "Field `ENDOFRSM` reader - End Of Resume Interrupt"]
pub type ENDOFRSM_R = crate::BitReader;
#[doc = "Field `UPSTR_RES` reader - Upstream Resume Interrupt"]
pub type UPSTR_RES_R = crate::BitReader;
#[doc = "Field `EPT_0` reader - Endpoint 0 Interrupt"]
pub type EPT_0_R = crate::BitReader;
#[doc = "Field `EPT_1` reader - Endpoint 1 Interrupt"]
pub type EPT_1_R = crate::BitReader;
#[doc = "Field `EPT_2` reader - Endpoint 2 Interrupt"]
pub type EPT_2_R = crate::BitReader;
#[doc = "Field `EPT_3` reader - Endpoint 3 Interrupt"]
pub type EPT_3_R = crate::BitReader;
#[doc = "Field `EPT_4` reader - Endpoint 4 Interrupt"]
pub type EPT_4_R = crate::BitReader;
#[doc = "Field `EPT_5` reader - Endpoint 5 Interrupt"]
pub type EPT_5_R = crate::BitReader;
#[doc = "Field `EPT_6` reader - Endpoint 6 Interrupt"]
pub type EPT_6_R = crate::BitReader;
#[doc = "Field `DMA_1` reader - DMA Channel 1 Interrupt"]
pub type DMA_1_R = crate::BitReader;
#[doc = "Field `DMA_2` reader - DMA Channel 2 Interrupt"]
pub type DMA_2_R = crate::BitReader;
#[doc = "Field `DMA_3` reader - DMA Channel 3 Interrupt"]
pub type DMA_3_R = crate::BitReader;
#[doc = "Field `DMA_4` reader - DMA Channel 4 Interrupt"]
pub type DMA_4_R = crate::BitReader;
#[doc = "Field `DMA_5` reader - DMA Channel 5 Interrupt"]
pub type DMA_5_R = crate::BitReader;
#[doc = "Field `DMA_6` reader - DMA Channel 6 Interrupt"]
pub type DMA_6_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Speed Status"]
    #[inline(always)]
    pub fn speed(&self) -> SPEED_R {
        SPEED_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Suspend Interrupt"]
    #[inline(always)]
    pub fn det_suspd(&self) -> DET_SUSPD_R {
        DET_SUSPD_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Micro Start Of Frame Interrupt"]
    #[inline(always)]
    pub fn micro_sof(&self) -> MICRO_SOF_R {
        MICRO_SOF_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Start Of Frame Interrupt"]
    #[inline(always)]
    pub fn int_sof(&self) -> INT_SOF_R {
        INT_SOF_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - End Of Reset Interrupt"]
    #[inline(always)]
    pub fn endreset(&self) -> ENDRESET_R {
        ENDRESET_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - Wake Up CPU Interrupt"]
    #[inline(always)]
    pub fn wake_up(&self) -> WAKE_UP_R {
        WAKE_UP_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - End Of Resume Interrupt"]
    #[inline(always)]
    pub fn endofrsm(&self) -> ENDOFRSM_R {
        ENDOFRSM_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - Upstream Resume Interrupt"]
    #[inline(always)]
    pub fn upstr_res(&self) -> UPSTR_RES_R {
        UPSTR_RES_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 8 - Endpoint 0 Interrupt"]
    #[inline(always)]
    pub fn ept_0(&self) -> EPT_0_R {
        EPT_0_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Endpoint 1 Interrupt"]
    #[inline(always)]
    pub fn ept_1(&self) -> EPT_1_R {
        EPT_1_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Endpoint 2 Interrupt"]
    #[inline(always)]
    pub fn ept_2(&self) -> EPT_2_R {
        EPT_2_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Endpoint 3 Interrupt"]
    #[inline(always)]
    pub fn ept_3(&self) -> EPT_3_R {
        EPT_3_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - Endpoint 4 Interrupt"]
    #[inline(always)]
    pub fn ept_4(&self) -> EPT_4_R {
        EPT_4_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - Endpoint 5 Interrupt"]
    #[inline(always)]
    pub fn ept_5(&self) -> EPT_5_R {
        EPT_5_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - Endpoint 6 Interrupt"]
    #[inline(always)]
    pub fn ept_6(&self) -> EPT_6_R {
        EPT_6_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 25 - DMA Channel 1 Interrupt"]
    #[inline(always)]
    pub fn dma_1(&self) -> DMA_1_R {
        DMA_1_R::new(((self.bits >> 25) & 1) != 0)
    }
    #[doc = "Bit 26 - DMA Channel 2 Interrupt"]
    #[inline(always)]
    pub fn dma_2(&self) -> DMA_2_R {
        DMA_2_R::new(((self.bits >> 26) & 1) != 0)
    }
    #[doc = "Bit 27 - DMA Channel 3 Interrupt"]
    #[inline(always)]
    pub fn dma_3(&self) -> DMA_3_R {
        DMA_3_R::new(((self.bits >> 27) & 1) != 0)
    }
    #[doc = "Bit 28 - DMA Channel 4 Interrupt"]
    #[inline(always)]
    pub fn dma_4(&self) -> DMA_4_R {
        DMA_4_R::new(((self.bits >> 28) & 1) != 0)
    }
    #[doc = "Bit 29 - DMA Channel 5 Interrupt"]
    #[inline(always)]
    pub fn dma_5(&self) -> DMA_5_R {
        DMA_5_R::new(((self.bits >> 29) & 1) != 0)
    }
    #[doc = "Bit 30 - DMA Channel 6 Interrupt"]
    #[inline(always)]
    pub fn dma_6(&self) -> DMA_6_R {
        DMA_6_R::new(((self.bits >> 30) & 1) != 0)
    }
}
#[doc = "UDPHS Interrupt Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`intsta::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct INTSTA_SPEC;
impl crate::RegisterSpec for INTSTA_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`intsta::R`](R) reader structure"]
impl crate::Readable for INTSTA_SPEC {}
#[doc = "`reset()` method sets INTSTA to value 0"]
impl crate::Resettable for INTSTA_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
