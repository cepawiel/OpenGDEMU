#[doc = "Register `IPFEATURES` reader"]
pub type R = crate::R<IPFEATURES_SPEC>;
#[doc = "Field `EPT_NBR_MAX` reader - Max Number of Endpoints"]
pub type EPT_NBR_MAX_R = crate::FieldReader;
#[doc = "Field `DMA_CHANNEL_NBR` reader - Number of DMA Channels"]
pub type DMA_CHANNEL_NBR_R = crate::FieldReader;
#[doc = "Field `DMA_B_SIZ` reader - DMA Buffer Size"]
pub type DMA_B_SIZ_R = crate::BitReader;
#[doc = "Field `DMA_FIFO_WORD_DEPTH` reader - DMA FIFO Depth in Words"]
pub type DMA_FIFO_WORD_DEPTH_R = crate::FieldReader;
#[doc = "Field `FIFO_MAX_SIZE` reader - DPRAM Size"]
pub type FIFO_MAX_SIZE_R = crate::FieldReader;
#[doc = "Field `BW_DPRAM` reader - DPRAM Byte Write Capability"]
pub type BW_DPRAM_R = crate::BitReader;
#[doc = "Field `DATAB16_8` reader - UTMI DataBus16_8"]
pub type DATAB16_8_R = crate::BitReader;
#[doc = "Field `ISO_EPT_1` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_1_R = crate::BitReader;
#[doc = "Field `ISO_EPT_2` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_2_R = crate::BitReader;
#[doc = "Field `ISO_EPT_3` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_3_R = crate::BitReader;
#[doc = "Field `ISO_EPT_4` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_4_R = crate::BitReader;
#[doc = "Field `ISO_EPT_5` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_5_R = crate::BitReader;
#[doc = "Field `ISO_EPT_6` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_6_R = crate::BitReader;
#[doc = "Field `ISO_EPT_7` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_7_R = crate::BitReader;
#[doc = "Field `ISO_EPT_8` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_8_R = crate::BitReader;
#[doc = "Field `ISO_EPT_9` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_9_R = crate::BitReader;
#[doc = "Field `ISO_EPT_10` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_10_R = crate::BitReader;
#[doc = "Field `ISO_EPT_11` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_11_R = crate::BitReader;
#[doc = "Field `ISO_EPT_12` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_12_R = crate::BitReader;
#[doc = "Field `ISO_EPT_13` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_13_R = crate::BitReader;
#[doc = "Field `ISO_EPT_14` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_14_R = crate::BitReader;
#[doc = "Field `ISO_EPT_15` reader - Endpointx High Bandwidth Isochronous Capability"]
pub type ISO_EPT_15_R = crate::BitReader;
impl R {
    #[doc = "Bits 0:3 - Max Number of Endpoints"]
    #[inline(always)]
    pub fn ept_nbr_max(&self) -> EPT_NBR_MAX_R {
        EPT_NBR_MAX_R::new((self.bits & 0x0f) as u8)
    }
    #[doc = "Bits 4:6 - Number of DMA Channels"]
    #[inline(always)]
    pub fn dma_channel_nbr(&self) -> DMA_CHANNEL_NBR_R {
        DMA_CHANNEL_NBR_R::new(((self.bits >> 4) & 7) as u8)
    }
    #[doc = "Bit 7 - DMA Buffer Size"]
    #[inline(always)]
    pub fn dma_b_siz(&self) -> DMA_B_SIZ_R {
        DMA_B_SIZ_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bits 8:11 - DMA FIFO Depth in Words"]
    #[inline(always)]
    pub fn dma_fifo_word_depth(&self) -> DMA_FIFO_WORD_DEPTH_R {
        DMA_FIFO_WORD_DEPTH_R::new(((self.bits >> 8) & 0x0f) as u8)
    }
    #[doc = "Bits 12:14 - DPRAM Size"]
    #[inline(always)]
    pub fn fifo_max_size(&self) -> FIFO_MAX_SIZE_R {
        FIFO_MAX_SIZE_R::new(((self.bits >> 12) & 7) as u8)
    }
    #[doc = "Bit 15 - DPRAM Byte Write Capability"]
    #[inline(always)]
    pub fn bw_dpram(&self) -> BW_DPRAM_R {
        BW_DPRAM_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bit 16 - UTMI DataBus16_8"]
    #[inline(always)]
    pub fn datab16_8(&self) -> DATAB16_8_R {
        DATAB16_8_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_1(&self) -> ISO_EPT_1_R {
        ISO_EPT_1_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_2(&self) -> ISO_EPT_2_R {
        ISO_EPT_2_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 19 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_3(&self) -> ISO_EPT_3_R {
        ISO_EPT_3_R::new(((self.bits >> 19) & 1) != 0)
    }
    #[doc = "Bit 20 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_4(&self) -> ISO_EPT_4_R {
        ISO_EPT_4_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bit 21 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_5(&self) -> ISO_EPT_5_R {
        ISO_EPT_5_R::new(((self.bits >> 21) & 1) != 0)
    }
    #[doc = "Bit 22 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_6(&self) -> ISO_EPT_6_R {
        ISO_EPT_6_R::new(((self.bits >> 22) & 1) != 0)
    }
    #[doc = "Bit 23 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_7(&self) -> ISO_EPT_7_R {
        ISO_EPT_7_R::new(((self.bits >> 23) & 1) != 0)
    }
    #[doc = "Bit 24 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_8(&self) -> ISO_EPT_8_R {
        ISO_EPT_8_R::new(((self.bits >> 24) & 1) != 0)
    }
    #[doc = "Bit 25 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_9(&self) -> ISO_EPT_9_R {
        ISO_EPT_9_R::new(((self.bits >> 25) & 1) != 0)
    }
    #[doc = "Bit 26 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_10(&self) -> ISO_EPT_10_R {
        ISO_EPT_10_R::new(((self.bits >> 26) & 1) != 0)
    }
    #[doc = "Bit 27 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_11(&self) -> ISO_EPT_11_R {
        ISO_EPT_11_R::new(((self.bits >> 27) & 1) != 0)
    }
    #[doc = "Bit 28 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_12(&self) -> ISO_EPT_12_R {
        ISO_EPT_12_R::new(((self.bits >> 28) & 1) != 0)
    }
    #[doc = "Bit 29 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_13(&self) -> ISO_EPT_13_R {
        ISO_EPT_13_R::new(((self.bits >> 29) & 1) != 0)
    }
    #[doc = "Bit 30 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_14(&self) -> ISO_EPT_14_R {
        ISO_EPT_14_R::new(((self.bits >> 30) & 1) != 0)
    }
    #[doc = "Bit 31 - Endpointx High Bandwidth Isochronous Capability"]
    #[inline(always)]
    pub fn iso_ept_15(&self) -> ISO_EPT_15_R {
        ISO_EPT_15_R::new(((self.bits >> 31) & 1) != 0)
    }
}
#[doc = "UDPHS Features Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ipfeatures::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IPFEATURES_SPEC;
impl crate::RegisterSpec for IPFEATURES_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ipfeatures::R`](R) reader structure"]
impl crate::Readable for IPFEATURES_SPEC {}
