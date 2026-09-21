#[doc = "Register `PMC_IMR` reader"]
pub type R = crate::R<PMC_IMR_SPEC>;
#[doc = "Field `MOSCXTS` reader - Main Crystal Oscillator Status Interrupt Mask"]
pub type MOSCXTS_R = crate::BitReader;
#[doc = "Field `LOCKA` reader - PLLA Lock Interrupt Mask"]
pub type LOCKA_R = crate::BitReader;
#[doc = "Field `MCKRDY` reader - Master Clock Ready Interrupt Mask"]
pub type MCKRDY_R = crate::BitReader;
#[doc = "Field `LOCKU` reader - UTMI PLL Lock Interrupt Mask"]
pub type LOCKU_R = crate::BitReader;
#[doc = "Field `PCKRDY0` reader - Programmable Clock Ready 0 Interrupt Mask"]
pub type PCKRDY0_R = crate::BitReader;
#[doc = "Field `PCKRDY1` reader - Programmable Clock Ready 1 Interrupt Mask"]
pub type PCKRDY1_R = crate::BitReader;
#[doc = "Field `PCKRDY2` reader - Programmable Clock Ready 2 Interrupt Mask"]
pub type PCKRDY2_R = crate::BitReader;
#[doc = "Field `MOSCSELS` reader - Main Oscillator Selection Status Interrupt Mask"]
pub type MOSCSELS_R = crate::BitReader;
#[doc = "Field `MOSCRCS` reader - Main On-Chip RC Status Interrupt Mask"]
pub type MOSCRCS_R = crate::BitReader;
#[doc = "Field `CFDEV` reader - Clock Failure Detector Event Interrupt Mask"]
pub type CFDEV_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Main Crystal Oscillator Status Interrupt Mask"]
    #[inline(always)]
    pub fn moscxts(&self) -> MOSCXTS_R {
        MOSCXTS_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - PLLA Lock Interrupt Mask"]
    #[inline(always)]
    pub fn locka(&self) -> LOCKA_R {
        LOCKA_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 3 - Master Clock Ready Interrupt Mask"]
    #[inline(always)]
    pub fn mckrdy(&self) -> MCKRDY_R {
        MCKRDY_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 6 - UTMI PLL Lock Interrupt Mask"]
    #[inline(always)]
    pub fn locku(&self) -> LOCKU_R {
        LOCKU_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 8 - Programmable Clock Ready 0 Interrupt Mask"]
    #[inline(always)]
    pub fn pckrdy0(&self) -> PCKRDY0_R {
        PCKRDY0_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Programmable Clock Ready 1 Interrupt Mask"]
    #[inline(always)]
    pub fn pckrdy1(&self) -> PCKRDY1_R {
        PCKRDY1_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Programmable Clock Ready 2 Interrupt Mask"]
    #[inline(always)]
    pub fn pckrdy2(&self) -> PCKRDY2_R {
        PCKRDY2_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 16 - Main Oscillator Selection Status Interrupt Mask"]
    #[inline(always)]
    pub fn moscsels(&self) -> MOSCSELS_R {
        MOSCSELS_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Main On-Chip RC Status Interrupt Mask"]
    #[inline(always)]
    pub fn moscrcs(&self) -> MOSCRCS_R {
        MOSCRCS_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - Clock Failure Detector Event Interrupt Mask"]
    #[inline(always)]
    pub fn cfdev(&self) -> CFDEV_R {
        CFDEV_R::new(((self.bits >> 18) & 1) != 0)
    }
}
#[doc = "Interrupt Mask Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`pmc_imr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_IMR_SPEC;
impl crate::RegisterSpec for PMC_IMR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`pmc_imr::R`](R) reader structure"]
impl crate::Readable for PMC_IMR_SPEC {}
#[doc = "`reset()` method sets PMC_IMR to value 0"]
impl crate::Resettable for PMC_IMR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
