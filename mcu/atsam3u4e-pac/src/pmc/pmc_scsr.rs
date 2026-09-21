#[doc = "Register `PMC_SCSR` reader"]
pub type R = crate::R<PMC_SCSR_SPEC>;
#[doc = "Field `PCK0` reader - Programmable Clock 0 Output Status"]
pub type PCK0_R = crate::BitReader;
#[doc = "Field `PCK1` reader - Programmable Clock 1 Output Status"]
pub type PCK1_R = crate::BitReader;
#[doc = "Field `PCK2` reader - Programmable Clock 2 Output Status"]
pub type PCK2_R = crate::BitReader;
impl R {
    #[doc = "Bit 8 - Programmable Clock 0 Output Status"]
    #[inline(always)]
    pub fn pck0(&self) -> PCK0_R {
        PCK0_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Programmable Clock 1 Output Status"]
    #[inline(always)]
    pub fn pck1(&self) -> PCK1_R {
        PCK1_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Programmable Clock 2 Output Status"]
    #[inline(always)]
    pub fn pck2(&self) -> PCK2_R {
        PCK2_R::new(((self.bits >> 10) & 1) != 0)
    }
}
#[doc = "System Clock Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`pmc_scsr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_SCSR_SPEC;
impl crate::RegisterSpec for PMC_SCSR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`pmc_scsr::R`](R) reader structure"]
impl crate::Readable for PMC_SCSR_SPEC {}
#[doc = "`reset()` method sets PMC_SCSR to value 0x01"]
impl crate::Resettable for PMC_SCSR_SPEC {
    const RESET_VALUE: Self::Ux = 0x01;
}
