#[doc = "Register `PMC_WPSR` reader"]
pub type R = crate::R<PMC_WPSR_SPEC>;
#[doc = "Field `WPVS` reader - Write Protect Violation Status"]
pub type WPVS_R = crate::BitReader;
#[doc = "Field `WPVSRC` reader - Write Protect Violation Source"]
pub type WPVSRC_R = crate::FieldReader<u16>;
impl R {
    #[doc = "Bit 0 - Write Protect Violation Status"]
    #[inline(always)]
    pub fn wpvs(&self) -> WPVS_R {
        WPVS_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 8:23 - Write Protect Violation Source"]
    #[inline(always)]
    pub fn wpvsrc(&self) -> WPVSRC_R {
        WPVSRC_R::new(((self.bits >> 8) & 0xffff) as u16)
    }
}
#[doc = "Write Protect Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`pmc_wpsr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct PMC_WPSR_SPEC;
impl crate::RegisterSpec for PMC_WPSR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`pmc_wpsr::R`](R) reader structure"]
impl crate::Readable for PMC_WPSR_SPEC {}
#[doc = "`reset()` method sets PMC_WPSR to value 0"]
impl crate::Resettable for PMC_WPSR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
