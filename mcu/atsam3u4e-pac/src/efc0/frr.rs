#[doc = "Register `FRR` reader"]
pub type R = crate::R<FRR_SPEC>;
#[doc = "Field `FVALUE` reader - Flash Result Value"]
pub type FVALUE_R = crate::FieldReader<u32>;
impl R {
    #[doc = "Bits 0:31 - Flash Result Value"]
    #[inline(always)]
    pub fn fvalue(&self) -> FVALUE_R {
        FVALUE_R::new(self.bits)
    }
}
#[doc = "EEFC Flash Result Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`frr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct FRR_SPEC;
impl crate::RegisterSpec for FRR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`frr::R`](R) reader structure"]
impl crate::Readable for FRR_SPEC {}
#[doc = "`reset()` method sets FRR to value 0"]
impl crate::Resettable for FRR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
