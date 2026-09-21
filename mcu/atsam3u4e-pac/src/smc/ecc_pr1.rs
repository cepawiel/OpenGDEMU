#[doc = "Register `ECC_PR1` reader"]
pub type R = crate::R<ECC_PR1_SPEC>;
#[doc = "Field `NPARITY` reader - Parity N"]
pub type NPARITY_R = crate::FieldReader<u16>;
impl R {
    #[doc = "Bits 0:15 - Parity N"]
    #[inline(always)]
    pub fn nparity(&self) -> NPARITY_R {
        NPARITY_R::new((self.bits & 0xffff) as u16)
    }
}
#[doc = "SMC ECC parity 1 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ecc_pr1::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct ECC_PR1_SPEC;
impl crate::RegisterSpec for ECC_PR1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ecc_pr1::R`](R) reader structure"]
impl crate::Readable for ECC_PR1_SPEC {}
#[doc = "`reset()` method sets ECC_PR1 to value 0"]
impl crate::Resettable for ECC_PR1_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
