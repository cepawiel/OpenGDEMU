#[doc = "Register `ECC_PR0` reader"]
pub type R = crate::R<ECC_PR0_SPEC>;
#[doc = "Field `BITADDR` reader - Bit Address"]
pub type BITADDR_R = crate::FieldReader;
#[doc = "Field `WORDADDR` reader - Word Address"]
pub type WORDADDR_R = crate::FieldReader<u16>;
impl R {
    #[doc = "Bits 0:3 - Bit Address"]
    #[inline(always)]
    pub fn bitaddr(&self) -> BITADDR_R {
        BITADDR_R::new((self.bits & 0x0f) as u8)
    }
    #[doc = "Bits 4:15 - Word Address"]
    #[inline(always)]
    pub fn wordaddr(&self) -> WORDADDR_R {
        WORDADDR_R::new(((self.bits >> 4) & 0x0fff) as u16)
    }
}
#[doc = "SMC ECC Parity 0 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ecc_pr0::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct ECC_PR0_SPEC;
impl crate::RegisterSpec for ECC_PR0_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ecc_pr0::R`](R) reader structure"]
impl crate::Readable for ECC_PR0_SPEC {}
#[doc = "`reset()` method sets ECC_PR0 to value 0"]
impl crate::Resettable for ECC_PR0_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
