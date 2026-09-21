#[doc = "Register `CV1` reader"]
pub type R = crate::R<CV1_SPEC>;
#[doc = "Field `CV` reader - Counter Value"]
pub type CV_R = crate::FieldReader<u32>;
impl R {
    #[doc = "Bits 0:31 - Counter Value"]
    #[inline(always)]
    pub fn cv(&self) -> CV_R {
        CV_R::new(self.bits)
    }
}
#[doc = "Counter Value (channel = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cv1::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CV1_SPEC;
impl crate::RegisterSpec for CV1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`cv1::R`](R) reader structure"]
impl crate::Readable for CV1_SPEC {}
#[doc = "`reset()` method sets CV1 to value 0"]
impl crate::Resettable for CV1_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
