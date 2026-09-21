#[doc = "Register `CV2` reader"]
pub type R = crate::R<CV2_SPEC>;
#[doc = "Field `CV` reader - Counter Value"]
pub type CV_R = crate::FieldReader<u32>;
impl R {
    #[doc = "Bits 0:31 - Counter Value"]
    #[inline(always)]
    pub fn cv(&self) -> CV_R {
        CV_R::new(self.bits)
    }
}
#[doc = "Counter Value (channel = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cv2::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CV2_SPEC;
impl crate::RegisterSpec for CV2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`cv2::R`](R) reader structure"]
impl crate::Readable for CV2_SPEC {}
#[doc = "`reset()` method sets CV2 to value 0"]
impl crate::Resettable for CV2_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
