#[doc = "Register `CCNT0` reader"]
pub type R = crate::R<CCNT0_SPEC>;
#[doc = "Field `CNT` reader - Channel Counter Register"]
pub type CNT_R = crate::FieldReader<u32>;
impl R {
    #[doc = "Bits 0:23 - Channel Counter Register"]
    #[inline(always)]
    pub fn cnt(&self) -> CNT_R {
        CNT_R::new(self.bits & 0x00ff_ffff)
    }
}
#[doc = "PWM Channel Counter Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ccnt0::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CCNT0_SPEC;
impl crate::RegisterSpec for CCNT0_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ccnt0::R`](R) reader structure"]
impl crate::Readable for CCNT0_SPEC {}
#[doc = "`reset()` method sets CCNT0 to value 0"]
impl crate::Resettable for CCNT0_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
