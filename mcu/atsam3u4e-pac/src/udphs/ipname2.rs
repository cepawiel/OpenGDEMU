#[doc = "Register `IPNAME2` reader"]
pub type R = crate::R<IPNAME2_SPEC>;
#[doc = "Field `IP_NAME2` reader - "]
pub type IP_NAME2_R = crate::FieldReader<u32>;
impl R {
    #[doc = "Bits 0:31"]
    #[inline(always)]
    pub fn ip_name2(&self) -> IP_NAME2_R {
        IP_NAME2_R::new(self.bits)
    }
}
#[doc = "UDPHS Name2 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ipname2::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IPNAME2_SPEC;
impl crate::RegisterSpec for IPNAME2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ipname2::R`](R) reader structure"]
impl crate::Readable for IPNAME2_SPEC {}
#[doc = "`reset()` method sets IPNAME2 to value 0x3244_4556"]
impl crate::Resettable for IPNAME2_SPEC {
    const RESET_VALUE: Self::Ux = 0x3244_4556;
}
