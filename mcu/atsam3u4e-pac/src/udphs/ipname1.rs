#[doc = "Register `IPNAME1` reader"]
pub type R = crate::R<IPNAME1_SPEC>;
#[doc = "Field `IP_NAME1` reader - "]
pub type IP_NAME1_R = crate::FieldReader<u32>;
impl R {
    #[doc = "Bits 0:31"]
    #[inline(always)]
    pub fn ip_name1(&self) -> IP_NAME1_R {
        IP_NAME1_R::new(self.bits)
    }
}
#[doc = "UDPHS Name1 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ipname1::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IPNAME1_SPEC;
impl crate::RegisterSpec for IPNAME1_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ipname1::R`](R) reader structure"]
impl crate::Readable for IPNAME1_SPEC {}
#[doc = "`reset()` method sets IPNAME1 to value 0x4855_5342"]
impl crate::Resettable for IPNAME1_SPEC {
    const RESET_VALUE: Self::Ux = 0x4855_5342;
}
