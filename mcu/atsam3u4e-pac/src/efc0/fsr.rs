#[doc = "Register `FSR` reader"]
pub type R = crate::R<FSR_SPEC>;
#[doc = "Field `FRDY` reader - Flash Ready Status"]
pub type FRDY_R = crate::BitReader;
#[doc = "Field `FCMDE` reader - Flash Command Error Status"]
pub type FCMDE_R = crate::BitReader;
#[doc = "Field `FLOCKE` reader - Flash Lock Error Status"]
pub type FLOCKE_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Flash Ready Status"]
    #[inline(always)]
    pub fn frdy(&self) -> FRDY_R {
        FRDY_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Flash Command Error Status"]
    #[inline(always)]
    pub fn fcmde(&self) -> FCMDE_R {
        FCMDE_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Flash Lock Error Status"]
    #[inline(always)]
    pub fn flocke(&self) -> FLOCKE_R {
        FLOCKE_R::new(((self.bits >> 2) & 1) != 0)
    }
}
#[doc = "EEFC Flash Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fsr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct FSR_SPEC;
impl crate::RegisterSpec for FSR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`fsr::R`](R) reader structure"]
impl crate::Readable for FSR_SPEC {}
#[doc = "`reset()` method sets FSR to value 0x01"]
impl crate::Resettable for FSR_SPEC {
    const RESET_VALUE: Self::Ux = 0x01;
}
