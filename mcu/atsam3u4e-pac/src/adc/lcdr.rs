#[doc = "Register `LCDR` reader"]
pub type R = crate::R<LCDR_SPEC>;
#[doc = "Field `LDATA` reader - Last Data Converted"]
pub type LDATA_R = crate::FieldReader<u16>;
impl R {
    #[doc = "Bits 0:9 - Last Data Converted"]
    #[inline(always)]
    pub fn ldata(&self) -> LDATA_R {
        LDATA_R::new((self.bits & 0x03ff) as u16)
    }
}
#[doc = "Last Converted Data Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`lcdr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct LCDR_SPEC;
impl crate::RegisterSpec for LCDR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`lcdr::R`](R) reader structure"]
impl crate::Readable for LCDR_SPEC {}
#[doc = "`reset()` method sets LCDR to value 0"]
impl crate::Resettable for LCDR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
