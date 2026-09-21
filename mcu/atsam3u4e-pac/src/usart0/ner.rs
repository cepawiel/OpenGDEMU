#[doc = "Register `NER` reader"]
pub type R = crate::R<NER_SPEC>;
#[doc = "Field `NB_ERRORS` reader - Number of Errors"]
pub type NB_ERRORS_R = crate::FieldReader;
impl R {
    #[doc = "Bits 0:7 - Number of Errors"]
    #[inline(always)]
    pub fn nb_errors(&self) -> NB_ERRORS_R {
        NB_ERRORS_R::new((self.bits & 0xff) as u8)
    }
}
#[doc = "Number of Errors Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ner::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct NER_SPEC;
impl crate::RegisterSpec for NER_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ner::R`](R) reader structure"]
impl crate::Readable for NER_SPEC {}
