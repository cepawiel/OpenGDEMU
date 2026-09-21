#[doc = "Register `QISR` reader"]
pub type R = crate::R<QISR_SPEC>;
#[doc = "Field `IDX` reader - InDeX"]
pub type IDX_R = crate::BitReader;
#[doc = "Field `DIRCHG` reader - DIRection CHanGe"]
pub type DIRCHG_R = crate::BitReader;
#[doc = "Field `QERR` reader - Quadrature ERRor"]
pub type QERR_R = crate::BitReader;
#[doc = "Field `DIR` reader - Direction"]
pub type DIR_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - InDeX"]
    #[inline(always)]
    pub fn idx(&self) -> IDX_R {
        IDX_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - DIRection CHanGe"]
    #[inline(always)]
    pub fn dirchg(&self) -> DIRCHG_R {
        DIRCHG_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Quadrature ERRor"]
    #[inline(always)]
    pub fn qerr(&self) -> QERR_R {
        QERR_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 8 - Direction"]
    #[inline(always)]
    pub fn dir(&self) -> DIR_R {
        DIR_R::new(((self.bits >> 8) & 1) != 0)
    }
}
#[doc = "QDEC Interrupt Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`qisr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct QISR_SPEC;
impl crate::RegisterSpec for QISR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`qisr::R`](R) reader structure"]
impl crate::Readable for QISR_SPEC {}
#[doc = "`reset()` method sets QISR to value 0"]
impl crate::Resettable for QISR_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
