#[doc = "Register `SR0` reader"]
pub type R = crate::R<SR0_SPEC>;
#[doc = "Field `COVFS` reader - Counter Overflow Status"]
pub type COVFS_R = crate::BitReader;
#[doc = "Field `LOVRS` reader - Load Overrun Status"]
pub type LOVRS_R = crate::BitReader;
#[doc = "Field `CPAS` reader - RA Compare Status"]
pub type CPAS_R = crate::BitReader;
#[doc = "Field `CPBS` reader - RB Compare Status"]
pub type CPBS_R = crate::BitReader;
#[doc = "Field `CPCS` reader - RC Compare Status"]
pub type CPCS_R = crate::BitReader;
#[doc = "Field `LDRAS` reader - RA Loading Status"]
pub type LDRAS_R = crate::BitReader;
#[doc = "Field `LDRBS` reader - RB Loading Status"]
pub type LDRBS_R = crate::BitReader;
#[doc = "Field `ETRGS` reader - External Trigger Status"]
pub type ETRGS_R = crate::BitReader;
#[doc = "Field `CLKSTA` reader - Clock Enabling Status"]
pub type CLKSTA_R = crate::BitReader;
#[doc = "Field `MTIOA` reader - TIOA Mirror"]
pub type MTIOA_R = crate::BitReader;
#[doc = "Field `MTIOB` reader - TIOB Mirror"]
pub type MTIOB_R = crate::BitReader;
impl R {
    #[doc = "Bit 0 - Counter Overflow Status"]
    #[inline(always)]
    pub fn covfs(&self) -> COVFS_R {
        COVFS_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Load Overrun Status"]
    #[inline(always)]
    pub fn lovrs(&self) -> LOVRS_R {
        LOVRS_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - RA Compare Status"]
    #[inline(always)]
    pub fn cpas(&self) -> CPAS_R {
        CPAS_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - RB Compare Status"]
    #[inline(always)]
    pub fn cpbs(&self) -> CPBS_R {
        CPBS_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - RC Compare Status"]
    #[inline(always)]
    pub fn cpcs(&self) -> CPCS_R {
        CPCS_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - RA Loading Status"]
    #[inline(always)]
    pub fn ldras(&self) -> LDRAS_R {
        LDRAS_R::new(((self.bits >> 5) & 1) != 0)
    }
    #[doc = "Bit 6 - RB Loading Status"]
    #[inline(always)]
    pub fn ldrbs(&self) -> LDRBS_R {
        LDRBS_R::new(((self.bits >> 6) & 1) != 0)
    }
    #[doc = "Bit 7 - External Trigger Status"]
    #[inline(always)]
    pub fn etrgs(&self) -> ETRGS_R {
        ETRGS_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 16 - Clock Enabling Status"]
    #[inline(always)]
    pub fn clksta(&self) -> CLKSTA_R {
        CLKSTA_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - TIOA Mirror"]
    #[inline(always)]
    pub fn mtioa(&self) -> MTIOA_R {
        MTIOA_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - TIOB Mirror"]
    #[inline(always)]
    pub fn mtiob(&self) -> MTIOB_R {
        MTIOB_R::new(((self.bits >> 18) & 1) != 0)
    }
}
#[doc = "Status Register (channel = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sr0::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct SR0_SPEC;
impl crate::RegisterSpec for SR0_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`sr0::R`](R) reader structure"]
impl crate::Readable for SR0_SPEC {}
#[doc = "`reset()` method sets SR0 to value 0"]
impl crate::Resettable for SR0_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
