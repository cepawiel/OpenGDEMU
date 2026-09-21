#[doc = "Register `WPSR` reader"]
pub type R = crate::R<WPSR_SPEC>;
#[doc = "Field `WP_VS` reader - Write Protection Violation Status"]
pub type WP_VS_R = crate::FieldReader<WP_VS_A>;
#[doc = "Write Protection Violation Status"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WP_VS_A {
    #[doc = "0: No Write Protection Violation occurred since the last read of this register (WP_SR)"]
    NONE = 0,
    #[doc = "1: Write Protection detected unauthorized attempt to write a control register had occurred (since the last read.)"]
    WRITE = 1,
    #[doc = "2: Software reset had been performed while Write Protection was enabled (since the last read)."]
    RESET = 2,
    #[doc = "3: Both Write Protection violation and software reset with Write Protection enabled have occurred since the last read."]
    BOTH = 3,
}
impl From<WP_VS_A> for u8 {
    #[inline(always)]
    fn from(variant: WP_VS_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for WP_VS_A {
    type Ux = u8;
}
impl WP_VS_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<WP_VS_A> {
        match self.bits {
            0 => Some(WP_VS_A::NONE),
            1 => Some(WP_VS_A::WRITE),
            2 => Some(WP_VS_A::RESET),
            3 => Some(WP_VS_A::BOTH),
            _ => None,
        }
    }
    #[doc = "No Write Protection Violation occurred since the last read of this register (WP_SR)"]
    #[inline(always)]
    pub fn is_none(&self) -> bool {
        *self == WP_VS_A::NONE
    }
    #[doc = "Write Protection detected unauthorized attempt to write a control register had occurred (since the last read.)"]
    #[inline(always)]
    pub fn is_write(&self) -> bool {
        *self == WP_VS_A::WRITE
    }
    #[doc = "Software reset had been performed while Write Protection was enabled (since the last read)."]
    #[inline(always)]
    pub fn is_reset(&self) -> bool {
        *self == WP_VS_A::RESET
    }
    #[doc = "Both Write Protection violation and software reset with Write Protection enabled have occurred since the last read."]
    #[inline(always)]
    pub fn is_both(&self) -> bool {
        *self == WP_VS_A::BOTH
    }
}
#[doc = "Field `WP_VSRC` reader - Write Protection Violation SouRCe"]
pub type WP_VSRC_R = crate::FieldReader<u16>;
impl R {
    #[doc = "Bits 0:3 - Write Protection Violation Status"]
    #[inline(always)]
    pub fn wp_vs(&self) -> WP_VS_R {
        WP_VS_R::new((self.bits & 0x0f) as u8)
    }
    #[doc = "Bits 8:23 - Write Protection Violation SouRCe"]
    #[inline(always)]
    pub fn wp_vsrc(&self) -> WP_VSRC_R {
        WP_VSRC_R::new(((self.bits >> 8) & 0xffff) as u16)
    }
}
#[doc = "Write Protection Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wpsr::R`](R).  See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct WPSR_SPEC;
impl crate::RegisterSpec for WPSR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`wpsr::R`](R) reader structure"]
impl crate::Readable for WPSR_SPEC {}
