#[doc = "Register `CKGR_PLLAR` reader"]
pub type R = crate::R<CKGR_PLLAR_SPEC>;
#[doc = "Register `CKGR_PLLAR` writer"]
pub type W = crate::W<CKGR_PLLAR_SPEC>;
#[doc = "Field `DIVA` reader - Divider"]
pub type DIVA_R = crate::FieldReader;
#[doc = "Field `DIVA` writer - Divider"]
pub type DIVA_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
#[doc = "Field `PLLACOUNT` reader - PLLA Counter"]
pub type PLLACOUNT_R = crate::FieldReader;
#[doc = "Field `PLLACOUNT` writer - PLLA Counter"]
pub type PLLACOUNT_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 6, O>;
#[doc = "Field `MULA` reader - PLLA Multiplier"]
pub type MULA_R = crate::FieldReader<u16>;
#[doc = "Field `MULA` writer - PLLA Multiplier"]
pub type MULA_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 11, O, u16>;
#[doc = "Field `ONE` reader - Must Be Set to 1"]
pub type ONE_R = crate::BitReader;
#[doc = "Field `ONE` writer - Must Be Set to 1"]
pub type ONE_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bits 0:7 - Divider"]
    #[inline(always)]
    pub fn diva(&self) -> DIVA_R {
        DIVA_R::new((self.bits & 0xff) as u8)
    }
    #[doc = "Bits 8:13 - PLLA Counter"]
    #[inline(always)]
    pub fn pllacount(&self) -> PLLACOUNT_R {
        PLLACOUNT_R::new(((self.bits >> 8) & 0x3f) as u8)
    }
    #[doc = "Bits 16:26 - PLLA Multiplier"]
    #[inline(always)]
    pub fn mula(&self) -> MULA_R {
        MULA_R::new(((self.bits >> 16) & 0x07ff) as u16)
    }
    #[doc = "Bit 29 - Must Be Set to 1"]
    #[inline(always)]
    pub fn one(&self) -> ONE_R {
        ONE_R::new(((self.bits >> 29) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:7 - Divider"]
    #[inline(always)]
    #[must_use]
    pub fn diva(&mut self) -> DIVA_W<CKGR_PLLAR_SPEC, 0> {
        DIVA_W::new(self)
    }
    #[doc = "Bits 8:13 - PLLA Counter"]
    #[inline(always)]
    #[must_use]
    pub fn pllacount(&mut self) -> PLLACOUNT_W<CKGR_PLLAR_SPEC, 8> {
        PLLACOUNT_W::new(self)
    }
    #[doc = "Bits 16:26 - PLLA Multiplier"]
    #[inline(always)]
    #[must_use]
    pub fn mula(&mut self) -> MULA_W<CKGR_PLLAR_SPEC, 16> {
        MULA_W::new(self)
    }
    #[doc = "Bit 29 - Must Be Set to 1"]
    #[inline(always)]
    #[must_use]
    pub fn one(&mut self) -> ONE_W<CKGR_PLLAR_SPEC, 29> {
        ONE_W::new(self)
    }
    #[doc = r" Writes raw bits to the register."]
    #[doc = r""]
    #[doc = r" # Safety"]
    #[doc = r""]
    #[doc = r" Passing incorrect value can cause undefined behaviour. See reference manual"]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.bits = bits;
        self
    }
}
#[doc = "PLLA Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ckgr_pllar::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ckgr_pllar::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CKGR_PLLAR_SPEC;
impl crate::RegisterSpec for CKGR_PLLAR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ckgr_pllar::R`](R) reader structure"]
impl crate::Readable for CKGR_PLLAR_SPEC {}
#[doc = "`write(|w| ..)` method takes [`ckgr_pllar::W`](W) writer structure"]
impl crate::Writable for CKGR_PLLAR_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CKGR_PLLAR to value 0x3f00"]
impl crate::Resettable for CKGR_PLLAR_SPEC {
    const RESET_VALUE: Self::Ux = 0x3f00;
}
