#[doc = "Register `CLK` reader"]
pub type R = crate::R<CLK_SPEC>;
#[doc = "Register `CLK` writer"]
pub type W = crate::W<CLK_SPEC>;
#[doc = "Field `DIVA` reader - CLKA, CLKB Divide Factor"]
pub type DIVA_R = crate::FieldReader;
#[doc = "Field `DIVA` writer - CLKA, CLKB Divide Factor"]
pub type DIVA_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
#[doc = "Field `PREA` reader - CLKA, CLKB Source Clock Selection"]
pub type PREA_R = crate::FieldReader;
#[doc = "Field `PREA` writer - CLKA, CLKB Source Clock Selection"]
pub type PREA_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
#[doc = "Field `DIVB` reader - CLKA, CLKB Divide Factor"]
pub type DIVB_R = crate::FieldReader;
#[doc = "Field `DIVB` writer - CLKA, CLKB Divide Factor"]
pub type DIVB_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 8, O>;
#[doc = "Field `PREB` reader - CLKA, CLKB Source Clock Selection"]
pub type PREB_R = crate::FieldReader;
#[doc = "Field `PREB` writer - CLKA, CLKB Source Clock Selection"]
pub type PREB_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 4, O>;
impl R {
    #[doc = "Bits 0:7 - CLKA, CLKB Divide Factor"]
    #[inline(always)]
    pub fn diva(&self) -> DIVA_R {
        DIVA_R::new((self.bits & 0xff) as u8)
    }
    #[doc = "Bits 8:11 - CLKA, CLKB Source Clock Selection"]
    #[inline(always)]
    pub fn prea(&self) -> PREA_R {
        PREA_R::new(((self.bits >> 8) & 0x0f) as u8)
    }
    #[doc = "Bits 16:23 - CLKA, CLKB Divide Factor"]
    #[inline(always)]
    pub fn divb(&self) -> DIVB_R {
        DIVB_R::new(((self.bits >> 16) & 0xff) as u8)
    }
    #[doc = "Bits 24:27 - CLKA, CLKB Source Clock Selection"]
    #[inline(always)]
    pub fn preb(&self) -> PREB_R {
        PREB_R::new(((self.bits >> 24) & 0x0f) as u8)
    }
}
impl W {
    #[doc = "Bits 0:7 - CLKA, CLKB Divide Factor"]
    #[inline(always)]
    #[must_use]
    pub fn diva(&mut self) -> DIVA_W<CLK_SPEC, 0> {
        DIVA_W::new(self)
    }
    #[doc = "Bits 8:11 - CLKA, CLKB Source Clock Selection"]
    #[inline(always)]
    #[must_use]
    pub fn prea(&mut self) -> PREA_W<CLK_SPEC, 8> {
        PREA_W::new(self)
    }
    #[doc = "Bits 16:23 - CLKA, CLKB Divide Factor"]
    #[inline(always)]
    #[must_use]
    pub fn divb(&mut self) -> DIVB_W<CLK_SPEC, 16> {
        DIVB_W::new(self)
    }
    #[doc = "Bits 24:27 - CLKA, CLKB Source Clock Selection"]
    #[inline(always)]
    #[must_use]
    pub fn preb(&mut self) -> PREB_W<CLK_SPEC, 24> {
        PREB_W::new(self)
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
#[doc = "PWM Clock Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`clk::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`clk::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CLK_SPEC;
impl crate::RegisterSpec for CLK_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`clk::R`](R) reader structure"]
impl crate::Readable for CLK_SPEC {}
#[doc = "`write(|w| ..)` method takes [`clk::W`](W) writer structure"]
impl crate::Writable for CLK_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CLK to value 0"]
impl crate::Resettable for CLK_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
