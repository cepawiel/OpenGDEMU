#[doc = "Register `CTRL` reader"]
pub type R = crate::R<CTRL_SPEC>;
#[doc = "Register `CTRL` writer"]
pub type W = crate::W<CTRL_SPEC>;
#[doc = "Field `DEV_ADDR` reader - UDPHS Address"]
pub type DEV_ADDR_R = crate::FieldReader;
#[doc = "Field `DEV_ADDR` writer - UDPHS Address"]
pub type DEV_ADDR_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 7, O>;
#[doc = "Field `FADDR_EN` reader - Function Address Enable"]
pub type FADDR_EN_R = crate::BitReader;
#[doc = "Field `FADDR_EN` writer - Function Address Enable"]
pub type FADDR_EN_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `EN_UDPHS` reader - UDPHS Enable"]
pub type EN_UDPHS_R = crate::BitReader;
#[doc = "Field `EN_UDPHS` writer - UDPHS Enable"]
pub type EN_UDPHS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DETACH` reader - Detach Command"]
pub type DETACH_R = crate::BitReader;
#[doc = "Field `DETACH` writer - Detach Command"]
pub type DETACH_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `REWAKEUP` reader - Send Remote Wake Up"]
pub type REWAKEUP_R = crate::BitReader;
#[doc = "Field `REWAKEUP` writer - Send Remote Wake Up"]
pub type REWAKEUP_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `PULLD_DIS` reader - Pull-Down Disable"]
pub type PULLD_DIS_R = crate::BitReader;
#[doc = "Field `PULLD_DIS` writer - Pull-Down Disable"]
pub type PULLD_DIS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bits 0:6 - UDPHS Address"]
    #[inline(always)]
    pub fn dev_addr(&self) -> DEV_ADDR_R {
        DEV_ADDR_R::new((self.bits & 0x7f) as u8)
    }
    #[doc = "Bit 7 - Function Address Enable"]
    #[inline(always)]
    pub fn faddr_en(&self) -> FADDR_EN_R {
        FADDR_EN_R::new(((self.bits >> 7) & 1) != 0)
    }
    #[doc = "Bit 8 - UDPHS Enable"]
    #[inline(always)]
    pub fn en_udphs(&self) -> EN_UDPHS_R {
        EN_UDPHS_R::new(((self.bits >> 8) & 1) != 0)
    }
    #[doc = "Bit 9 - Detach Command"]
    #[inline(always)]
    pub fn detach(&self) -> DETACH_R {
        DETACH_R::new(((self.bits >> 9) & 1) != 0)
    }
    #[doc = "Bit 10 - Send Remote Wake Up"]
    #[inline(always)]
    pub fn rewakeup(&self) -> REWAKEUP_R {
        REWAKEUP_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Pull-Down Disable"]
    #[inline(always)]
    pub fn pulld_dis(&self) -> PULLD_DIS_R {
        PULLD_DIS_R::new(((self.bits >> 11) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:6 - UDPHS Address"]
    #[inline(always)]
    #[must_use]
    pub fn dev_addr(&mut self) -> DEV_ADDR_W<CTRL_SPEC, 0> {
        DEV_ADDR_W::new(self)
    }
    #[doc = "Bit 7 - Function Address Enable"]
    #[inline(always)]
    #[must_use]
    pub fn faddr_en(&mut self) -> FADDR_EN_W<CTRL_SPEC, 7> {
        FADDR_EN_W::new(self)
    }
    #[doc = "Bit 8 - UDPHS Enable"]
    #[inline(always)]
    #[must_use]
    pub fn en_udphs(&mut self) -> EN_UDPHS_W<CTRL_SPEC, 8> {
        EN_UDPHS_W::new(self)
    }
    #[doc = "Bit 9 - Detach Command"]
    #[inline(always)]
    #[must_use]
    pub fn detach(&mut self) -> DETACH_W<CTRL_SPEC, 9> {
        DETACH_W::new(self)
    }
    #[doc = "Bit 10 - Send Remote Wake Up"]
    #[inline(always)]
    #[must_use]
    pub fn rewakeup(&mut self) -> REWAKEUP_W<CTRL_SPEC, 10> {
        REWAKEUP_W::new(self)
    }
    #[doc = "Bit 11 - Pull-Down Disable"]
    #[inline(always)]
    #[must_use]
    pub fn pulld_dis(&mut self) -> PULLD_DIS_W<CTRL_SPEC, 11> {
        PULLD_DIS_W::new(self)
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
#[doc = "UDPHS Control Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrl::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrl::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CTRL_SPEC;
impl crate::RegisterSpec for CTRL_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ctrl::R`](R) reader structure"]
impl crate::Readable for CTRL_SPEC {}
#[doc = "`write(|w| ..)` method takes [`ctrl::W`](W) writer structure"]
impl crate::Writable for CTRL_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CTRL to value 0x0200"]
impl crate::Resettable for CTRL_SPEC {
    const RESET_VALUE: Self::Ux = 0x0200;
}
