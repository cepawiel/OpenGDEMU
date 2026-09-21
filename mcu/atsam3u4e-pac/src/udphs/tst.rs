#[doc = "Register `TST` reader"]
pub type R = crate::R<TST_SPEC>;
#[doc = "Register `TST` writer"]
pub type W = crate::W<TST_SPEC>;
#[doc = "Field `SPEED_CFG` reader - Speed Configuration"]
pub type SPEED_CFG_R = crate::FieldReader<SPEED_CFG_A>;
#[doc = "Speed Configuration\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SPEED_CFG_A {
    #[doc = "0: Normal Mode: The macro is in Full Speed mode, ready to make a High Speed identification, if the host supports it and then to automatically switch to High Speed mode"]
    NORMAL = 0,
    #[doc = "2: Force High Speed: Set this value to force the hardware to work in High Speed mode. Only for debug or test purpose."]
    HIGH_SPEED = 2,
    #[doc = "3: Force Full Speed: Set this value to force the hardware to work only in Full Speed mode. In this configuration, the macro will not respond to a High Speed reset handshake."]
    FULL_SPEED = 3,
}
impl From<SPEED_CFG_A> for u8 {
    #[inline(always)]
    fn from(variant: SPEED_CFG_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for SPEED_CFG_A {
    type Ux = u8;
}
impl SPEED_CFG_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<SPEED_CFG_A> {
        match self.bits {
            0 => Some(SPEED_CFG_A::NORMAL),
            2 => Some(SPEED_CFG_A::HIGH_SPEED),
            3 => Some(SPEED_CFG_A::FULL_SPEED),
            _ => None,
        }
    }
    #[doc = "Normal Mode: The macro is in Full Speed mode, ready to make a High Speed identification, if the host supports it and then to automatically switch to High Speed mode"]
    #[inline(always)]
    pub fn is_normal(&self) -> bool {
        *self == SPEED_CFG_A::NORMAL
    }
    #[doc = "Force High Speed: Set this value to force the hardware to work in High Speed mode. Only for debug or test purpose."]
    #[inline(always)]
    pub fn is_high_speed(&self) -> bool {
        *self == SPEED_CFG_A::HIGH_SPEED
    }
    #[doc = "Force Full Speed: Set this value to force the hardware to work only in Full Speed mode. In this configuration, the macro will not respond to a High Speed reset handshake."]
    #[inline(always)]
    pub fn is_full_speed(&self) -> bool {
        *self == SPEED_CFG_A::FULL_SPEED
    }
}
#[doc = "Field `SPEED_CFG` writer - Speed Configuration"]
pub type SPEED_CFG_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 2, O, SPEED_CFG_A>;
impl<'a, REG, const O: u8> SPEED_CFG_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Normal Mode: The macro is in Full Speed mode, ready to make a High Speed identification, if the host supports it and then to automatically switch to High Speed mode"]
    #[inline(always)]
    pub fn normal(self) -> &'a mut crate::W<REG> {
        self.variant(SPEED_CFG_A::NORMAL)
    }
    #[doc = "Force High Speed: Set this value to force the hardware to work in High Speed mode. Only for debug or test purpose."]
    #[inline(always)]
    pub fn high_speed(self) -> &'a mut crate::W<REG> {
        self.variant(SPEED_CFG_A::HIGH_SPEED)
    }
    #[doc = "Force Full Speed: Set this value to force the hardware to work only in Full Speed mode. In this configuration, the macro will not respond to a High Speed reset handshake."]
    #[inline(always)]
    pub fn full_speed(self) -> &'a mut crate::W<REG> {
        self.variant(SPEED_CFG_A::FULL_SPEED)
    }
}
#[doc = "Field `TST_J` reader - Test J Mode"]
pub type TST_J_R = crate::BitReader;
#[doc = "Field `TST_J` writer - Test J Mode"]
pub type TST_J_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TST_K` reader - Test K Mode"]
pub type TST_K_R = crate::BitReader;
#[doc = "Field `TST_K` writer - Test K Mode"]
pub type TST_K_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TST_PKT` reader - Test Packet Mode"]
pub type TST_PKT_R = crate::BitReader;
#[doc = "Field `TST_PKT` writer - Test Packet Mode"]
pub type TST_PKT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `OPMODE2` reader - OpMode2"]
pub type OPMODE2_R = crate::BitReader;
#[doc = "Field `OPMODE2` writer - OpMode2"]
pub type OPMODE2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bits 0:1 - Speed Configuration"]
    #[inline(always)]
    pub fn speed_cfg(&self) -> SPEED_CFG_R {
        SPEED_CFG_R::new((self.bits & 3) as u8)
    }
    #[doc = "Bit 2 - Test J Mode"]
    #[inline(always)]
    pub fn tst_j(&self) -> TST_J_R {
        TST_J_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Test K Mode"]
    #[inline(always)]
    pub fn tst_k(&self) -> TST_K_R {
        TST_K_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 4 - Test Packet Mode"]
    #[inline(always)]
    pub fn tst_pkt(&self) -> TST_PKT_R {
        TST_PKT_R::new(((self.bits >> 4) & 1) != 0)
    }
    #[doc = "Bit 5 - OpMode2"]
    #[inline(always)]
    pub fn opmode2(&self) -> OPMODE2_R {
        OPMODE2_R::new(((self.bits >> 5) & 1) != 0)
    }
}
impl W {
    #[doc = "Bits 0:1 - Speed Configuration"]
    #[inline(always)]
    #[must_use]
    pub fn speed_cfg(&mut self) -> SPEED_CFG_W<TST_SPEC, 0> {
        SPEED_CFG_W::new(self)
    }
    #[doc = "Bit 2 - Test J Mode"]
    #[inline(always)]
    #[must_use]
    pub fn tst_j(&mut self) -> TST_J_W<TST_SPEC, 2> {
        TST_J_W::new(self)
    }
    #[doc = "Bit 3 - Test K Mode"]
    #[inline(always)]
    #[must_use]
    pub fn tst_k(&mut self) -> TST_K_W<TST_SPEC, 3> {
        TST_K_W::new(self)
    }
    #[doc = "Bit 4 - Test Packet Mode"]
    #[inline(always)]
    #[must_use]
    pub fn tst_pkt(&mut self) -> TST_PKT_W<TST_SPEC, 4> {
        TST_PKT_W::new(self)
    }
    #[doc = "Bit 5 - OpMode2"]
    #[inline(always)]
    #[must_use]
    pub fn opmode2(&mut self) -> OPMODE2_W<TST_SPEC, 5> {
        OPMODE2_W::new(self)
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
#[doc = "UDPHS Test Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tst::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tst::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct TST_SPEC;
impl crate::RegisterSpec for TST_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`tst::R`](R) reader structure"]
impl crate::Readable for TST_SPEC {}
#[doc = "`write(|w| ..)` method takes [`tst::W`](W) writer structure"]
impl crate::Writable for TST_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets TST to value 0"]
impl crate::Resettable for TST_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
