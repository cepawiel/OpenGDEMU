#[doc = "Register `EPTSETSTA2` writer"]
pub type W = crate::W<EPTSETSTA2_SPEC>;
#[doc = "Field `FRCESTALL` writer - Stall Handshake Request Set"]
pub type FRCESTALL_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `KILL_BANK` writer - KILL Bank Set (for IN Endpoint)"]
pub type KILL_BANK_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TX_PK_RDY` writer - TX Packet Ready Set"]
pub type TX_PK_RDY_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl W {
    #[doc = "Bit 5 - Stall Handshake Request Set"]
    #[inline(always)]
    #[must_use]
    pub fn frcestall(&mut self) -> FRCESTALL_W<EPTSETSTA2_SPEC, 5> {
        FRCESTALL_W::new(self)
    }
    #[doc = "Bit 9 - KILL Bank Set (for IN Endpoint)"]
    #[inline(always)]
    #[must_use]
    pub fn kill_bank(&mut self) -> KILL_BANK_W<EPTSETSTA2_SPEC, 9> {
        KILL_BANK_W::new(self)
    }
    #[doc = "Bit 11 - TX Packet Ready Set"]
    #[inline(always)]
    #[must_use]
    pub fn tx_pk_rdy(&mut self) -> TX_PK_RDY_W<EPTSETSTA2_SPEC, 11> {
        TX_PK_RDY_W::new(self)
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
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta2::W`](W). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct EPTSETSTA2_SPEC;
impl crate::RegisterSpec for EPTSETSTA2_SPEC {
    type Ux = u32;
}
#[doc = "`write(|w| ..)` method takes [`eptsetsta2::W`](W) writer structure"]
impl crate::Writable for EPTSETSTA2_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
