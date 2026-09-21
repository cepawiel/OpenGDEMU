#[doc = "Register `ECC_MD` reader"]
pub type R = crate::R<ECC_MD_SPEC>;
#[doc = "Register `ECC_MD` writer"]
pub type W = crate::W<ECC_MD_SPEC>;
#[doc = "Field `ECC_PAGESIZE` reader - ECC Page Size"]
pub type ECC_PAGESIZE_R = crate::FieldReader<ECC_PAGESIZE_A>;
#[doc = "ECC Page Size\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ECC_PAGESIZE_A {
    #[doc = "0: Main area 512 Bytes + Spare area 16 Bytes = 528 Bytes"]
    PS512_16 = 0,
    #[doc = "1: Main area 1024 Bytes + Spare area 32 Bytes = 1056 Bytes"]
    PS1024_32 = 1,
    #[doc = "2: Main area 2048 Bytes + Spare area 64 Bytes = 2112 Bytes"]
    PS2048_64 = 2,
    #[doc = "3: Main area 4096 Bytes + Spare area 128 Bytes = 4224 Bytes"]
    PS4096_128 = 3,
}
impl From<ECC_PAGESIZE_A> for u8 {
    #[inline(always)]
    fn from(variant: ECC_PAGESIZE_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for ECC_PAGESIZE_A {
    type Ux = u8;
}
impl ECC_PAGESIZE_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ECC_PAGESIZE_A {
        match self.bits {
            0 => ECC_PAGESIZE_A::PS512_16,
            1 => ECC_PAGESIZE_A::PS1024_32,
            2 => ECC_PAGESIZE_A::PS2048_64,
            3 => ECC_PAGESIZE_A::PS4096_128,
            _ => unreachable!(),
        }
    }
    #[doc = "Main area 512 Bytes + Spare area 16 Bytes = 528 Bytes"]
    #[inline(always)]
    pub fn is_ps512_16(&self) -> bool {
        *self == ECC_PAGESIZE_A::PS512_16
    }
    #[doc = "Main area 1024 Bytes + Spare area 32 Bytes = 1056 Bytes"]
    #[inline(always)]
    pub fn is_ps1024_32(&self) -> bool {
        *self == ECC_PAGESIZE_A::PS1024_32
    }
    #[doc = "Main area 2048 Bytes + Spare area 64 Bytes = 2112 Bytes"]
    #[inline(always)]
    pub fn is_ps2048_64(&self) -> bool {
        *self == ECC_PAGESIZE_A::PS2048_64
    }
    #[doc = "Main area 4096 Bytes + Spare area 128 Bytes = 4224 Bytes"]
    #[inline(always)]
    pub fn is_ps4096_128(&self) -> bool {
        *self == ECC_PAGESIZE_A::PS4096_128
    }
}
#[doc = "Field `ECC_PAGESIZE` writer - ECC Page Size"]
pub type ECC_PAGESIZE_W<'a, REG, const O: u8> =
    crate::FieldWriterSafe<'a, REG, 2, O, ECC_PAGESIZE_A>;
impl<'a, REG, const O: u8> ECC_PAGESIZE_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "Main area 512 Bytes + Spare area 16 Bytes = 528 Bytes"]
    #[inline(always)]
    pub fn ps512_16(self) -> &'a mut crate::W<REG> {
        self.variant(ECC_PAGESIZE_A::PS512_16)
    }
    #[doc = "Main area 1024 Bytes + Spare area 32 Bytes = 1056 Bytes"]
    #[inline(always)]
    pub fn ps1024_32(self) -> &'a mut crate::W<REG> {
        self.variant(ECC_PAGESIZE_A::PS1024_32)
    }
    #[doc = "Main area 2048 Bytes + Spare area 64 Bytes = 2112 Bytes"]
    #[inline(always)]
    pub fn ps2048_64(self) -> &'a mut crate::W<REG> {
        self.variant(ECC_PAGESIZE_A::PS2048_64)
    }
    #[doc = "Main area 4096 Bytes + Spare area 128 Bytes = 4224 Bytes"]
    #[inline(always)]
    pub fn ps4096_128(self) -> &'a mut crate::W<REG> {
        self.variant(ECC_PAGESIZE_A::PS4096_128)
    }
}
#[doc = "Field `TYPCORREC` reader - Type of Correction"]
pub type TYPCORREC_R = crate::FieldReader<TYPCORREC_A>;
#[doc = "Type of Correction\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TYPCORREC_A {
    #[doc = "0: 1 bit correction for a page of 512/1024/2048/4096 Bytes (for 8 or 16-bit NAND Flash)"]
    CPAGE = 0,
    #[doc = "1: 1 bit correction for 256 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only)"]
    C256B = 1,
    #[doc = "2: 1 bit correction for 512 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only)"]
    C512B = 2,
}
impl From<TYPCORREC_A> for u8 {
    #[inline(always)]
    fn from(variant: TYPCORREC_A) -> Self {
        variant as _
    }
}
impl crate::FieldSpec for TYPCORREC_A {
    type Ux = u8;
}
impl TYPCORREC_R {
    #[doc = "Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<TYPCORREC_A> {
        match self.bits {
            0 => Some(TYPCORREC_A::CPAGE),
            1 => Some(TYPCORREC_A::C256B),
            2 => Some(TYPCORREC_A::C512B),
            _ => None,
        }
    }
    #[doc = "1 bit correction for a page of 512/1024/2048/4096 Bytes (for 8 or 16-bit NAND Flash)"]
    #[inline(always)]
    pub fn is_cpage(&self) -> bool {
        *self == TYPCORREC_A::CPAGE
    }
    #[doc = "1 bit correction for 256 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only)"]
    #[inline(always)]
    pub fn is_c256b(&self) -> bool {
        *self == TYPCORREC_A::C256B
    }
    #[doc = "1 bit correction for 512 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only)"]
    #[inline(always)]
    pub fn is_c512b(&self) -> bool {
        *self == TYPCORREC_A::C512B
    }
}
#[doc = "Field `TYPCORREC` writer - Type of Correction"]
pub type TYPCORREC_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 2, O, TYPCORREC_A>;
impl<'a, REG, const O: u8> TYPCORREC_W<'a, REG, O>
where
    REG: crate::Writable + crate::RegisterSpec,
    REG::Ux: From<u8>,
{
    #[doc = "1 bit correction for a page of 512/1024/2048/4096 Bytes (for 8 or 16-bit NAND Flash)"]
    #[inline(always)]
    pub fn cpage(self) -> &'a mut crate::W<REG> {
        self.variant(TYPCORREC_A::CPAGE)
    }
    #[doc = "1 bit correction for 256 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only)"]
    #[inline(always)]
    pub fn c256b(self) -> &'a mut crate::W<REG> {
        self.variant(TYPCORREC_A::C256B)
    }
    #[doc = "1 bit correction for 512 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only)"]
    #[inline(always)]
    pub fn c512b(self) -> &'a mut crate::W<REG> {
        self.variant(TYPCORREC_A::C512B)
    }
}
impl R {
    #[doc = "Bits 0:1 - ECC Page Size"]
    #[inline(always)]
    pub fn ecc_pagesize(&self) -> ECC_PAGESIZE_R {
        ECC_PAGESIZE_R::new((self.bits & 3) as u8)
    }
    #[doc = "Bits 4:5 - Type of Correction"]
    #[inline(always)]
    pub fn typcorrec(&self) -> TYPCORREC_R {
        TYPCORREC_R::new(((self.bits >> 4) & 3) as u8)
    }
}
impl W {
    #[doc = "Bits 0:1 - ECC Page Size"]
    #[inline(always)]
    #[must_use]
    pub fn ecc_pagesize(&mut self) -> ECC_PAGESIZE_W<ECC_MD_SPEC, 0> {
        ECC_PAGESIZE_W::new(self)
    }
    #[doc = "Bits 4:5 - Type of Correction"]
    #[inline(always)]
    #[must_use]
    pub fn typcorrec(&mut self) -> TYPCORREC_W<ECC_MD_SPEC, 4> {
        TYPCORREC_W::new(self)
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
#[doc = "SMC ECC Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ecc_md::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ecc_md::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct ECC_MD_SPEC;
impl crate::RegisterSpec for ECC_MD_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ecc_md::R`](R) reader structure"]
impl crate::Readable for ECC_MD_SPEC {}
#[doc = "`write(|w| ..)` method takes [`ecc_md::W`](W) writer structure"]
impl crate::Writable for ECC_MD_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets ECC_MD to value 0"]
impl crate::Resettable for ECC_MD_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
