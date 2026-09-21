#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00 - DMAC Global Configuration Register"]
    pub gcfg: GCFG,
    #[doc = "0x04 - DMAC Enable Register"]
    pub en: EN,
    #[doc = "0x08 - DMAC Software Single Request Register"]
    pub sreq: SREQ,
    #[doc = "0x0c - DMAC Software Chunk Transfer Request Register"]
    pub creq: CREQ,
    #[doc = "0x10 - DMAC Software Last Transfer Flag Register"]
    pub last: LAST,
    _reserved5: [u8; 0x04],
    #[doc = "0x18 - DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register."]
    pub ebcier: EBCIER,
    #[doc = "0x1c - DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register."]
    pub ebcidr: EBCIDR,
    #[doc = "0x20 - DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register."]
    pub ebcimr: EBCIMR,
    #[doc = "0x24 - DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register."]
    pub ebcisr: EBCISR,
    #[doc = "0x28 - DMAC Channel Handler Enable Register"]
    pub cher: CHER,
    #[doc = "0x2c - DMAC Channel Handler Disable Register"]
    pub chdr: CHDR,
    #[doc = "0x30 - DMAC Channel Handler Status Register"]
    pub chsr: CHSR,
    _reserved12: [u8; 0x08],
    #[doc = "0x3c - DMAC Channel Source Address Register (ch_num = 0)"]
    pub saddr0: SADDR0,
    #[doc = "0x40 - DMAC Channel Destination Address Register (ch_num = 0)"]
    pub daddr0: DADDR0,
    #[doc = "0x44 - DMAC Channel Descriptor Address Register (ch_num = 0)"]
    pub dscr0: DSCR0,
    #[doc = "0x48 - DMAC Channel Control A Register (ch_num = 0)"]
    pub ctrla0: CTRLA0,
    #[doc = "0x4c - DMAC Channel Control B Register (ch_num = 0)"]
    pub ctrlb0: CTRLB0,
    #[doc = "0x50 - DMAC Channel Configuration Register (ch_num = 0)"]
    pub cfg0: CFG0,
    _reserved18: [u8; 0x10],
    #[doc = "0x64 - DMAC Channel Source Address Register (ch_num = 1)"]
    pub saddr1: SADDR1,
    #[doc = "0x68 - DMAC Channel Destination Address Register (ch_num = 1)"]
    pub daddr1: DADDR1,
    #[doc = "0x6c - DMAC Channel Descriptor Address Register (ch_num = 1)"]
    pub dscr1: DSCR1,
    #[doc = "0x70 - DMAC Channel Control A Register (ch_num = 1)"]
    pub ctrla1: CTRLA1,
    #[doc = "0x74 - DMAC Channel Control B Register (ch_num = 1)"]
    pub ctrlb1: CTRLB1,
    #[doc = "0x78 - DMAC Channel Configuration Register (ch_num = 1)"]
    pub cfg1: CFG1,
    _reserved24: [u8; 0x10],
    #[doc = "0x8c - DMAC Channel Source Address Register (ch_num = 2)"]
    pub saddr2: SADDR2,
    #[doc = "0x90 - DMAC Channel Destination Address Register (ch_num = 2)"]
    pub daddr2: DADDR2,
    #[doc = "0x94 - DMAC Channel Descriptor Address Register (ch_num = 2)"]
    pub dscr2: DSCR2,
    #[doc = "0x98 - DMAC Channel Control A Register (ch_num = 2)"]
    pub ctrla2: CTRLA2,
    #[doc = "0x9c - DMAC Channel Control B Register (ch_num = 2)"]
    pub ctrlb2: CTRLB2,
    #[doc = "0xa0 - DMAC Channel Configuration Register (ch_num = 2)"]
    pub cfg2: CFG2,
    _reserved30: [u8; 0x10],
    #[doc = "0xb4 - DMAC Channel Source Address Register (ch_num = 3)"]
    pub saddr3: SADDR3,
    #[doc = "0xb8 - DMAC Channel Destination Address Register (ch_num = 3)"]
    pub daddr3: DADDR3,
    #[doc = "0xbc - DMAC Channel Descriptor Address Register (ch_num = 3)"]
    pub dscr3: DSCR3,
    #[doc = "0xc0 - DMAC Channel Control A Register (ch_num = 3)"]
    pub ctrla3: CTRLA3,
    #[doc = "0xc4 - DMAC Channel Control B Register (ch_num = 3)"]
    pub ctrlb3: CTRLB3,
    #[doc = "0xc8 - DMAC Channel Configuration Register (ch_num = 3)"]
    pub cfg3: CFG3,
    _reserved36: [u8; 0x0118],
    #[doc = "0x1e4 - DMAC Write Protect Mode Register"]
    pub wpmr: WPMR,
    #[doc = "0x1e8 - DMAC Write Protect Status Register"]
    pub wpsr: WPSR,
}
#[doc = "GCFG (rw) register accessor: DMAC Global Configuration Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`gcfg::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`gcfg::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`gcfg`]
module"]
pub type GCFG = crate::Reg<gcfg::GCFG_SPEC>;
#[doc = "DMAC Global Configuration Register"]
pub mod gcfg;
#[doc = "EN (rw) register accessor: DMAC Enable Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`en::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`en::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`en`]
module"]
pub type EN = crate::Reg<en::EN_SPEC>;
#[doc = "DMAC Enable Register"]
pub mod en;
#[doc = "SREQ (rw) register accessor: DMAC Software Single Request Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sreq::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`sreq::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`sreq`]
module"]
pub type SREQ = crate::Reg<sreq::SREQ_SPEC>;
#[doc = "DMAC Software Single Request Register"]
pub mod sreq;
#[doc = "CREQ (rw) register accessor: DMAC Software Chunk Transfer Request Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`creq::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`creq::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`creq`]
module"]
pub type CREQ = crate::Reg<creq::CREQ_SPEC>;
#[doc = "DMAC Software Chunk Transfer Request Register"]
pub mod creq;
#[doc = "LAST (rw) register accessor: DMAC Software Last Transfer Flag Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`last::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`last::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`last`]
module"]
pub type LAST = crate::Reg<last::LAST_SPEC>;
#[doc = "DMAC Software Last Transfer Flag Register"]
pub mod last;
#[doc = "EBCIER (w) register accessor: DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register.\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ebcier::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ebcier`]
module"]
pub type EBCIER = crate::Reg<ebcier::EBCIER_SPEC>;
#[doc = "DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register."]
pub mod ebcier;
#[doc = "EBCIDR (w) register accessor: DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register.\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ebcidr::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ebcidr`]
module"]
pub type EBCIDR = crate::Reg<ebcidr::EBCIDR_SPEC>;
#[doc = "DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register."]
pub mod ebcidr;
#[doc = "EBCIMR (r) register accessor: DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register.\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ebcimr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ebcimr`]
module"]
pub type EBCIMR = crate::Reg<ebcimr::EBCIMR_SPEC>;
#[doc = "DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register."]
pub mod ebcimr;
#[doc = "EBCISR (r) register accessor: DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register.\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ebcisr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ebcisr`]
module"]
pub type EBCISR = crate::Reg<ebcisr::EBCISR_SPEC>;
#[doc = "DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register."]
pub mod ebcisr;
#[doc = "CHER (w) register accessor: DMAC Channel Handler Enable Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cher::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cher`]
module"]
pub type CHER = crate::Reg<cher::CHER_SPEC>;
#[doc = "DMAC Channel Handler Enable Register"]
pub mod cher;
#[doc = "CHDR (w) register accessor: DMAC Channel Handler Disable Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`chdr::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`chdr`]
module"]
pub type CHDR = crate::Reg<chdr::CHDR_SPEC>;
#[doc = "DMAC Channel Handler Disable Register"]
pub mod chdr;
#[doc = "CHSR (r) register accessor: DMAC Channel Handler Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`chsr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`chsr`]
module"]
pub type CHSR = crate::Reg<chsr::CHSR_SPEC>;
#[doc = "DMAC Channel Handler Status Register"]
pub mod chsr;
#[doc = "SADDR0 (rw) register accessor: DMAC Channel Source Address Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`saddr0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`saddr0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`saddr0`]
module"]
pub type SADDR0 = crate::Reg<saddr0::SADDR0_SPEC>;
#[doc = "DMAC Channel Source Address Register (ch_num = 0)"]
pub mod saddr0;
#[doc = "DADDR0 (rw) register accessor: DMAC Channel Destination Address Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`daddr0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`daddr0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`daddr0`]
module"]
pub type DADDR0 = crate::Reg<daddr0::DADDR0_SPEC>;
#[doc = "DMAC Channel Destination Address Register (ch_num = 0)"]
pub mod daddr0;
#[doc = "DSCR0 (rw) register accessor: DMAC Channel Descriptor Address Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dscr0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dscr0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dscr0`]
module"]
pub type DSCR0 = crate::Reg<dscr0::DSCR0_SPEC>;
#[doc = "DMAC Channel Descriptor Address Register (ch_num = 0)"]
pub mod dscr0;
#[doc = "CTRLA0 (rw) register accessor: DMAC Channel Control A Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrla0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrla0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrla0`]
module"]
pub type CTRLA0 = crate::Reg<ctrla0::CTRLA0_SPEC>;
#[doc = "DMAC Channel Control A Register (ch_num = 0)"]
pub mod ctrla0;
#[doc = "CTRLB0 (rw) register accessor: DMAC Channel Control B Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrlb0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrlb0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrlb0`]
module"]
pub type CTRLB0 = crate::Reg<ctrlb0::CTRLB0_SPEC>;
#[doc = "DMAC Channel Control B Register (ch_num = 0)"]
pub mod ctrlb0;
#[doc = "CFG0 (rw) register accessor: DMAC Channel Configuration Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cfg0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cfg0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cfg0`]
module"]
pub type CFG0 = crate::Reg<cfg0::CFG0_SPEC>;
#[doc = "DMAC Channel Configuration Register (ch_num = 0)"]
pub mod cfg0;
#[doc = "SADDR1 (rw) register accessor: DMAC Channel Source Address Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`saddr1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`saddr1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`saddr1`]
module"]
pub type SADDR1 = crate::Reg<saddr1::SADDR1_SPEC>;
#[doc = "DMAC Channel Source Address Register (ch_num = 1)"]
pub mod saddr1;
#[doc = "DADDR1 (rw) register accessor: DMAC Channel Destination Address Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`daddr1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`daddr1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`daddr1`]
module"]
pub type DADDR1 = crate::Reg<daddr1::DADDR1_SPEC>;
#[doc = "DMAC Channel Destination Address Register (ch_num = 1)"]
pub mod daddr1;
#[doc = "DSCR1 (rw) register accessor: DMAC Channel Descriptor Address Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dscr1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dscr1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dscr1`]
module"]
pub type DSCR1 = crate::Reg<dscr1::DSCR1_SPEC>;
#[doc = "DMAC Channel Descriptor Address Register (ch_num = 1)"]
pub mod dscr1;
#[doc = "CTRLA1 (rw) register accessor: DMAC Channel Control A Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrla1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrla1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrla1`]
module"]
pub type CTRLA1 = crate::Reg<ctrla1::CTRLA1_SPEC>;
#[doc = "DMAC Channel Control A Register (ch_num = 1)"]
pub mod ctrla1;
#[doc = "CTRLB1 (rw) register accessor: DMAC Channel Control B Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrlb1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrlb1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrlb1`]
module"]
pub type CTRLB1 = crate::Reg<ctrlb1::CTRLB1_SPEC>;
#[doc = "DMAC Channel Control B Register (ch_num = 1)"]
pub mod ctrlb1;
#[doc = "CFG1 (rw) register accessor: DMAC Channel Configuration Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cfg1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cfg1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cfg1`]
module"]
pub type CFG1 = crate::Reg<cfg1::CFG1_SPEC>;
#[doc = "DMAC Channel Configuration Register (ch_num = 1)"]
pub mod cfg1;
#[doc = "SADDR2 (rw) register accessor: DMAC Channel Source Address Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`saddr2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`saddr2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`saddr2`]
module"]
pub type SADDR2 = crate::Reg<saddr2::SADDR2_SPEC>;
#[doc = "DMAC Channel Source Address Register (ch_num = 2)"]
pub mod saddr2;
#[doc = "DADDR2 (rw) register accessor: DMAC Channel Destination Address Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`daddr2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`daddr2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`daddr2`]
module"]
pub type DADDR2 = crate::Reg<daddr2::DADDR2_SPEC>;
#[doc = "DMAC Channel Destination Address Register (ch_num = 2)"]
pub mod daddr2;
#[doc = "DSCR2 (rw) register accessor: DMAC Channel Descriptor Address Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dscr2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dscr2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dscr2`]
module"]
pub type DSCR2 = crate::Reg<dscr2::DSCR2_SPEC>;
#[doc = "DMAC Channel Descriptor Address Register (ch_num = 2)"]
pub mod dscr2;
#[doc = "CTRLA2 (rw) register accessor: DMAC Channel Control A Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrla2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrla2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrla2`]
module"]
pub type CTRLA2 = crate::Reg<ctrla2::CTRLA2_SPEC>;
#[doc = "DMAC Channel Control A Register (ch_num = 2)"]
pub mod ctrla2;
#[doc = "CTRLB2 (rw) register accessor: DMAC Channel Control B Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrlb2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrlb2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrlb2`]
module"]
pub type CTRLB2 = crate::Reg<ctrlb2::CTRLB2_SPEC>;
#[doc = "DMAC Channel Control B Register (ch_num = 2)"]
pub mod ctrlb2;
#[doc = "CFG2 (rw) register accessor: DMAC Channel Configuration Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cfg2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cfg2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cfg2`]
module"]
pub type CFG2 = crate::Reg<cfg2::CFG2_SPEC>;
#[doc = "DMAC Channel Configuration Register (ch_num = 2)"]
pub mod cfg2;
#[doc = "SADDR3 (rw) register accessor: DMAC Channel Source Address Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`saddr3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`saddr3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`saddr3`]
module"]
pub type SADDR3 = crate::Reg<saddr3::SADDR3_SPEC>;
#[doc = "DMAC Channel Source Address Register (ch_num = 3)"]
pub mod saddr3;
#[doc = "DADDR3 (rw) register accessor: DMAC Channel Destination Address Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`daddr3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`daddr3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`daddr3`]
module"]
pub type DADDR3 = crate::Reg<daddr3::DADDR3_SPEC>;
#[doc = "DMAC Channel Destination Address Register (ch_num = 3)"]
pub mod daddr3;
#[doc = "DSCR3 (rw) register accessor: DMAC Channel Descriptor Address Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dscr3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dscr3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dscr3`]
module"]
pub type DSCR3 = crate::Reg<dscr3::DSCR3_SPEC>;
#[doc = "DMAC Channel Descriptor Address Register (ch_num = 3)"]
pub mod dscr3;
#[doc = "CTRLA3 (rw) register accessor: DMAC Channel Control A Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrla3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrla3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrla3`]
module"]
pub type CTRLA3 = crate::Reg<ctrla3::CTRLA3_SPEC>;
#[doc = "DMAC Channel Control A Register (ch_num = 3)"]
pub mod ctrla3;
#[doc = "CTRLB3 (rw) register accessor: DMAC Channel Control B Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrlb3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrlb3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrlb3`]
module"]
pub type CTRLB3 = crate::Reg<ctrlb3::CTRLB3_SPEC>;
#[doc = "DMAC Channel Control B Register (ch_num = 3)"]
pub mod ctrlb3;
#[doc = "CFG3 (rw) register accessor: DMAC Channel Configuration Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cfg3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cfg3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cfg3`]
module"]
pub type CFG3 = crate::Reg<cfg3::CFG3_SPEC>;
#[doc = "DMAC Channel Configuration Register (ch_num = 3)"]
pub mod cfg3;
#[doc = "WPMR (rw) register accessor: DMAC Write Protect Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wpmr::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`wpmr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`wpmr`]
module"]
pub type WPMR = crate::Reg<wpmr::WPMR_SPEC>;
#[doc = "DMAC Write Protect Mode Register"]
pub mod wpmr;
#[doc = "WPSR (r) register accessor: DMAC Write Protect Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wpsr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`wpsr`]
module"]
pub type WPSR = crate::Reg<wpsr::WPSR_SPEC>;
#[doc = "DMAC Write Protect Status Register"]
pub mod wpsr;
