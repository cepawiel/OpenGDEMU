#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00 - UDPHS Control Register"]
    pub ctrl: CTRL,
    #[doc = "0x04 - UDPHS Frame Number Register"]
    pub fnum: FNUM,
    _reserved2: [u8; 0x08],
    #[doc = "0x10 - UDPHS Interrupt Enable Register"]
    pub ien: IEN,
    #[doc = "0x14 - UDPHS Interrupt Status Register"]
    pub intsta: INTSTA,
    #[doc = "0x18 - UDPHS Clear Interrupt Register"]
    pub clrint: CLRINT,
    #[doc = "0x1c - UDPHS Endpoints Reset Register"]
    pub eptrst: EPTRST,
    _reserved6: [u8; 0xc0],
    #[doc = "0xe0 - UDPHS Test Register"]
    pub tst: TST,
    _reserved7: [u8; 0x0c],
    #[doc = "0xf0 - UDPHS Name1 Register"]
    pub ipname1: IPNAME1,
    #[doc = "0xf4 - UDPHS Name2 Register"]
    pub ipname2: IPNAME2,
    #[doc = "0xf8 - UDPHS Features Register"]
    pub ipfeatures: IPFEATURES,
    _reserved10: [u8; 0x04],
    #[doc = "0x100 - UDPHS Endpoint Configuration Register (endpoint = 0)"]
    pub eptcfg0: EPTCFG0,
    #[doc = "0x104 - UDPHS Endpoint Control Enable Register (endpoint = 0)"]
    pub eptctlenb0: EPTCTLENB0,
    #[doc = "0x108 - UDPHS Endpoint Control Disable Register (endpoint = 0)"]
    pub eptctldis0: EPTCTLDIS0,
    #[doc = "0x10c - UDPHS Endpoint Control Register (endpoint = 0)"]
    pub eptctl0: EPTCTL0,
    _reserved14: [u8; 0x04],
    #[doc = "0x114 - UDPHS Endpoint Set Status Register (endpoint = 0)"]
    pub eptsetsta0: EPTSETSTA0,
    #[doc = "0x118 - UDPHS Endpoint Clear Status Register (endpoint = 0)"]
    pub eptclrsta0: EPTCLRSTA0,
    #[doc = "0x11c - UDPHS Endpoint Status Register (endpoint = 0)"]
    pub eptsta0: EPTSTA0,
    #[doc = "0x120 - UDPHS Endpoint Configuration Register (endpoint = 1)"]
    pub eptcfg1: EPTCFG1,
    #[doc = "0x124 - UDPHS Endpoint Control Enable Register (endpoint = 1)"]
    pub eptctlenb1: EPTCTLENB1,
    #[doc = "0x128 - UDPHS Endpoint Control Disable Register (endpoint = 1)"]
    pub eptctldis1: EPTCTLDIS1,
    #[doc = "0x12c - UDPHS Endpoint Control Register (endpoint = 1)"]
    pub eptctl1: EPTCTL1,
    _reserved21: [u8; 0x04],
    #[doc = "0x134 - UDPHS Endpoint Set Status Register (endpoint = 1)"]
    pub eptsetsta1: EPTSETSTA1,
    #[doc = "0x138 - UDPHS Endpoint Clear Status Register (endpoint = 1)"]
    pub eptclrsta1: EPTCLRSTA1,
    #[doc = "0x13c - UDPHS Endpoint Status Register (endpoint = 1)"]
    pub eptsta1: EPTSTA1,
    #[doc = "0x140 - UDPHS Endpoint Configuration Register (endpoint = 2)"]
    pub eptcfg2: EPTCFG2,
    #[doc = "0x144 - UDPHS Endpoint Control Enable Register (endpoint = 2)"]
    pub eptctlenb2: EPTCTLENB2,
    #[doc = "0x148 - UDPHS Endpoint Control Disable Register (endpoint = 2)"]
    pub eptctldis2: EPTCTLDIS2,
    #[doc = "0x14c - UDPHS Endpoint Control Register (endpoint = 2)"]
    pub eptctl2: EPTCTL2,
    _reserved28: [u8; 0x04],
    #[doc = "0x154 - UDPHS Endpoint Set Status Register (endpoint = 2)"]
    pub eptsetsta2: EPTSETSTA2,
    #[doc = "0x158 - UDPHS Endpoint Clear Status Register (endpoint = 2)"]
    pub eptclrsta2: EPTCLRSTA2,
    #[doc = "0x15c - UDPHS Endpoint Status Register (endpoint = 2)"]
    pub eptsta2: EPTSTA2,
    #[doc = "0x160 - UDPHS Endpoint Configuration Register (endpoint = 3)"]
    pub eptcfg3: EPTCFG3,
    #[doc = "0x164 - UDPHS Endpoint Control Enable Register (endpoint = 3)"]
    pub eptctlenb3: EPTCTLENB3,
    #[doc = "0x168 - UDPHS Endpoint Control Disable Register (endpoint = 3)"]
    pub eptctldis3: EPTCTLDIS3,
    #[doc = "0x16c - UDPHS Endpoint Control Register (endpoint = 3)"]
    pub eptctl3: EPTCTL3,
    _reserved35: [u8; 0x04],
    #[doc = "0x174 - UDPHS Endpoint Set Status Register (endpoint = 3)"]
    pub eptsetsta3: EPTSETSTA3,
    #[doc = "0x178 - UDPHS Endpoint Clear Status Register (endpoint = 3)"]
    pub eptclrsta3: EPTCLRSTA3,
    #[doc = "0x17c - UDPHS Endpoint Status Register (endpoint = 3)"]
    pub eptsta3: EPTSTA3,
    #[doc = "0x180 - UDPHS Endpoint Configuration Register (endpoint = 4)"]
    pub eptcfg4: EPTCFG4,
    #[doc = "0x184 - UDPHS Endpoint Control Enable Register (endpoint = 4)"]
    pub eptctlenb4: EPTCTLENB4,
    #[doc = "0x188 - UDPHS Endpoint Control Disable Register (endpoint = 4)"]
    pub eptctldis4: EPTCTLDIS4,
    #[doc = "0x18c - UDPHS Endpoint Control Register (endpoint = 4)"]
    pub eptctl4: EPTCTL4,
    _reserved42: [u8; 0x04],
    #[doc = "0x194 - UDPHS Endpoint Set Status Register (endpoint = 4)"]
    pub eptsetsta4: EPTSETSTA4,
    #[doc = "0x198 - UDPHS Endpoint Clear Status Register (endpoint = 4)"]
    pub eptclrsta4: EPTCLRSTA4,
    #[doc = "0x19c - UDPHS Endpoint Status Register (endpoint = 4)"]
    pub eptsta4: EPTSTA4,
    #[doc = "0x1a0 - UDPHS Endpoint Configuration Register (endpoint = 5)"]
    pub eptcfg5: EPTCFG5,
    #[doc = "0x1a4 - UDPHS Endpoint Control Enable Register (endpoint = 5)"]
    pub eptctlenb5: EPTCTLENB5,
    #[doc = "0x1a8 - UDPHS Endpoint Control Disable Register (endpoint = 5)"]
    pub eptctldis5: EPTCTLDIS5,
    #[doc = "0x1ac - UDPHS Endpoint Control Register (endpoint = 5)"]
    pub eptctl5: EPTCTL5,
    _reserved49: [u8; 0x04],
    #[doc = "0x1b4 - UDPHS Endpoint Set Status Register (endpoint = 5)"]
    pub eptsetsta5: EPTSETSTA5,
    #[doc = "0x1b8 - UDPHS Endpoint Clear Status Register (endpoint = 5)"]
    pub eptclrsta5: EPTCLRSTA5,
    #[doc = "0x1bc - UDPHS Endpoint Status Register (endpoint = 5)"]
    pub eptsta5: EPTSTA5,
    #[doc = "0x1c0 - UDPHS Endpoint Configuration Register (endpoint = 6)"]
    pub eptcfg6: EPTCFG6,
    #[doc = "0x1c4 - UDPHS Endpoint Control Enable Register (endpoint = 6)"]
    pub eptctlenb6: EPTCTLENB6,
    #[doc = "0x1c8 - UDPHS Endpoint Control Disable Register (endpoint = 6)"]
    pub eptctldis6: EPTCTLDIS6,
    #[doc = "0x1cc - UDPHS Endpoint Control Register (endpoint = 6)"]
    pub eptctl6: EPTCTL6,
    _reserved56: [u8; 0x04],
    #[doc = "0x1d4 - UDPHS Endpoint Set Status Register (endpoint = 6)"]
    pub eptsetsta6: EPTSETSTA6,
    #[doc = "0x1d8 - UDPHS Endpoint Clear Status Register (endpoint = 6)"]
    pub eptclrsta6: EPTCLRSTA6,
    #[doc = "0x1dc - UDPHS Endpoint Status Register (endpoint = 6)"]
    pub eptsta6: EPTSTA6,
    _reserved59: [u8; 0x0120],
    #[doc = "0x300 - UDPHS DMA Next Descriptor Address Register (channel = 0)"]
    pub dmanxtdsc0: DMANXTDSC0,
    #[doc = "0x304 - UDPHS DMA Channel Address Register (channel = 0)"]
    pub dmaaddress0: DMAADDRESS0,
    #[doc = "0x308 - UDPHS DMA Channel Control Register (channel = 0)"]
    pub dmacontrol0: DMACONTROL0,
    #[doc = "0x30c - UDPHS DMA Channel Status Register (channel = 0)"]
    pub dmastatus0: DMASTATUS0,
    #[doc = "0x310 - UDPHS DMA Next Descriptor Address Register (channel = 1)"]
    pub dmanxtdsc1: DMANXTDSC1,
    #[doc = "0x314 - UDPHS DMA Channel Address Register (channel = 1)"]
    pub dmaaddress1: DMAADDRESS1,
    #[doc = "0x318 - UDPHS DMA Channel Control Register (channel = 1)"]
    pub dmacontrol1: DMACONTROL1,
    #[doc = "0x31c - UDPHS DMA Channel Status Register (channel = 1)"]
    pub dmastatus1: DMASTATUS1,
    #[doc = "0x320 - UDPHS DMA Next Descriptor Address Register (channel = 2)"]
    pub dmanxtdsc2: DMANXTDSC2,
    #[doc = "0x324 - UDPHS DMA Channel Address Register (channel = 2)"]
    pub dmaaddress2: DMAADDRESS2,
    #[doc = "0x328 - UDPHS DMA Channel Control Register (channel = 2)"]
    pub dmacontrol2: DMACONTROL2,
    #[doc = "0x32c - UDPHS DMA Channel Status Register (channel = 2)"]
    pub dmastatus2: DMASTATUS2,
    #[doc = "0x330 - UDPHS DMA Next Descriptor Address Register (channel = 3)"]
    pub dmanxtdsc3: DMANXTDSC3,
    #[doc = "0x334 - UDPHS DMA Channel Address Register (channel = 3)"]
    pub dmaaddress3: DMAADDRESS3,
    #[doc = "0x338 - UDPHS DMA Channel Control Register (channel = 3)"]
    pub dmacontrol3: DMACONTROL3,
    #[doc = "0x33c - UDPHS DMA Channel Status Register (channel = 3)"]
    pub dmastatus3: DMASTATUS3,
    #[doc = "0x340 - UDPHS DMA Next Descriptor Address Register (channel = 4)"]
    pub dmanxtdsc4: DMANXTDSC4,
    #[doc = "0x344 - UDPHS DMA Channel Address Register (channel = 4)"]
    pub dmaaddress4: DMAADDRESS4,
    #[doc = "0x348 - UDPHS DMA Channel Control Register (channel = 4)"]
    pub dmacontrol4: DMACONTROL4,
    #[doc = "0x34c - UDPHS DMA Channel Status Register (channel = 4)"]
    pub dmastatus4: DMASTATUS4,
    #[doc = "0x350 - UDPHS DMA Next Descriptor Address Register (channel = 5)"]
    pub dmanxtdsc5: DMANXTDSC5,
    #[doc = "0x354 - UDPHS DMA Channel Address Register (channel = 5)"]
    pub dmaaddress5: DMAADDRESS5,
    #[doc = "0x358 - UDPHS DMA Channel Control Register (channel = 5)"]
    pub dmacontrol5: DMACONTROL5,
    #[doc = "0x35c - UDPHS DMA Channel Status Register (channel = 5)"]
    pub dmastatus5: DMASTATUS5,
}
#[doc = "CTRL (rw) register accessor: UDPHS Control Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ctrl::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ctrl::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ctrl`]
module"]
pub type CTRL = crate::Reg<ctrl::CTRL_SPEC>;
#[doc = "UDPHS Control Register"]
pub mod ctrl;
#[doc = "FNUM (r) register accessor: UDPHS Frame Number Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fnum::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`fnum`]
module"]
pub type FNUM = crate::Reg<fnum::FNUM_SPEC>;
#[doc = "UDPHS Frame Number Register"]
pub mod fnum;
#[doc = "IEN (rw) register accessor: UDPHS Interrupt Enable Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ien::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ien::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ien`]
module"]
pub type IEN = crate::Reg<ien::IEN_SPEC>;
#[doc = "UDPHS Interrupt Enable Register"]
pub mod ien;
#[doc = "INTSTA (r) register accessor: UDPHS Interrupt Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`intsta::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`intsta`]
module"]
pub type INTSTA = crate::Reg<intsta::INTSTA_SPEC>;
#[doc = "UDPHS Interrupt Status Register"]
pub mod intsta;
#[doc = "CLRINT (w) register accessor: UDPHS Clear Interrupt Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`clrint::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`clrint`]
module"]
pub type CLRINT = crate::Reg<clrint::CLRINT_SPEC>;
#[doc = "UDPHS Clear Interrupt Register"]
pub mod clrint;
#[doc = "EPTRST (w) register accessor: UDPHS Endpoints Reset Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptrst::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptrst`]
module"]
pub type EPTRST = crate::Reg<eptrst::EPTRST_SPEC>;
#[doc = "UDPHS Endpoints Reset Register"]
pub mod eptrst;
#[doc = "TST (rw) register accessor: UDPHS Test Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tst::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tst::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`tst`]
module"]
pub type TST = crate::Reg<tst::TST_SPEC>;
#[doc = "UDPHS Test Register"]
pub mod tst;
#[doc = "IPNAME1 (r) register accessor: UDPHS Name1 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ipname1::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ipname1`]
module"]
pub type IPNAME1 = crate::Reg<ipname1::IPNAME1_SPEC>;
#[doc = "UDPHS Name1 Register"]
pub mod ipname1;
#[doc = "IPNAME2 (r) register accessor: UDPHS Name2 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ipname2::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ipname2`]
module"]
pub type IPNAME2 = crate::Reg<ipname2::IPNAME2_SPEC>;
#[doc = "UDPHS Name2 Register"]
pub mod ipname2;
#[doc = "IPFEATURES (r) register accessor: UDPHS Features Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ipfeatures::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ipfeatures`]
module"]
pub type IPFEATURES = crate::Reg<ipfeatures::IPFEATURES_SPEC>;
#[doc = "UDPHS Features Register"]
pub mod ipfeatures;
#[doc = "EPTCFG0 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg0`]
module"]
pub type EPTCFG0 = crate::Reg<eptcfg0::EPTCFG0_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 0)"]
pub mod eptcfg0;
#[doc = "EPTCTLENB0 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb0`]
module"]
pub type EPTCTLENB0 = crate::Reg<eptctlenb0::EPTCTLENB0_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 0)"]
pub mod eptctlenb0;
#[doc = "EPTCTLDIS0 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis0`]
module"]
pub type EPTCTLDIS0 = crate::Reg<eptctldis0::EPTCTLDIS0_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 0)"]
pub mod eptctldis0;
#[doc = "EPTCTL0 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl0::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl0`]
module"]
pub type EPTCTL0 = crate::Reg<eptctl0::EPTCTL0_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 0)"]
pub mod eptctl0;
#[doc = "EPTSETSTA0 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta0`]
module"]
pub type EPTSETSTA0 = crate::Reg<eptsetsta0::EPTSETSTA0_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 0)"]
pub mod eptsetsta0;
#[doc = "EPTCLRSTA0 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta0`]
module"]
pub type EPTCLRSTA0 = crate::Reg<eptclrsta0::EPTCLRSTA0_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 0)"]
pub mod eptclrsta0;
#[doc = "EPTSTA0 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta0::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta0`]
module"]
pub type EPTSTA0 = crate::Reg<eptsta0::EPTSTA0_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 0)"]
pub mod eptsta0;
#[doc = "EPTCFG1 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg1`]
module"]
pub type EPTCFG1 = crate::Reg<eptcfg1::EPTCFG1_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 1)"]
pub mod eptcfg1;
#[doc = "EPTCTLENB1 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb1`]
module"]
pub type EPTCTLENB1 = crate::Reg<eptctlenb1::EPTCTLENB1_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 1)"]
pub mod eptctlenb1;
#[doc = "EPTCTLDIS1 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis1`]
module"]
pub type EPTCTLDIS1 = crate::Reg<eptctldis1::EPTCTLDIS1_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 1)"]
pub mod eptctldis1;
#[doc = "EPTCTL1 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl1::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl1`]
module"]
pub type EPTCTL1 = crate::Reg<eptctl1::EPTCTL1_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 1)"]
pub mod eptctl1;
#[doc = "EPTSETSTA1 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta1`]
module"]
pub type EPTSETSTA1 = crate::Reg<eptsetsta1::EPTSETSTA1_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 1)"]
pub mod eptsetsta1;
#[doc = "EPTCLRSTA1 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta1`]
module"]
pub type EPTCLRSTA1 = crate::Reg<eptclrsta1::EPTCLRSTA1_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 1)"]
pub mod eptclrsta1;
#[doc = "EPTSTA1 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta1::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta1`]
module"]
pub type EPTSTA1 = crate::Reg<eptsta1::EPTSTA1_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 1)"]
pub mod eptsta1;
#[doc = "EPTCFG2 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg2`]
module"]
pub type EPTCFG2 = crate::Reg<eptcfg2::EPTCFG2_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 2)"]
pub mod eptcfg2;
#[doc = "EPTCTLENB2 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb2`]
module"]
pub type EPTCTLENB2 = crate::Reg<eptctlenb2::EPTCTLENB2_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 2)"]
pub mod eptctlenb2;
#[doc = "EPTCTLDIS2 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis2`]
module"]
pub type EPTCTLDIS2 = crate::Reg<eptctldis2::EPTCTLDIS2_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 2)"]
pub mod eptctldis2;
#[doc = "EPTCTL2 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl2::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl2`]
module"]
pub type EPTCTL2 = crate::Reg<eptctl2::EPTCTL2_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 2)"]
pub mod eptctl2;
#[doc = "EPTSETSTA2 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta2`]
module"]
pub type EPTSETSTA2 = crate::Reg<eptsetsta2::EPTSETSTA2_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 2)"]
pub mod eptsetsta2;
#[doc = "EPTCLRSTA2 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta2`]
module"]
pub type EPTCLRSTA2 = crate::Reg<eptclrsta2::EPTCLRSTA2_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 2)"]
pub mod eptclrsta2;
#[doc = "EPTSTA2 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta2::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta2`]
module"]
pub type EPTSTA2 = crate::Reg<eptsta2::EPTSTA2_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 2)"]
pub mod eptsta2;
#[doc = "EPTCFG3 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg3`]
module"]
pub type EPTCFG3 = crate::Reg<eptcfg3::EPTCFG3_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 3)"]
pub mod eptcfg3;
#[doc = "EPTCTLENB3 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb3`]
module"]
pub type EPTCTLENB3 = crate::Reg<eptctlenb3::EPTCTLENB3_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 3)"]
pub mod eptctlenb3;
#[doc = "EPTCTLDIS3 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis3`]
module"]
pub type EPTCTLDIS3 = crate::Reg<eptctldis3::EPTCTLDIS3_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 3)"]
pub mod eptctldis3;
#[doc = "EPTCTL3 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl3::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl3`]
module"]
pub type EPTCTL3 = crate::Reg<eptctl3::EPTCTL3_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 3)"]
pub mod eptctl3;
#[doc = "EPTSETSTA3 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta3`]
module"]
pub type EPTSETSTA3 = crate::Reg<eptsetsta3::EPTSETSTA3_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 3)"]
pub mod eptsetsta3;
#[doc = "EPTCLRSTA3 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta3`]
module"]
pub type EPTCLRSTA3 = crate::Reg<eptclrsta3::EPTCLRSTA3_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 3)"]
pub mod eptclrsta3;
#[doc = "EPTSTA3 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta3::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta3`]
module"]
pub type EPTSTA3 = crate::Reg<eptsta3::EPTSTA3_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 3)"]
pub mod eptsta3;
#[doc = "EPTCFG4 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg4`]
module"]
pub type EPTCFG4 = crate::Reg<eptcfg4::EPTCFG4_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 4)"]
pub mod eptcfg4;
#[doc = "EPTCTLENB4 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 4)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb4::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb4`]
module"]
pub type EPTCTLENB4 = crate::Reg<eptctlenb4::EPTCTLENB4_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 4)"]
pub mod eptctlenb4;
#[doc = "EPTCTLDIS4 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 4)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis4::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis4`]
module"]
pub type EPTCTLDIS4 = crate::Reg<eptctldis4::EPTCTLDIS4_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 4)"]
pub mod eptctldis4;
#[doc = "EPTCTL4 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl4::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl4`]
module"]
pub type EPTCTL4 = crate::Reg<eptctl4::EPTCTL4_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 4)"]
pub mod eptctl4;
#[doc = "EPTSETSTA4 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 4)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta4::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta4`]
module"]
pub type EPTSETSTA4 = crate::Reg<eptsetsta4::EPTSETSTA4_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 4)"]
pub mod eptsetsta4;
#[doc = "EPTCLRSTA4 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 4)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta4::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta4`]
module"]
pub type EPTCLRSTA4 = crate::Reg<eptclrsta4::EPTCLRSTA4_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 4)"]
pub mod eptclrsta4;
#[doc = "EPTSTA4 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta4::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta4`]
module"]
pub type EPTSTA4 = crate::Reg<eptsta4::EPTSTA4_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 4)"]
pub mod eptsta4;
#[doc = "EPTCFG5 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg5`]
module"]
pub type EPTCFG5 = crate::Reg<eptcfg5::EPTCFG5_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 5)"]
pub mod eptcfg5;
#[doc = "EPTCTLENB5 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 5)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb5::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb5`]
module"]
pub type EPTCTLENB5 = crate::Reg<eptctlenb5::EPTCTLENB5_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 5)"]
pub mod eptctlenb5;
#[doc = "EPTCTLDIS5 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 5)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis5::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis5`]
module"]
pub type EPTCTLDIS5 = crate::Reg<eptctldis5::EPTCTLDIS5_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 5)"]
pub mod eptctldis5;
#[doc = "EPTCTL5 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl5::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl5`]
module"]
pub type EPTCTL5 = crate::Reg<eptctl5::EPTCTL5_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 5)"]
pub mod eptctl5;
#[doc = "EPTSETSTA5 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 5)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta5::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta5`]
module"]
pub type EPTSETSTA5 = crate::Reg<eptsetsta5::EPTSETSTA5_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 5)"]
pub mod eptsetsta5;
#[doc = "EPTCLRSTA5 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 5)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta5::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta5`]
module"]
pub type EPTCLRSTA5 = crate::Reg<eptclrsta5::EPTCLRSTA5_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 5)"]
pub mod eptclrsta5;
#[doc = "EPTSTA5 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta5::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta5`]
module"]
pub type EPTSTA5 = crate::Reg<eptsta5::EPTSTA5_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 5)"]
pub mod eptsta5;
#[doc = "EPTCFG6 (rw) register accessor: UDPHS Endpoint Configuration Register (endpoint = 6)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptcfg6::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptcfg6::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptcfg6`]
module"]
pub type EPTCFG6 = crate::Reg<eptcfg6::EPTCFG6_SPEC>;
#[doc = "UDPHS Endpoint Configuration Register (endpoint = 6)"]
pub mod eptcfg6;
#[doc = "EPTCTLENB6 (w) register accessor: UDPHS Endpoint Control Enable Register (endpoint = 6)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctlenb6::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctlenb6`]
module"]
pub type EPTCTLENB6 = crate::Reg<eptctlenb6::EPTCTLENB6_SPEC>;
#[doc = "UDPHS Endpoint Control Enable Register (endpoint = 6)"]
pub mod eptctlenb6;
#[doc = "EPTCTLDIS6 (w) register accessor: UDPHS Endpoint Control Disable Register (endpoint = 6)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptctldis6::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctldis6`]
module"]
pub type EPTCTLDIS6 = crate::Reg<eptctldis6::EPTCTLDIS6_SPEC>;
#[doc = "UDPHS Endpoint Control Disable Register (endpoint = 6)"]
pub mod eptctldis6;
#[doc = "EPTCTL6 (r) register accessor: UDPHS Endpoint Control Register (endpoint = 6)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptctl6::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptctl6`]
module"]
pub type EPTCTL6 = crate::Reg<eptctl6::EPTCTL6_SPEC>;
#[doc = "UDPHS Endpoint Control Register (endpoint = 6)"]
pub mod eptctl6;
#[doc = "EPTSETSTA6 (w) register accessor: UDPHS Endpoint Set Status Register (endpoint = 6)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptsetsta6::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsetsta6`]
module"]
pub type EPTSETSTA6 = crate::Reg<eptsetsta6::EPTSETSTA6_SPEC>;
#[doc = "UDPHS Endpoint Set Status Register (endpoint = 6)"]
pub mod eptsetsta6;
#[doc = "EPTCLRSTA6 (w) register accessor: UDPHS Endpoint Clear Status Register (endpoint = 6)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`eptclrsta6::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptclrsta6`]
module"]
pub type EPTCLRSTA6 = crate::Reg<eptclrsta6::EPTCLRSTA6_SPEC>;
#[doc = "UDPHS Endpoint Clear Status Register (endpoint = 6)"]
pub mod eptclrsta6;
#[doc = "EPTSTA6 (r) register accessor: UDPHS Endpoint Status Register (endpoint = 6)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`eptsta6::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`eptsta6`]
module"]
pub type EPTSTA6 = crate::Reg<eptsta6::EPTSTA6_SPEC>;
#[doc = "UDPHS Endpoint Status Register (endpoint = 6)"]
pub mod eptsta6;
#[doc = "DMANXTDSC0 (rw) register accessor: UDPHS DMA Next Descriptor Address Register (channel = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmanxtdsc0`]
module"]
pub type DMANXTDSC0 = crate::Reg<dmanxtdsc0::DMANXTDSC0_SPEC>;
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 0)"]
pub mod dmanxtdsc0;
#[doc = "DMAADDRESS0 (rw) register accessor: UDPHS DMA Channel Address Register (channel = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmaaddress0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmaaddress0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmaaddress0`]
module"]
pub type DMAADDRESS0 = crate::Reg<dmaaddress0::DMAADDRESS0_SPEC>;
#[doc = "UDPHS DMA Channel Address Register (channel = 0)"]
pub mod dmaaddress0;
#[doc = "DMACONTROL0 (rw) register accessor: UDPHS DMA Channel Control Register (channel = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmacontrol0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmacontrol0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmacontrol0`]
module"]
pub type DMACONTROL0 = crate::Reg<dmacontrol0::DMACONTROL0_SPEC>;
#[doc = "UDPHS DMA Channel Control Register (channel = 0)"]
pub mod dmacontrol0;
#[doc = "DMASTATUS0 (rw) register accessor: UDPHS DMA Channel Status Register (channel = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmastatus0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmastatus0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmastatus0`]
module"]
pub type DMASTATUS0 = crate::Reg<dmastatus0::DMASTATUS0_SPEC>;
#[doc = "UDPHS DMA Channel Status Register (channel = 0)"]
pub mod dmastatus0;
#[doc = "DMANXTDSC1 (rw) register accessor: UDPHS DMA Next Descriptor Address Register (channel = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmanxtdsc1`]
module"]
pub type DMANXTDSC1 = crate::Reg<dmanxtdsc1::DMANXTDSC1_SPEC>;
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 1)"]
pub mod dmanxtdsc1;
#[doc = "DMAADDRESS1 (rw) register accessor: UDPHS DMA Channel Address Register (channel = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmaaddress1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmaaddress1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmaaddress1`]
module"]
pub type DMAADDRESS1 = crate::Reg<dmaaddress1::DMAADDRESS1_SPEC>;
#[doc = "UDPHS DMA Channel Address Register (channel = 1)"]
pub mod dmaaddress1;
#[doc = "DMACONTROL1 (rw) register accessor: UDPHS DMA Channel Control Register (channel = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmacontrol1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmacontrol1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmacontrol1`]
module"]
pub type DMACONTROL1 = crate::Reg<dmacontrol1::DMACONTROL1_SPEC>;
#[doc = "UDPHS DMA Channel Control Register (channel = 1)"]
pub mod dmacontrol1;
#[doc = "DMASTATUS1 (rw) register accessor: UDPHS DMA Channel Status Register (channel = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmastatus1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmastatus1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmastatus1`]
module"]
pub type DMASTATUS1 = crate::Reg<dmastatus1::DMASTATUS1_SPEC>;
#[doc = "UDPHS DMA Channel Status Register (channel = 1)"]
pub mod dmastatus1;
#[doc = "DMANXTDSC2 (rw) register accessor: UDPHS DMA Next Descriptor Address Register (channel = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmanxtdsc2`]
module"]
pub type DMANXTDSC2 = crate::Reg<dmanxtdsc2::DMANXTDSC2_SPEC>;
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 2)"]
pub mod dmanxtdsc2;
#[doc = "DMAADDRESS2 (rw) register accessor: UDPHS DMA Channel Address Register (channel = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmaaddress2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmaaddress2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmaaddress2`]
module"]
pub type DMAADDRESS2 = crate::Reg<dmaaddress2::DMAADDRESS2_SPEC>;
#[doc = "UDPHS DMA Channel Address Register (channel = 2)"]
pub mod dmaaddress2;
#[doc = "DMACONTROL2 (rw) register accessor: UDPHS DMA Channel Control Register (channel = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmacontrol2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmacontrol2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmacontrol2`]
module"]
pub type DMACONTROL2 = crate::Reg<dmacontrol2::DMACONTROL2_SPEC>;
#[doc = "UDPHS DMA Channel Control Register (channel = 2)"]
pub mod dmacontrol2;
#[doc = "DMASTATUS2 (rw) register accessor: UDPHS DMA Channel Status Register (channel = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmastatus2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmastatus2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmastatus2`]
module"]
pub type DMASTATUS2 = crate::Reg<dmastatus2::DMASTATUS2_SPEC>;
#[doc = "UDPHS DMA Channel Status Register (channel = 2)"]
pub mod dmastatus2;
#[doc = "DMANXTDSC3 (rw) register accessor: UDPHS DMA Next Descriptor Address Register (channel = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmanxtdsc3`]
module"]
pub type DMANXTDSC3 = crate::Reg<dmanxtdsc3::DMANXTDSC3_SPEC>;
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 3)"]
pub mod dmanxtdsc3;
#[doc = "DMAADDRESS3 (rw) register accessor: UDPHS DMA Channel Address Register (channel = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmaaddress3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmaaddress3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmaaddress3`]
module"]
pub type DMAADDRESS3 = crate::Reg<dmaaddress3::DMAADDRESS3_SPEC>;
#[doc = "UDPHS DMA Channel Address Register (channel = 3)"]
pub mod dmaaddress3;
#[doc = "DMACONTROL3 (rw) register accessor: UDPHS DMA Channel Control Register (channel = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmacontrol3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmacontrol3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmacontrol3`]
module"]
pub type DMACONTROL3 = crate::Reg<dmacontrol3::DMACONTROL3_SPEC>;
#[doc = "UDPHS DMA Channel Control Register (channel = 3)"]
pub mod dmacontrol3;
#[doc = "DMASTATUS3 (rw) register accessor: UDPHS DMA Channel Status Register (channel = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmastatus3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmastatus3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmastatus3`]
module"]
pub type DMASTATUS3 = crate::Reg<dmastatus3::DMASTATUS3_SPEC>;
#[doc = "UDPHS DMA Channel Status Register (channel = 3)"]
pub mod dmastatus3;
#[doc = "DMANXTDSC4 (rw) register accessor: UDPHS DMA Next Descriptor Address Register (channel = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmanxtdsc4`]
module"]
pub type DMANXTDSC4 = crate::Reg<dmanxtdsc4::DMANXTDSC4_SPEC>;
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 4)"]
pub mod dmanxtdsc4;
#[doc = "DMAADDRESS4 (rw) register accessor: UDPHS DMA Channel Address Register (channel = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmaaddress4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmaaddress4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmaaddress4`]
module"]
pub type DMAADDRESS4 = crate::Reg<dmaaddress4::DMAADDRESS4_SPEC>;
#[doc = "UDPHS DMA Channel Address Register (channel = 4)"]
pub mod dmaaddress4;
#[doc = "DMACONTROL4 (rw) register accessor: UDPHS DMA Channel Control Register (channel = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmacontrol4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmacontrol4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmacontrol4`]
module"]
pub type DMACONTROL4 = crate::Reg<dmacontrol4::DMACONTROL4_SPEC>;
#[doc = "UDPHS DMA Channel Control Register (channel = 4)"]
pub mod dmacontrol4;
#[doc = "DMASTATUS4 (rw) register accessor: UDPHS DMA Channel Status Register (channel = 4)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmastatus4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmastatus4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmastatus4`]
module"]
pub type DMASTATUS4 = crate::Reg<dmastatus4::DMASTATUS4_SPEC>;
#[doc = "UDPHS DMA Channel Status Register (channel = 4)"]
pub mod dmastatus4;
#[doc = "DMANXTDSC5 (rw) register accessor: UDPHS DMA Next Descriptor Address Register (channel = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmanxtdsc5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmanxtdsc5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmanxtdsc5`]
module"]
pub type DMANXTDSC5 = crate::Reg<dmanxtdsc5::DMANXTDSC5_SPEC>;
#[doc = "UDPHS DMA Next Descriptor Address Register (channel = 5)"]
pub mod dmanxtdsc5;
#[doc = "DMAADDRESS5 (rw) register accessor: UDPHS DMA Channel Address Register (channel = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmaaddress5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmaaddress5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmaaddress5`]
module"]
pub type DMAADDRESS5 = crate::Reg<dmaaddress5::DMAADDRESS5_SPEC>;
#[doc = "UDPHS DMA Channel Address Register (channel = 5)"]
pub mod dmaaddress5;
#[doc = "DMACONTROL5 (rw) register accessor: UDPHS DMA Channel Control Register (channel = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmacontrol5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmacontrol5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmacontrol5`]
module"]
pub type DMACONTROL5 = crate::Reg<dmacontrol5::DMACONTROL5_SPEC>;
#[doc = "UDPHS DMA Channel Control Register (channel = 5)"]
pub mod dmacontrol5;
#[doc = "DMASTATUS5 (rw) register accessor: UDPHS DMA Channel Status Register (channel = 5)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dmastatus5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dmastatus5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dmastatus5`]
module"]
pub type DMASTATUS5 = crate::Reg<dmastatus5::DMASTATUS5_SPEC>;
#[doc = "UDPHS DMA Channel Status Register (channel = 5)"]
pub mod dmastatus5;
