#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00 - PWM Clock Register"]
    pub clk: CLK,
    #[doc = "0x04 - PWM Enable Register"]
    pub ena: ENA,
    #[doc = "0x08 - PWM Disable Register"]
    pub dis: DIS,
    #[doc = "0x0c - PWM Status Register"]
    pub sr: SR,
    #[doc = "0x10 - PWM Interrupt Enable Register 1"]
    pub ier1: IER1,
    #[doc = "0x14 - PWM Interrupt Disable Register 1"]
    pub idr1: IDR1,
    #[doc = "0x18 - PWM Interrupt Mask Register 1"]
    pub imr1: IMR1,
    #[doc = "0x1c - PWM Interrupt Status Register 1"]
    pub isr1: ISR1,
    #[doc = "0x20 - PWM Sync Channels Mode Register"]
    pub scm: SCM,
    _reserved9: [u8; 0x04],
    #[doc = "0x28 - PWM Sync Channels Update Control Register"]
    pub scuc: SCUC,
    #[doc = "0x2c - PWM Sync Channels Update Period Register"]
    pub scup: SCUP,
    #[doc = "0x30 - PWM Sync Channels Update Period Update Register"]
    pub scupupd: SCUPUPD,
    #[doc = "0x34 - PWM Interrupt Enable Register 2"]
    pub ier2: IER2,
    #[doc = "0x38 - PWM Interrupt Disable Register 2"]
    pub idr2: IDR2,
    #[doc = "0x3c - PWM Interrupt Mask Register 2"]
    pub imr2: IMR2,
    #[doc = "0x40 - PWM Interrupt Status Register 2"]
    pub isr2: ISR2,
    #[doc = "0x44 - PWM Output Override Value Register"]
    pub oov: OOV,
    #[doc = "0x48 - PWM Output Selection Register"]
    pub os: OS,
    #[doc = "0x4c - PWM Output Selection Set Register"]
    pub oss: OSS,
    #[doc = "0x50 - PWM Output Selection Clear Register"]
    pub osc: OSC,
    #[doc = "0x54 - PWM Output Selection Set Update Register"]
    pub ossupd: OSSUPD,
    #[doc = "0x58 - PWM Output Selection Clear Update Register"]
    pub oscupd: OSCUPD,
    #[doc = "0x5c - PWM Fault Mode Register"]
    pub fmr: FMR,
    #[doc = "0x60 - PWM Fault Status Register"]
    pub fsr: FSR,
    #[doc = "0x64 - PWM Fault Clear Register"]
    pub fcr: FCR,
    #[doc = "0x68 - PWM Fault Protection Value Register"]
    pub fpv: FPV,
    #[doc = "0x6c - PWM Fault Protection Enable Register"]
    pub fpe: FPE,
    _reserved27: [u8; 0x0c],
    #[doc = "0x7c..0x84 - PWM Event Line 0 Mode Register"]
    pub elmr: [ELMR; 2],
    _reserved28: [u8; 0x60],
    #[doc = "0xe4 - PWM Write Protect Control Register"]
    pub wpcr: WPCR,
    #[doc = "0xe8 - PWM Write Protect Status Register"]
    pub wpsr: WPSR,
    _reserved30: [u8; 0x1c],
    #[doc = "0x108 - Transmit Pointer Register"]
    pub tpr: TPR,
    #[doc = "0x10c - Transmit Counter Register"]
    pub tcr: TCR,
    _reserved32: [u8; 0x08],
    #[doc = "0x118 - Transmit Next Pointer Register"]
    pub tnpr: TNPR,
    #[doc = "0x11c - Transmit Next Counter Register"]
    pub tncr: TNCR,
    #[doc = "0x120 - Transfer Control Register"]
    pub ptcr: PTCR,
    #[doc = "0x124 - Transfer Status Register"]
    pub ptsr: PTSR,
    _reserved36: [u8; 0x08],
    #[doc = "0x130 - PWM Comparison 0 Value Register"]
    pub cmpv0: CMPV0,
    #[doc = "0x134 - PWM Comparison 0 Value Update Register"]
    pub cmpvupd0: CMPVUPD0,
    #[doc = "0x138 - PWM Comparison 0 Mode Register"]
    pub cmpm0: CMPM0,
    #[doc = "0x13c - PWM Comparison 0 Mode Update Register"]
    pub cmpmupd0: CMPMUPD0,
    #[doc = "0x140 - PWM Comparison 1 Value Register"]
    pub cmpv1: CMPV1,
    #[doc = "0x144 - PWM Comparison 1 Value Update Register"]
    pub cmpvupd1: CMPVUPD1,
    #[doc = "0x148 - PWM Comparison 1 Mode Register"]
    pub cmpm1: CMPM1,
    #[doc = "0x14c - PWM Comparison 1 Mode Update Register"]
    pub cmpmupd1: CMPMUPD1,
    #[doc = "0x150 - PWM Comparison 2 Value Register"]
    pub cmpv2: CMPV2,
    #[doc = "0x154 - PWM Comparison 2 Value Update Register"]
    pub cmpvupd2: CMPVUPD2,
    #[doc = "0x158 - PWM Comparison 2 Mode Register"]
    pub cmpm2: CMPM2,
    #[doc = "0x15c - PWM Comparison 2 Mode Update Register"]
    pub cmpmupd2: CMPMUPD2,
    #[doc = "0x160 - PWM Comparison 3 Value Register"]
    pub cmpv3: CMPV3,
    #[doc = "0x164 - PWM Comparison 3 Value Update Register"]
    pub cmpvupd3: CMPVUPD3,
    #[doc = "0x168 - PWM Comparison 3 Mode Register"]
    pub cmpm3: CMPM3,
    #[doc = "0x16c - PWM Comparison 3 Mode Update Register"]
    pub cmpmupd3: CMPMUPD3,
    #[doc = "0x170 - PWM Comparison 4 Value Register"]
    pub cmpv4: CMPV4,
    #[doc = "0x174 - PWM Comparison 4 Value Update Register"]
    pub cmpvupd4: CMPVUPD4,
    #[doc = "0x178 - PWM Comparison 4 Mode Register"]
    pub cmpm4: CMPM4,
    #[doc = "0x17c - PWM Comparison 4 Mode Update Register"]
    pub cmpmupd4: CMPMUPD4,
    #[doc = "0x180 - PWM Comparison 5 Value Register"]
    pub cmpv5: CMPV5,
    #[doc = "0x184 - PWM Comparison 5 Value Update Register"]
    pub cmpvupd5: CMPVUPD5,
    #[doc = "0x188 - PWM Comparison 5 Mode Register"]
    pub cmpm5: CMPM5,
    #[doc = "0x18c - PWM Comparison 5 Mode Update Register"]
    pub cmpmupd5: CMPMUPD5,
    #[doc = "0x190 - PWM Comparison 6 Value Register"]
    pub cmpv6: CMPV6,
    #[doc = "0x194 - PWM Comparison 6 Value Update Register"]
    pub cmpvupd6: CMPVUPD6,
    #[doc = "0x198 - PWM Comparison 6 Mode Register"]
    pub cmpm6: CMPM6,
    #[doc = "0x19c - PWM Comparison 6 Mode Update Register"]
    pub cmpmupd6: CMPMUPD6,
    #[doc = "0x1a0 - PWM Comparison 7 Value Register"]
    pub cmpv7: CMPV7,
    #[doc = "0x1a4 - PWM Comparison 7 Value Update Register"]
    pub cmpvupd7: CMPVUPD7,
    #[doc = "0x1a8 - PWM Comparison 7 Mode Register"]
    pub cmpm7: CMPM7,
    #[doc = "0x1ac - PWM Comparison 7 Mode Update Register"]
    pub cmpmupd7: CMPMUPD7,
    _reserved68: [u8; 0x50],
    #[doc = "0x200 - PWM Channel Mode Register (ch_num = 0)"]
    pub cmr0: CMR0,
    #[doc = "0x204 - PWM Channel Duty Cycle Register (ch_num = 0)"]
    pub cdty0: CDTY0,
    #[doc = "0x208 - PWM Channel Duty Cycle Update Register (ch_num = 0)"]
    pub cdtyupd0: CDTYUPD0,
    #[doc = "0x20c - PWM Channel Period Register (ch_num = 0)"]
    pub cprd0: CPRD0,
    #[doc = "0x210 - PWM Channel Period Update Register (ch_num = 0)"]
    pub cprdupd0: CPRDUPD0,
    #[doc = "0x214 - PWM Channel Counter Register (ch_num = 0)"]
    pub ccnt0: CCNT0,
    #[doc = "0x218 - PWM Channel Dead Time Register (ch_num = 0)"]
    pub dt0: DT0,
    #[doc = "0x21c - PWM Channel Dead Time Update Register (ch_num = 0)"]
    pub dtupd0: DTUPD0,
    #[doc = "0x220 - PWM Channel Mode Register (ch_num = 1)"]
    pub cmr1: CMR1,
    #[doc = "0x224 - PWM Channel Duty Cycle Register (ch_num = 1)"]
    pub cdty1: CDTY1,
    #[doc = "0x228 - PWM Channel Duty Cycle Update Register (ch_num = 1)"]
    pub cdtyupd1: CDTYUPD1,
    #[doc = "0x22c - PWM Channel Period Register (ch_num = 1)"]
    pub cprd1: CPRD1,
    #[doc = "0x230 - PWM Channel Period Update Register (ch_num = 1)"]
    pub cprdupd1: CPRDUPD1,
    #[doc = "0x234 - PWM Channel Counter Register (ch_num = 1)"]
    pub ccnt1: CCNT1,
    #[doc = "0x238 - PWM Channel Dead Time Register (ch_num = 1)"]
    pub dt1: DT1,
    #[doc = "0x23c - PWM Channel Dead Time Update Register (ch_num = 1)"]
    pub dtupd1: DTUPD1,
    #[doc = "0x240 - PWM Channel Mode Register (ch_num = 2)"]
    pub cmr2: CMR2,
    #[doc = "0x244 - PWM Channel Duty Cycle Register (ch_num = 2)"]
    pub cdty2: CDTY2,
    #[doc = "0x248 - PWM Channel Duty Cycle Update Register (ch_num = 2)"]
    pub cdtyupd2: CDTYUPD2,
    #[doc = "0x24c - PWM Channel Period Register (ch_num = 2)"]
    pub cprd2: CPRD2,
    #[doc = "0x250 - PWM Channel Period Update Register (ch_num = 2)"]
    pub cprdupd2: CPRDUPD2,
    #[doc = "0x254 - PWM Channel Counter Register (ch_num = 2)"]
    pub ccnt2: CCNT2,
    #[doc = "0x258 - PWM Channel Dead Time Register (ch_num = 2)"]
    pub dt2: DT2,
    #[doc = "0x25c - PWM Channel Dead Time Update Register (ch_num = 2)"]
    pub dtupd2: DTUPD2,
    #[doc = "0x260 - PWM Channel Mode Register (ch_num = 3)"]
    pub cmr3: CMR3,
    #[doc = "0x264 - PWM Channel Duty Cycle Register (ch_num = 3)"]
    pub cdty3: CDTY3,
    #[doc = "0x268 - PWM Channel Duty Cycle Update Register (ch_num = 3)"]
    pub cdtyupd3: CDTYUPD3,
    #[doc = "0x26c - PWM Channel Period Register (ch_num = 3)"]
    pub cprd3: CPRD3,
    #[doc = "0x270 - PWM Channel Period Update Register (ch_num = 3)"]
    pub cprdupd3: CPRDUPD3,
    #[doc = "0x274 - PWM Channel Counter Register (ch_num = 3)"]
    pub ccnt3: CCNT3,
    #[doc = "0x278 - PWM Channel Dead Time Register (ch_num = 3)"]
    pub dt3: DT3,
    #[doc = "0x27c - PWM Channel Dead Time Update Register (ch_num = 3)"]
    pub dtupd3: DTUPD3,
}
#[doc = "CLK (rw) register accessor: PWM Clock Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`clk::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`clk::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`clk`]
module"]
pub type CLK = crate::Reg<clk::CLK_SPEC>;
#[doc = "PWM Clock Register"]
pub mod clk;
#[doc = "ENA (w) register accessor: PWM Enable Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ena::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ena`]
module"]
pub type ENA = crate::Reg<ena::ENA_SPEC>;
#[doc = "PWM Enable Register"]
pub mod ena;
#[doc = "DIS (w) register accessor: PWM Disable Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dis::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dis`]
module"]
pub type DIS = crate::Reg<dis::DIS_SPEC>;
#[doc = "PWM Disable Register"]
pub mod dis;
#[doc = "SR (r) register accessor: PWM Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`sr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`sr`]
module"]
pub type SR = crate::Reg<sr::SR_SPEC>;
#[doc = "PWM Status Register"]
pub mod sr;
#[doc = "IER1 (w) register accessor: PWM Interrupt Enable Register 1\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ier1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ier1`]
module"]
pub type IER1 = crate::Reg<ier1::IER1_SPEC>;
#[doc = "PWM Interrupt Enable Register 1"]
pub mod ier1;
#[doc = "IDR1 (w) register accessor: PWM Interrupt Disable Register 1\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`idr1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`idr1`]
module"]
pub type IDR1 = crate::Reg<idr1::IDR1_SPEC>;
#[doc = "PWM Interrupt Disable Register 1"]
pub mod idr1;
#[doc = "IMR1 (r) register accessor: PWM Interrupt Mask Register 1\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`imr1::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`imr1`]
module"]
pub type IMR1 = crate::Reg<imr1::IMR1_SPEC>;
#[doc = "PWM Interrupt Mask Register 1"]
pub mod imr1;
#[doc = "ISR1 (r) register accessor: PWM Interrupt Status Register 1\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`isr1::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`isr1`]
module"]
pub type ISR1 = crate::Reg<isr1::ISR1_SPEC>;
#[doc = "PWM Interrupt Status Register 1"]
pub mod isr1;
#[doc = "SCM (rw) register accessor: PWM Sync Channels Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`scm::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`scm::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`scm`]
module"]
pub type SCM = crate::Reg<scm::SCM_SPEC>;
#[doc = "PWM Sync Channels Mode Register"]
pub mod scm;
#[doc = "SCUC (rw) register accessor: PWM Sync Channels Update Control Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`scuc::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`scuc::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`scuc`]
module"]
pub type SCUC = crate::Reg<scuc::SCUC_SPEC>;
#[doc = "PWM Sync Channels Update Control Register"]
pub mod scuc;
#[doc = "SCUP (rw) register accessor: PWM Sync Channels Update Period Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`scup::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`scup::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`scup`]
module"]
pub type SCUP = crate::Reg<scup::SCUP_SPEC>;
#[doc = "PWM Sync Channels Update Period Register"]
pub mod scup;
#[doc = "SCUPUPD (w) register accessor: PWM Sync Channels Update Period Update Register\n\nYou can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`scupupd::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`scupupd`]
module"]
pub type SCUPUPD = crate::Reg<scupupd::SCUPUPD_SPEC>;
#[doc = "PWM Sync Channels Update Period Update Register"]
pub mod scupupd;
#[doc = "IER2 (w) register accessor: PWM Interrupt Enable Register 2\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ier2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ier2`]
module"]
pub type IER2 = crate::Reg<ier2::IER2_SPEC>;
#[doc = "PWM Interrupt Enable Register 2"]
pub mod ier2;
#[doc = "IDR2 (w) register accessor: PWM Interrupt Disable Register 2\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`idr2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`idr2`]
module"]
pub type IDR2 = crate::Reg<idr2::IDR2_SPEC>;
#[doc = "PWM Interrupt Disable Register 2"]
pub mod idr2;
#[doc = "IMR2 (r) register accessor: PWM Interrupt Mask Register 2\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`imr2::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`imr2`]
module"]
pub type IMR2 = crate::Reg<imr2::IMR2_SPEC>;
#[doc = "PWM Interrupt Mask Register 2"]
pub mod imr2;
#[doc = "ISR2 (r) register accessor: PWM Interrupt Status Register 2\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`isr2::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`isr2`]
module"]
pub type ISR2 = crate::Reg<isr2::ISR2_SPEC>;
#[doc = "PWM Interrupt Status Register 2"]
pub mod isr2;
#[doc = "OOV (rw) register accessor: PWM Output Override Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`oov::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`oov::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`oov`]
module"]
pub type OOV = crate::Reg<oov::OOV_SPEC>;
#[doc = "PWM Output Override Value Register"]
pub mod oov;
#[doc = "OS (rw) register accessor: PWM Output Selection Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`os::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`os::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`os`]
module"]
pub type OS = crate::Reg<os::OS_SPEC>;
#[doc = "PWM Output Selection Register"]
pub mod os;
#[doc = "OSS (w) register accessor: PWM Output Selection Set Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`oss::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`oss`]
module"]
pub type OSS = crate::Reg<oss::OSS_SPEC>;
#[doc = "PWM Output Selection Set Register"]
pub mod oss;
#[doc = "OSC (w) register accessor: PWM Output Selection Clear Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`osc::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`osc`]
module"]
pub type OSC = crate::Reg<osc::OSC_SPEC>;
#[doc = "PWM Output Selection Clear Register"]
pub mod osc;
#[doc = "OSSUPD (w) register accessor: PWM Output Selection Set Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ossupd::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ossupd`]
module"]
pub type OSSUPD = crate::Reg<ossupd::OSSUPD_SPEC>;
#[doc = "PWM Output Selection Set Update Register"]
pub mod ossupd;
#[doc = "OSCUPD (w) register accessor: PWM Output Selection Clear Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`oscupd::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`oscupd`]
module"]
pub type OSCUPD = crate::Reg<oscupd::OSCUPD_SPEC>;
#[doc = "PWM Output Selection Clear Update Register"]
pub mod oscupd;
#[doc = "FMR (rw) register accessor: PWM Fault Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fmr::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`fmr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`fmr`]
module"]
pub type FMR = crate::Reg<fmr::FMR_SPEC>;
#[doc = "PWM Fault Mode Register"]
pub mod fmr;
#[doc = "FSR (r) register accessor: PWM Fault Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fsr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`fsr`]
module"]
pub type FSR = crate::Reg<fsr::FSR_SPEC>;
#[doc = "PWM Fault Status Register"]
pub mod fsr;
#[doc = "FCR (w) register accessor: PWM Fault Clear Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`fcr::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`fcr`]
module"]
pub type FCR = crate::Reg<fcr::FCR_SPEC>;
#[doc = "PWM Fault Clear Register"]
pub mod fcr;
#[doc = "FPV (rw) register accessor: PWM Fault Protection Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fpv::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`fpv::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`fpv`]
module"]
pub type FPV = crate::Reg<fpv::FPV_SPEC>;
#[doc = "PWM Fault Protection Value Register"]
pub mod fpv;
#[doc = "FPE (rw) register accessor: PWM Fault Protection Enable Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`fpe::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`fpe::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`fpe`]
module"]
pub type FPE = crate::Reg<fpe::FPE_SPEC>;
#[doc = "PWM Fault Protection Enable Register"]
pub mod fpe;
#[doc = "ELMR (rw) register accessor: PWM Event Line 0 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`elmr::R`].  You can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`elmr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`elmr`]
module"]
pub type ELMR = crate::Reg<elmr::ELMR_SPEC>;
#[doc = "PWM Event Line 0 Mode Register"]
pub mod elmr;
#[doc = "WPCR (w) register accessor: PWM Write Protect Control Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`wpcr::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`wpcr`]
module"]
pub type WPCR = crate::Reg<wpcr::WPCR_SPEC>;
#[doc = "PWM Write Protect Control Register"]
pub mod wpcr;
#[doc = "WPSR (r) register accessor: PWM Write Protect Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`wpsr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`wpsr`]
module"]
pub type WPSR = crate::Reg<wpsr::WPSR_SPEC>;
#[doc = "PWM Write Protect Status Register"]
pub mod wpsr;
#[doc = "CMPV0 (rw) register accessor: PWM Comparison 0 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv0`]
module"]
pub type CMPV0 = crate::Reg<cmpv0::CMPV0_SPEC>;
#[doc = "PWM Comparison 0 Value Register"]
pub mod cmpv0;
#[doc = "CMPVUPD0 (w) register accessor: PWM Comparison 0 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd0`]
module"]
pub type CMPVUPD0 = crate::Reg<cmpvupd0::CMPVUPD0_SPEC>;
#[doc = "PWM Comparison 0 Value Update Register"]
pub mod cmpvupd0;
#[doc = "CMPM0 (rw) register accessor: PWM Comparison 0 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm0`]
module"]
pub type CMPM0 = crate::Reg<cmpm0::CMPM0_SPEC>;
#[doc = "PWM Comparison 0 Mode Register"]
pub mod cmpm0;
#[doc = "CMPMUPD0 (w) register accessor: PWM Comparison 0 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd0`]
module"]
pub type CMPMUPD0 = crate::Reg<cmpmupd0::CMPMUPD0_SPEC>;
#[doc = "PWM Comparison 0 Mode Update Register"]
pub mod cmpmupd0;
#[doc = "CMPV1 (rw) register accessor: PWM Comparison 1 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv1`]
module"]
pub type CMPV1 = crate::Reg<cmpv1::CMPV1_SPEC>;
#[doc = "PWM Comparison 1 Value Register"]
pub mod cmpv1;
#[doc = "CMPVUPD1 (w) register accessor: PWM Comparison 1 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd1`]
module"]
pub type CMPVUPD1 = crate::Reg<cmpvupd1::CMPVUPD1_SPEC>;
#[doc = "PWM Comparison 1 Value Update Register"]
pub mod cmpvupd1;
#[doc = "CMPM1 (rw) register accessor: PWM Comparison 1 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm1`]
module"]
pub type CMPM1 = crate::Reg<cmpm1::CMPM1_SPEC>;
#[doc = "PWM Comparison 1 Mode Register"]
pub mod cmpm1;
#[doc = "CMPMUPD1 (w) register accessor: PWM Comparison 1 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd1`]
module"]
pub type CMPMUPD1 = crate::Reg<cmpmupd1::CMPMUPD1_SPEC>;
#[doc = "PWM Comparison 1 Mode Update Register"]
pub mod cmpmupd1;
#[doc = "CMPV2 (rw) register accessor: PWM Comparison 2 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv2`]
module"]
pub type CMPV2 = crate::Reg<cmpv2::CMPV2_SPEC>;
#[doc = "PWM Comparison 2 Value Register"]
pub mod cmpv2;
#[doc = "CMPVUPD2 (w) register accessor: PWM Comparison 2 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd2`]
module"]
pub type CMPVUPD2 = crate::Reg<cmpvupd2::CMPVUPD2_SPEC>;
#[doc = "PWM Comparison 2 Value Update Register"]
pub mod cmpvupd2;
#[doc = "CMPM2 (rw) register accessor: PWM Comparison 2 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm2`]
module"]
pub type CMPM2 = crate::Reg<cmpm2::CMPM2_SPEC>;
#[doc = "PWM Comparison 2 Mode Register"]
pub mod cmpm2;
#[doc = "CMPMUPD2 (w) register accessor: PWM Comparison 2 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd2`]
module"]
pub type CMPMUPD2 = crate::Reg<cmpmupd2::CMPMUPD2_SPEC>;
#[doc = "PWM Comparison 2 Mode Update Register"]
pub mod cmpmupd2;
#[doc = "CMPV3 (rw) register accessor: PWM Comparison 3 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv3`]
module"]
pub type CMPV3 = crate::Reg<cmpv3::CMPV3_SPEC>;
#[doc = "PWM Comparison 3 Value Register"]
pub mod cmpv3;
#[doc = "CMPVUPD3 (w) register accessor: PWM Comparison 3 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd3`]
module"]
pub type CMPVUPD3 = crate::Reg<cmpvupd3::CMPVUPD3_SPEC>;
#[doc = "PWM Comparison 3 Value Update Register"]
pub mod cmpvupd3;
#[doc = "CMPM3 (rw) register accessor: PWM Comparison 3 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm3`]
module"]
pub type CMPM3 = crate::Reg<cmpm3::CMPM3_SPEC>;
#[doc = "PWM Comparison 3 Mode Register"]
pub mod cmpm3;
#[doc = "CMPMUPD3 (w) register accessor: PWM Comparison 3 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd3`]
module"]
pub type CMPMUPD3 = crate::Reg<cmpmupd3::CMPMUPD3_SPEC>;
#[doc = "PWM Comparison 3 Mode Update Register"]
pub mod cmpmupd3;
#[doc = "CMPV4 (rw) register accessor: PWM Comparison 4 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv4`]
module"]
pub type CMPV4 = crate::Reg<cmpv4::CMPV4_SPEC>;
#[doc = "PWM Comparison 4 Value Register"]
pub mod cmpv4;
#[doc = "CMPVUPD4 (w) register accessor: PWM Comparison 4 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd4::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd4`]
module"]
pub type CMPVUPD4 = crate::Reg<cmpvupd4::CMPVUPD4_SPEC>;
#[doc = "PWM Comparison 4 Value Update Register"]
pub mod cmpvupd4;
#[doc = "CMPM4 (rw) register accessor: PWM Comparison 4 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm4::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm4::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm4`]
module"]
pub type CMPM4 = crate::Reg<cmpm4::CMPM4_SPEC>;
#[doc = "PWM Comparison 4 Mode Register"]
pub mod cmpm4;
#[doc = "CMPMUPD4 (w) register accessor: PWM Comparison 4 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd4::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd4`]
module"]
pub type CMPMUPD4 = crate::Reg<cmpmupd4::CMPMUPD4_SPEC>;
#[doc = "PWM Comparison 4 Mode Update Register"]
pub mod cmpmupd4;
#[doc = "CMPV5 (rw) register accessor: PWM Comparison 5 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv5`]
module"]
pub type CMPV5 = crate::Reg<cmpv5::CMPV5_SPEC>;
#[doc = "PWM Comparison 5 Value Register"]
pub mod cmpv5;
#[doc = "CMPVUPD5 (w) register accessor: PWM Comparison 5 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd5::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd5`]
module"]
pub type CMPVUPD5 = crate::Reg<cmpvupd5::CMPVUPD5_SPEC>;
#[doc = "PWM Comparison 5 Value Update Register"]
pub mod cmpvupd5;
#[doc = "CMPM5 (rw) register accessor: PWM Comparison 5 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm5::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm5::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm5`]
module"]
pub type CMPM5 = crate::Reg<cmpm5::CMPM5_SPEC>;
#[doc = "PWM Comparison 5 Mode Register"]
pub mod cmpm5;
#[doc = "CMPMUPD5 (w) register accessor: PWM Comparison 5 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd5::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd5`]
module"]
pub type CMPMUPD5 = crate::Reg<cmpmupd5::CMPMUPD5_SPEC>;
#[doc = "PWM Comparison 5 Mode Update Register"]
pub mod cmpmupd5;
#[doc = "CMPV6 (rw) register accessor: PWM Comparison 6 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv6::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv6::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv6`]
module"]
pub type CMPV6 = crate::Reg<cmpv6::CMPV6_SPEC>;
#[doc = "PWM Comparison 6 Value Register"]
pub mod cmpv6;
#[doc = "CMPVUPD6 (w) register accessor: PWM Comparison 6 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd6::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd6`]
module"]
pub type CMPVUPD6 = crate::Reg<cmpvupd6::CMPVUPD6_SPEC>;
#[doc = "PWM Comparison 6 Value Update Register"]
pub mod cmpvupd6;
#[doc = "CMPM6 (rw) register accessor: PWM Comparison 6 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm6::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm6::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm6`]
module"]
pub type CMPM6 = crate::Reg<cmpm6::CMPM6_SPEC>;
#[doc = "PWM Comparison 6 Mode Register"]
pub mod cmpm6;
#[doc = "CMPMUPD6 (w) register accessor: PWM Comparison 6 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd6::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd6`]
module"]
pub type CMPMUPD6 = crate::Reg<cmpmupd6::CMPMUPD6_SPEC>;
#[doc = "PWM Comparison 6 Mode Update Register"]
pub mod cmpmupd6;
#[doc = "CMPV7 (rw) register accessor: PWM Comparison 7 Value Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpv7::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpv7::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpv7`]
module"]
pub type CMPV7 = crate::Reg<cmpv7::CMPV7_SPEC>;
#[doc = "PWM Comparison 7 Value Register"]
pub mod cmpv7;
#[doc = "CMPVUPD7 (w) register accessor: PWM Comparison 7 Value Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpvupd7::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpvupd7`]
module"]
pub type CMPVUPD7 = crate::Reg<cmpvupd7::CMPVUPD7_SPEC>;
#[doc = "PWM Comparison 7 Value Update Register"]
pub mod cmpvupd7;
#[doc = "CMPM7 (rw) register accessor: PWM Comparison 7 Mode Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmpm7::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpm7::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpm7`]
module"]
pub type CMPM7 = crate::Reg<cmpm7::CMPM7_SPEC>;
#[doc = "PWM Comparison 7 Mode Register"]
pub mod cmpm7;
#[doc = "CMPMUPD7 (w) register accessor: PWM Comparison 7 Mode Update Register\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmpmupd7::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmpmupd7`]
module"]
pub type CMPMUPD7 = crate::Reg<cmpmupd7::CMPMUPD7_SPEC>;
#[doc = "PWM Comparison 7 Mode Update Register"]
pub mod cmpmupd7;
#[doc = "CMR0 (rw) register accessor: PWM Channel Mode Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmr0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmr0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmr0`]
module"]
pub type CMR0 = crate::Reg<cmr0::CMR0_SPEC>;
#[doc = "PWM Channel Mode Register (ch_num = 0)"]
pub mod cmr0;
#[doc = "CDTY0 (rw) register accessor: PWM Channel Duty Cycle Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cdty0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdty0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdty0`]
module"]
pub type CDTY0 = crate::Reg<cdty0::CDTY0_SPEC>;
#[doc = "PWM Channel Duty Cycle Register (ch_num = 0)"]
pub mod cdty0;
#[doc = "CDTYUPD0 (w) register accessor: PWM Channel Duty Cycle Update Register (ch_num = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdtyupd0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdtyupd0`]
module"]
pub type CDTYUPD0 = crate::Reg<cdtyupd0::CDTYUPD0_SPEC>;
#[doc = "PWM Channel Duty Cycle Update Register (ch_num = 0)"]
pub mod cdtyupd0;
#[doc = "CPRD0 (rw) register accessor: PWM Channel Period Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cprd0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprd0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprd0`]
module"]
pub type CPRD0 = crate::Reg<cprd0::CPRD0_SPEC>;
#[doc = "PWM Channel Period Register (ch_num = 0)"]
pub mod cprd0;
#[doc = "CPRDUPD0 (w) register accessor: PWM Channel Period Update Register (ch_num = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprdupd0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprdupd0`]
module"]
pub type CPRDUPD0 = crate::Reg<cprdupd0::CPRDUPD0_SPEC>;
#[doc = "PWM Channel Period Update Register (ch_num = 0)"]
pub mod cprdupd0;
#[doc = "CCNT0 (r) register accessor: PWM Channel Counter Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ccnt0::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ccnt0`]
module"]
pub type CCNT0 = crate::Reg<ccnt0::CCNT0_SPEC>;
#[doc = "PWM Channel Counter Register (ch_num = 0)"]
pub mod ccnt0;
#[doc = "DT0 (rw) register accessor: PWM Channel Dead Time Register (ch_num = 0)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dt0::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dt0::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dt0`]
module"]
pub type DT0 = crate::Reg<dt0::DT0_SPEC>;
#[doc = "PWM Channel Dead Time Register (ch_num = 0)"]
pub mod dt0;
#[doc = "DTUPD0 (w) register accessor: PWM Channel Dead Time Update Register (ch_num = 0)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dtupd0::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dtupd0`]
module"]
pub type DTUPD0 = crate::Reg<dtupd0::DTUPD0_SPEC>;
#[doc = "PWM Channel Dead Time Update Register (ch_num = 0)"]
pub mod dtupd0;
#[doc = "CMR1 (rw) register accessor: PWM Channel Mode Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmr1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmr1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmr1`]
module"]
pub type CMR1 = crate::Reg<cmr1::CMR1_SPEC>;
#[doc = "PWM Channel Mode Register (ch_num = 1)"]
pub mod cmr1;
#[doc = "CDTY1 (rw) register accessor: PWM Channel Duty Cycle Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cdty1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdty1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdty1`]
module"]
pub type CDTY1 = crate::Reg<cdty1::CDTY1_SPEC>;
#[doc = "PWM Channel Duty Cycle Register (ch_num = 1)"]
pub mod cdty1;
#[doc = "CDTYUPD1 (w) register accessor: PWM Channel Duty Cycle Update Register (ch_num = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdtyupd1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdtyupd1`]
module"]
pub type CDTYUPD1 = crate::Reg<cdtyupd1::CDTYUPD1_SPEC>;
#[doc = "PWM Channel Duty Cycle Update Register (ch_num = 1)"]
pub mod cdtyupd1;
#[doc = "CPRD1 (rw) register accessor: PWM Channel Period Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cprd1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprd1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprd1`]
module"]
pub type CPRD1 = crate::Reg<cprd1::CPRD1_SPEC>;
#[doc = "PWM Channel Period Register (ch_num = 1)"]
pub mod cprd1;
#[doc = "CPRDUPD1 (w) register accessor: PWM Channel Period Update Register (ch_num = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprdupd1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprdupd1`]
module"]
pub type CPRDUPD1 = crate::Reg<cprdupd1::CPRDUPD1_SPEC>;
#[doc = "PWM Channel Period Update Register (ch_num = 1)"]
pub mod cprdupd1;
#[doc = "CCNT1 (r) register accessor: PWM Channel Counter Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ccnt1::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ccnt1`]
module"]
pub type CCNT1 = crate::Reg<ccnt1::CCNT1_SPEC>;
#[doc = "PWM Channel Counter Register (ch_num = 1)"]
pub mod ccnt1;
#[doc = "DT1 (rw) register accessor: PWM Channel Dead Time Register (ch_num = 1)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dt1::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dt1::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dt1`]
module"]
pub type DT1 = crate::Reg<dt1::DT1_SPEC>;
#[doc = "PWM Channel Dead Time Register (ch_num = 1)"]
pub mod dt1;
#[doc = "DTUPD1 (w) register accessor: PWM Channel Dead Time Update Register (ch_num = 1)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dtupd1::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dtupd1`]
module"]
pub type DTUPD1 = crate::Reg<dtupd1::DTUPD1_SPEC>;
#[doc = "PWM Channel Dead Time Update Register (ch_num = 1)"]
pub mod dtupd1;
#[doc = "CMR2 (rw) register accessor: PWM Channel Mode Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmr2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmr2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmr2`]
module"]
pub type CMR2 = crate::Reg<cmr2::CMR2_SPEC>;
#[doc = "PWM Channel Mode Register (ch_num = 2)"]
pub mod cmr2;
#[doc = "CDTY2 (rw) register accessor: PWM Channel Duty Cycle Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cdty2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdty2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdty2`]
module"]
pub type CDTY2 = crate::Reg<cdty2::CDTY2_SPEC>;
#[doc = "PWM Channel Duty Cycle Register (ch_num = 2)"]
pub mod cdty2;
#[doc = "CDTYUPD2 (w) register accessor: PWM Channel Duty Cycle Update Register (ch_num = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdtyupd2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdtyupd2`]
module"]
pub type CDTYUPD2 = crate::Reg<cdtyupd2::CDTYUPD2_SPEC>;
#[doc = "PWM Channel Duty Cycle Update Register (ch_num = 2)"]
pub mod cdtyupd2;
#[doc = "CPRD2 (rw) register accessor: PWM Channel Period Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cprd2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprd2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprd2`]
module"]
pub type CPRD2 = crate::Reg<cprd2::CPRD2_SPEC>;
#[doc = "PWM Channel Period Register (ch_num = 2)"]
pub mod cprd2;
#[doc = "CPRDUPD2 (w) register accessor: PWM Channel Period Update Register (ch_num = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprdupd2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprdupd2`]
module"]
pub type CPRDUPD2 = crate::Reg<cprdupd2::CPRDUPD2_SPEC>;
#[doc = "PWM Channel Period Update Register (ch_num = 2)"]
pub mod cprdupd2;
#[doc = "CCNT2 (r) register accessor: PWM Channel Counter Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ccnt2::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ccnt2`]
module"]
pub type CCNT2 = crate::Reg<ccnt2::CCNT2_SPEC>;
#[doc = "PWM Channel Counter Register (ch_num = 2)"]
pub mod ccnt2;
#[doc = "DT2 (rw) register accessor: PWM Channel Dead Time Register (ch_num = 2)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dt2::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dt2::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dt2`]
module"]
pub type DT2 = crate::Reg<dt2::DT2_SPEC>;
#[doc = "PWM Channel Dead Time Register (ch_num = 2)"]
pub mod dt2;
#[doc = "DTUPD2 (w) register accessor: PWM Channel Dead Time Update Register (ch_num = 2)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dtupd2::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dtupd2`]
module"]
pub type DTUPD2 = crate::Reg<dtupd2::DTUPD2_SPEC>;
#[doc = "PWM Channel Dead Time Update Register (ch_num = 2)"]
pub mod dtupd2;
#[doc = "CMR3 (rw) register accessor: PWM Channel Mode Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cmr3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cmr3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cmr3`]
module"]
pub type CMR3 = crate::Reg<cmr3::CMR3_SPEC>;
#[doc = "PWM Channel Mode Register (ch_num = 3)"]
pub mod cmr3;
#[doc = "CDTY3 (rw) register accessor: PWM Channel Duty Cycle Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cdty3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdty3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdty3`]
module"]
pub type CDTY3 = crate::Reg<cdty3::CDTY3_SPEC>;
#[doc = "PWM Channel Duty Cycle Register (ch_num = 3)"]
pub mod cdty3;
#[doc = "CDTYUPD3 (w) register accessor: PWM Channel Duty Cycle Update Register (ch_num = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cdtyupd3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cdtyupd3`]
module"]
pub type CDTYUPD3 = crate::Reg<cdtyupd3::CDTYUPD3_SPEC>;
#[doc = "PWM Channel Duty Cycle Update Register (ch_num = 3)"]
pub mod cdtyupd3;
#[doc = "CPRD3 (rw) register accessor: PWM Channel Period Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`cprd3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprd3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprd3`]
module"]
pub type CPRD3 = crate::Reg<cprd3::CPRD3_SPEC>;
#[doc = "PWM Channel Period Register (ch_num = 3)"]
pub mod cprd3;
#[doc = "CPRDUPD3 (w) register accessor: PWM Channel Period Update Register (ch_num = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`cprdupd3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`cprdupd3`]
module"]
pub type CPRDUPD3 = crate::Reg<cprdupd3::CPRDUPD3_SPEC>;
#[doc = "PWM Channel Period Update Register (ch_num = 3)"]
pub mod cprdupd3;
#[doc = "CCNT3 (r) register accessor: PWM Channel Counter Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ccnt3::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ccnt3`]
module"]
pub type CCNT3 = crate::Reg<ccnt3::CCNT3_SPEC>;
#[doc = "PWM Channel Counter Register (ch_num = 3)"]
pub mod ccnt3;
#[doc = "DT3 (rw) register accessor: PWM Channel Dead Time Register (ch_num = 3)\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`dt3::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dt3::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dt3`]
module"]
pub type DT3 = crate::Reg<dt3::DT3_SPEC>;
#[doc = "PWM Channel Dead Time Register (ch_num = 3)"]
pub mod dt3;
#[doc = "DTUPD3 (w) register accessor: PWM Channel Dead Time Update Register (ch_num = 3)\n\nYou can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`dtupd3::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`dtupd3`]
module"]
pub type DTUPD3 = crate::Reg<dtupd3::DTUPD3_SPEC>;
#[doc = "PWM Channel Dead Time Update Register (ch_num = 3)"]
pub mod dtupd3;
#[doc = "TPR (rw) register accessor: Transmit Pointer Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tpr::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tpr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`tpr`]
module"]
pub type TPR = crate::Reg<tpr::TPR_SPEC>;
#[doc = "Transmit Pointer Register"]
pub mod tpr;
#[doc = "TCR (rw) register accessor: Transmit Counter Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tcr::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tcr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`tcr`]
module"]
pub type TCR = crate::Reg<tcr::TCR_SPEC>;
#[doc = "Transmit Counter Register"]
pub mod tcr;
#[doc = "TNPR (rw) register accessor: Transmit Next Pointer Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tnpr::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tnpr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`tnpr`]
module"]
pub type TNPR = crate::Reg<tnpr::TNPR_SPEC>;
#[doc = "Transmit Next Pointer Register"]
pub mod tnpr;
#[doc = "TNCR (rw) register accessor: Transmit Next Counter Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`tncr::R`].  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`tncr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`tncr`]
module"]
pub type TNCR = crate::Reg<tncr::TNCR_SPEC>;
#[doc = "Transmit Next Counter Register"]
pub mod tncr;
#[doc = "PTCR (w) register accessor: Transfer Control Register\n\nYou can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ptcr::W`]. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ptcr`]
module"]
pub type PTCR = crate::Reg<ptcr::PTCR_SPEC>;
#[doc = "Transfer Control Register"]
pub mod ptcr;
#[doc = "PTSR (r) register accessor: Transfer Status Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ptsr::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`ptsr`]
module"]
pub type PTSR = crate::Reg<ptsr::PTSR_SPEC>;
#[doc = "Transfer Status Register"]
pub mod ptsr;
