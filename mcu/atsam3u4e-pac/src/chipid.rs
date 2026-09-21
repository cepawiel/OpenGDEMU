#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    _reserved0: [u8; 0x04],
    #[doc = "0x04 - Chip ID Extension Register"]
    pub exid: EXID,
}
#[doc = "EXID (r) register accessor: Chip ID Extension Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`exid::R`].  See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`exid`]
module"]
pub type EXID = crate::Reg<exid::EXID_SPEC>;
#[doc = "Chip ID Extension Register"]
pub mod exid;
