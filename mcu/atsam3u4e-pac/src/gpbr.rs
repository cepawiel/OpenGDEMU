#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00..0x10 - General Purpose Backup Register"]
    pub gpbr: [GPBR; 4],
}
#[doc = "GPBR (rw) register accessor: General Purpose Backup Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`gpbr::R`].  You can [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`gpbr::W`]. You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [`gpbr`]
module"]
pub type GPBR = crate::Reg<gpbr::GPBR_SPEC>;
#[doc = "General Purpose Backup Register"]
pub mod gpbr;
