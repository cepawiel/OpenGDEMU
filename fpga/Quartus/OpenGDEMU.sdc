## Generated SDC file "OpenGDEMU.sdc"

## Copyright (C) 1991-2013 Altera Corporation
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, Altera MegaCore Function License 
## Agreement, or other applicable license agreement, including, 
## without limitation, that your use is for the sole purpose of 
## programming logic devices manufactured by Altera and sold by 
## Altera or its authorized distributors.  Please refer to the 
## applicable agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus II"
## VERSION "Version 13.0.1 Build 232 06/12/2013 Service Pack 1 SJ Web Edition"

## DATE    "Mon Mar  7 14:44:51 2022"

##
## DEVICE  "EP2C5T144C8"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {CLK_11_2896_MHz} -period 88.577 -waveform { 0.000 44.290 } [get_ports {CLK_11_2896_MHz}]
derive_pll_clocks
derive_clock_uncertainty
create_clock -name {CLK_48_MHz} -period 20.833 -waveform { 0.000 10.416 } [get_ports {CLK_48_MHz}]


#**************************************************************
# Create Generated Clock
#**************************************************************

# The 33.8688 MHz CDCLK PLL output is picked up by derive_pll_clocks above.


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

# CLK_48_MHz (core) and CLK_11_2896_MHz (audio xtal, plus its PLL'd
# 33.8688 MHz CDCLK) are unrelated. Everything crossing between them goes
# through a 2-FF synchronizer (audio_tick heartbeat, the CDDA frame
# request toggle) or is a register that settles ~20 us before it is next
# sampled (the CDDA sample pair); none of it should be timed cross-domain.
# Without this assertion the timing analyzer reports -1 ns setup slack on
# the audio_div → audio_sync path.
set_clock_groups -asynchronous \
    -group {CLK_48_MHz} \
    -group [get_clocks {CLK_11_2896_MHz audio_pll_inst|altpll_component|pll|clk[0]}]


#**************************************************************
# Set False Path
#**************************************************************



#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************



#**************************************************************
# External bus constraints
#**************************************************************
# Both external buses are asynchronous -- neither the Dreamcast nor the SAM3U
# hands us a clock -- so without these TimeQuest checks nothing on 56 input and
# 37 output ports, and an "8.5 ns slack" result only ever described internal
# register-to-register paths. That is how adding a debug capture buffer could
# change real bus behaviour while static timing stayed green.

# --- SAM3U SMC bus -------------------------------------------------------
# Read access budget: 1 MCK setup + 8 MCK pulse at 96 MHz is ~94 ns, less the
# SAM3U's own data setup and board delay. 60 ns leaves margin and is what the
# path should comfortably meet now that MCU_DATA comes from a register.
set_max_delay -from [get_ports {MCU_ADDR[*] MCU_NCS MCU_NRE MCU_NBS[*]}] \
              -to   [get_ports {MCU_DATA[*]}] 60.000
set_min_delay -from [get_ports {MCU_ADDR[*] MCU_NCS MCU_NRE MCU_NBS[*]}] \
              -to   [get_ports {MCU_DATA[*]}] 0.000

# Write data and the strobes are captured in the 48 MHz domain through
# synchronizers; bound the skew between them rather than leaving it unchecked.
set_max_delay -from [get_ports {MCU_DATA[*] MCU_ADDR[*] MCU_NCS MCU_NWE MCU_NBS[*]}] \
              -to   [get_clocks {CLK_48_MHz}] 20.000

# --- Dreamcast G1 IDE bus ------------------------------------------------
# Outputs are sampled by the host against its own strobe. Multiword DMA mode 2
# gives tE = 50 ns from DIOR- asserted to data valid. 30 ns was inside that on
# paper but left the fitter no reason to do better than ~25 ns. The read data
# path is M4K address register -> RAM -> output mux -> pad, which is ~16 ns
# at best (12 ns was tried: -4.3 ns slack, unmeetable without an output
# register), so 20 ns is the tightest honest budget. A constraint that can
# never be met just hides the next real regression behind a permanent
# "timing requirements not met". (The read-DMA corruption that prompted
# tightening this turned out to be a FIFO refill race, fixed in
# ide_device.py, not an output-timing problem.)
set_max_delay -from [get_clocks {CLK_48_MHz}] \
              -to   [get_ports {DC_DATA[*] DC_INTRQ DC_DMARQ DC_IORDY}] 20.000

# Inputs: what matters is the skew between the data/address/CS lines and the
# strobe that qualifies them, since ide_data_latch samples data gated on the
# raw strobe level. Bound them together rather than false-pathing them.
set_max_delay -from [get_ports {DC_DATA[*] DC_ADDR[*] DC_CSn[*] DC_RDn DC_WRn DC_DMACKn}] \
              -to   [get_clocks {CLK_48_MHz}] 20.000

# DC_RSTn only gates tri-states and the reset state machine; it has no timing
# relationship worth checking.
set_false_path -from [get_ports {DC_RSTn}]

# --- CDDA serial audio ---------------------------------------------------
# The AICA samples SDAT/LRCK on the rising edge of SCK; all three are
# registered on the same 11.2896 MHz edge, so their relative skew is one
# pad delay each. Bound the clock-to-pad so a regression shows up.
set_max_delay -from [get_clocks {CLK_11_2896_MHz}] \
              -to   [get_ports {DC_SCK DC_SDAT DC_LRCK}] 15.000
