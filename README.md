# OpenGDEMU

### Warning:
At the moment this is a WIP. DO NOT ATTEMPT to flash unless you are okay with having a non-functional
GDEMU (or clone) since I do not have a way to revert to the stock firmware. Even the notes aren't going
to be 100% perfect, I've tried to put little questions when I can't remember perfectly and hope to erase
some of those questions as time goes on.

### Required Pre-Reqs
- #### cmake
- #### arm-none-eabi-gcc
- #### docker
    - Generating Verilog from SpinalHDL
    - Running Quartus 2
        - This program is a nightmare to get working locally on any modern
            linux machine. Will need to put your license.dat file into the Containers/QuartusII folder.
            Lots of broken dependencies and old static libraries, so avoid trying a local install. 
            Open source bitstream generation when :'(

### Current Status
- Can successfully execute code on the SAM3U
- Communicate between SDCard and SAM3U
- Able to load bitstream into FPGA
- Generate the clock signal (for audio iirc?) so a Dreamcast will boot properly
- Communicate with registers in FPGA using SAM3U Static Memory Controller*
- Dreamcast can Read IDE registers*

\* These arent working great. Last I touched this I was having issues I believe came down to bus timings. 
Was seeing symptoms like some bits flipped on some bytes. Was kinda a pain to hookup any kind of logic
analyzer, would be nice to make a board that broke out some of the extra unused fpga pins for debug :) 

### Goals
I really wanted to be able to try and see what other features could be added past the support in the
official (or clone) firmware. Initially it was the complaint that you can't use a GDEMU with a secondary
IDE disk. In addition I'd like to support some further features that I beleieve would be useful for 
both development and future homebrew could take advantage of. Something like the following:

1. Add a custom commannd to "eject" a GDROM image
2. Emulate an IDE hard drive on the secondary interface that just exposes the SDCard to the
   Dreamcast. Preferibly with write support. This would allow a piece of homebrew (ex. a menu launcher) to update CDIs/GDIs. Perhaps extending dcload to write your updated assets image in a developers build script?
3. Add a custom command with a path parameter on the secondary IDE device to "eject" itself and
   remount a GDROM image


### My Current TODOs
- [ ] Fix hard coded paths, I know some are to my user directory :sweat_smile:
- [ ] Check if FatFS is working
- [ ] Answer documentation questions that are lack of research
- [ ] KiCAD PCB Design
- [ ] Make it work
- [ ] Put some work into breaking up above task into portions ;)

### Just on the off chance that maybe things go too easily
- Would be pretty cool to compile the Verilog to C with Verilator and hook up to Lxdream-nitro. 
Might be too slow, never used Verilator before this just sounded like a good excuse to try and learn it


### Projects in Use
- [CMSIS]()
- [Trice](https://github.com/rokath/trice) MIT
- [Argon RTOS ](https://github.com/flit/argon-rtos) BSD-3-Clause
- [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) [FatFS License](http://elm-chan.org/fsw/ff/doc/appnote.html#license)
- [Microchip ASF](https://github.com/alfred-ai/microchip-asf)
- [umm_malloc](https://github.com/rhempel/umm_malloc) MIT
- [SpinalHDL](https://github.com/SpinalHDL/SpinalHDL)
- [VexRiscv](https://github.com/SpinalHDL/VexRiscv)

Any omissions are not intentional. Please open an issue if you notice any missed attributions.