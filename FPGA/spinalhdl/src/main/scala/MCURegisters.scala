package opengdemu

import spinal.core._
import spinal.lib._

case class MCURegisters(mcuBus : MCUInterface, pataRegs : PATARegisters) extends Area {
    mcuBus.DATA_OUT := 0

    var irqDelay = History(pataRegs.COMMAND_PEND, 4)
    mcuBus.IRQ := irqDelay(3)
    // mcuBus.IRQ := pataRegs.COMMAND_PEND

    pataRegs.readFIFOStream.io.push.valid       := False
    pataRegs.readFIFOStream.io.push.payload     := mcuBus.DATA_IN
    pataRegs.writeFIFOStream.io.pop.ready       := False

    when (mcuBus.ADDR === 0) {
        if(pataRegs.devID == 0) { // if master
            mcuBus.DATA_OUT(0) := False
        } else {
            mcuBus.DATA_OUT(0) := True
        }

        mcuBus.DATA_OUT(1) := pataRegs.COMMAND_PEND

        when (mcuBus.WR(0)) {
            pataRegs.COMMAND_PEND := mcuBus.DATA_IN(1)
            pataRegs.readFIFOStream.io.flush := True
        }
    }

    when (mcuBus.ADDR === 1) {
        mcuBus.DATA_OUT(0) := pataRegs.ERR
        mcuBus.DATA_OUT(3) := pataRegs.DRQ
        mcuBus.DATA_OUT(5) := pataRegs.DF
        mcuBus.DATA_OUT(6) := pataRegs.DRDY
        mcuBus.DATA_OUT(7) := pataRegs.BSY
            
        when (mcuBus.WR(0)) {
            pataRegs.ERR    := mcuBus.DATA_IN(0)
            pataRegs.DRQ    := mcuBus.DATA_IN(3)
            pataRegs.DF     := mcuBus.DATA_IN(5)
            pataRegs.DRDY   := mcuBus.DATA_IN(6)
            pataRegs.BSY    := mcuBus.DATA_IN(7)
        }
    }

    when (mcuBus.ADDR === 2) {
        mcuBus.DATA_OUT(7 downto 0) := pataRegs.DEVICE
        when (mcuBus.WR(0)) {
            pataRegs.DEVICE         := mcuBus.DATA_IN(7 downto 0)
        }
    }

    when (mcuBus.ADDR === 3) {
        mcuBus.DATA_OUT(3)           := pataRegs.ABRT
        when (mcuBus.WR(0)) {
            pataRegs.ABRT             := mcuBus.DATA_IN(3)
        }
    }

    when (mcuBus.ADDR === 4) {
        mcuBus.DATA_OUT(7 downto 0)         := pataRegs.COMMAND
        mcuBus.DATA_OUT(15 downto 8)        := pataRegs.Features
        when (mcuBus.WR(0)) {
            pataRegs.COMMAND                := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            pataRegs.Features               := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 5) {
        mcuBus.DATA_OUT     := pataRegs.SectorCount
        
        when (mcuBus.WR(0)) {
            pataRegs.SectorCount(7 downto 0)    := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            pataRegs.SectorCount(15 downto 8)    := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 6) {
        mcuBus.DATA_OUT(7 downto 0)  := pataRegs.LBALow
        mcuBus.DATA_OUT(15 downto 8) := pataRegs.LBAMid
        when (mcuBus.WR(0)) {
            pataRegs.LBALow                := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            pataRegs.LBAMid                := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 7) {
        // LBA(31:16)
        mcuBus.DATA_OUT(7 downto 0)  := pataRegs.LBAHigh
        mcuBus.DATA_OUT(15 downto 8) := pataRegs.LBALowPrev
        when (mcuBus.WR(0)) {
            pataRegs.LBAHigh               := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            pataRegs.LBALowPrev            := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 8) {
        // LBA(47:32)
        mcuBus.DATA_OUT(7 downto 0)  := pataRegs.LBAMidPrev
        mcuBus.DATA_OUT(15 downto 8) := pataRegs.LBAHighPrev
        when (mcuBus.WR(0)) {
            pataRegs.LBAMidPrev            := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            pataRegs.LBAHighPrev           := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 9) {
        mcuBus.DATA_OUT                         := pataRegs.writeFIFOStream.io.pop.payload
        pataRegs.writeFIFOStream.io.pop.ready   := True
        
        when (mcuBus.WR(0) & mcuBus.WR(1)) {
            pataRegs.readFIFOStream.io.push.valid       := True
        }
    }
    
    when (mcuBus.ADDR === 10) {
        mcuBus.DATA_OUT(10 downto 0)        := pataRegs.readFIFOStream.io.occupancy.asBits
    }

    when (mcuBus.ADDR === 11) {
        mcuBus.DATA_OUT(10 downto 0)        := pataRegs.writeFIFOStream.io.occupancy.asBits
    }

    when (mcuBus.ADDR === 12) {
        mcuBus.DATA_OUT                  := "16'xCAFE"
    }

}
