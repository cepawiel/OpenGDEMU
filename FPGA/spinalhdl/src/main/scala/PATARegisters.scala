package opengdemu

import spinal.core._
import spinal.lib._

case class PATARegisters(bus : PATAInterface, deviceID : Int) extends Area {
    val regBlock0 = (bus.CS === B"01")   // CS1 = 0         CS0 = 1
    val regBlock1 = (bus.CS === B"10")   // CS1 = 1         CS0 = 0
    val dmaTransfer = (bus.CS === B"00") && bus.DMARQ && bus.DMACK

    bus.DMARQ := False
    bus.INTRQ := False
    bus.IORDY := True

    bus.DATA_OUT_EN := bus.RD & (regBlock0 | regBlock1 | dmaTransfer)
    
    val RD = bus.RD
    val WR = bus.WR
    val WR_STROBE = bus.WR.fall

    //=====================
    // Device Register
    //=====================
    val DEVICE = Reg(Bits(8 bits)) init(0)
    val DEV = DEVICE(4)

    // Device Select, MASTER=0, SLAVE=1
    val devID = deviceID
    val deviceSelected = if (deviceID == 0) (DEV === False) else (DEV === True);

    // High Order Bit, controls read access to upper bits for LBA48
    val HOB = Reg(Bool()) init(False)

    // Software Reset
    val SRST = Reg(Bool()) init(False)

    // Interupt Enable for INTRQ
    val nIEN = Reg(Bool()) init(False)
    // Assert IRQ when nIEN=1 and device is selected
    // bus.INTRQ := !nIEN && DEV

    //=====================
    // Status Register
    //=====================
    // Busy
    val BSY = Reg(Bool()) init(True)
    // Device Ready
    val DRDY = Reg(Bool()) init(False)
    // Device Fault
    val DF = Reg(Bool()) init(False)
    // Data request
    val DRQ = Reg(Bool()) init(False)
    // Error
    val ERR = Reg(Bool()) init(False)

    //=====================
    // Command Register
    //=====================
    val COMMAND = Reg(Bits(8 bits)) init(0)
    val COMMAND_PEND = Reg(Bool()) init(False)

    //=====================
    // LBA Registers
    //=====================
    val LBALow          = Reg(Bits(8 bits)) init(0)
    val LBALowPrev      = Reg(Bits(8 bits)) init(0)
    val LBAMid          = Reg(Bits(8 bits)) init(0)
    val LBAMidPrev      = Reg(Bits(8 bits)) init(0)
    val LBAHigh         = Reg(Bits(8 bits)) init(0)
    val LBAHighPrev     = Reg(Bits(8 bits)) init(0)

    //=====================
    // Error Registers
    //=====================
    val ABRT            = Reg(Bool()) init(False)

    //=====================
    // Features Registers
    //=====================
    val Features        = Reg(Bits(8 bits)) init(0)

    //=====================
    // Sector Count Register
    //=====================
    val SectorCount     = Reg(Bits(16 bits)) init(0)

    //=====================
    // DATA FIFOs (2KBytes)
    //=====================
    // val readFIFO = new scfifo(16, 1024)
    // val writeFIFO = new scfifo(16, 1024)
    val readFIFOStream = new StreamFifo(dataType = Bits(16 bits), depth = 1024)
    val writeFIFOStream = new StreamFifo(dataType = Bits(16 bits), depth = 1024)


    readFIFOStream.io.pop.ready     := False
    writeFIFOStream.io.push.valid   := False
    writeFIFOStream.io.push.payload := bus.DATA_IN

    // Default Data Output
    // bus.DATA_OUT := "16'xDADA"
    bus.DATA_OUT := 0

    readFIFOStream.io.flush := False
    writeFIFOStream.io.flush := False

    when (SRST | bus.RST) {
        BSY         := True
        DRDY        := False
        DRQ         := False

        // clear fifos
        readFIFOStream.io.flush := True
        writeFIFOStream.io.flush := True

        COMMAND     := 0
        LBALow      := 0
        LBALowPrev  := 0
        LBAMid      := 0
        LBAMidPrev  := 0
        LBAHigh     := 0
        LBAHighPrev := 0
        COMMAND_PEND:= False
    }.elsewhen(SRST.fall) {
        BSY         := False
        DRDY        := True
    }

    when (regBlock0 && bus.ADDR === U"110" && WR) {
        // DEV - Device select. Cleared to Zero selects Device 0. Set to one selects Device 1.
        DEVICE     := bus.DATA_IN(7 downto 0)
    }

    when (deviceSelected) {
        when (dmaTransfer) {
            // transfer data 16 bits
            bus.DATA_OUT                    := "16'xDADA"
        }

        when (regBlock0) {
            when (bus.ADDR === U"000") {
                // 0        Data Register (16 Bits)
                // transfer data 16 bits
                when (RD) {
                    bus.DATA_OUT               := readFIFOStream.io.pop.payload
                }
                when (RD.fall) {
                    readFIFOStream.io.pop.ready  := True
                }
                when (WR) {
                    writeFIFOStream.io.push.valid := True
                }
            }
            when (bus.ADDR === U"001") {
                when (RD) {
                    // 1        Error Register (R)
                    // valid when ERR=1 and BSY=DRQ=0
                    bus.DATA_OUT(2)     := ABRT
                }
                when (WR) {
                    // 1        Features Register (W)
                    Features    := bus.DATA_IN(7 downto 0)
                }
            }
            when (bus.ADDR === U"010") {
                // 2        Sector Count Register
                // 0 = 256 Sectors
                // 255 = 255 Sectors
                // decremented each time a sector is read
                when (RD) {
                    when (HOB) {
                        bus.DATA_OUT(7 downto 0)    := SectorCount(15 downto 8)
                    } otherwise {
                        bus.DATA_OUT(7 downto 0)    := SectorCount(7 downto 0)
                    }
                }
                when (WR) {
                    SectorCount(15 downto 8)     := SectorCount(7 downto 0)
                    SectorCount(7 downto 0)      := bus.DATA_IN(7 downto 0)
                }
            }
            when (bus.ADDR === U"011") {
                // LBA Low Register
                // Bits 0-7 of LBA
                // Updated at end of command to reflect last accessed register
                // Multi Sector Txfr update to next sector if previous was successful
                when (RD) {
                    when (HOB) {
                        bus.DATA_OUT(7 downto 0)    := LBALowPrev
                    } otherwise {
                        bus.DATA_OUT(7 downto 0)    := LBALow
                    }
                }
                when (WR) {
                    LBALowPrev      := LBALow
                    LBALow          := bus.DATA_IN(7 downto 0)
                }
            }
            when (bus.ADDR === U"100") {
                // LBA Mid Register
                // Bits 8-15 of LBA
                when (RD) {
                    when (HOB) {
                        bus.DATA_OUT(7 downto 0)    := LBAMidPrev
                    } otherwise {
                        bus.DATA_OUT(7 downto 0)    := LBAMid
                    }
                }
                when (WR) {
                    LBAMidPrev      := LBAMid
                    LBAMid          := bus.DATA_IN(7 downto 0)
                }
            }
            when (bus.ADDR === U"101") {
                // LBA High Register
                // Bits 16-23 of LBA
                when (RD) {
                    when (HOB) {
                        bus.DATA_OUT(7 downto 0)    := LBAHighPrev
                    } otherwise {
                        bus.DATA_OUT(7 downto 0)    := LBAHigh
                    }
                }
                when (WR) {
                    LBAHighPrev     := LBAHigh
                    LBAHigh         := bus.DATA_IN(7 downto 0)
                }
            }
            when (bus.ADDR === U"110") {
                // Device Register
                // Only be written when both BSY and DRQ are cleared to zero
                // and DMACK is not asserted.

                // Only valid when BSY=0. 
                // DEV - Device select. Cleared to Zero selects Device 0. Set to one selects Device 1.
                when (RD) {
                    bus.DATA_OUT(7 downto 0) := DEVICE
                }
                when (WR) {
                    // DEVICE     := bus.DATA_IN(7 downto 0)
                }
            }
            when (bus.ADDR === U"111") {
                when (RD) {
                    // 7        Status Register (R)
                    // Reading this Reg will clear pending interrupts
                    bus.DATA_OUT(7) := BSY
                    bus.DATA_OUT(6) := DRDY
                    bus.DATA_OUT(5) := DF
                    bus.DATA_OUT(3) := DRQ
                    bus.DATA_OUT(0) := ERR
                }
                when (WR) {
                    // 7        Command Register (W)
                    COMMAND      := bus.DATA_IN(7 downto 0)
                    BSY          := True
                    COMMAND_PEND := True
                }
                when (WR.fall) {
                    // DRDY         := False
                    // DRQ          := False
                    // ERR          := False
                    // HOB          := False
                }

            }
        }

        when (regBlock1) {
            when (bus.ADDR === U"000") {
                when (RD) {
                    bus.DATA_OUT(7 downto 0)        := COMMAND
                    bus.DATA_OUT(15 downto 8)       := "8'x5A"
                }
            }
            when (bus.ADDR === U"110") {
                when (RD) {
                    // Alternative Status Register (R)
                    bus.DATA_OUT(7) := BSY
                    bus.DATA_OUT(6) := DRDY
                    bus.DATA_OUT(5) := DF
                    bus.DATA_OUT(3) := DRQ
                    bus.DATA_OUT(0) := ERR
                }
                when (WR) {
                    // Device Control Register (W)
                    HOB  := bus.DATA_IN(7)
                    SRST := bus.DATA_IN(2)
                    nIEN := bus.DATA_IN(1)
                                    
                }
            }
            when (bus.ADDR === U"111") {
                // DBG
                when (RD) {
                    bus.DATA_OUT(15 downto 11)       := "5'b00000"
                    bus.DATA_OUT(10 downto 0)        := readFIFOStream.io.occupancy.asBits
                }
            }
        }
    }
}