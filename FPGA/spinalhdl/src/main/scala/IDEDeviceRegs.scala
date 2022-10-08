package opengdemu

import spinal.core._
import spinal.lib._

class IDEDeviceRegs(bus : IDEBus, mcuBus: MCUBus, primary : Boolean) extends Area {
    val dataOutput   = Reg(Bits(16 bits)) init(0xFFFF)
    val dataOutputEn = Reg(Bool()) init(False) allowUnsetRegToAvoidLatch

    val mcuDataOutput = Reg(Bits(16 bits)) init(0xFFFF) allowUnsetRegToAvoidLatch
    val mcuDataOutputEn = Reg(Bits(2 bits)) init(0) allowUnsetRegToAvoidLatch
    val mcuIRQ = Reg(Bool()) init(False)

    val RD = ~bus.RDn
    val WR = ~bus.WRn
    val WR_STROBE = WR.fall

    //==============================================================
    // Device Register
    //==============================================================
    val DEVICE = Reg(Bits(8 bits)) init(0)
    val DEV = DEVICE(4)

    // High Order Bit, controls read access to upper bits for LBA48
    val HOB = Reg(Bool()) init(False)

    // Software Reset
    val SRST = Reg(Bool()) init(False)

    // Interupt Enable for INTRQ
    val nIEN = Reg(Bool()) init(False)
    // Assert IRQ when nIEN=1 and device is selected
    // bus.INTRQ := !nIEN && DEV

    //==============================================================
    // Status Register
    //==============================================================
    // Busy
    val BSY = Reg(Bool()) init(True)
    // Device Ready
    val DRDY = Reg(Bool()) init(False)
    // Device Fault
    val DF = Reg(Bool()) init(False) allowUnsetRegToAvoidLatch
    // Data request
    val DRQ = Reg(Bool()) init(False)
    // Error
    val ERR = Reg(Bool()) init(False) allowUnsetRegToAvoidLatch

    //==============================================================
    // Command Register
    //==============================================================
    val COMMAND = Reg(Bits(8 bits)) init(0)
    val COMMAND_PEND = Reg(Bool()) init(False)

    //==============================================================
    // LBA Registers
    //==============================================================
    val LBALow          = Reg(Bits(8 bits)) init(0)
    val LBALowPrev      = RegNextWhen(LBALow, False) init(0)
    val LBAMid          = Reg(Bits(8 bits)) init(0)
    val LBAMidPrev      = RegNextWhen(LBAMid, False) init(0)
    val LBAHigh         = Reg(Bits(8 bits)) init(0)
    val LBAHighPrev     = RegNextWhen(LBAHigh, False) init(0)

    //==============================================================
    // Error Registers
    //==============================================================
    val ABRT            = Reg(Bool()) init(False) allowUnsetRegToAvoidLatch

    //==============================================================
    // Features Registers
    //==============================================================
    val Features        = Reg(Bits(8 bits)) init(0)

    //==============================================================
    // Sector Count Register
    //==============================================================
    val SectorCount     = Reg(Bits(16 bits)) init(0)

    //==============================================================
    // DATA FIFOs (2KBytes)
    //==============================================================
    val readFIFOStream = new StreamFifo(dataType = Bits(16 bits), depth = 1024)
    val writeFIFOStream = new StreamFifo(dataType = Bits(16 bits), depth = 1024)

    readFIFOStream.io.flush := False
    writeFIFOStream.io.flush := False

    readFIFOStream.io.pop.ready     := False
    writeFIFOStream.io.push.valid   := False
    writeFIFOStream.io.push.payload := bus.DATA_IN

    //==============================================================
    // readFIFOStream.io.push.valid := False
    // readFIFOStream.io.push.payload := 0
    val testReg = Reg(Bits(16 bits)) init(0xCAFE)
    //==============================================================

    //==============================================================
    // Helper Functions
    //==============================================================

    //==============================================================
    // Reset Logic
    //==============================================================
    when (SRST | ~bus.RSTn) {
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

    // Reset dataOutputEn Reg when read is finished
    when (dataOutputEn && bus.RDn) {
        dataOutputEn := False
    }

    //==============================================================
    // IDE Reg Decode Logic
    //==============================================================
    // val regBlock0 = (bus.CSn === B"10")   // CS1 = 0         CS0 = 1
    // val regBlock1 = (bus.CSn === B"01")   // CS1 = 1         CS0 = 0
    val dmaTransfer = (bus.CSn === B"11") && bus.DMARQ && ~bus.DMACKn

    val ideDecode = IDEAddressDecode();
    ideDecode.io.CSn := bus.CSn
    ideDecode.io.ADDR := bus.ADDR

    // Device Select, MASTER=0, SLAVE=1
    val deviceSelected = if (primary) (DEV === False) else (DEV === True);


    //==============================================================
    // IDE Device Select Bit
    //==============================================================
    when (ideDecode.io.DeviceRegister && WR) {
        // DEV - Device select. Cleared to Zero selects Device 0. Set to one selects Device 1.
        DEVICE     := bus.DATA_IN(7 downto 0)
    }

    //==============================================================
    // IDE DMA
    //==============================================================
    when (deviceSelected && dmaTransfer) {
        // transfer data 16 bits
        dataOutput                    := "16'xDADA"
        //dataOutputEn := True
    }

    //==============================================================
    // IDE Regs
    //==============================================================
    // val dataRegister = IDERegister(0);

    when ( deviceSelected && ideDecode.io.DataRegister ) {
        // 0        Data Register (16 Bits)
        // transfer data 16 bits
        dataOutput               := readFIFOStream.io.pop.payload
        when (RD) {
            dataOutputEn := True
        }
        when (RD.fall) {
            readFIFOStream.io.pop.ready  := True
        }
        when (WR) {
            writeFIFOStream.io.push.valid := True
        }
    }

    when ( deviceSelected && ideDecode.io.ErrorRegister ) {
        // 1        Error Register (R)
        // valid when ERR=1 and BSY=DRQ=0
        dataOutput(2)     := ABRT
        when (RD) {
            dataOutputEn := True
        }
    }
    when ( deviceSelected && ideDecode.io.FeaturesRegister ) {
        when (WR) {
            // 1        Features Register (W)
            Features    := bus.DATA_IN(7 downto 0)
        }
    }

    when ( deviceSelected && ideDecode.io.SectorCountRegister ) {
        // 2        Sector Count Register
        // 0 = 256 Sectors
        // 255 = 255 Sectors
        // decremented each time a sector is read
        when (HOB) {
            dataOutput(7 downto 0)    := SectorCount(15 downto 8)
        } otherwise {
            dataOutput(7 downto 0)    := SectorCount(7 downto 0)
        }
        when (RD) {   
            dataOutputEn := True
        }
        when (WR) {
            SectorCount(15 downto 8)     := SectorCount(7 downto 0)
            SectorCount(7 downto 0)      := bus.DATA_IN(7 downto 0)
        }
    }

    when ( deviceSelected && ideDecode.io.LBALowRegister ) {
        // LBA Low Register
        // Bits 0-7 of LBA
        // Updated at end of command to reflect last accessed register
        // Multi Sector Txfr update to next sector if previous was successful
        when (HOB) {
            dataOutput(7 downto 0)    := LBALowPrev
        } otherwise {
            dataOutput(7 downto 0)    := LBALow
        }
        when (RD) {
            dataOutputEn := True
        }
        when (WR) {
            // LBALowPrev      := LBALow
            LBALow          := bus.DATA_IN(7 downto 0)
        }
    }
    when ( deviceSelected && ideDecode.io.LBAMidRegister ) {
        // LBA Mid Register
        // Bits 8-15 of LBA
        when (HOB) {
            dataOutput(7 downto 0)    := LBAMidPrev
        } otherwise {
            dataOutput(7 downto 0)    := LBAMid
        }
        when (RD) {
            dataOutputEn := True
        }
        when (WR) {
            // LBAMidPrev      := LBAMid
            LBAMid          := bus.DATA_IN(7 downto 0)
        }
    }
    when ( deviceSelected && ideDecode.io.LBAHighRegister ) {
        // LBA High Register
        // Bits 16-23 of LBA
        when (HOB) {
            dataOutput(7 downto 0)    := LBAHighPrev
        } otherwise {
            dataOutput(7 downto 0)    := LBAHigh
        }
        when (RD) {
            dataOutputEn := True
        }
        when (WR) {
            // LBAHighPrev     := LBAHigh
            LBAHigh         := bus.DATA_IN(7 downto 0)
        }
    }
    when ( deviceSelected && ideDecode.io.DeviceRegister ) {
        // Device Register
        // Only be written when both BSY and DRQ are cleared to zero
        // and DMACK is not asserted.

        // Only valid when BSY=0. 
        // DEV - Device select. Cleared to Zero selects Device 0. Set to one selects Device 1.
        dataOutput(7 downto 0) := DEVICE
        when (RD) {
            dataOutputEn := True
        }
        when (WR) {
            // DEVICE     := bus.DATA_IN(7 downto 0)
        }
    }
    when ( deviceSelected && ideDecode.io.StatusRegister ) {
        dataOutput(7) := BSY
        dataOutput(6) := DRDY
        dataOutput(5) := DF
        dataOutput(3) := DRQ
        dataOutput(0) := ERR
        when (RD) {
            // 7        Status Register (R)
            // Reading this Reg will clear pending interrupts
            dataOutputEn := True
        }
    }
    when ( deviceSelected && ideDecode.io.CommandRegister ) {
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

    // when ( ideRegAddressBlock(1)(0) ) {
    //     dataOutput(7 downto 0)        := COMMAND
    //     dataOutput(15 downto 8)       := "8'x5A"
    //     when (RD) {
    //         dataOutputEn := True
    //     }
    // }
    

    when ( deviceSelected && ideDecode.io.AlternateStatusRegister ) {
        dataOutput(7) := BSY
        dataOutput(6) := DRDY
        dataOutput(5) := DF
        dataOutput(3) := DRQ
        dataOutput(0) := ERR
        when (RD) {
            // Alternative Status Register (R)
            dataOutputEn := True
        }
    }
    when ( deviceSelected && ideDecode.io.DeviceControlRegister ) {
        when (WR) {
            // Device Control Register (W)
            HOB  := bus.DATA_IN(7)
            SRST := bus.DATA_IN(2)
            nIEN := bus.DATA_IN(1)
                            
        }
    }
    when ( deviceSelected && ideDecode.io.DebugTestpad ) {
        // DBG Reg (RW)
        dataOutput(15 downto 0)         := testReg
        when (RD) {
            dataOutputEn := True
        }
        when (WR) {
            testReg                 := dataOutput(15 downto 0)
        }
    }

    //==============================================================
    // MCU Regs
    //==============================================================
    var irqDelay = History(COMMAND_PEND, 4)
    mcuIRQ := irqDelay(3)
    // mcuBus.IRQ := pataRegs.COMMAND_PEND

    readFIFOStream.io.push.valid       := False
    readFIFOStream.io.push.payload     := mcuBus.DATA_IN
    writeFIFOStream.io.pop.ready       := False

    when (mcuBus.ADDR === 0) {
        if(primary) { // if master
            mcuDataOutput(0) := False
        } else {
            mcuDataOutput(0) := True
        }

        mcuDataOutput(1) := COMMAND_PEND

        when (mcuBus.WR(0)) {
            COMMAND_PEND := mcuBus.DATA_IN(1)
            readFIFOStream.io.flush := True
        }
    }

    Console.printf("Creating MCURegisterManagerGenerator\n");
    val mcuRegGen = MCURegisterManagerGenerator()
    
    mcuRegGen.add("ERR",  0x02, 0, 1, 0)  
    mcuRegGen.add("DRQ",  0x02, 3, 1, 0) 
    mcuRegGen.add("DF",   0x02, 5, 1, 0)  
    mcuRegGen.add("DRDY", 0x02, 6, 1, 0) 
    mcuRegGen.add("BSY",  0x02, 7, 1, 0) 


    mcuRegGen.generate()

    when (mcuBus.ADDR === 1) {
        mcuDataOutput(0) := ERR
        mcuDataOutput(3) := DRQ
        mcuDataOutput(5) := DF
        mcuDataOutput(6) := DRDY
        mcuDataOutput(7) := BSY
            
        when (mcuBus.WR(0)) {
            ERR    := mcuBus.DATA_IN(0)
            DRQ    := mcuBus.DATA_IN(3)
            DF     := mcuBus.DATA_IN(5)
            DRDY   := mcuBus.DATA_IN(6)
            BSY    := mcuBus.DATA_IN(7)
        }
    }

    when (mcuBus.ADDR === 2) {
        mcuDataOutput(7 downto 0) := DEVICE
        when (mcuBus.WR(0)) {
            DEVICE         := mcuBus.DATA_IN(7 downto 0)
        }
    }

    when (mcuBus.ADDR === 3) {
        mcuDataOutput(3)           := ABRT
        when (mcuBus.WR(0)) {
            ABRT             := mcuBus.DATA_IN(3)
        }
    }

    when (mcuBus.ADDR === 4) {
        mcuDataOutput(7 downto 0)         := COMMAND
        mcuDataOutput(15 downto 8)        := Features
        when (mcuBus.WR(0)) {
            COMMAND                := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            Features               := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 5) {
        mcuDataOutput     := SectorCount
        
        when (mcuBus.WR(0)) {
            SectorCount(7 downto 0)    := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            SectorCount(15 downto 8)    := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 6) {
        mcuDataOutput(7 downto 0)  := LBALow
        mcuDataOutput(15 downto 8) := LBAMid
        when (mcuBus.WR(0)) {
            LBALow                := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            LBAMid                := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 7) {
        // LBA(31:16)
        mcuDataOutput(7 downto 0)  := LBAHigh
        mcuDataOutput(15 downto 8) := LBALowPrev
        when (mcuBus.WR(0)) {
            LBAHigh               := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            LBALowPrev            := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 8) {
        // LBA(47:32)
        mcuDataOutput(7 downto 0)  := LBAMidPrev
        mcuDataOutput(15 downto 8) := LBAHighPrev
        when (mcuBus.WR(0)) {
            LBAMidPrev            := mcuBus.DATA_IN(7 downto 0)
        }
        when (mcuBus.WR(1)) {
            LBAHighPrev           := mcuBus.DATA_IN(15 downto 8)
        }
    }

    when (mcuBus.ADDR === 9) {
        mcuDataOutput                         := writeFIFOStream.io.pop.payload
        writeFIFOStream.io.pop.ready   := True
        
        when (mcuBus.WR(0) & mcuBus.WR(1)) {
            readFIFOStream.io.push.valid       := True
        }
    }
    
    when (mcuBus.ADDR === 10) {
        mcuDataOutput(10 downto 0)        := readFIFOStream.io.occupancy.asBits
    }

    when (mcuBus.ADDR === 11) {
        mcuDataOutput(10 downto 0)        := writeFIFOStream.io.occupancy.asBits
    }

    when (mcuBus.ADDR === 12) {
        mcuDataOutput                  := "16'xC0DE"
    }
}

case class PrimaryIDEDeviceRegs(bus : IDEBus, mcuBus : MCUBus) extends IDEDeviceRegs(bus, mcuBus, true) {}
case class SecondaryIDEDeviceRegs(bus : IDEBus, mcuBus : MCUBus) extends IDEDeviceRegs(bus, mcuBus, false) {}

// import spinal.core.sim._

// object TestIDEDeviceController {
//     def simulate(workspacePath : String) {
//         Console.println("Simulating TestIDEDeviceController");
//         val cfg = SimConfig
//             .withWave
//             .withIVerilog
//             .workspacePath(workspacePath)
//             .compile(new IDEDeviceController())


//         cfg.doSim("IDEDeviceController_tb") { dut =>
//                 dut.clockDomain.forkStimulus(period = 20000)

//                 dut.io.ideBus.ADDR #= 0
//                 dut.io.ideBus.CSn #= 3
//                 dut.io.ideBus.DATA_IN #= 0xA5

//                 dut.io.ideBus.DMACKn #= true
//                 dut.io.ideBus.RDn #= true
//                 dut.io.ideBus.WRn #= true

//                 dut.io.ideBus.RSTn #= false
//                 dut.clockDomain.waitFallingEdge()
                
//                 dut.io.ideBus.RSTn #= true
//                 dut.clockDomain.waitFallingEdge()

//                 def ideReadReg(csn: Int, address : Int) {
//                     dut.io.ideBus.CSn #= csn
//                     dut.io.ideBus.ADDR #= address
//                     dut.io.ideBus.RDn #= false
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     //assert(dut.io.ideBus.DATA_OUT_EN == true, "Failed to read IDE Reg")
//                     dut.io.ideBus.RDn #= true
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     //dut.io.ideBus.DATA_OUT.toInt
//                 }

//                 def ideWriteReg(csn: Int, address : Int, data : Int) {
//                     dut.io.ideBus.CSn   #= csn
//                     dut.io.ideBus.ADDR  #= address
//                     dut.io.ideBus.DATA_IN      #= data
//                     dut.io.ideBus.WRn   #= false
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.io.ideBus.WRn   #= true
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                     dut.clockDomain.waitFallingEdge()
//                 }

//                 dut.clockDomain.waitFallingEdge()
//                 dut.clockDomain.waitFallingEdge()
//                 dut.clockDomain.waitFallingEdge()
//                 dut.clockDomain.waitFallingEdge()

//                 // debug reg
//                 ideReadReg(1, 7)
//                 ideWriteReg(1, 7, 0x5A)
//                 ideReadReg(1, 7)

//                 dut.clockDomain.waitFallingEdge()

//         }
//     }
    
// }