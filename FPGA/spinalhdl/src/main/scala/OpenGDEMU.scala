package opengdemu

import spinal.core._
import spinal.lib._
import java.time.Instant

//Hardware definition
class OpenGDEMU extends Component {
    val io = new Bundle {
        // Microcontroller Connections
        val MCU_CLK         = in Bool()
        val MCU_RST         = in Bool()
        val MCU             = MCUInterface(8)

        // DC Connections
        // DC Audio
        val DC = new Bundle {
            val CDCLK        = in Bool()
            val SCK          = in Bool()
            val SDAT         = in Bool()
            val LRCK         = in Bool()
            val EMPH         = in Bool()

            // DC Data
            val RST          = in Bool()
            val CSn          = in Bits(2 bits)
            val WR           = in Bool()
            val RD           = in Bool()
            val ADDR         = in UInt(3 bits)
            val DATA_IN      = in Bits(16 bits)
            val DATA_OUT     = out Bits(16 bits)
            val DATA_OUT_EN  = out Bool()

            // ?
            val DMARQ        = out Bool()
            val DMACK        = in Bool()
            val IORDY        = out Bool()
            val INTRQ        = out Bool()
        }
    }

    val clockDom = ClockDomain(
        clock = io.MCU_CLK,
        reset = io.MCU_RST
    )

    val clockArea = new ClockingArea(clockDom) {
        val globalRegsCS : Bool = io.MCU.ADDR(7 downto 6) === 0
        val masterRegsCS : Bool = (io.MCU.ADDR(7) === True) & (io.MCU.ADDR(6) === False)
        val slaveRegsCS  : Bool = (io.MCU.ADDR(7) === True) & (io.MCU.ADDR(6) === True)
        // val extraRegs := io.MCU.ADDR(7 downto 6) === 3

        io.MCU.DATA_OUT                 := 0

        // Global Registers
        when (globalRegsCS) {
            val ts : Long = Instant.now.getEpochSecond
            Console.printf("Compiled at 0x%X\n", ts)
            val tsWords = Array(
                B((ts >> 0).toInt  & 0xFFFF, 16 bits),
                B((ts >> 16).toInt & 0xFFFF, 16 bits),
                B((ts >> 32).toInt & 0xFFFF, 16 bits),
                B((ts >> 48).toInt & 0xFFFF, 16 bits))

            // Version Registers (0-3)
            for( i <- 0 to 3 )
                when (io.MCU.ADDR === i) { io.MCU.DATA_OUT := tsWords(i) }

            // Test Data Register
            val testReg = Reg(Bits(16 bits)) init(0x5AA5)
            when (io.MCU.ADDR === 4) {
                io.MCU.DATA_OUT := testReg
                when (io.MCU.WR(0)) { testReg(7 downto 0) := io.MCU.DATA_IN(7 downto 0) }
                when (io.MCU.WR(1)) { testReg(15 downto 8) := io.MCU.DATA_IN(15 downto 8) }
            }
        }

        // val CS0_SYNC = { BufferCC(~io.DC.CSn(0), False) }
        // val CS1_SYNC = { BufferCC(~io.DC.CSn(1), False) }
        val WR_SYNC = BufferCC(io.DC.WR, False)
        val RD_SYNC = BufferCC(io.DC.RD, False)

        val DC_DATA_IN_SYNC = Reg(Bits(16 bits)) init(0)
        DC_DATA_IN_SYNC := io.DC.DATA_IN
        val DC_ADDR_SYNC = Reg(UInt(3 bits)) init(0)
        DC_ADDR_SYNC := io.DC.ADDR
        val DC_CS_SYNC = Reg(Bits(2 bits)) init(0)
        DC_CS_SYNC := ~io.DC.CSn
        

        // val masterDevice;
        // create hdd as slave device
        val slaveDevice = new PATA_HDD(1);
        slaveDevice.io.pataBus.RST         := io.DC.RST
        slaveDevice.io.pataBus.CS          := DC_CS_SYNC
        slaveDevice.io.pataBus.WR          := WR_SYNC
        slaveDevice.io.pataBus.RD          := RD_SYNC
        slaveDevice.io.pataBus.ADDR        := DC_ADDR_SYNC
        slaveDevice.io.pataBus.DATA_IN     := DC_DATA_IN_SYNC
        slaveDevice.io.pataBus.DMACK       := io.DC.DMACK
        io.DC.DATA_OUT                     := slaveDevice.io.pataBus.DATA_OUT
        io.DC.DATA_OUT_EN                  := slaveDevice.io.pataBus.DATA_OUT_EN
        io.DC.DMARQ                        := slaveDevice.io.pataBus.DMARQ
        io.DC.INTRQ                        := slaveDevice.io.pataBus.INTRQ
        io.DC.IORDY                        := slaveDevice.io.pataBus.IORDY

        slaveDevice.io.mcuBus.WR           := slaveRegsCS ? io.MCU.WR | 0 
        slaveDevice.io.mcuBus.RD           := slaveRegsCS ? io.MCU.RD | False
        slaveDevice.io.mcuBus.ADDR         := io.MCU.ADDR(5 downto 0)
        slaveDevice.io.mcuBus.DATA_IN      := io.MCU.DATA_IN
        
        when (masterRegsCS) {
            io.MCU.DATA_OUT                := "16'xC0DE"
        }
        when (slaveRegsCS) {
            io.MCU.DATA_OUT                := slaveDevice.io.mcuBus.DATA_OUT
            // io.MCU.DATA_OUT                := "16'xCAFE"

        }
        io.MCU.IRQ                         := slaveDevice.io.mcuBus.IRQ
    }
}

import spinal.sim._
import spinal.core.sim._

object OpenGDEMUVerilog {
    def main(args: Array[String]) {
        if(args.length == 0) {
            Console.err.println("Must pass targetPath as parameter");
            sys.exit(1)
        }


        Console.println("Generating OpenGDEMU Verilog")
        SpinalConfig(
            mode=Verilog,
            targetDirectory=args(0)
        ).generate(new OpenGDEMU).printPruned()
        // SpinalVerilog(new OpenGDEMU).printPruned()
        

        // SimConfig
        //     .withWave
        //     .withIVerilog
        //     .workspacePath("test_output")
        //     .compile(new OpenGDEMU).doSim("OpenGDEMU_tb") 
        //     { dut =>
        //         var clk = dut.clockDom

        //         def mcuRead(address : Int) {
        //             dut.io.MCU.ADDR #= address
        //             dut.io.MCU.RD #= true
        //             clk.waitFallingEdge()
        //             dut.io.MCU.RD #= false
        //         }

        //         def mcuWrite(address : Int, value : Int) {
        //             dut.io.MCU.ADDR #= address
        //             dut.io.MCU.DATA_IN #= value
        //             dut.io.MCU.WR #= 3  // Write both bytes
        //             clk.waitFallingEdge()
        //             dut.io.MCU.WR #= 0
        //         }

        //         def dcWrite(csn : Int, addr : Int, data : Int) {
        //             dut.io.DC.CSn #= csn
        //             dut.io.DC.ADDR #= addr
        //             dut.io.DC.DATA_IN #= data
        //             dut.io.DC.WR #= true
        //             clk.waitFallingEdge();
        //             dut.io.DC.WR #= false
        //             clk.waitFallingEdge();
        //         }

        //         def dcRead(csn : Int, addr : Int) {
        //             dut.io.DC.CSn #= csn
        //             dut.io.DC.ADDR #= addr
        //             dut.io.DC.DATA_IN #= 0
        //             dut.io.DC.RD #= true
        //             clk.waitFallingEdge();
        //             dut.io.DC.RD #= false
        //             clk.waitFallingEdge();
        //         }

        //         // set initial values for mcu
        //         dut.io.MCU.WR #= 0
        //         dut.io.MCU.RD #= false
        //         dut.io.MCU.ADDR #= 0
        //         dut.io.MCU.DATA_IN #= 0

        //         // set initial values for dc
        //         dut.io.DC.RST #= false
        //         dut.io.DC.CSn #= 3
        //         dut.io.DC.RD #= false
        //         dut.io.DC.WR #= false
        //         dut.io.DC.ADDR #= 0
        //         dut.io.DC.DATA_IN #= 0

        //         clk.forkStimulus(period = 10)
        //         clk.waitRisingEdge()
        //         clk.waitFallingEdge()

        //         mcuRead(0x04)
        //         mcuWrite(0x04, 0xDEAD)
        //         mcuRead(0x04)

        //         clk.waitFallingEdge()
        //         clk.waitFallingEdge()

        //         // select device
        //         dcWrite(2, 6, 0x10)

        //         // write sector count
        //         dcWrite(2, 2, 0xAA55)

        //         // write command
        //         dcWrite(2, 7, 0xEC)
                
        //         dcRead(2, 2)

        //         // clk.waitFallingEdge()
        //         // clk.waitFallingEdge()

        //         mcuRead(0xC4)

        //         clk.waitFallingEdge()
        //         clk.waitFallingEdge()

        //     }
    }
}



