package opengdemu

import spinal.core._
import spinal.lib._
import spinal.lib.bus.simple._
import java.time.Instant

//Hardware definition
class OpenGDEMU extends Component {
    val io = new Bundle {
        // Microcontroller Connections
        val MCU_CLK         = in Bool()
        val MCU_RST         = in Bool()
        val MCU             = slave(MCUBus(8))

        // DC Data Connection
        val IDE          = slave(IDEBus())

        // DC Audio
        val DC = new Bundle {
            val CDCLK        = in Bool()
            val SCK          = in Bool()
            val SDAT         = in Bool()
            val LRCK         = in Bool()
            val EMPH         = in Bool()
        }
        
    }

    val clockDom = ClockDomain(
        clock = io.MCU_CLK,
        reset = io.MCU_RST
    )

    val clockArea = new ClockingArea(clockDom) {
        val globalRegsCS : Bool = ~io.MCU.ADDR(7)
        val ideRegsCS : Bool = io.MCU.ADDR(7)
        val idePrimaryRegsCS : Bool = ~io.MCU.ADDR(6);
        val ideSecondaryRegsCS : Bool = io.MCU.ADDR(6);
        
        val primary = PrimaryIDEDeviceRegs(io.IDE, io.MCU);
        val secondary = SecondaryIDEDeviceRegs(io.IDE, io.MCU);
   
        io.IDE.DATA_OUT := primary.dataOutputEn ? primary.dataOutput | secondary.dataOutput
        // io.IDE.DATA_OUT := 0
        io.IDE.DATA_OUT_EN := primary.dataOutputEn | secondary.dataOutputEn
        // io.IDE.DATA_OUT_EN := False
        io.IDE.DMARQ := False
        io.IDE.IORDY := False
        io.IDE.INTRQ := False


        // io.MCU.IRQ := primary.mcuIRQ | secondary.mcuIRQ
        io.MCU.IRQ := False
        io.MCU.DATA_OUT := 0xC0DE
        io.MCU.DATA_OUT_EN := 0
        when (globalRegsCS) {
            val timestamp : Long = Instant.now.getEpochSecond;
            val timestampBits = U(timestamp, 64 bits)
            Console.printf("Compiled at 0x%X\n", timestamp)

            // Version Registers (0-3)
            switch (io.MCU.ADDR) {
                is (0, 1, 2, 3) {
                    // io.MCU.DATA_OUT := tsWords(io.MCU.ADDR) 
                    io.MCU.DATA_OUT := timestampBits.subdivideIn(16 bits)(io.MCU.ADDR(1 downto 0)).asBits
                }
                is (4) {
                    val testReg = Reg(Bits(16 bits)) init(0x5AA5)
                    io.MCU.DATA_OUT := testReg
                    when (io.MCU.WR(0)) { testReg(7 downto 0) := io.MCU.DATA_IN(7 downto 0) }
                    when (io.MCU.WR(1)) { testReg(15 downto 8) := io.MCU.DATA_IN(15 downto 8) }
                }
                default {
                    io.MCU.DATA_OUT := 0
                    io.MCU.DATA_OUT_EN := 0
                }
            }

        }.otherwise {
            when ( idePrimaryRegsCS ) {
                io.MCU.DATA_OUT := primary.mcuDataOutput
                io.MCU.DATA_OUT_EN := primary.mcuDataOutputEn
            }.otherwise {
                io.MCU.DATA_OUT := secondary.mcuDataOutput
                io.MCU.DATA_OUT_EN := secondary.mcuDataOutputEn
            }
        }
    

        // val WR_SYNC = BufferCC(~io.DC.WRn, False)
        // val RD_SYNC = BufferCC(~io.DC.RDn, False)

        // val DC_DATA_IN_SYNC = Reg(Bits(16 bits)) init(0)
        // DC_DATA_IN_SYNC := io.DC.DATA_IN
        // val DC_ADDR_SYNC = Reg(UInt(3 bits)) init(0)
        // DC_ADDR_SYNC := io.DC.ADDR
        // val DC_CS_SYNC = Reg(Bits(2 bits)) init(0)
        // DC_CS_SYNC := ~io.DC.CSn
        
    }
}

import spinal.sim._
import spinal.core.sim._

object OpenGDEMUVerilog {
    def checkSims() {
        val testbenchOutput : String = "out";

        TestIDERegister.simulate(testbenchOutput);
        TestMCURegister.simulate(testbenchOutput);
        TestIDEAddressDecode.simulate(testbenchOutput);

    }

    def main(args: Array[String]) {
        if(args.length == 0) {
            Console.err.println("Must pass targetPath as parameter");
            sys.exit(1)
        }
        
        checkSims()

        Console.println("Generating OpenGDEMU Verilog");
        SpinalConfig(
            mode=Verilog,
            targetDirectory=args(0)
        )
        .generate(new OpenGDEMU)
        .printPruned();
        

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




