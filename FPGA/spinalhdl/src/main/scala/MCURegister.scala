package opengdemu

import spinal.core._
import spinal.lib._

case class FlowWithBS() extends Bundle with IMasterSlave {
    val upper   = Flow(Bits(8 bits))
    val lower   = Flow(Bits(8 bits))

    override def asMaster(): Unit = {
        master(upper, lower)
    }
}

case class MCURegister(bitOffset : Int, bitCount : Int, initVal : Int) extends Component {
    val io = new Bundle {
        val MCU_UPDATE  = slave(FlowWithBS())
        val IDE_UPDATE  = slave Flow(Bits(16 bits))

        val DATA_OUT    = out(Bits(bitCount bits))
    }

    val reg = Reg(Bits(bitCount bits)) init(initVal)

    io.DATA_OUT := reg

    val lowestBit = bitOffset;
    val highestBit = bitOffset + (bitCount - 1);

    if ( lowestBit <= 7 ) {
        val localHigh = (highestBit % 8)
        //Console.printf("Lower: (%d downto %d)\n", localHigh, lowestBit);
        when (io.MCU_UPDATE.lower.valid) {
            reg(localHigh downto lowestBit) := io.MCU_UPDATE.lower.payload
        }
    }
    
    if ( highestBit >= 8 ) {        
        val localLow : Int = if ( lowestBit >= 8) lowestBit else 8;
        //Console.printf("Higher: (%d downto %d)\n", highestBit, localLow);
        when (io.MCU_UPDATE.upper.valid) {
            reg(highestBit downto localLow) := io.MCU_UPDATE.upper.payload
        }
    }

    when (io.IDE_UPDATE.valid) {
        reg := io.IDE_UPDATE.payload
    }
}

import spinal.core.sim._

object TestMCURegister {
    def simulate(workspacePath : String) {
        Console.println("Simulating MCURegister");
        val cfg = SimConfig
            .withWave
            .withIVerilog
            .workspacePath(workspacePath)
            .compile(MCURegister(0, 16, 0))


        cfg.doSim("MCURegister_tb") { dut =>
                dut.clockDomain.forkStimulus(period = 20000)

                dut.io.MCU_UPDATE.upper.valid #= false
                dut.io.MCU_UPDATE.lower.valid #= false
                dut.io.MCU_UPDATE.upper.payload #= 0
                dut.io.MCU_UPDATE.lower.payload #= 0
                
                dut.io.IDE_UPDATE.valid #= false
                dut.io.IDE_UPDATE.payload #= 0

                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0, "Failed Check Init Value")


                dut.io.MCU_UPDATE.lower.valid #= true   
                dut.io.MCU_UPDATE.upper.valid #= true 
                dut.io.MCU_UPDATE.upper.payload #= 0xA5
                dut.io.MCU_UPDATE.lower.payload #= 0x5A
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0xA55A, "Failed Write Check 0xA55A")
                dut.io.MCU_UPDATE.lower.valid #= false   
                dut.io.MCU_UPDATE.upper.valid #= false 

                dut.io.MCU_UPDATE.lower.valid #= true   
                dut.io.MCU_UPDATE.upper.valid #= true 
                dut.io.MCU_UPDATE.upper.payload #= 0xC0
                dut.io.MCU_UPDATE.lower.payload #= 0xDE
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0xC0DE, "Failed Write Check 0xC0DE")
                dut.io.MCU_UPDATE.lower.valid #= false   
                dut.io.MCU_UPDATE.upper.valid #= false 
                 
                dut.io.MCU_UPDATE.upper.valid #= true 
                dut.io.MCU_UPDATE.upper.payload #= 0xFF
                dut.clockDomain.waitFallingEdge()
                assert((dut.io.DATA_OUT.toInt & 0xFF00) == 0xFF00, "Failed Write Check 0xFF00") 
                dut.io.MCU_UPDATE.upper.valid #= false 

                dut.io.MCU_UPDATE.lower.valid #= true 
                dut.io.MCU_UPDATE.lower.payload #= 0xFF
                dut.clockDomain.waitFallingEdge()
                assert((dut.io.DATA_OUT.toInt & 0x00FF) == 0x00FF, "Failed Write Check 0x00FF") 
                dut.io.MCU_UPDATE.lower.valid #= false 

                dut.io.IDE_UPDATE.payload #= 0x3333
                dut.io.IDE_UPDATE.valid #= true      
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0x3333, "Failed Write Check 0x3333")
                dut.io.IDE_UPDATE.valid #= false

                dut.io.IDE_UPDATE.payload #= 0x4444
                dut.io.IDE_UPDATE.valid #= true      
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0x4444, "Failed Write Check 0x4444")
                dut.io.IDE_UPDATE.valid #= false

                dut.clockDomain.waitFallingEdge()
                dut.clockDomain.waitFallingEdge()

        }
    }
    
}