package opengdemu

import spinal.core._
import spinal.lib._

case class IDEBus() extends Bundle with IMasterSlave {
    val RSTn        = Bool()
    val CSn         = Bits(2 bits)
    val WRn         = Bool()
    val RDn         = Bool()
    val ADDR        = UInt(3 bits)
    val DATA_IN     = Bits(16 bits)
    val DATA_OUT    = Bits(16 bits)
    val DATA_OUT_EN = Bool()

    val DMACKn      = Bool()
    val DMARQ       = Bool()
    
    val IORDY       = Bool()
    val INTRQ       = Bool()

    override def asMaster(): Unit = {
        out(RSTn, CSn, WRn, RDn, ADDR, DATA_IN, DMACKn)
        in(DATA_OUT, DATA_OUT_EN, DMARQ, IORDY, INTRQ)
    }
}

case class IDERegisterMap() extends Component {
    val io = new Bundle {
        // val IDEBus = slave(IDEBus())

    }

    // val dataArray;
}

case class IDERegister(initVal: Int = 0) extends Component {
    val io = new Bundle {
        val ENABLE      = in Bool()
        val WRITE       = in Bool()
        val DATA_IN     = in Bits(16 bits)

        val DATA_OUT    = out Bits(16 bits)
    }


    val reg = RegNextWhen(io.DATA_IN, io.ENABLE && io.WRITE) init(initVal)    
    io.DATA_OUT := reg
}

import spinal.core.sim._

object TestIDERegister {
    def simulate(workspacePath : String) {
        Console.println("Simulating IDERegister");
        val cfg = SimConfig
            .withWave
            .withIVerilog
            .workspacePath(workspacePath)
            .compile(IDERegister())


        cfg.doSim("IDERegister_tb") { dut =>
                dut.clockDomain.forkStimulus(period = 20000)

                dut.io.ENABLE #= false
                dut.io.WRITE #= false
                dut.io.DATA_IN #= 0xA55A

                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0, "Failed Check Init Value")

                dut.io.WRITE #= true                
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0, "Failed Write Only Check")
                dut.io.WRITE #= false   

                dut.io.ENABLE #= true                
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0, "Failed Enable Only Check")
                dut.io.ENABLE #= false   

                dut.io.ENABLE #= true                
                dut.io.WRITE #= true       
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0xA55A, "Failed Write Check 0xA55A")
                dut.io.ENABLE #= false   
                dut.io.WRITE #= true       
                
                dut.io.DATA_IN #= 0xC0DE
                dut.io.ENABLE #= true                
                dut.io.WRITE #= true       
                dut.clockDomain.waitFallingEdge()
                assert(dut.io.DATA_OUT.toInt == 0xC0DE, "Failed Write Check 0xC0DE")
                dut.io.ENABLE #= false   
                dut.io.WRITE #= true   

                dut.clockDomain.waitFallingEdge()
                dut.clockDomain.waitFallingEdge()

        }
    }
    
}