package opengdemu

import spinal.core._
import spinal.lib._
import spinal.lib.bus.simple._

// https://pdos.csail.mit.edu/6.828/2018/readings/hardware/ATA-d1410r3a.pdf
// p. 63-74

case class IDEAddressDecode() extends Component {
    val io = new Bundle {
        val CSn         = in Bits(2 bits)
        val ADDR        = in UInt(3 bits)

        // 0
        val DataRegister = out Bool()       // 0 - RW
        val ErrorRegister = out Bool()      // 1 - R
        val FeaturesRegister = out Bool()   // 1 - W
        val SectorCountRegister = out Bool()// 2 - RW
        val LBALowRegister = out Bool()     // 3 - RW
        val LBAMidRegister = out Bool()     // 4 - RW
        val LBAHighRegister = out Bool()    // 5 - RW
        val DeviceRegister = out Bool()     // 6 - RW
        val CommandRegister = out Bool()    // 7 - W
        val StatusRegister = out Bool()     // 7 - R
    
        // 1
        val DeviceControlRegister = out Bool()     // 6 - W
        val AlternateStatusRegister = out Bool()   // 6 - R

        // Non-Spec Used for Debug
        val DebugTestpad = out Bool()              // 7 - RW
        
    }

    val regBlock0 = (io.CSn === B"10")   // CS1 = 0         CS0 = 1
    val regBlock1 = (io.CSn === B"01")   // CS1 = 1         CS0 = 0
    // val dmaTransfer = (io.CSn === B"11") && bus.DMARQ && ~bus.DMACKn

    val ideRegAddress = new Array[Bool](8);
    val ideRegAddressBlock = Array.ofDim[Bool](2, 8);
    for ( addr <- 0 to 7) {
        ideRegAddress(addr) = io.ADDR === addr
        ideRegAddressBlock(0)(addr) = ideRegAddress(addr) && regBlock0
        ideRegAddressBlock(1)(addr) = ideRegAddress(addr) && regBlock1
    }

    io.DataRegister := ideRegAddressBlock(0)(0)
    io.ErrorRegister := ideRegAddressBlock(0)(1)
    io.FeaturesRegister := ideRegAddressBlock(0)(1)
    io.SectorCountRegister := ideRegAddressBlock(0)(2)
    io.LBALowRegister := ideRegAddressBlock(0)(3)
    io.LBAMidRegister := ideRegAddressBlock(0)(4)
    io.LBAHighRegister := ideRegAddressBlock(0)(5)
    io.DeviceRegister := ideRegAddressBlock(0)(6)
    io.CommandRegister := ideRegAddressBlock(0)(7)
    io.StatusRegister := ideRegAddressBlock(0)(7)

    io.DeviceControlRegister := ideRegAddressBlock(1)(6)
    io.AlternateStatusRegister := ideRegAddressBlock(1)(6)

    io.DebugTestpad := ideRegAddressBlock(1)(7)
}

import spinal.core.sim._

object TestIDEAddressDecode {
    def simulate(workspacePath : String) {
        Console.println("Simulating IDEAddressDecode");
        val cfg = SimConfig
            .withWave
            .withIVerilog
            .workspacePath(workspacePath)
            .compile(IDEAddressDecode())


        cfg.doSim("IDEAddressDecode_tb") { dut =>
            // check when both CSn pins are low, no registers should be active
            for( csn <- List(3, 0)) {
                // check all addresses don't activate when CSn pins are not valid
                for( addr <- 0 to 7) {
                    dut.io.CSn #= csn
                    dut.io.ADDR #= addr
                    sleep(1)
                    assert(dut.io.DataRegister.toBoolean == false);
                    assert(dut.io.ErrorRegister.toBoolean == false);
                    assert(dut.io.FeaturesRegister.toBoolean == false);
                    assert(dut.io.SectorCountRegister.toBoolean == false);
                    assert(dut.io.LBALowRegister.toBoolean == false);
                    assert(dut.io.LBAMidRegister.toBoolean == false);
                    assert(dut.io.LBAHighRegister.toBoolean == false);
                    assert(dut.io.DeviceRegister.toBoolean == false);
                    assert(dut.io.CommandRegister.toBoolean == false);
                    assert(dut.io.StatusRegister.toBoolean == false);
                    assert(dut.io.DeviceControlRegister.toBoolean == false);
                    assert(dut.io.AlternateStatusRegister.toBoolean == false);
                    assert(dut.io.DebugTestpad.toBoolean == false);
                }
            }

            // Check Block 1 Addr 0
            dut.io.CSn #= 2
            dut.io.ADDR #= 0
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == true);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 1
            dut.io.CSn #= 2
            dut.io.ADDR #= 1
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == true);
            assert(dut.io.FeaturesRegister.toBoolean == true);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 2
            dut.io.CSn #= 2
            dut.io.ADDR #= 2
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == true);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 3
            dut.io.CSn #= 2
            dut.io.ADDR #= 3
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == true);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 4
            dut.io.CSn #= 2
            dut.io.ADDR #= 4
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == true);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 5
            dut.io.CSn #= 2
            dut.io.ADDR #= 5
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == true);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 6
            dut.io.CSn #= 2
            dut.io.ADDR #= 6
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == true);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 1 Addr 7
            dut.io.CSn #= 2
            dut.io.ADDR #= 7
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == true);
            assert(dut.io.StatusRegister.toBoolean == true);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 2 Addr 6
            dut.io.CSn #= 1
            dut.io.ADDR #= 6
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == true);
            assert(dut.io.AlternateStatusRegister.toBoolean == true);
            assert(dut.io.DebugTestpad.toBoolean == false);

            // Check Block 2 Addr 7
            dut.io.CSn #= 1
            dut.io.ADDR #= 7
            sleep(1)
            assert(dut.io.DataRegister.toBoolean == false);
            assert(dut.io.ErrorRegister.toBoolean == false);
            assert(dut.io.FeaturesRegister.toBoolean == false);
            assert(dut.io.SectorCountRegister.toBoolean == false);
            assert(dut.io.LBALowRegister.toBoolean == false);
            assert(dut.io.LBAMidRegister.toBoolean == false);
            assert(dut.io.LBAHighRegister.toBoolean == false);
            assert(dut.io.DeviceRegister.toBoolean == false);
            assert(dut.io.CommandRegister.toBoolean == false);
            assert(dut.io.StatusRegister.toBoolean == false);
            assert(dut.io.DeviceControlRegister.toBoolean == false);
            assert(dut.io.AlternateStatusRegister.toBoolean == false);
            assert(dut.io.DebugTestpad.toBoolean == true);
                
        }
    }
    
}