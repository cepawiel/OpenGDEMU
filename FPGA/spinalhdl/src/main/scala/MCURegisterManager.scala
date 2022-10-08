package opengdemu

import scala.collection.mutable.ArrayBuffer
import spinal.core._
import spinal.lib._

case class MCURegisterConfig(
    name          : String,
    address       : Int,
    bitOffset     : Int,
    bitCount      : Int,
    initVal       : Int
)

case class MCURegisterManagerGenerator() extends Component {
    val configs = ArrayBuffer[MCURegisterConfig]()

    def add(name          : String,
            address       : Int,
            bitOffset     : Int,
            bitCount      : Int,
            initVal       : Int = 0) {
        assert(bitCount > 0, "bitCount must be greater than 0")
        configs.append(MCURegisterConfig(name = name, address = address, 
                                         bitOffset = bitOffset, bitCount = bitCount, 
                                         initVal = initVal))
    }

    def generate() {
        for ( cfg <- configs ) {
            if (cfg.bitCount > 1) {
                Console.printf("Setting up %s at 0x%X(%d downto %d)\n", cfg.name, cfg.address, cfg.bitOffset, cfg.bitOffset + cfg.bitCount - 1);
            } else {
                Console.printf("Setting up %s at 0x%X(%d)\n", cfg.name, cfg.address, cfg.bitOffset);
            }
            
        }
    }
}

case class MCURegisterManager(busSize : Int) extends Component {
    val io = new Bundle {
        val MCU_BUS = slave(MCUBus(busSize))
    }
}