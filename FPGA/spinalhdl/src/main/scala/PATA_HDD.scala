package opengdemu

import spinal.core._
import spinal.lib._

case class PATA_HDD(deviceID : Int) extends Component {
    val io = new Bundle {
        val pataBus = PATAInterface();
        val mcuBus = MCUInterface(6);
    }

    val ideRegs = new PATARegisters(io.pataBus, deviceID);
    val mcuRegs = new MCURegisters(io.mcuBus, ideRegs)
}