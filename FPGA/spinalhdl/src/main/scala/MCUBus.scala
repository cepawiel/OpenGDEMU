package opengdemu

import spinal.core._
import spinal.lib._

case class MCUBus(addrLen : Int) extends Bundle with IMasterSlave {
    val RD          = Bool()
    val WR          = Bits(2 bits)
    val ADDR        = UInt(addrLen bits)
    val DATA_IN     = Bits(16 bits)
    val DATA_OUT    = Bits(16 bits)
    val DATA_OUT_EN = Bits(2 bits)
    val IRQ         = Bool()

    override def asMaster(): Unit = {
        out(RD, WR, ADDR, DATA_IN)
        in(DATA_OUT, DATA_OUT_EN, IRQ)
    }
}

// case class MCUBusSplitter( splitBit : Int ) extends Component {
//     var io = new Bundle {
//         val Upstream = slave(MCUBus(splitBit + 1));
//         val Upper = master(MCUBus(splitBit));
//         val Lower = master(MCUBus(splitBit));
//     }
    
//     val bit = io.Upstream.ADDR(splitBit);
    
//     io.Upper.RD := io.Upstream.RD && bit
//     io.Upper.WR := bit ? io.Upstream.WR | B(0, 2 bits)
//     io.Upper.ADDR := io.Upstream.ADDR(splitBit - 1 downto 0)
//     io.Upper.DATA_IN := io.Upstream.DATA_IN

//     io.Lower.RD := io.Upstream.RD && ~bit
//     io.Lower.WR := ~bit ? io.Upstream.WR | 0
//     io.Lower.ADDR := io.Upstream.ADDR(splitBit - 1 downto 0)
//     io.Lower.DATA_IN := io.Upstream.DATA_IN
    
//     when (bit) {
//         io.Upstream.DATA_OUT     := io.Upper.DATA_OUT
//         io.Upstream.DATA_OUT_EN  := io.Upper.DATA_OUT_EN
//     }.otherwise {
//         io.Upstream.DATA_OUT     := io.Lower.DATA_OUT
//         io.Upstream.DATA_OUT_EN  := io.Lower.DATA_OUT_EN
//     }

//     io.Upstream.IRQ := io.Upper.IRQ && io.Lower.IRQ
// }