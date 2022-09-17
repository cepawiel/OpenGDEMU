package opengdemu

import spinal.core._
import spinal.lib._

case class MCUInterface(addrLen : Int) extends Bundle {
    val RD          = in Bool()
    val WR          = in Bits(2 bits)
    val ADDR        = in UInt(addrLen bits)
    val DATA_IN     = in Bits(16 bits)
    val DATA_OUT    = out Bits(16 bits)
    val IRQ         = out Bool()
}