package opengdemu

import spinal.core._
import spinal.lib._

case class PATAInterface() extends Bundle {
    val RST         = in Bool()
    val CS          = in Bits(2 bits)
    val WR          = in Bool()
    val RD          = in Bool()
    val ADDR        = in UInt(3 bits)
    val DATA_IN     = in Bits(16 bits)
    val DATA_OUT    = out Bits(16 bits)
    val DATA_OUT_EN = out Bool()

    val DMARQ       = out Bool()
    val DMACK       = in Bool()
    
    val IORDY       = out Bool()
    val INTRQ       = out Bool()
}