
package opengdemu

import spinal.core._
import spinal.lib._
import scala.math._

// case class scfifo(
//         width : Int, 
//         depth : Int, 
//         almostEmpty : Int = 0, 
//         almostFull : Int = 0
//     ) extends BlackBox {

//     addGeneric("lpm_width", width)
//     addGeneric("lpm_numwords", depth)

//     addGeneric("intended_device_family", "Cyclone II")
//     addGeneric("lpm_showahead", "ON")
//     addGeneric("almost_full_value", almostFull)
//     addGeneric("almost_empty_value", almostEmpty)
//     addGeneric("lpm_type", "scfifo")
//     addGeneric("overflow_checking", "ON")
//     addGeneric("underflow_checking", "ON")
//     addGeneric("use_eab", "ON")     // Use RAM Blocks
//     addGeneric("add_usedw_msb_bit", "ON")

//     val countWidth : Int = ceil(log(depth)/log(2)).toInt;
//     addGeneric("lpm_widthu", countWidth)

//     val io = new Bundle {
//         val aclr            = in Bool()
//         val clock           = in Bool()
//         val sclr            = in Bool()

//         val wrreq           = in Bool()
//         val data            = in Bits(width bits)

//         val rdreq           = in Bool()
//         val q               = out Bits(width bits)
    
//         val empty           = out Bool()
//         val full            = out Bool()

//         val almost_empty    = out Bool()
//         val almost_full     = out Bool()

//         val usedw           = out UInt(countWidth bits)
//     }

//     noIoPrefix()
//     mapClockDomain(clock=io.clock, reset=io.aclr)
// }

case class scfifo(
        width : Int, 
        depth : Int, 
        almostEmpty : Int = 0, 
        almostFull : Int = 0
    ) extends Component {

    val countWidth : Int = ceil(log(depth)/log(2)).toInt;


    val io = new Bundle {
        val sclr            = in Bool()

        val wrreq           = in Bool()
        val data            = in Bits(width bits)

        val rdreq           = in Bool()
        val q               = out Bits(width bits)
    
        val empty           = out Bool()
        val full            = out Bool()

        val almost_empty    = out Bool()
        val almost_full     = out Bool()

        val usedw           = out UInt(countWidth bits)
    }

    io.q := 0;
    io.empty := True;
    io.full := False;
    io.almost_empty := False;
    io.almost_full := False;
    io.usedw := 0;
}

// case class FIFO(width : Int, depth : Int) extends Component {
//     val countWidth : Int = ceil(log(depth)/log(2)).toInt;

//     val io = new Bundle {
//         val clear           = in Bool()

//         val write_en        = in Bool()
//         val data_in         = in Bits(width bits)
//         val overflow        = out Bool()
//         val full            = out Bool()

//         val read_en         = in Bool()
//         val data_out        = out Bits(width bits)
//         val underflow       = out Bool()
//         val empty           = out Bool()
        
//         val usedw           = out UInt(countWidth bits)
//     }

//     io.overflow := False
//     io.underflow := False

//     val readPos = Reg(Bits(countWidth bits)) init(0);
//     val writePos = Reg(Bits(countWidth bits)) init(0);
//     val empty = Reg(Bool()) init(true);

//     val memory = Mem(Bits(width bits), wordCount = depth)
//     io.data_out := memory.readSync(readPos)
//     memory.write(writePos, io.data_in, io.write_en)

//     when (io.write_en) {
//         when ((writePos == readPos) && !empty) {
//             io.overflow := True
//         }.otherwise {
//             writePos = writePos + 1
//         }
        
//     }

//     when (io.read_en) {
//         when (empty) {
//             io.underflow := True
//         }.otherwise {
//             readPos = readPos + 1
//             when (readPos == writePos) {
//                 empty := True
//             }
//         }
//     }


// }