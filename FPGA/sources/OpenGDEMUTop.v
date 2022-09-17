module OpenGDEMUTop (
   // Input Clocks
   input CLK_11_2896_MHz,
   input CLK_48_MHz,

   // Connection to SAM3U
   input MCU_RSTn,
   input MCU_NCS,
   input MCU_NRE,
   input MCU_NWE,
   input [1:0]  MCU_NBS,
   input [8:1]  MCU_ADDR,
   inout [15:0] MCU_DATA,
   output MCU_IRQn,

	// Dreamcast G1 Connector
	output DC_CDCLK,
	output DC_SCK,
	output DC_SDAT,
	output DC_LRCK,
   output DC_EMPH,

	input DC_RSTn,
	input [1:0] DC_CSn,
	input DC_WRn,
	input DC_RDn,
	input [2:0] DC_ADDR,
	inout [15:0] DC_DATA,

	output DC_DMARQ,
	input DC_DMACKn,
	output DC_IORDY,
	output DC_INTRQ
	);

   // CDCLK PLL
   wire CDCLK_PLL_OUT;
   assign DC_CDCLK = CDCLK_PLL_OUT;
   Audio_PLL cdda_pll(CLK_11_2896_MHz, CDCLK_PLL_OUT);

   // wire CLK_96_MHz;
   // Double_PLL corePLL(CLK_48_MHz, CLK_96_MHz);


   wire MCU_READ  = (~MCU_NCS & ~MCU_NRE);
   wire MCU_WRITE = (~MCU_NCS & ~MCU_NWE);

   wire [15:0] MCU_DATA_OUT;
   wire [1:0] MCU_DATA_OUT_EN = MCU_READ ? ~MCU_NBS : 2'b00;
   assign MCU_DATA[15:8] = MCU_DATA_OUT_EN[1] ? MCU_DATA_OUT[15:8] : 8'hZZ;
   assign MCU_DATA[7:0] = MCU_DATA_OUT_EN[0] ? MCU_DATA_OUT[7:0] : 8'hZZ;

   wire [15:0] DC_DATA_OUT;
   wire DC_DATA_OUT_EN;
   assign DC_DATA = (DC_DATA_OUT_EN) ? DC_DATA_OUT : 16'hZZZZ;

   wire MCU_IRQ;
   assign MCU_IRQn = ~MCU_IRQ;

   OpenGDEMU gdemu1(
      .io_MCU_CLK(CLK_48_MHz),
      .io_MCU_RST(~MCU_RSTn),
      .io_MCU_RD(MCU_READ),
      .io_MCU_WR(MCU_WRITE ? ~MCU_NBS : 2'b00),
      .io_MCU_ADDR(MCU_ADDR),
      .io_MCU_DATA_IN(MCU_DATA),
      .io_MCU_DATA_OUT(MCU_DATA_OUT),
      .io_MCU_IRQ(MCU_IRQ),

      .io_DC_CDCLK(DC_CDCLK),
      .io_DC_SCK(DC_SCK),
      .io_DC_SDAT(DC_SDAT),
      .io_DC_LRCK(DC_LRCK),
      .io_DC_EMPH(DC_EMPH),

      .io_DC_RST(~DC_RSTn),
      .io_DC_CSn(DC_CSn),
      .io_DC_WR(~DC_WRn),
      .io_DC_RD(~DC_RDn),
      .io_DC_ADDR(DC_ADDR),
      .io_DC_DATA_IN(DC_DATA),
      .io_DC_DATA_OUT(DC_DATA_OUT),
      .io_DC_DATA_OUT_EN(DC_DATA_OUT_EN),

      .io_DC_DMARQ(DC_DMARQ),
      .io_DC_DMACK(~DC_DMACKn),
      .io_DC_IORDY(DC_IORDY),
      .io_DC_INTRQ(DC_INTRQ)
   );

   assign DC_SCK = 0;
   assign DC_SDAT = 1;
   assign DC_LRCK = 0;
   assign DC_EMPH = 1;

endmodule // top
