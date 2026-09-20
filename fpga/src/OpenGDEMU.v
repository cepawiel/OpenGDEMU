module OpenGDEMU (
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

	wire core_reset;
	wire core_clock;

	core_pll	core_pll_inst (
		.areset ( ~MCU_RSTn ),
		.inclk0 ( CLK_48_MHz ),
		.c0 ( core_clock ),
		.locked ( core_reset )
	);

   wire MCU_READ  = (~MCU_NCS & ~MCU_NRE);

   wire [15:0] MCU_DATA_OUT;
   assign MCU_DATA[15:8] = (~MCU_NBS[1] & MCU_READ) ? MCU_DATA_OUT[15:8] : 8'hZZ;
   assign MCU_DATA[7:0]  = (~MCU_NBS[0] & MCU_READ) ? MCU_DATA_OUT[7:0]  : 8'hZZ;

   // Audio crystal heartbeat. The wrapper owns this so the divider
   // sits in CLK_11_2896_MHz's domain; bit 15 (~172 Hz square wave) is
   // sampled by the core's 48 MHz logic to count xtal liveness.
   reg [15:0] audio_div;
   always @(posedge CLK_11_2896_MHz)
      audio_div <= audio_div + 1'b1;
   wire audio_tick = audio_div[15];

   wire DC_CS = ~DC_CSn[0] | ~DC_CSn[1];
   wire DC_RD = DC_CS & ~DC_RDn;
   // During a DMA read the host deasserts CSn and pulses RDn under DMACKn.
   // Treat that as a separate "drive DATA" condition; the core decides what
   // to actually put on the bus (FIFO data, gated on its own dma_mode).
   wire DC_DMA_RD = ~DC_DMACKn & ~DC_RDn;

   // While the DC holds its bus reset asserted (DC_RSTn low), keep every
   // line we drive at hi-Z. Cyclone II tristates outputs only during
   // configuration, so without this gating we'd start actively driving
   // INTRQ/DMARQ/IORDY the moment the bitstream is loaded — which is
   // long before the DC's south bridge has come out of its own reset.
   // Active drives during that window appear to fight the bridge's
   // initialization handshake; tristating until DC_RSTn rises lets the
   // DC's pulls settle the lines while it boots.
   wire core_intrq;
   wire core_dmarq;
   wire core_iordy;

   wire [15:0] DC_DATA_OUT;
   assign DC_DATA  = (DC_RSTn & (DC_RD | DC_DMA_RD)) ? DC_DATA_OUT : 16'hZZZZ;
   assign DC_INTRQ = DC_RSTn ? core_intrq : 1'bZ;
   assign DC_DMARQ = DC_RSTn ? core_dmarq : 1'bZ;
   assign DC_IORDY = DC_RSTn ? core_iordy : 1'bZ;

   // GD-ROM audio master clock: 33.8688 MHz (768 x 44.1 kHz), the same
   // rate a real drive and iceGDROM put on this pin, made by tripling the
   // 11.2896 MHz xtal in a PLL. The serial audio itself (SCK/LRCK/SDAT)
   // is generated inside the core straight from the xtal.
   wire cdclk;
   Audio_PLL audio_pll_inst (
      .inclk0 ( CLK_11_2896_MHz ),
      .c0 ( cdclk )
   );
   assign DC_CDCLK = cdclk;

   // Second sample of the IDE read strobe, taken on the falling clock edge.
   // Multiword DMA mode 2 allows only tKR = 25 ns between consecutive DIOR-
   // pulses (ATA/ATAPI-5 table 50), which is 1.2 periods of the 48 MHz core
   // clock. Sampling on one edge only, that gap is sometimes missed entirely,
   // two strobes merge into a single falling edge, and the read FIFO pops once
   // instead of twice -- the host's DMA counter then runs ahead of what we
   // actually handed over and every subsequent word is shifted. Capturing on
   // both edges halves the effective sample period to 10.4 ns.
   reg dc_rd_half;
   always @(negedge CLK_48_MHz)
      dc_rd_half <= ~DC_RDn;

   // Same for the write strobe: a PIO data-out phase is a burst of
   // back-to-back writes with the same narrow inter-strobe gap.
   reg dc_wr_half;
   always @(negedge CLK_48_MHz)
      dc_wr_half <= ~DC_WRn;

   wire mcuIRQ;
   assign MCU_IRQn = ~mcuIRQ;

   OpenGDEMUCore core (
      .clk(CLK_48_MHz),
      .rst(~MCU_RSTn),

      .io_MCU_CS(~MCU_NCS),
      .io_MCU_BS(~MCU_NBS),
      .io_MCU_RD(~MCU_NRE),
      .io_MCU_WR(~MCU_NWE),
      .io_MCU_ADDR(MCU_ADDR),
      .io_MCU_DATA_IN(MCU_DATA),
      .io_MCU_DATA_OUT(MCU_DATA_OUT),
      .io_MCU_IRQ(mcuIRQ),

      // CDCLK comes from the wrapper's PLL (above); the core's serialiser
      // runs on the raw xtal and drives SCK/SDAT/LRCK.
      .io_AUDIO_CLK(CLK_11_2896_MHz),
      .io_DC_CDCLK(),
      .io_DC_SCK(DC_SCK),
      .io_DC_SDAT(DC_SDAT),
      .io_DC_LRCK(DC_LRCK),
      .io_DC_EMPH(DC_EMPH),
      .io_audio_tick(audio_tick),

      .io_IDE_RSTn(DC_RSTn),
      .io_IDE_CSn(DC_CSn),
      .io_IDE_WR(~DC_WRn),
      .io_IDE_RD(~DC_RDn),
      .io_IDE_RD_H(dc_rd_half),
      .io_IDE_WR_H(dc_wr_half),
      .io_IDE_ADDR(DC_ADDR),
      .io_IDE_DATA_IN(DC_DATA),
      .io_IDE_DATA_OUT(DC_DATA_OUT),

      .io_IDE_DMARQ(core_dmarq),
      .io_IDE_DMACKn(DC_DMACKn),
      .io_IDE_IORDY(core_iordy),
      .io_IDE_INTRQ(core_intrq)
   );

endmodule // top
