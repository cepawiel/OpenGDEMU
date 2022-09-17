// Generator : SpinalHDL v1.6.4    git head : 598c18959149eb18e5eee5b0aa3eef01ecaa41a1
// Component : OpenGDEMU

`timescale 1ns/1ps 

module OpenGDEMU (
  input               io_MCU_CLK,
  input               io_MCU_RST,
  input               io_MCU_RD,
  input      [1:0]    io_MCU_WR,
  input      [7:0]    io_MCU_ADDR,
  input      [15:0]   io_MCU_DATA_IN,
  output reg [15:0]   io_MCU_DATA_OUT,
  output              io_MCU_IRQ,
  input               io_DC_CDCLK,
  input               io_DC_SCK,
  input               io_DC_SDAT,
  input               io_DC_LRCK,
  input               io_DC_EMPH,
  input               io_DC_RST,
  input      [1:0]    io_DC_CSn,
  input               io_DC_WR,
  input               io_DC_RD,
  input      [2:0]    io_DC_ADDR,
  input      [15:0]   io_DC_DATA_IN,
  output     [15:0]   io_DC_DATA_OUT,
  output              io_DC_DATA_OUT_EN,
  output              io_DC_DMARQ,
  input               io_DC_DMACK,
  output              io_DC_IORDY,
  output              io_DC_INTRQ
);

  wire                clockArea_slaveDevice_io_mcuBus_RD;
  wire       [1:0]    clockArea_slaveDevice_io_mcuBus_WR;
  wire       [5:0]    clockArea_slaveDevice_io_mcuBus_ADDR;
  wire                io_DC_WR_buffercc_io_dataOut;
  wire                io_DC_RD_buffercc_io_dataOut;
  wire       [15:0]   clockArea_slaveDevice_io_pataBus_DATA_OUT;
  wire                clockArea_slaveDevice_io_pataBus_DATA_OUT_EN;
  wire                clockArea_slaveDevice_io_pataBus_DMARQ;
  wire                clockArea_slaveDevice_io_pataBus_IORDY;
  wire                clockArea_slaveDevice_io_pataBus_INTRQ;
  wire       [15:0]   clockArea_slaveDevice_io_mcuBus_DATA_OUT;
  wire                clockArea_slaveDevice_io_mcuBus_IRQ;
  wire                clockArea_globalRegsCS;
  wire                clockArea_masterRegsCS;
  wire                clockArea_slaveRegsCS;
  wire                when_OpenGDEMU_l67;
  wire                when_OpenGDEMU_l67_1;
  wire                when_OpenGDEMU_l67_2;
  wire                when_OpenGDEMU_l67_3;
  reg        [15:0]   _zz_io_MCU_DATA_OUT;
  wire                when_OpenGDEMU_l71;
  wire                when_OpenGDEMU_l73;
  wire                when_OpenGDEMU_l74;
  wire                clockArea_WR_SYNC;
  wire                clockArea_RD_SYNC;
  reg        [15:0]   clockArea_DC_DATA_IN_SYNC;
  reg        [2:0]    clockArea_DC_ADDR_SYNC;
  reg        [1:0]    clockArea_DC_CS_SYNC;

  BufferCC io_DC_WR_buffercc (
    .io_dataIn     (io_DC_WR                      ), //i
    .io_dataOut    (io_DC_WR_buffercc_io_dataOut  ), //o
    .io_MCU_CLK    (io_MCU_CLK                    ), //i
    .io_MCU_RST    (io_MCU_RST                    )  //i
  );
  BufferCC io_DC_RD_buffercc (
    .io_dataIn     (io_DC_RD                      ), //i
    .io_dataOut    (io_DC_RD_buffercc_io_dataOut  ), //o
    .io_MCU_CLK    (io_MCU_CLK                    ), //i
    .io_MCU_RST    (io_MCU_RST                    )  //i
  );
  PATA_HDD clockArea_slaveDevice (
    .io_pataBus_RST            (io_DC_RST                                        ), //i
    .io_pataBus_CS             (clockArea_DC_CS_SYNC[1:0]                        ), //i
    .io_pataBus_WR             (clockArea_WR_SYNC                                ), //i
    .io_pataBus_RD             (clockArea_RD_SYNC                                ), //i
    .io_pataBus_ADDR           (clockArea_DC_ADDR_SYNC[2:0]                      ), //i
    .io_pataBus_DATA_IN        (clockArea_DC_DATA_IN_SYNC[15:0]                  ), //i
    .io_pataBus_DATA_OUT       (clockArea_slaveDevice_io_pataBus_DATA_OUT[15:0]  ), //o
    .io_pataBus_DATA_OUT_EN    (clockArea_slaveDevice_io_pataBus_DATA_OUT_EN     ), //o
    .io_pataBus_DMARQ          (clockArea_slaveDevice_io_pataBus_DMARQ           ), //o
    .io_pataBus_DMACK          (io_DC_DMACK                                      ), //i
    .io_pataBus_IORDY          (clockArea_slaveDevice_io_pataBus_IORDY           ), //o
    .io_pataBus_INTRQ          (clockArea_slaveDevice_io_pataBus_INTRQ           ), //o
    .io_mcuBus_RD              (clockArea_slaveDevice_io_mcuBus_RD               ), //i
    .io_mcuBus_WR              (clockArea_slaveDevice_io_mcuBus_WR[1:0]          ), //i
    .io_mcuBus_ADDR            (clockArea_slaveDevice_io_mcuBus_ADDR[5:0]        ), //i
    .io_mcuBus_DATA_IN         (io_MCU_DATA_IN[15:0]                             ), //i
    .io_mcuBus_DATA_OUT        (clockArea_slaveDevice_io_mcuBus_DATA_OUT[15:0]   ), //o
    .io_mcuBus_IRQ             (clockArea_slaveDevice_io_mcuBus_IRQ              ), //o
    .io_MCU_CLK                (io_MCU_CLK                                       ), //i
    .io_MCU_RST                (io_MCU_RST                                       )  //i
  );
  assign clockArea_globalRegsCS = (io_MCU_ADDR[7 : 6] == 2'b00);
  assign clockArea_masterRegsCS = ((io_MCU_ADDR[7] == 1'b1) && (io_MCU_ADDR[6] == 1'b0));
  assign clockArea_slaveRegsCS = ((io_MCU_ADDR[7] == 1'b1) && (io_MCU_ADDR[6] == 1'b1));
  always @(*) begin
    io_MCU_DATA_OUT = 16'h0;
    if(clockArea_globalRegsCS) begin
      if(when_OpenGDEMU_l67) begin
        io_MCU_DATA_OUT = 16'h3cf8;
      end
      if(when_OpenGDEMU_l67_1) begin
        io_MCU_DATA_OUT = 16'h62c2;
      end
      if(when_OpenGDEMU_l67_2) begin
        io_MCU_DATA_OUT = 16'h0;
      end
      if(when_OpenGDEMU_l67_3) begin
        io_MCU_DATA_OUT = 16'h0;
      end
      if(when_OpenGDEMU_l71) begin
        io_MCU_DATA_OUT = _zz_io_MCU_DATA_OUT;
      end
    end
    if(clockArea_masterRegsCS) begin
      io_MCU_DATA_OUT = 16'hc0de;
    end
    if(clockArea_slaveRegsCS) begin
      io_MCU_DATA_OUT = clockArea_slaveDevice_io_mcuBus_DATA_OUT;
    end
  end

  assign when_OpenGDEMU_l67 = (io_MCU_ADDR == 8'h0);
  assign when_OpenGDEMU_l67_1 = (io_MCU_ADDR == 8'h01);
  assign when_OpenGDEMU_l67_2 = (io_MCU_ADDR == 8'h02);
  assign when_OpenGDEMU_l67_3 = (io_MCU_ADDR == 8'h03);
  assign when_OpenGDEMU_l71 = (io_MCU_ADDR == 8'h04);
  assign when_OpenGDEMU_l73 = io_MCU_WR[0];
  assign when_OpenGDEMU_l74 = io_MCU_WR[1];
  assign clockArea_WR_SYNC = io_DC_WR_buffercc_io_dataOut;
  assign clockArea_RD_SYNC = io_DC_RD_buffercc_io_dataOut;
  assign io_DC_DATA_OUT = clockArea_slaveDevice_io_pataBus_DATA_OUT;
  assign io_DC_DATA_OUT_EN = clockArea_slaveDevice_io_pataBus_DATA_OUT_EN;
  assign io_DC_DMARQ = clockArea_slaveDevice_io_pataBus_DMARQ;
  assign io_DC_INTRQ = clockArea_slaveDevice_io_pataBus_INTRQ;
  assign io_DC_IORDY = clockArea_slaveDevice_io_pataBus_IORDY;
  assign clockArea_slaveDevice_io_mcuBus_WR = (clockArea_slaveRegsCS ? io_MCU_WR : 2'b00);
  assign clockArea_slaveDevice_io_mcuBus_RD = (clockArea_slaveRegsCS ? io_MCU_RD : 1'b0);
  assign clockArea_slaveDevice_io_mcuBus_ADDR = io_MCU_ADDR[5 : 0];
  assign io_MCU_IRQ = clockArea_slaveDevice_io_mcuBus_IRQ;
  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      _zz_io_MCU_DATA_OUT <= 16'h5aa5;
    end else begin
      if(when_OpenGDEMU_l71) begin
        if(when_OpenGDEMU_l73) begin
          _zz_io_MCU_DATA_OUT[7 : 0] <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_OpenGDEMU_l74) begin
          _zz_io_MCU_DATA_OUT[15 : 8] <= io_MCU_DATA_IN[15 : 8];
        end
      end
    end
  end

  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      clockArea_DC_DATA_IN_SYNC <= 16'h0;
      clockArea_DC_ADDR_SYNC <= 3'b000;
      clockArea_DC_CS_SYNC <= 2'b00;
    end else begin
      clockArea_DC_DATA_IN_SYNC <= io_DC_DATA_IN;
      clockArea_DC_ADDR_SYNC <= io_DC_ADDR;
      clockArea_DC_CS_SYNC <= (~ io_DC_CSn);
    end
  end


endmodule

module PATA_HDD (
  input               io_pataBus_RST,
  input      [1:0]    io_pataBus_CS,
  input               io_pataBus_WR,
  input               io_pataBus_RD,
  input      [2:0]    io_pataBus_ADDR,
  input      [15:0]   io_pataBus_DATA_IN,
  output reg [15:0]   io_pataBus_DATA_OUT,
  output              io_pataBus_DATA_OUT_EN,
  output              io_pataBus_DMARQ,
  input               io_pataBus_DMACK,
  output              io_pataBus_IORDY,
  output              io_pataBus_INTRQ,
  input               io_mcuBus_RD,
  input      [1:0]    io_mcuBus_WR,
  input      [5:0]    io_mcuBus_ADDR,
  input      [15:0]   io_mcuBus_DATA_IN,
  output reg [15:0]   io_mcuBus_DATA_OUT,
  output              io_mcuBus_IRQ,
  input               io_MCU_CLK,
  input               io_MCU_RST
);

  reg                 ideRegs_readFIFOStream_io_push_valid;
  reg                 ideRegs_readFIFOStream_io_pop_ready;
  reg                 ideRegs_readFIFOStream_io_flush;
  reg                 ideRegs_writeFIFOStream_io_push_valid;
  reg                 ideRegs_writeFIFOStream_io_pop_ready;
  reg                 ideRegs_writeFIFOStream_io_flush;
  wire                ideRegs_readFIFOStream_io_push_ready;
  wire                ideRegs_readFIFOStream_io_pop_valid;
  wire       [15:0]   ideRegs_readFIFOStream_io_pop_payload;
  wire       [10:0]   ideRegs_readFIFOStream_io_occupancy;
  wire       [10:0]   ideRegs_readFIFOStream_io_availability;
  wire                ideRegs_writeFIFOStream_io_push_ready;
  wire                ideRegs_writeFIFOStream_io_pop_valid;
  wire       [15:0]   ideRegs_writeFIFOStream_io_pop_payload;
  wire       [10:0]   ideRegs_writeFIFOStream_io_occupancy;
  wire       [10:0]   ideRegs_writeFIFOStream_io_availability;
  wire                ideRegs_regBlock0;
  wire                ideRegs_regBlock1;
  wire                ideRegs_dmaTransfer;
  reg                 io_pataBus_WR_regNext;
  wire                ideRegs_WR_STROBE;
  reg        [7:0]    ideRegs_DEVICE;
  wire                ideRegs_DEV;
  wire                ideRegs_deviceSelected;
  reg                 ideRegs_HOB;
  reg                 ideRegs_SRST;
  reg                 ideRegs_nIEN;
  reg                 ideRegs_BSY;
  reg                 ideRegs_DRDY;
  reg                 ideRegs_DF;
  reg                 ideRegs_DRQ;
  reg                 ideRegs_ERR;
  reg        [7:0]    ideRegs_COMMAND;
  reg                 ideRegs_COMMAND_PEND;
  reg        [7:0]    ideRegs_LBALow;
  reg        [7:0]    ideRegs_LBALowPrev;
  reg        [7:0]    ideRegs_LBAMid;
  reg        [7:0]    ideRegs_LBAMidPrev;
  reg        [7:0]    ideRegs_LBAHigh;
  reg        [7:0]    ideRegs_LBAHighPrev;
  reg                 ideRegs_ABRT;
  reg        [7:0]    ideRegs_Features;
  reg        [15:0]   ideRegs_SectorCount;
  wire                when_PATARegisters_l107;
  reg                 ideRegs_SRST_regNext;
  wire                when_PATARegisters_l124;
  wire                when_PATARegisters_l129;
  wire                when_PATARegisters_l141;
  reg                 io_pataBus_RD_regNext;
  wire                when_PATARegisters_l147;
  wire                when_PATARegisters_l154;
  wire                when_PATARegisters_l165;
  wire                when_PATARegisters_l182;
  wire                when_PATARegisters_l199;
  wire                when_PATARegisters_l214;
  wire                when_PATARegisters_l229;
  wire                when_PATARegisters_l243;
  reg                 io_pataBus_WR_regNext_1;
  wire                when_PATARegisters_l270;
  wire                when_PATARegisters_l276;
  wire                mcuRegs_irqDelay_0;
  reg                 mcuRegs_irqDelay_1;
  reg                 mcuRegs_irqDelay_2;
  reg                 mcuRegs_irqDelay_3;
  wire                when_MCURegisters_l17;
  wire                when_MCURegisters_l26;
  wire                when_MCURegisters_l32;
  wire                when_MCURegisters_l39;
  wire                when_MCURegisters_l48;
  wire                when_MCURegisters_l50;
  wire                when_MCURegisters_l55;
  wire                when_MCURegisters_l57;
  wire                when_MCURegisters_l62;
  wire                when_MCURegisters_l65;
  wire                when_MCURegisters_l68;
  wire                when_MCURegisters_l73;
  wire                when_MCURegisters_l76;
  wire                when_MCURegisters_l79;
  wire                when_MCURegisters_l84;
  wire                when_MCURegisters_l87;
  wire                when_MCURegisters_l90;
  wire                when_MCURegisters_l95;
  wire                when_MCURegisters_l99;
  wire                when_MCURegisters_l102;
  wire                when_MCURegisters_l107;
  wire                when_MCURegisters_l111;
  wire                when_MCURegisters_l114;
  wire                when_MCURegisters_l119;
  wire                when_MCURegisters_l123;
  wire                when_MCURegisters_l128;
  wire                when_MCURegisters_l132;
  wire                when_MCURegisters_l136;

  StreamFifo ideRegs_readFIFOStream (
    .io_push_valid      (ideRegs_readFIFOStream_io_push_valid          ), //i
    .io_push_ready      (ideRegs_readFIFOStream_io_push_ready          ), //o
    .io_push_payload    (io_mcuBus_DATA_IN[15:0]                       ), //i
    .io_pop_valid       (ideRegs_readFIFOStream_io_pop_valid           ), //o
    .io_pop_ready       (ideRegs_readFIFOStream_io_pop_ready           ), //i
    .io_pop_payload     (ideRegs_readFIFOStream_io_pop_payload[15:0]   ), //o
    .io_flush           (ideRegs_readFIFOStream_io_flush               ), //i
    .io_occupancy       (ideRegs_readFIFOStream_io_occupancy[10:0]     ), //o
    .io_availability    (ideRegs_readFIFOStream_io_availability[10:0]  ), //o
    .io_MCU_CLK         (io_MCU_CLK                                    ), //i
    .io_MCU_RST         (io_MCU_RST                                    )  //i
  );
  StreamFifo ideRegs_writeFIFOStream (
    .io_push_valid      (ideRegs_writeFIFOStream_io_push_valid          ), //i
    .io_push_ready      (ideRegs_writeFIFOStream_io_push_ready          ), //o
    .io_push_payload    (io_pataBus_DATA_IN[15:0]                       ), //i
    .io_pop_valid       (ideRegs_writeFIFOStream_io_pop_valid           ), //o
    .io_pop_ready       (ideRegs_writeFIFOStream_io_pop_ready           ), //i
    .io_pop_payload     (ideRegs_writeFIFOStream_io_pop_payload[15:0]   ), //o
    .io_flush           (ideRegs_writeFIFOStream_io_flush               ), //i
    .io_occupancy       (ideRegs_writeFIFOStream_io_occupancy[10:0]     ), //o
    .io_availability    (ideRegs_writeFIFOStream_io_availability[10:0]  ), //o
    .io_MCU_CLK         (io_MCU_CLK                                     ), //i
    .io_MCU_RST         (io_MCU_RST                                     )  //i
  );
  assign ideRegs_regBlock0 = (io_pataBus_CS == 2'b01);
  assign ideRegs_regBlock1 = (io_pataBus_CS == 2'b10);
  assign ideRegs_dmaTransfer = (((io_pataBus_CS == 2'b00) && io_pataBus_DMARQ) && io_pataBus_DMACK);
  assign io_pataBus_DMARQ = 1'b0;
  assign io_pataBus_INTRQ = 1'b0;
  assign io_pataBus_IORDY = 1'b1;
  assign io_pataBus_DATA_OUT_EN = (io_pataBus_RD && ((ideRegs_regBlock0 || ideRegs_regBlock1) || ideRegs_dmaTransfer));
  assign ideRegs_WR_STROBE = ((! io_pataBus_WR) && io_pataBus_WR_regNext);
  assign ideRegs_DEV = ideRegs_DEVICE[4];
  assign ideRegs_deviceSelected = (ideRegs_DEV == 1'b1);
  always @(*) begin
    ideRegs_readFIFOStream_io_pop_ready = 1'b0;
    if(ideRegs_deviceSelected) begin
      if(ideRegs_regBlock0) begin
        if(when_PATARegisters_l141) begin
          if(when_PATARegisters_l147) begin
            ideRegs_readFIFOStream_io_pop_ready = 1'b1;
          end
        end
      end
    end
  end

  always @(*) begin
    ideRegs_writeFIFOStream_io_push_valid = 1'b0;
    if(ideRegs_deviceSelected) begin
      if(ideRegs_regBlock0) begin
        if(when_PATARegisters_l141) begin
          if(io_pataBus_WR) begin
            ideRegs_writeFIFOStream_io_push_valid = 1'b1;
          end
        end
      end
    end
  end

  always @(*) begin
    io_pataBus_DATA_OUT = 16'h0;
    if(ideRegs_deviceSelected) begin
      if(ideRegs_dmaTransfer) begin
        io_pataBus_DATA_OUT = 16'hdada;
      end
      if(ideRegs_regBlock0) begin
        if(when_PATARegisters_l141) begin
          if(io_pataBus_RD) begin
            io_pataBus_DATA_OUT = ideRegs_readFIFOStream_io_pop_payload;
          end
        end
        if(when_PATARegisters_l154) begin
          if(io_pataBus_RD) begin
            io_pataBus_DATA_OUT[2] = ideRegs_ABRT;
          end
        end
        if(when_PATARegisters_l165) begin
          if(io_pataBus_RD) begin
            if(ideRegs_HOB) begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_SectorCount[15 : 8];
            end else begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_SectorCount[7 : 0];
            end
          end
        end
        if(when_PATARegisters_l182) begin
          if(io_pataBus_RD) begin
            if(ideRegs_HOB) begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_LBALowPrev;
            end else begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_LBALow;
            end
          end
        end
        if(when_PATARegisters_l199) begin
          if(io_pataBus_RD) begin
            if(ideRegs_HOB) begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_LBAMidPrev;
            end else begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_LBAMid;
            end
          end
        end
        if(when_PATARegisters_l214) begin
          if(io_pataBus_RD) begin
            if(ideRegs_HOB) begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_LBAHighPrev;
            end else begin
              io_pataBus_DATA_OUT[7 : 0] = ideRegs_LBAHigh;
            end
          end
        end
        if(when_PATARegisters_l229) begin
          if(io_pataBus_RD) begin
            io_pataBus_DATA_OUT[7 : 0] = ideRegs_DEVICE;
          end
        end
        if(when_PATARegisters_l243) begin
          if(io_pataBus_RD) begin
            io_pataBus_DATA_OUT[7] = ideRegs_BSY;
            io_pataBus_DATA_OUT[6] = ideRegs_DRDY;
            io_pataBus_DATA_OUT[5] = ideRegs_DF;
            io_pataBus_DATA_OUT[3] = ideRegs_DRQ;
            io_pataBus_DATA_OUT[0] = ideRegs_ERR;
          end
        end
      end
      if(ideRegs_regBlock1) begin
        if(when_PATARegisters_l270) begin
          if(io_pataBus_RD) begin
            io_pataBus_DATA_OUT[7 : 0] = ideRegs_COMMAND;
            io_pataBus_DATA_OUT[15 : 8] = 8'h5a;
          end
        end
        if(when_PATARegisters_l276) begin
          if(io_pataBus_RD) begin
            io_pataBus_DATA_OUT[7] = ideRegs_BSY;
            io_pataBus_DATA_OUT[6] = ideRegs_DRDY;
            io_pataBus_DATA_OUT[5] = ideRegs_DF;
            io_pataBus_DATA_OUT[3] = ideRegs_DRQ;
            io_pataBus_DATA_OUT[0] = ideRegs_ERR;
          end
        end
      end
    end
  end

  always @(*) begin
    ideRegs_readFIFOStream_io_flush = 1'b0;
    if(when_PATARegisters_l107) begin
      ideRegs_readFIFOStream_io_flush = 1'b1;
    end
    if(when_MCURegisters_l17) begin
      if(when_MCURegisters_l26) begin
        ideRegs_readFIFOStream_io_flush = 1'b1;
      end
    end
  end

  always @(*) begin
    ideRegs_writeFIFOStream_io_flush = 1'b0;
    if(when_PATARegisters_l107) begin
      ideRegs_writeFIFOStream_io_flush = 1'b1;
    end
  end

  assign when_PATARegisters_l107 = (ideRegs_SRST || io_pataBus_RST);
  assign when_PATARegisters_l124 = ((! ideRegs_SRST) && ideRegs_SRST_regNext);
  assign when_PATARegisters_l129 = ((ideRegs_regBlock0 && (io_pataBus_ADDR == 3'b110)) && io_pataBus_WR);
  assign when_PATARegisters_l141 = (io_pataBus_ADDR == 3'b000);
  assign when_PATARegisters_l147 = ((! io_pataBus_RD) && io_pataBus_RD_regNext);
  assign when_PATARegisters_l154 = (io_pataBus_ADDR == 3'b001);
  assign when_PATARegisters_l165 = (io_pataBus_ADDR == 3'b010);
  assign when_PATARegisters_l182 = (io_pataBus_ADDR == 3'b011);
  assign when_PATARegisters_l199 = (io_pataBus_ADDR == 3'b100);
  assign when_PATARegisters_l214 = (io_pataBus_ADDR == 3'b101);
  assign when_PATARegisters_l229 = (io_pataBus_ADDR == 3'b110);
  assign when_PATARegisters_l243 = (io_pataBus_ADDR == 3'b111);
  assign when_PATARegisters_l270 = (io_pataBus_ADDR == 3'b000);
  assign when_PATARegisters_l276 = (io_pataBus_ADDR == 3'b110);
  always @(*) begin
    io_mcuBus_DATA_OUT = 16'h0;
    if(when_MCURegisters_l17) begin
      io_mcuBus_DATA_OUT[0] = 1'b1;
      io_mcuBus_DATA_OUT[1] = ideRegs_COMMAND_PEND;
    end
    if(when_MCURegisters_l32) begin
      io_mcuBus_DATA_OUT[0] = ideRegs_ERR;
      io_mcuBus_DATA_OUT[3] = ideRegs_DRQ;
      io_mcuBus_DATA_OUT[5] = ideRegs_DF;
      io_mcuBus_DATA_OUT[6] = ideRegs_DRDY;
      io_mcuBus_DATA_OUT[7] = ideRegs_BSY;
    end
    if(when_MCURegisters_l48) begin
      io_mcuBus_DATA_OUT[7 : 0] = ideRegs_DEVICE;
    end
    if(when_MCURegisters_l55) begin
      io_mcuBus_DATA_OUT[3] = ideRegs_ABRT;
    end
    if(when_MCURegisters_l62) begin
      io_mcuBus_DATA_OUT[7 : 0] = ideRegs_COMMAND;
      io_mcuBus_DATA_OUT[15 : 8] = ideRegs_Features;
    end
    if(when_MCURegisters_l73) begin
      io_mcuBus_DATA_OUT = ideRegs_SectorCount;
    end
    if(when_MCURegisters_l84) begin
      io_mcuBus_DATA_OUT[7 : 0] = ideRegs_LBALow;
      io_mcuBus_DATA_OUT[15 : 8] = ideRegs_LBAMid;
    end
    if(when_MCURegisters_l95) begin
      io_mcuBus_DATA_OUT[7 : 0] = ideRegs_LBAHigh;
      io_mcuBus_DATA_OUT[15 : 8] = ideRegs_LBALowPrev;
    end
    if(when_MCURegisters_l107) begin
      io_mcuBus_DATA_OUT[7 : 0] = ideRegs_LBAMidPrev;
      io_mcuBus_DATA_OUT[15 : 8] = ideRegs_LBAHighPrev;
    end
    if(when_MCURegisters_l119) begin
      io_mcuBus_DATA_OUT = ideRegs_writeFIFOStream_io_pop_payload;
    end
    if(when_MCURegisters_l128) begin
      io_mcuBus_DATA_OUT[10 : 0] = ideRegs_readFIFOStream_io_occupancy;
    end
    if(when_MCURegisters_l132) begin
      io_mcuBus_DATA_OUT[10 : 0] = ideRegs_writeFIFOStream_io_occupancy;
    end
    if(when_MCURegisters_l136) begin
      io_mcuBus_DATA_OUT = 16'hcafe;
    end
  end

  assign mcuRegs_irqDelay_0 = ideRegs_COMMAND_PEND;
  assign io_mcuBus_IRQ = mcuRegs_irqDelay_3;
  always @(*) begin
    ideRegs_readFIFOStream_io_push_valid = 1'b0;
    if(when_MCURegisters_l119) begin
      if(when_MCURegisters_l123) begin
        ideRegs_readFIFOStream_io_push_valid = 1'b1;
      end
    end
  end

  always @(*) begin
    ideRegs_writeFIFOStream_io_pop_ready = 1'b0;
    if(when_MCURegisters_l119) begin
      ideRegs_writeFIFOStream_io_pop_ready = 1'b1;
    end
  end

  assign when_MCURegisters_l17 = (io_mcuBus_ADDR == 6'h0);
  assign when_MCURegisters_l26 = io_mcuBus_WR[0];
  assign when_MCURegisters_l32 = (io_mcuBus_ADDR == 6'h01);
  assign when_MCURegisters_l39 = io_mcuBus_WR[0];
  assign when_MCURegisters_l48 = (io_mcuBus_ADDR == 6'h02);
  assign when_MCURegisters_l50 = io_mcuBus_WR[0];
  assign when_MCURegisters_l55 = (io_mcuBus_ADDR == 6'h03);
  assign when_MCURegisters_l57 = io_mcuBus_WR[0];
  assign when_MCURegisters_l62 = (io_mcuBus_ADDR == 6'h04);
  assign when_MCURegisters_l65 = io_mcuBus_WR[0];
  assign when_MCURegisters_l68 = io_mcuBus_WR[1];
  assign when_MCURegisters_l73 = (io_mcuBus_ADDR == 6'h05);
  assign when_MCURegisters_l76 = io_mcuBus_WR[0];
  assign when_MCURegisters_l79 = io_mcuBus_WR[1];
  assign when_MCURegisters_l84 = (io_mcuBus_ADDR == 6'h06);
  assign when_MCURegisters_l87 = io_mcuBus_WR[0];
  assign when_MCURegisters_l90 = io_mcuBus_WR[1];
  assign when_MCURegisters_l95 = (io_mcuBus_ADDR == 6'h07);
  assign when_MCURegisters_l99 = io_mcuBus_WR[0];
  assign when_MCURegisters_l102 = io_mcuBus_WR[1];
  assign when_MCURegisters_l107 = (io_mcuBus_ADDR == 6'h08);
  assign when_MCURegisters_l111 = io_mcuBus_WR[0];
  assign when_MCURegisters_l114 = io_mcuBus_WR[1];
  assign when_MCURegisters_l119 = (io_mcuBus_ADDR == 6'h09);
  assign when_MCURegisters_l123 = (io_mcuBus_WR[0] && io_mcuBus_WR[1]);
  assign when_MCURegisters_l128 = (io_mcuBus_ADDR == 6'h0a);
  assign when_MCURegisters_l132 = (io_mcuBus_ADDR == 6'h0b);
  assign when_MCURegisters_l136 = (io_mcuBus_ADDR == 6'h0c);
  always @(posedge io_MCU_CLK) begin
    io_pataBus_WR_regNext <= io_pataBus_WR;
    ideRegs_SRST_regNext <= ideRegs_SRST;
    mcuRegs_irqDelay_1 <= mcuRegs_irqDelay_0;
    mcuRegs_irqDelay_2 <= mcuRegs_irqDelay_1;
    mcuRegs_irqDelay_3 <= mcuRegs_irqDelay_2;
  end

  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      ideRegs_DEVICE <= 8'h0;
      ideRegs_HOB <= 1'b0;
      ideRegs_SRST <= 1'b0;
      ideRegs_nIEN <= 1'b0;
      ideRegs_BSY <= 1'b1;
      ideRegs_DRDY <= 1'b0;
      ideRegs_DF <= 1'b0;
      ideRegs_DRQ <= 1'b0;
      ideRegs_ERR <= 1'b0;
      ideRegs_COMMAND <= 8'h0;
      ideRegs_COMMAND_PEND <= 1'b0;
      ideRegs_LBALow <= 8'h0;
      ideRegs_LBALowPrev <= 8'h0;
      ideRegs_LBAMid <= 8'h0;
      ideRegs_LBAMidPrev <= 8'h0;
      ideRegs_LBAHigh <= 8'h0;
      ideRegs_LBAHighPrev <= 8'h0;
      ideRegs_ABRT <= 1'b0;
      ideRegs_Features <= 8'h0;
      ideRegs_SectorCount <= 16'h0;
    end else begin
      if(when_PATARegisters_l107) begin
        ideRegs_BSY <= 1'b1;
        ideRegs_DRDY <= 1'b0;
        ideRegs_DRQ <= 1'b0;
        ideRegs_COMMAND <= 8'h0;
        ideRegs_LBALow <= 8'h0;
        ideRegs_LBALowPrev <= 8'h0;
        ideRegs_LBAMid <= 8'h0;
        ideRegs_LBAMidPrev <= 8'h0;
        ideRegs_LBAHigh <= 8'h0;
        ideRegs_LBAHighPrev <= 8'h0;
        ideRegs_COMMAND_PEND <= 1'b0;
      end else begin
        if(when_PATARegisters_l124) begin
          ideRegs_BSY <= 1'b0;
          ideRegs_DRDY <= 1'b1;
        end
      end
      if(when_PATARegisters_l129) begin
        ideRegs_DEVICE <= io_pataBus_DATA_IN[7 : 0];
      end
      if(ideRegs_deviceSelected) begin
        if(ideRegs_regBlock0) begin
          if(when_PATARegisters_l154) begin
            if(io_pataBus_WR) begin
              ideRegs_Features <= io_pataBus_DATA_IN[7 : 0];
            end
          end
          if(when_PATARegisters_l165) begin
            if(io_pataBus_WR) begin
              ideRegs_SectorCount[15 : 8] <= ideRegs_SectorCount[7 : 0];
              ideRegs_SectorCount[7 : 0] <= io_pataBus_DATA_IN[7 : 0];
            end
          end
          if(when_PATARegisters_l182) begin
            if(io_pataBus_WR) begin
              ideRegs_LBALowPrev <= ideRegs_LBALow;
              ideRegs_LBALow <= io_pataBus_DATA_IN[7 : 0];
            end
          end
          if(when_PATARegisters_l199) begin
            if(io_pataBus_WR) begin
              ideRegs_LBAMidPrev <= ideRegs_LBAMid;
              ideRegs_LBAMid <= io_pataBus_DATA_IN[7 : 0];
            end
          end
          if(when_PATARegisters_l214) begin
            if(io_pataBus_WR) begin
              ideRegs_LBAHighPrev <= ideRegs_LBAHigh;
              ideRegs_LBAHigh <= io_pataBus_DATA_IN[7 : 0];
            end
          end
          if(when_PATARegisters_l243) begin
            if(io_pataBus_WR) begin
              ideRegs_COMMAND <= io_pataBus_DATA_IN[7 : 0];
              ideRegs_BSY <= 1'b1;
              ideRegs_COMMAND_PEND <= 1'b1;
            end
          end
        end
        if(ideRegs_regBlock1) begin
          if(when_PATARegisters_l276) begin
            if(io_pataBus_WR) begin
              ideRegs_HOB <= io_pataBus_DATA_IN[7];
              ideRegs_SRST <= io_pataBus_DATA_IN[2];
              ideRegs_nIEN <= io_pataBus_DATA_IN[1];
            end
          end
        end
      end
      if(when_MCURegisters_l17) begin
        if(when_MCURegisters_l26) begin
          ideRegs_COMMAND_PEND <= io_mcuBus_DATA_IN[1];
        end
      end
      if(when_MCURegisters_l32) begin
        if(when_MCURegisters_l39) begin
          ideRegs_ERR <= io_mcuBus_DATA_IN[0];
          ideRegs_DRQ <= io_mcuBus_DATA_IN[3];
          ideRegs_DF <= io_mcuBus_DATA_IN[5];
          ideRegs_DRDY <= io_mcuBus_DATA_IN[6];
          ideRegs_BSY <= io_mcuBus_DATA_IN[7];
        end
      end
      if(when_MCURegisters_l48) begin
        if(when_MCURegisters_l50) begin
          ideRegs_DEVICE <= io_mcuBus_DATA_IN[7 : 0];
        end
      end
      if(when_MCURegisters_l55) begin
        if(when_MCURegisters_l57) begin
          ideRegs_ABRT <= io_mcuBus_DATA_IN[3];
        end
      end
      if(when_MCURegisters_l62) begin
        if(when_MCURegisters_l65) begin
          ideRegs_COMMAND <= io_mcuBus_DATA_IN[7 : 0];
        end
        if(when_MCURegisters_l68) begin
          ideRegs_Features <= io_mcuBus_DATA_IN[15 : 8];
        end
      end
      if(when_MCURegisters_l73) begin
        if(when_MCURegisters_l76) begin
          ideRegs_SectorCount[7 : 0] <= io_mcuBus_DATA_IN[7 : 0];
        end
        if(when_MCURegisters_l79) begin
          ideRegs_SectorCount[15 : 8] <= io_mcuBus_DATA_IN[15 : 8];
        end
      end
      if(when_MCURegisters_l84) begin
        if(when_MCURegisters_l87) begin
          ideRegs_LBALow <= io_mcuBus_DATA_IN[7 : 0];
        end
        if(when_MCURegisters_l90) begin
          ideRegs_LBAMid <= io_mcuBus_DATA_IN[15 : 8];
        end
      end
      if(when_MCURegisters_l95) begin
        if(when_MCURegisters_l99) begin
          ideRegs_LBAHigh <= io_mcuBus_DATA_IN[7 : 0];
        end
        if(when_MCURegisters_l102) begin
          ideRegs_LBALowPrev <= io_mcuBus_DATA_IN[15 : 8];
        end
      end
      if(when_MCURegisters_l107) begin
        if(when_MCURegisters_l111) begin
          ideRegs_LBAMidPrev <= io_mcuBus_DATA_IN[7 : 0];
        end
        if(when_MCURegisters_l114) begin
          ideRegs_LBAHighPrev <= io_mcuBus_DATA_IN[15 : 8];
        end
      end
    end
  end

  always @(posedge io_MCU_CLK) begin
    io_pataBus_RD_regNext <= io_pataBus_RD;
  end

  always @(posedge io_MCU_CLK) begin
    io_pataBus_WR_regNext_1 <= io_pataBus_WR;
  end


endmodule

//BufferCC replaced by BufferCC

module BufferCC (
  input               io_dataIn,
  output              io_dataOut,
  input               io_MCU_CLK,
  input               io_MCU_RST
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

//StreamFifo replaced by StreamFifo

module StreamFifo (
  input               io_push_valid,
  output              io_push_ready,
  input      [15:0]   io_push_payload,
  output              io_pop_valid,
  input               io_pop_ready,
  output     [15:0]   io_pop_payload,
  input               io_flush,
  output     [10:0]   io_occupancy,
  output     [10:0]   io_availability,
  input               io_MCU_CLK,
  input               io_MCU_RST
);

  reg        [15:0]   _zz_logic_ram_port0;
  wire       [9:0]    _zz_logic_pushPtr_valueNext;
  wire       [0:0]    _zz_logic_pushPtr_valueNext_1;
  wire       [9:0]    _zz_logic_popPtr_valueNext;
  wire       [0:0]    _zz_logic_popPtr_valueNext_1;
  wire                _zz_logic_ram_port;
  wire                _zz_io_pop_payload;
  wire       [9:0]    _zz_io_availability;
  reg                 _zz_1;
  reg                 logic_pushPtr_willIncrement;
  reg                 logic_pushPtr_willClear;
  reg        [9:0]    logic_pushPtr_valueNext;
  reg        [9:0]    logic_pushPtr_value;
  wire                logic_pushPtr_willOverflowIfInc;
  wire                logic_pushPtr_willOverflow;
  reg                 logic_popPtr_willIncrement;
  reg                 logic_popPtr_willClear;
  reg        [9:0]    logic_popPtr_valueNext;
  reg        [9:0]    logic_popPtr_value;
  wire                logic_popPtr_willOverflowIfInc;
  wire                logic_popPtr_willOverflow;
  wire                logic_ptrMatch;
  reg                 logic_risingOccupancy;
  wire                logic_pushing;
  wire                logic_popping;
  wire                logic_empty;
  wire                logic_full;
  reg                 _zz_io_pop_valid;
  wire                when_Stream_l954;
  wire       [9:0]    logic_ptrDif;
  reg [15:0] logic_ram [0:1023];

  assign _zz_logic_pushPtr_valueNext_1 = logic_pushPtr_willIncrement;
  assign _zz_logic_pushPtr_valueNext = {9'd0, _zz_logic_pushPtr_valueNext_1};
  assign _zz_logic_popPtr_valueNext_1 = logic_popPtr_willIncrement;
  assign _zz_logic_popPtr_valueNext = {9'd0, _zz_logic_popPtr_valueNext_1};
  assign _zz_io_availability = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_io_pop_payload = 1'b1;
  always @(posedge io_MCU_CLK) begin
    if(_zz_io_pop_payload) begin
      _zz_logic_ram_port0 <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @(posedge io_MCU_CLK) begin
    if(_zz_1) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @(*) begin
    _zz_1 = 1'b0;
    if(logic_pushing) begin
      _zz_1 = 1'b1;
    end
  end

  always @(*) begin
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing) begin
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @(*) begin
    logic_pushPtr_willClear = 1'b0;
    if(io_flush) begin
      logic_pushPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == 10'h3ff);
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @(*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_logic_pushPtr_valueNext);
    if(logic_pushPtr_willClear) begin
      logic_pushPtr_valueNext = 10'h0;
    end
  end

  always @(*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping) begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  always @(*) begin
    logic_popPtr_willClear = 1'b0;
    if(io_flush) begin
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == 10'h3ff);
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @(*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_logic_popPtr_valueNext);
    if(logic_popPtr_willClear) begin
      logic_popPtr_valueNext = 10'h0;
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_io_pop_valid && (! logic_full))));
  assign io_pop_payload = _zz_logic_ram_port0;
  assign when_Stream_l954 = (logic_pushing != logic_popping);
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_io_availability};
  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      logic_pushPtr_value <= 10'h0;
      logic_popPtr_value <= 10'h0;
      logic_risingOccupancy <= 1'b0;
      _zz_io_pop_valid <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_io_pop_valid <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if(when_Stream_l954) begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush) begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end


endmodule
