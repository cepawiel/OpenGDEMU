// Generator : SpinalHDL v1.7.3    git head : ed8004c489ee8a38c2cab309d0447b543fe9d5b8
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
  output reg [1:0]    io_MCU_DATA_OUT_EN,
  output              io_MCU_IRQ,
  input               io_IDE_RSTn,
  input      [1:0]    io_IDE_CSn,
  input               io_IDE_WRn,
  input               io_IDE_RDn,
  input      [2:0]    io_IDE_ADDR,
  input      [15:0]   io_IDE_DATA_IN,
  output     [15:0]   io_IDE_DATA_OUT,
  output              io_IDE_DATA_OUT_EN,
  input               io_IDE_DMACKn,
  output              io_IDE_DMARQ,
  output              io_IDE_IORDY,
  output              io_IDE_INTRQ,
  input               io_DC_CDCLK,
  input               io_DC_SCK,
  input               io_DC_SDAT,
  input               io_DC_LRCK,
  input               io_DC_EMPH
);

  reg                 clockArea_primary_readFIFOStream_io_push_valid;
  reg                 clockArea_primary_readFIFOStream_io_pop_ready;
  reg                 clockArea_primary_readFIFOStream_io_flush;
  reg                 clockArea_primary_writeFIFOStream_io_push_valid;
  reg                 clockArea_primary_writeFIFOStream_io_pop_ready;
  reg                 clockArea_primary_writeFIFOStream_io_flush;
  reg                 clockArea_secondary_readFIFOStream_io_push_valid;
  reg                 clockArea_secondary_readFIFOStream_io_pop_ready;
  reg                 clockArea_secondary_readFIFOStream_io_flush;
  reg                 clockArea_secondary_writeFIFOStream_io_push_valid;
  reg                 clockArea_secondary_writeFIFOStream_io_pop_ready;
  reg                 clockArea_secondary_writeFIFOStream_io_flush;
  wire                clockArea_primary_readFIFOStream_io_push_ready;
  wire                clockArea_primary_readFIFOStream_io_pop_valid;
  wire       [15:0]   clockArea_primary_readFIFOStream_io_pop_payload;
  wire       [10:0]   clockArea_primary_readFIFOStream_io_occupancy;
  wire       [10:0]   clockArea_primary_readFIFOStream_io_availability;
  wire                clockArea_primary_writeFIFOStream_io_push_ready;
  wire                clockArea_primary_writeFIFOStream_io_pop_valid;
  wire       [15:0]   clockArea_primary_writeFIFOStream_io_pop_payload;
  wire       [10:0]   clockArea_primary_writeFIFOStream_io_occupancy;
  wire       [10:0]   clockArea_primary_writeFIFOStream_io_availability;
  wire                clockArea_primary_ideDecode_io_DataRegister;
  wire                clockArea_primary_ideDecode_io_ErrorRegister;
  wire                clockArea_primary_ideDecode_io_FeaturesRegister;
  wire                clockArea_primary_ideDecode_io_SectorCountRegister;
  wire                clockArea_primary_ideDecode_io_LBALowRegister;
  wire                clockArea_primary_ideDecode_io_LBAMidRegister;
  wire                clockArea_primary_ideDecode_io_LBAHighRegister;
  wire                clockArea_primary_ideDecode_io_DeviceRegister;
  wire                clockArea_primary_ideDecode_io_CommandRegister;
  wire                clockArea_primary_ideDecode_io_StatusRegister;
  wire                clockArea_primary_ideDecode_io_DeviceControlRegister;
  wire                clockArea_primary_ideDecode_io_AlternateStatusRegister;
  wire                clockArea_primary_ideDecode_io_DebugTestpad;
  wire                clockArea_secondary_readFIFOStream_io_push_ready;
  wire                clockArea_secondary_readFIFOStream_io_pop_valid;
  wire       [15:0]   clockArea_secondary_readFIFOStream_io_pop_payload;
  wire       [10:0]   clockArea_secondary_readFIFOStream_io_occupancy;
  wire       [10:0]   clockArea_secondary_readFIFOStream_io_availability;
  wire                clockArea_secondary_writeFIFOStream_io_push_ready;
  wire                clockArea_secondary_writeFIFOStream_io_pop_valid;
  wire       [15:0]   clockArea_secondary_writeFIFOStream_io_pop_payload;
  wire       [10:0]   clockArea_secondary_writeFIFOStream_io_occupancy;
  wire       [10:0]   clockArea_secondary_writeFIFOStream_io_availability;
  wire                clockArea_secondary_ideDecode_io_DataRegister;
  wire                clockArea_secondary_ideDecode_io_ErrorRegister;
  wire                clockArea_secondary_ideDecode_io_FeaturesRegister;
  wire                clockArea_secondary_ideDecode_io_SectorCountRegister;
  wire                clockArea_secondary_ideDecode_io_LBALowRegister;
  wire                clockArea_secondary_ideDecode_io_LBAMidRegister;
  wire                clockArea_secondary_ideDecode_io_LBAHighRegister;
  wire                clockArea_secondary_ideDecode_io_DeviceRegister;
  wire                clockArea_secondary_ideDecode_io_CommandRegister;
  wire                clockArea_secondary_ideDecode_io_StatusRegister;
  wire                clockArea_secondary_ideDecode_io_DeviceControlRegister;
  wire                clockArea_secondary_ideDecode_io_AlternateStatusRegister;
  wire                clockArea_secondary_ideDecode_io_DebugTestpad;
  reg        [15:0]   _zz_io_MCU_DATA_OUT_2;
  wire       [1:0]    _zz_io_MCU_DATA_OUT_3;
  wire                clockArea_globalRegsCS;
  wire                clockArea_ideRegsCS;
  wire                clockArea_idePrimaryRegsCS;
  wire                clockArea_ideSecondaryRegsCS;
  reg        [15:0]   clockArea_primary_dataOutput;
  reg                 clockArea_primary_dataOutputEn;
  reg        [15:0]   clockArea_primary_mcuDataOutput;
  wire       [1:0]    clockArea_primary_mcuDataOutputEn;
  reg                 clockArea_primary_mcuIRQ;
  wire                clockArea_primary_RD;
  wire                clockArea_primary_WR;
  reg                 clockArea_primary_WR_regNext;
  wire                clockArea_primary_WR_STROBE;
  reg        [7:0]    clockArea_primary_DEVICE;
  wire                clockArea_primary_DEV;
  reg                 clockArea_primary_HOB;
  reg                 clockArea_primary_SRST;
  reg                 clockArea_primary_nIEN;
  reg                 clockArea_primary_BSY;
  reg                 clockArea_primary_DRDY;
  reg                 clockArea_primary_DF;
  reg                 clockArea_primary_DRQ;
  reg                 clockArea_primary_ERR;
  reg        [7:0]    clockArea_primary_COMMAND;
  reg                 clockArea_primary_COMMAND_PEND;
  reg        [7:0]    clockArea_primary_LBALow;
  wire                when_IDEDeviceRegs_l59;
  reg        [7:0]    clockArea_primary_LBALowPrev;
  reg        [7:0]    clockArea_primary_LBAMid;
  wire                when_IDEDeviceRegs_l61;
  reg        [7:0]    clockArea_primary_LBAMidPrev;
  reg        [7:0]    clockArea_primary_LBAHigh;
  wire                when_IDEDeviceRegs_l63;
  reg        [7:0]    clockArea_primary_LBAHighPrev;
  reg                 clockArea_primary_ABRT;
  reg        [7:0]    clockArea_primary_Features;
  reg        [15:0]   clockArea_primary_SectorCount;
  reg        [15:0]   clockArea_primary_testReg;
  wire                when_IDEDeviceRegs_l106;
  reg                 clockArea_primary_SRST_regNext;
  wire                when_IDEDeviceRegs_l123;
  wire                when_IDEDeviceRegs_l129;
  wire                clockArea_primary_dmaTransfer;
  wire                clockArea_primary_deviceSelected;
  wire                when_IDEDeviceRegs_l151;
  wire                when_IDEDeviceRegs_l159;
  wire                when_IDEDeviceRegs_l170;
  reg                 clockArea_primary_RD_regNext;
  wire                when_IDEDeviceRegs_l177;
  wire                when_IDEDeviceRegs_l185;
  wire                when_IDEDeviceRegs_l193;
  wire                when_IDEDeviceRegs_l200;
  wire                when_IDEDeviceRegs_l219;
  wire                when_IDEDeviceRegs_l237;
  wire                when_IDEDeviceRegs_l253;
  wire                when_IDEDeviceRegs_l269;
  wire                when_IDEDeviceRegs_l284;
  wire                when_IDEDeviceRegs_l296;
  reg                 clockArea_primary_WR_regNext_1;
  wire                when_IDEDeviceRegs_l320;
  wire                when_IDEDeviceRegs_l331;
  wire                when_IDEDeviceRegs_l340;
  wire                clockArea_primary_irqDelay_0;
  reg                 clockArea_primary_irqDelay_1;
  reg                 clockArea_primary_irqDelay_2;
  reg                 clockArea_primary_irqDelay_3;
  wire                when_IDEDeviceRegs_l362;
  wire                when_IDEDeviceRegs_l371;
  wire                when_IDEDeviceRegs_l389;
  wire                when_IDEDeviceRegs_l396;
  wire                when_IDEDeviceRegs_l405;
  wire                when_IDEDeviceRegs_l407;
  wire                when_IDEDeviceRegs_l412;
  wire                when_IDEDeviceRegs_l414;
  wire                when_IDEDeviceRegs_l419;
  wire                when_IDEDeviceRegs_l422;
  wire                when_IDEDeviceRegs_l425;
  wire                when_IDEDeviceRegs_l430;
  wire                when_IDEDeviceRegs_l433;
  wire                when_IDEDeviceRegs_l436;
  wire                when_IDEDeviceRegs_l441;
  wire                when_IDEDeviceRegs_l444;
  wire                when_IDEDeviceRegs_l447;
  wire                when_IDEDeviceRegs_l452;
  wire                when_IDEDeviceRegs_l456;
  wire                when_IDEDeviceRegs_l459;
  wire                when_IDEDeviceRegs_l464;
  wire                when_IDEDeviceRegs_l468;
  wire                when_IDEDeviceRegs_l471;
  wire                when_IDEDeviceRegs_l476;
  wire                when_IDEDeviceRegs_l480;
  wire                when_IDEDeviceRegs_l485;
  wire                when_IDEDeviceRegs_l489;
  wire                when_IDEDeviceRegs_l493;
  reg        [15:0]   clockArea_secondary_dataOutput;
  reg                 clockArea_secondary_dataOutputEn;
  reg        [15:0]   clockArea_secondary_mcuDataOutput;
  wire       [1:0]    clockArea_secondary_mcuDataOutputEn;
  reg                 clockArea_secondary_mcuIRQ;
  wire                clockArea_secondary_RD;
  wire                clockArea_secondary_WR;
  reg                 clockArea_secondary_WR_regNext;
  wire                clockArea_secondary_WR_STROBE;
  reg        [7:0]    clockArea_secondary_DEVICE;
  wire                clockArea_secondary_DEV;
  reg                 clockArea_secondary_HOB;
  reg                 clockArea_secondary_SRST;
  reg                 clockArea_secondary_nIEN;
  reg                 clockArea_secondary_BSY;
  reg                 clockArea_secondary_DRDY;
  reg                 clockArea_secondary_DF;
  reg                 clockArea_secondary_DRQ;
  reg                 clockArea_secondary_ERR;
  reg        [7:0]    clockArea_secondary_COMMAND;
  reg                 clockArea_secondary_COMMAND_PEND;
  reg        [7:0]    clockArea_secondary_LBALow;
  wire                when_IDEDeviceRegs_l59_1;
  reg        [7:0]    clockArea_secondary_LBALowPrev;
  reg        [7:0]    clockArea_secondary_LBAMid;
  wire                when_IDEDeviceRegs_l61_1;
  reg        [7:0]    clockArea_secondary_LBAMidPrev;
  reg        [7:0]    clockArea_secondary_LBAHigh;
  wire                when_IDEDeviceRegs_l63_1;
  reg        [7:0]    clockArea_secondary_LBAHighPrev;
  reg                 clockArea_secondary_ABRT;
  reg        [7:0]    clockArea_secondary_Features;
  reg        [15:0]   clockArea_secondary_SectorCount;
  reg        [15:0]   clockArea_secondary_testReg;
  wire                when_IDEDeviceRegs_l106_1;
  reg                 clockArea_secondary_SRST_regNext;
  wire                when_IDEDeviceRegs_l123_1;
  wire                when_IDEDeviceRegs_l129_1;
  wire                clockArea_secondary_dmaTransfer;
  wire                clockArea_secondary_deviceSelected;
  wire                when_IDEDeviceRegs_l151_1;
  wire                when_IDEDeviceRegs_l159_1;
  wire                when_IDEDeviceRegs_l170_1;
  reg                 clockArea_secondary_RD_regNext;
  wire                when_IDEDeviceRegs_l177_1;
  wire                when_IDEDeviceRegs_l185_1;
  wire                when_IDEDeviceRegs_l193_1;
  wire                when_IDEDeviceRegs_l200_1;
  wire                when_IDEDeviceRegs_l219_1;
  wire                when_IDEDeviceRegs_l237_1;
  wire                when_IDEDeviceRegs_l253_1;
  wire                when_IDEDeviceRegs_l269_1;
  wire                when_IDEDeviceRegs_l284_1;
  wire                when_IDEDeviceRegs_l296_1;
  reg                 clockArea_secondary_WR_regNext_1;
  wire                when_IDEDeviceRegs_l320_1;
  wire                when_IDEDeviceRegs_l331_1;
  wire                when_IDEDeviceRegs_l340_1;
  wire                clockArea_secondary_irqDelay_0;
  reg                 clockArea_secondary_irqDelay_1;
  reg                 clockArea_secondary_irqDelay_2;
  reg                 clockArea_secondary_irqDelay_3;
  wire                when_IDEDeviceRegs_l362_1;
  wire                when_IDEDeviceRegs_l371_1;
  wire                when_IDEDeviceRegs_l389_1;
  wire                when_IDEDeviceRegs_l396_1;
  wire                when_IDEDeviceRegs_l405_1;
  wire                when_IDEDeviceRegs_l407_1;
  wire                when_IDEDeviceRegs_l412_1;
  wire                when_IDEDeviceRegs_l414_1;
  wire                when_IDEDeviceRegs_l419_1;
  wire                when_IDEDeviceRegs_l422_1;
  wire                when_IDEDeviceRegs_l425_1;
  wire                when_IDEDeviceRegs_l430_1;
  wire                when_IDEDeviceRegs_l433_1;
  wire                when_IDEDeviceRegs_l436_1;
  wire                when_IDEDeviceRegs_l441_1;
  wire                when_IDEDeviceRegs_l444_1;
  wire                when_IDEDeviceRegs_l447_1;
  wire                when_IDEDeviceRegs_l452_1;
  wire                when_IDEDeviceRegs_l456_1;
  wire                when_IDEDeviceRegs_l459_1;
  wire                when_IDEDeviceRegs_l464_1;
  wire                when_IDEDeviceRegs_l468_1;
  wire                when_IDEDeviceRegs_l471_1;
  wire                when_IDEDeviceRegs_l476_1;
  wire                when_IDEDeviceRegs_l480_1;
  wire                when_IDEDeviceRegs_l485_1;
  wire                when_IDEDeviceRegs_l489_1;
  wire                when_IDEDeviceRegs_l493_1;
  wire       [63:0]   _zz_io_MCU_DATA_OUT;
  reg        [15:0]   _zz_io_MCU_DATA_OUT_1;
  wire                when_OpenGDEMU_l71;
  wire                when_OpenGDEMU_l72;

  assign _zz_io_MCU_DATA_OUT_3 = io_MCU_ADDR[1 : 0];
  StreamFifo clockArea_primary_readFIFOStream (
    .io_push_valid   (clockArea_primary_readFIFOStream_io_push_valid        ), //i
    .io_push_ready   (clockArea_primary_readFIFOStream_io_push_ready        ), //o
    .io_push_payload (io_MCU_DATA_IN[15:0]                                  ), //i
    .io_pop_valid    (clockArea_primary_readFIFOStream_io_pop_valid         ), //o
    .io_pop_ready    (clockArea_primary_readFIFOStream_io_pop_ready         ), //i
    .io_pop_payload  (clockArea_primary_readFIFOStream_io_pop_payload[15:0] ), //o
    .io_flush        (clockArea_primary_readFIFOStream_io_flush             ), //i
    .io_occupancy    (clockArea_primary_readFIFOStream_io_occupancy[10:0]   ), //o
    .io_availability (clockArea_primary_readFIFOStream_io_availability[10:0]), //o
    .io_MCU_CLK      (io_MCU_CLK                                            ), //i
    .io_MCU_RST      (io_MCU_RST                                            )  //i
  );
  StreamFifo clockArea_primary_writeFIFOStream (
    .io_push_valid   (clockArea_primary_writeFIFOStream_io_push_valid        ), //i
    .io_push_ready   (clockArea_primary_writeFIFOStream_io_push_ready        ), //o
    .io_push_payload (io_IDE_DATA_IN[15:0]                                   ), //i
    .io_pop_valid    (clockArea_primary_writeFIFOStream_io_pop_valid         ), //o
    .io_pop_ready    (clockArea_primary_writeFIFOStream_io_pop_ready         ), //i
    .io_pop_payload  (clockArea_primary_writeFIFOStream_io_pop_payload[15:0] ), //o
    .io_flush        (clockArea_primary_writeFIFOStream_io_flush             ), //i
    .io_occupancy    (clockArea_primary_writeFIFOStream_io_occupancy[10:0]   ), //o
    .io_availability (clockArea_primary_writeFIFOStream_io_availability[10:0]), //o
    .io_MCU_CLK      (io_MCU_CLK                                             ), //i
    .io_MCU_RST      (io_MCU_RST                                             )  //i
  );
  IDEAddressDecode clockArea_primary_ideDecode (
    .io_CSn                     (io_IDE_CSn[1:0]                                       ), //i
    .io_ADDR                    (io_IDE_ADDR[2:0]                                      ), //i
    .io_DataRegister            (clockArea_primary_ideDecode_io_DataRegister           ), //o
    .io_ErrorRegister           (clockArea_primary_ideDecode_io_ErrorRegister          ), //o
    .io_FeaturesRegister        (clockArea_primary_ideDecode_io_FeaturesRegister       ), //o
    .io_SectorCountRegister     (clockArea_primary_ideDecode_io_SectorCountRegister    ), //o
    .io_LBALowRegister          (clockArea_primary_ideDecode_io_LBALowRegister         ), //o
    .io_LBAMidRegister          (clockArea_primary_ideDecode_io_LBAMidRegister         ), //o
    .io_LBAHighRegister         (clockArea_primary_ideDecode_io_LBAHighRegister        ), //o
    .io_DeviceRegister          (clockArea_primary_ideDecode_io_DeviceRegister         ), //o
    .io_CommandRegister         (clockArea_primary_ideDecode_io_CommandRegister        ), //o
    .io_StatusRegister          (clockArea_primary_ideDecode_io_StatusRegister         ), //o
    .io_DeviceControlRegister   (clockArea_primary_ideDecode_io_DeviceControlRegister  ), //o
    .io_AlternateStatusRegister (clockArea_primary_ideDecode_io_AlternateStatusRegister), //o
    .io_DebugTestpad            (clockArea_primary_ideDecode_io_DebugTestpad           )  //o
  );
  MCURegisterManagerGenerator clockArea_primary_mcuRegGen (
  );
  StreamFifo clockArea_secondary_readFIFOStream (
    .io_push_valid   (clockArea_secondary_readFIFOStream_io_push_valid        ), //i
    .io_push_ready   (clockArea_secondary_readFIFOStream_io_push_ready        ), //o
    .io_push_payload (io_MCU_DATA_IN[15:0]                                    ), //i
    .io_pop_valid    (clockArea_secondary_readFIFOStream_io_pop_valid         ), //o
    .io_pop_ready    (clockArea_secondary_readFIFOStream_io_pop_ready         ), //i
    .io_pop_payload  (clockArea_secondary_readFIFOStream_io_pop_payload[15:0] ), //o
    .io_flush        (clockArea_secondary_readFIFOStream_io_flush             ), //i
    .io_occupancy    (clockArea_secondary_readFIFOStream_io_occupancy[10:0]   ), //o
    .io_availability (clockArea_secondary_readFIFOStream_io_availability[10:0]), //o
    .io_MCU_CLK      (io_MCU_CLK                                              ), //i
    .io_MCU_RST      (io_MCU_RST                                              )  //i
  );
  StreamFifo clockArea_secondary_writeFIFOStream (
    .io_push_valid   (clockArea_secondary_writeFIFOStream_io_push_valid        ), //i
    .io_push_ready   (clockArea_secondary_writeFIFOStream_io_push_ready        ), //o
    .io_push_payload (io_IDE_DATA_IN[15:0]                                     ), //i
    .io_pop_valid    (clockArea_secondary_writeFIFOStream_io_pop_valid         ), //o
    .io_pop_ready    (clockArea_secondary_writeFIFOStream_io_pop_ready         ), //i
    .io_pop_payload  (clockArea_secondary_writeFIFOStream_io_pop_payload[15:0] ), //o
    .io_flush        (clockArea_secondary_writeFIFOStream_io_flush             ), //i
    .io_occupancy    (clockArea_secondary_writeFIFOStream_io_occupancy[10:0]   ), //o
    .io_availability (clockArea_secondary_writeFIFOStream_io_availability[10:0]), //o
    .io_MCU_CLK      (io_MCU_CLK                                               ), //i
    .io_MCU_RST      (io_MCU_RST                                               )  //i
  );
  IDEAddressDecode clockArea_secondary_ideDecode (
    .io_CSn                     (io_IDE_CSn[1:0]                                         ), //i
    .io_ADDR                    (io_IDE_ADDR[2:0]                                        ), //i
    .io_DataRegister            (clockArea_secondary_ideDecode_io_DataRegister           ), //o
    .io_ErrorRegister           (clockArea_secondary_ideDecode_io_ErrorRegister          ), //o
    .io_FeaturesRegister        (clockArea_secondary_ideDecode_io_FeaturesRegister       ), //o
    .io_SectorCountRegister     (clockArea_secondary_ideDecode_io_SectorCountRegister    ), //o
    .io_LBALowRegister          (clockArea_secondary_ideDecode_io_LBALowRegister         ), //o
    .io_LBAMidRegister          (clockArea_secondary_ideDecode_io_LBAMidRegister         ), //o
    .io_LBAHighRegister         (clockArea_secondary_ideDecode_io_LBAHighRegister        ), //o
    .io_DeviceRegister          (clockArea_secondary_ideDecode_io_DeviceRegister         ), //o
    .io_CommandRegister         (clockArea_secondary_ideDecode_io_CommandRegister        ), //o
    .io_StatusRegister          (clockArea_secondary_ideDecode_io_StatusRegister         ), //o
    .io_DeviceControlRegister   (clockArea_secondary_ideDecode_io_DeviceControlRegister  ), //o
    .io_AlternateStatusRegister (clockArea_secondary_ideDecode_io_AlternateStatusRegister), //o
    .io_DebugTestpad            (clockArea_secondary_ideDecode_io_DebugTestpad           )  //o
  );
  MCURegisterManagerGenerator clockArea_secondary_mcuRegGen (
  );
  always @(*) begin
    case(_zz_io_MCU_DATA_OUT_3)
      2'b00 : _zz_io_MCU_DATA_OUT_2 = _zz_io_MCU_DATA_OUT[15 : 0];
      2'b01 : _zz_io_MCU_DATA_OUT_2 = _zz_io_MCU_DATA_OUT[31 : 16];
      2'b10 : _zz_io_MCU_DATA_OUT_2 = _zz_io_MCU_DATA_OUT[47 : 32];
      default : _zz_io_MCU_DATA_OUT_2 = _zz_io_MCU_DATA_OUT[63 : 48];
    endcase
  end

  assign clockArea_globalRegsCS = (! io_MCU_ADDR[7]);
  assign clockArea_ideRegsCS = io_MCU_ADDR[7];
  assign clockArea_idePrimaryRegsCS = (! io_MCU_ADDR[6]);
  assign clockArea_ideSecondaryRegsCS = io_MCU_ADDR[6];
  assign clockArea_primary_mcuDataOutputEn = 2'b00;
  assign clockArea_primary_RD = (! io_IDE_RDn);
  assign clockArea_primary_WR = (! io_IDE_WRn);
  assign clockArea_primary_WR_STROBE = ((! clockArea_primary_WR) && clockArea_primary_WR_regNext);
  assign clockArea_primary_DEV = clockArea_primary_DEVICE[4];
  assign when_IDEDeviceRegs_l59 = 1'b0;
  assign when_IDEDeviceRegs_l61 = 1'b0;
  assign when_IDEDeviceRegs_l63 = 1'b0;
  always @(*) begin
    clockArea_primary_readFIFOStream_io_flush = 1'b0;
    if(when_IDEDeviceRegs_l106) begin
      clockArea_primary_readFIFOStream_io_flush = 1'b1;
    end
    if(when_IDEDeviceRegs_l362) begin
      if(when_IDEDeviceRegs_l371) begin
        clockArea_primary_readFIFOStream_io_flush = 1'b1;
      end
    end
  end

  always @(*) begin
    clockArea_primary_writeFIFOStream_io_flush = 1'b0;
    if(when_IDEDeviceRegs_l106) begin
      clockArea_primary_writeFIFOStream_io_flush = 1'b1;
    end
  end

  always @(*) begin
    clockArea_primary_readFIFOStream_io_pop_ready = 1'b0;
    if(when_IDEDeviceRegs_l170) begin
      if(when_IDEDeviceRegs_l177) begin
        clockArea_primary_readFIFOStream_io_pop_ready = 1'b1;
      end
    end
  end

  always @(*) begin
    clockArea_primary_writeFIFOStream_io_push_valid = 1'b0;
    if(when_IDEDeviceRegs_l170) begin
      if(clockArea_primary_WR) begin
        clockArea_primary_writeFIFOStream_io_push_valid = 1'b1;
      end
    end
  end

  assign when_IDEDeviceRegs_l106 = (clockArea_primary_SRST || (! io_IDE_RSTn));
  assign when_IDEDeviceRegs_l123 = ((! clockArea_primary_SRST) && clockArea_primary_SRST_regNext);
  assign when_IDEDeviceRegs_l129 = (clockArea_primary_dataOutputEn && io_IDE_RDn);
  assign clockArea_primary_dmaTransfer = (((io_IDE_CSn == 2'b11) && io_IDE_DMARQ) && (! io_IDE_DMACKn));
  assign clockArea_primary_deviceSelected = (clockArea_primary_DEV == 1'b0);
  assign when_IDEDeviceRegs_l151 = (clockArea_primary_ideDecode_io_DeviceRegister && clockArea_primary_WR);
  assign when_IDEDeviceRegs_l159 = (clockArea_primary_deviceSelected && clockArea_primary_dmaTransfer);
  assign when_IDEDeviceRegs_l170 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_DataRegister);
  assign when_IDEDeviceRegs_l177 = ((! clockArea_primary_RD) && clockArea_primary_RD_regNext);
  assign when_IDEDeviceRegs_l185 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_ErrorRegister);
  assign when_IDEDeviceRegs_l193 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_FeaturesRegister);
  assign when_IDEDeviceRegs_l200 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_SectorCountRegister);
  assign when_IDEDeviceRegs_l219 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_LBALowRegister);
  assign when_IDEDeviceRegs_l237 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_LBAMidRegister);
  assign when_IDEDeviceRegs_l253 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_LBAHighRegister);
  assign when_IDEDeviceRegs_l269 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_DeviceRegister);
  assign when_IDEDeviceRegs_l284 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_StatusRegister);
  assign when_IDEDeviceRegs_l296 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_CommandRegister);
  assign when_IDEDeviceRegs_l320 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_AlternateStatusRegister);
  assign when_IDEDeviceRegs_l331 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_DeviceControlRegister);
  assign when_IDEDeviceRegs_l340 = (clockArea_primary_deviceSelected && clockArea_primary_ideDecode_io_DebugTestpad);
  assign clockArea_primary_irqDelay_0 = clockArea_primary_COMMAND_PEND;
  always @(*) begin
    clockArea_primary_readFIFOStream_io_push_valid = 1'b0;
    if(when_IDEDeviceRegs_l476) begin
      if(when_IDEDeviceRegs_l480) begin
        clockArea_primary_readFIFOStream_io_push_valid = 1'b1;
      end
    end
  end

  always @(*) begin
    clockArea_primary_writeFIFOStream_io_pop_ready = 1'b0;
    if(when_IDEDeviceRegs_l476) begin
      clockArea_primary_writeFIFOStream_io_pop_ready = 1'b1;
    end
  end

  assign when_IDEDeviceRegs_l362 = (io_MCU_ADDR == 8'h0);
  assign when_IDEDeviceRegs_l371 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l389 = (io_MCU_ADDR == 8'h01);
  assign when_IDEDeviceRegs_l396 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l405 = (io_MCU_ADDR == 8'h02);
  assign when_IDEDeviceRegs_l407 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l412 = (io_MCU_ADDR == 8'h03);
  assign when_IDEDeviceRegs_l414 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l419 = (io_MCU_ADDR == 8'h04);
  assign when_IDEDeviceRegs_l422 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l425 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l430 = (io_MCU_ADDR == 8'h05);
  assign when_IDEDeviceRegs_l433 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l436 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l441 = (io_MCU_ADDR == 8'h06);
  assign when_IDEDeviceRegs_l444 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l447 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l452 = (io_MCU_ADDR == 8'h07);
  assign when_IDEDeviceRegs_l456 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l459 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l464 = (io_MCU_ADDR == 8'h08);
  assign when_IDEDeviceRegs_l468 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l471 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l476 = (io_MCU_ADDR == 8'h09);
  assign when_IDEDeviceRegs_l480 = (io_MCU_WR[0] && io_MCU_WR[1]);
  assign when_IDEDeviceRegs_l485 = (io_MCU_ADDR == 8'h0a);
  assign when_IDEDeviceRegs_l489 = (io_MCU_ADDR == 8'h0b);
  assign when_IDEDeviceRegs_l493 = (io_MCU_ADDR == 8'h0c);
  assign clockArea_secondary_mcuDataOutputEn = 2'b00;
  assign clockArea_secondary_RD = (! io_IDE_RDn);
  assign clockArea_secondary_WR = (! io_IDE_WRn);
  assign clockArea_secondary_WR_STROBE = ((! clockArea_secondary_WR) && clockArea_secondary_WR_regNext);
  assign clockArea_secondary_DEV = clockArea_secondary_DEVICE[4];
  assign when_IDEDeviceRegs_l59_1 = 1'b0;
  assign when_IDEDeviceRegs_l61_1 = 1'b0;
  assign when_IDEDeviceRegs_l63_1 = 1'b0;
  always @(*) begin
    clockArea_secondary_readFIFOStream_io_flush = 1'b0;
    if(when_IDEDeviceRegs_l106_1) begin
      clockArea_secondary_readFIFOStream_io_flush = 1'b1;
    end
    if(when_IDEDeviceRegs_l362_1) begin
      if(when_IDEDeviceRegs_l371_1) begin
        clockArea_secondary_readFIFOStream_io_flush = 1'b1;
      end
    end
  end

  always @(*) begin
    clockArea_secondary_writeFIFOStream_io_flush = 1'b0;
    if(when_IDEDeviceRegs_l106_1) begin
      clockArea_secondary_writeFIFOStream_io_flush = 1'b1;
    end
  end

  always @(*) begin
    clockArea_secondary_readFIFOStream_io_pop_ready = 1'b0;
    if(when_IDEDeviceRegs_l170_1) begin
      if(when_IDEDeviceRegs_l177_1) begin
        clockArea_secondary_readFIFOStream_io_pop_ready = 1'b1;
      end
    end
  end

  always @(*) begin
    clockArea_secondary_writeFIFOStream_io_push_valid = 1'b0;
    if(when_IDEDeviceRegs_l170_1) begin
      if(clockArea_secondary_WR) begin
        clockArea_secondary_writeFIFOStream_io_push_valid = 1'b1;
      end
    end
  end

  assign when_IDEDeviceRegs_l106_1 = (clockArea_secondary_SRST || (! io_IDE_RSTn));
  assign when_IDEDeviceRegs_l123_1 = ((! clockArea_secondary_SRST) && clockArea_secondary_SRST_regNext);
  assign when_IDEDeviceRegs_l129_1 = (clockArea_secondary_dataOutputEn && io_IDE_RDn);
  assign clockArea_secondary_dmaTransfer = (((io_IDE_CSn == 2'b11) && io_IDE_DMARQ) && (! io_IDE_DMACKn));
  assign clockArea_secondary_deviceSelected = (clockArea_secondary_DEV == 1'b1);
  assign when_IDEDeviceRegs_l151_1 = (clockArea_secondary_ideDecode_io_DeviceRegister && clockArea_secondary_WR);
  assign when_IDEDeviceRegs_l159_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_dmaTransfer);
  assign when_IDEDeviceRegs_l170_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_DataRegister);
  assign when_IDEDeviceRegs_l177_1 = ((! clockArea_secondary_RD) && clockArea_secondary_RD_regNext);
  assign when_IDEDeviceRegs_l185_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_ErrorRegister);
  assign when_IDEDeviceRegs_l193_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_FeaturesRegister);
  assign when_IDEDeviceRegs_l200_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_SectorCountRegister);
  assign when_IDEDeviceRegs_l219_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_LBALowRegister);
  assign when_IDEDeviceRegs_l237_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_LBAMidRegister);
  assign when_IDEDeviceRegs_l253_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_LBAHighRegister);
  assign when_IDEDeviceRegs_l269_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_DeviceRegister);
  assign when_IDEDeviceRegs_l284_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_StatusRegister);
  assign when_IDEDeviceRegs_l296_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_CommandRegister);
  assign when_IDEDeviceRegs_l320_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_AlternateStatusRegister);
  assign when_IDEDeviceRegs_l331_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_DeviceControlRegister);
  assign when_IDEDeviceRegs_l340_1 = (clockArea_secondary_deviceSelected && clockArea_secondary_ideDecode_io_DebugTestpad);
  assign clockArea_secondary_irqDelay_0 = clockArea_secondary_COMMAND_PEND;
  always @(*) begin
    clockArea_secondary_readFIFOStream_io_push_valid = 1'b0;
    if(when_IDEDeviceRegs_l476_1) begin
      if(when_IDEDeviceRegs_l480_1) begin
        clockArea_secondary_readFIFOStream_io_push_valid = 1'b1;
      end
    end
  end

  always @(*) begin
    clockArea_secondary_writeFIFOStream_io_pop_ready = 1'b0;
    if(when_IDEDeviceRegs_l476_1) begin
      clockArea_secondary_writeFIFOStream_io_pop_ready = 1'b1;
    end
  end

  assign when_IDEDeviceRegs_l362_1 = (io_MCU_ADDR == 8'h0);
  assign when_IDEDeviceRegs_l371_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l389_1 = (io_MCU_ADDR == 8'h01);
  assign when_IDEDeviceRegs_l396_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l405_1 = (io_MCU_ADDR == 8'h02);
  assign when_IDEDeviceRegs_l407_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l412_1 = (io_MCU_ADDR == 8'h03);
  assign when_IDEDeviceRegs_l414_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l419_1 = (io_MCU_ADDR == 8'h04);
  assign when_IDEDeviceRegs_l422_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l425_1 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l430_1 = (io_MCU_ADDR == 8'h05);
  assign when_IDEDeviceRegs_l433_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l436_1 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l441_1 = (io_MCU_ADDR == 8'h06);
  assign when_IDEDeviceRegs_l444_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l447_1 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l452_1 = (io_MCU_ADDR == 8'h07);
  assign when_IDEDeviceRegs_l456_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l459_1 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l464_1 = (io_MCU_ADDR == 8'h08);
  assign when_IDEDeviceRegs_l468_1 = io_MCU_WR[0];
  assign when_IDEDeviceRegs_l471_1 = io_MCU_WR[1];
  assign when_IDEDeviceRegs_l476_1 = (io_MCU_ADDR == 8'h09);
  assign when_IDEDeviceRegs_l480_1 = (io_MCU_WR[0] && io_MCU_WR[1]);
  assign when_IDEDeviceRegs_l485_1 = (io_MCU_ADDR == 8'h0a);
  assign when_IDEDeviceRegs_l489_1 = (io_MCU_ADDR == 8'h0b);
  assign when_IDEDeviceRegs_l493_1 = (io_MCU_ADDR == 8'h0c);
  assign io_IDE_DATA_OUT = (clockArea_primary_dataOutputEn ? clockArea_primary_dataOutput : clockArea_secondary_dataOutput);
  assign io_IDE_DATA_OUT_EN = (clockArea_primary_dataOutputEn || clockArea_secondary_dataOutputEn);
  assign io_IDE_DMARQ = 1'b0;
  assign io_IDE_IORDY = 1'b0;
  assign io_IDE_INTRQ = 1'b0;
  assign io_MCU_IRQ = 1'b0;
  always @(*) begin
    io_MCU_DATA_OUT = 16'hc0de;
    if(clockArea_globalRegsCS) begin
      case(io_MCU_ADDR)
        8'h0, 8'h01, 8'h02, 8'h03 : begin
          io_MCU_DATA_OUT = _zz_io_MCU_DATA_OUT_2;
        end
        8'h04 : begin
          io_MCU_DATA_OUT = _zz_io_MCU_DATA_OUT_1;
        end
        default : begin
          io_MCU_DATA_OUT = 16'h0;
        end
      endcase
    end else begin
      if(clockArea_idePrimaryRegsCS) begin
        io_MCU_DATA_OUT = clockArea_primary_mcuDataOutput;
      end else begin
        io_MCU_DATA_OUT = clockArea_secondary_mcuDataOutput;
      end
    end
  end

  always @(*) begin
    io_MCU_DATA_OUT_EN = 2'b00;
    if(clockArea_globalRegsCS) begin
      case(io_MCU_ADDR)
        8'h0, 8'h01, 8'h02, 8'h03 : begin
        end
        8'h04 : begin
        end
        default : begin
          io_MCU_DATA_OUT_EN = 2'b00;
        end
      endcase
    end else begin
      if(clockArea_idePrimaryRegsCS) begin
        io_MCU_DATA_OUT_EN = clockArea_primary_mcuDataOutputEn;
      end else begin
        io_MCU_DATA_OUT_EN = clockArea_secondary_mcuDataOutputEn;
      end
    end
  end

  assign _zz_io_MCU_DATA_OUT = 64'h0000000063407782;
  assign when_OpenGDEMU_l71 = io_MCU_WR[0];
  assign when_OpenGDEMU_l72 = io_MCU_WR[1];
  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      clockArea_primary_dataOutput <= 16'hffff;
      clockArea_primary_dataOutputEn <= 1'b0;
      clockArea_primary_mcuDataOutput <= 16'hffff;
      clockArea_primary_mcuIRQ <= 1'b0;
      clockArea_primary_DEVICE <= 8'h0;
      clockArea_primary_HOB <= 1'b0;
      clockArea_primary_SRST <= 1'b0;
      clockArea_primary_nIEN <= 1'b0;
      clockArea_primary_BSY <= 1'b1;
      clockArea_primary_DRDY <= 1'b0;
      clockArea_primary_DF <= 1'b0;
      clockArea_primary_DRQ <= 1'b0;
      clockArea_primary_ERR <= 1'b0;
      clockArea_primary_COMMAND <= 8'h0;
      clockArea_primary_COMMAND_PEND <= 1'b0;
      clockArea_primary_LBALow <= 8'h0;
      clockArea_primary_LBALowPrev <= 8'h0;
      clockArea_primary_LBAMid <= 8'h0;
      clockArea_primary_LBAMidPrev <= 8'h0;
      clockArea_primary_LBAHigh <= 8'h0;
      clockArea_primary_LBAHighPrev <= 8'h0;
      clockArea_primary_ABRT <= 1'b0;
      clockArea_primary_Features <= 8'h0;
      clockArea_primary_SectorCount <= 16'h0;
      clockArea_primary_testReg <= 16'hcafe;
      clockArea_secondary_dataOutput <= 16'hffff;
      clockArea_secondary_dataOutputEn <= 1'b0;
      clockArea_secondary_mcuDataOutput <= 16'hffff;
      clockArea_secondary_mcuIRQ <= 1'b0;
      clockArea_secondary_DEVICE <= 8'h0;
      clockArea_secondary_HOB <= 1'b0;
      clockArea_secondary_SRST <= 1'b0;
      clockArea_secondary_nIEN <= 1'b0;
      clockArea_secondary_BSY <= 1'b1;
      clockArea_secondary_DRDY <= 1'b0;
      clockArea_secondary_DF <= 1'b0;
      clockArea_secondary_DRQ <= 1'b0;
      clockArea_secondary_ERR <= 1'b0;
      clockArea_secondary_COMMAND <= 8'h0;
      clockArea_secondary_COMMAND_PEND <= 1'b0;
      clockArea_secondary_LBALow <= 8'h0;
      clockArea_secondary_LBALowPrev <= 8'h0;
      clockArea_secondary_LBAMid <= 8'h0;
      clockArea_secondary_LBAMidPrev <= 8'h0;
      clockArea_secondary_LBAHigh <= 8'h0;
      clockArea_secondary_LBAHighPrev <= 8'h0;
      clockArea_secondary_ABRT <= 1'b0;
      clockArea_secondary_Features <= 8'h0;
      clockArea_secondary_SectorCount <= 16'h0;
      clockArea_secondary_testReg <= 16'hcafe;
    end else begin
      if(when_IDEDeviceRegs_l59) begin
        clockArea_primary_LBALowPrev <= clockArea_primary_LBALow;
      end
      if(when_IDEDeviceRegs_l61) begin
        clockArea_primary_LBAMidPrev <= clockArea_primary_LBAMid;
      end
      if(when_IDEDeviceRegs_l63) begin
        clockArea_primary_LBAHighPrev <= clockArea_primary_LBAHigh;
      end
      if(when_IDEDeviceRegs_l106) begin
        clockArea_primary_BSY <= 1'b1;
        clockArea_primary_DRDY <= 1'b0;
        clockArea_primary_DRQ <= 1'b0;
        clockArea_primary_COMMAND <= 8'h0;
        clockArea_primary_LBALow <= 8'h0;
        clockArea_primary_LBALowPrev <= 8'h0;
        clockArea_primary_LBAMid <= 8'h0;
        clockArea_primary_LBAMidPrev <= 8'h0;
        clockArea_primary_LBAHigh <= 8'h0;
        clockArea_primary_LBAHighPrev <= 8'h0;
        clockArea_primary_COMMAND_PEND <= 1'b0;
      end else begin
        if(when_IDEDeviceRegs_l123) begin
          clockArea_primary_BSY <= 1'b0;
          clockArea_primary_DRDY <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l129) begin
        clockArea_primary_dataOutputEn <= 1'b0;
      end
      if(when_IDEDeviceRegs_l151) begin
        clockArea_primary_DEVICE <= io_IDE_DATA_IN[7 : 0];
      end
      if(when_IDEDeviceRegs_l159) begin
        clockArea_primary_dataOutput <= 16'hdada;
      end
      if(when_IDEDeviceRegs_l170) begin
        clockArea_primary_dataOutput <= clockArea_primary_readFIFOStream_io_pop_payload;
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l185) begin
        clockArea_primary_dataOutput[2] <= clockArea_primary_ABRT;
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l193) begin
        if(clockArea_primary_WR) begin
          clockArea_primary_Features <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l200) begin
        if(clockArea_primary_HOB) begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_SectorCount[15 : 8];
        end else begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_SectorCount[7 : 0];
        end
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
        if(clockArea_primary_WR) begin
          clockArea_primary_SectorCount[15 : 8] <= clockArea_primary_SectorCount[7 : 0];
          clockArea_primary_SectorCount[7 : 0] <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l219) begin
        if(clockArea_primary_HOB) begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_LBALowPrev;
        end else begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_LBALow;
        end
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
        if(clockArea_primary_WR) begin
          clockArea_primary_LBALow <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l237) begin
        if(clockArea_primary_HOB) begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_LBAMidPrev;
        end else begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_LBAMid;
        end
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
        if(clockArea_primary_WR) begin
          clockArea_primary_LBAMid <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l253) begin
        if(clockArea_primary_HOB) begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_LBAHighPrev;
        end else begin
          clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_LBAHigh;
        end
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
        if(clockArea_primary_WR) begin
          clockArea_primary_LBAHigh <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l269) begin
        clockArea_primary_dataOutput[7 : 0] <= clockArea_primary_DEVICE;
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l284) begin
        clockArea_primary_dataOutput[7] <= clockArea_primary_BSY;
        clockArea_primary_dataOutput[6] <= clockArea_primary_DRDY;
        clockArea_primary_dataOutput[5] <= clockArea_primary_DF;
        clockArea_primary_dataOutput[3] <= clockArea_primary_DRQ;
        clockArea_primary_dataOutput[0] <= clockArea_primary_ERR;
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l296) begin
        if(clockArea_primary_WR) begin
          clockArea_primary_COMMAND <= io_IDE_DATA_IN[7 : 0];
          clockArea_primary_BSY <= 1'b1;
          clockArea_primary_COMMAND_PEND <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l320) begin
        clockArea_primary_dataOutput[7] <= clockArea_primary_BSY;
        clockArea_primary_dataOutput[6] <= clockArea_primary_DRDY;
        clockArea_primary_dataOutput[5] <= clockArea_primary_DF;
        clockArea_primary_dataOutput[3] <= clockArea_primary_DRQ;
        clockArea_primary_dataOutput[0] <= clockArea_primary_ERR;
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l331) begin
        if(clockArea_primary_WR) begin
          clockArea_primary_HOB <= io_IDE_DATA_IN[7];
          clockArea_primary_SRST <= io_IDE_DATA_IN[2];
          clockArea_primary_nIEN <= io_IDE_DATA_IN[1];
        end
      end
      if(when_IDEDeviceRegs_l340) begin
        clockArea_primary_dataOutput[15 : 0] <= clockArea_primary_testReg;
        if(clockArea_primary_RD) begin
          clockArea_primary_dataOutputEn <= 1'b1;
        end
        if(clockArea_primary_WR) begin
          clockArea_primary_testReg <= clockArea_primary_dataOutput[15 : 0];
        end
      end
      clockArea_primary_mcuIRQ <= clockArea_primary_irqDelay_3;
      if(when_IDEDeviceRegs_l362) begin
        clockArea_primary_mcuDataOutput[0] <= 1'b0;
        clockArea_primary_mcuDataOutput[1] <= clockArea_primary_COMMAND_PEND;
        if(when_IDEDeviceRegs_l371) begin
          clockArea_primary_COMMAND_PEND <= io_MCU_DATA_IN[1];
        end
      end
      if(when_IDEDeviceRegs_l389) begin
        clockArea_primary_mcuDataOutput[0] <= clockArea_primary_ERR;
        clockArea_primary_mcuDataOutput[3] <= clockArea_primary_DRQ;
        clockArea_primary_mcuDataOutput[5] <= clockArea_primary_DF;
        clockArea_primary_mcuDataOutput[6] <= clockArea_primary_DRDY;
        clockArea_primary_mcuDataOutput[7] <= clockArea_primary_BSY;
        if(when_IDEDeviceRegs_l396) begin
          clockArea_primary_ERR <= io_MCU_DATA_IN[0];
          clockArea_primary_DRQ <= io_MCU_DATA_IN[3];
          clockArea_primary_DF <= io_MCU_DATA_IN[5];
          clockArea_primary_DRDY <= io_MCU_DATA_IN[6];
          clockArea_primary_BSY <= io_MCU_DATA_IN[7];
        end
      end
      if(when_IDEDeviceRegs_l405) begin
        clockArea_primary_mcuDataOutput[7 : 0] <= clockArea_primary_DEVICE;
        if(when_IDEDeviceRegs_l407) begin
          clockArea_primary_DEVICE <= io_MCU_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l412) begin
        clockArea_primary_mcuDataOutput[3] <= clockArea_primary_ABRT;
        if(when_IDEDeviceRegs_l414) begin
          clockArea_primary_ABRT <= io_MCU_DATA_IN[3];
        end
      end
      if(when_IDEDeviceRegs_l419) begin
        clockArea_primary_mcuDataOutput[7 : 0] <= clockArea_primary_COMMAND;
        clockArea_primary_mcuDataOutput[15 : 8] <= clockArea_primary_Features;
        if(when_IDEDeviceRegs_l422) begin
          clockArea_primary_COMMAND <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l425) begin
          clockArea_primary_Features <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l430) begin
        clockArea_primary_mcuDataOutput <= clockArea_primary_SectorCount;
        if(when_IDEDeviceRegs_l433) begin
          clockArea_primary_SectorCount[7 : 0] <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l436) begin
          clockArea_primary_SectorCount[15 : 8] <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l441) begin
        clockArea_primary_mcuDataOutput[7 : 0] <= clockArea_primary_LBALow;
        clockArea_primary_mcuDataOutput[15 : 8] <= clockArea_primary_LBAMid;
        if(when_IDEDeviceRegs_l444) begin
          clockArea_primary_LBALow <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l447) begin
          clockArea_primary_LBAMid <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l452) begin
        clockArea_primary_mcuDataOutput[7 : 0] <= clockArea_primary_LBAHigh;
        clockArea_primary_mcuDataOutput[15 : 8] <= clockArea_primary_LBALowPrev;
        if(when_IDEDeviceRegs_l456) begin
          clockArea_primary_LBAHigh <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l459) begin
          clockArea_primary_LBALowPrev <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l464) begin
        clockArea_primary_mcuDataOutput[7 : 0] <= clockArea_primary_LBAMidPrev;
        clockArea_primary_mcuDataOutput[15 : 8] <= clockArea_primary_LBAHighPrev;
        if(when_IDEDeviceRegs_l468) begin
          clockArea_primary_LBAMidPrev <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l471) begin
          clockArea_primary_LBAHighPrev <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l476) begin
        clockArea_primary_mcuDataOutput <= clockArea_primary_writeFIFOStream_io_pop_payload;
      end
      if(when_IDEDeviceRegs_l485) begin
        clockArea_primary_mcuDataOutput[10 : 0] <= clockArea_primary_readFIFOStream_io_occupancy;
      end
      if(when_IDEDeviceRegs_l489) begin
        clockArea_primary_mcuDataOutput[10 : 0] <= clockArea_primary_writeFIFOStream_io_occupancy;
      end
      if(when_IDEDeviceRegs_l493) begin
        clockArea_primary_mcuDataOutput <= 16'hc0de;
      end
      if(when_IDEDeviceRegs_l59_1) begin
        clockArea_secondary_LBALowPrev <= clockArea_secondary_LBALow;
      end
      if(when_IDEDeviceRegs_l61_1) begin
        clockArea_secondary_LBAMidPrev <= clockArea_secondary_LBAMid;
      end
      if(when_IDEDeviceRegs_l63_1) begin
        clockArea_secondary_LBAHighPrev <= clockArea_secondary_LBAHigh;
      end
      if(when_IDEDeviceRegs_l106_1) begin
        clockArea_secondary_BSY <= 1'b1;
        clockArea_secondary_DRDY <= 1'b0;
        clockArea_secondary_DRQ <= 1'b0;
        clockArea_secondary_COMMAND <= 8'h0;
        clockArea_secondary_LBALow <= 8'h0;
        clockArea_secondary_LBALowPrev <= 8'h0;
        clockArea_secondary_LBAMid <= 8'h0;
        clockArea_secondary_LBAMidPrev <= 8'h0;
        clockArea_secondary_LBAHigh <= 8'h0;
        clockArea_secondary_LBAHighPrev <= 8'h0;
        clockArea_secondary_COMMAND_PEND <= 1'b0;
      end else begin
        if(when_IDEDeviceRegs_l123_1) begin
          clockArea_secondary_BSY <= 1'b0;
          clockArea_secondary_DRDY <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l129_1) begin
        clockArea_secondary_dataOutputEn <= 1'b0;
      end
      if(when_IDEDeviceRegs_l151_1) begin
        clockArea_secondary_DEVICE <= io_IDE_DATA_IN[7 : 0];
      end
      if(when_IDEDeviceRegs_l159_1) begin
        clockArea_secondary_dataOutput <= 16'hdada;
      end
      if(when_IDEDeviceRegs_l170_1) begin
        clockArea_secondary_dataOutput <= clockArea_secondary_readFIFOStream_io_pop_payload;
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l185_1) begin
        clockArea_secondary_dataOutput[2] <= clockArea_secondary_ABRT;
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l193_1) begin
        if(clockArea_secondary_WR) begin
          clockArea_secondary_Features <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l200_1) begin
        if(clockArea_secondary_HOB) begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_SectorCount[15 : 8];
        end else begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_SectorCount[7 : 0];
        end
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
        if(clockArea_secondary_WR) begin
          clockArea_secondary_SectorCount[15 : 8] <= clockArea_secondary_SectorCount[7 : 0];
          clockArea_secondary_SectorCount[7 : 0] <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l219_1) begin
        if(clockArea_secondary_HOB) begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_LBALowPrev;
        end else begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_LBALow;
        end
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
        if(clockArea_secondary_WR) begin
          clockArea_secondary_LBALow <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l237_1) begin
        if(clockArea_secondary_HOB) begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_LBAMidPrev;
        end else begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_LBAMid;
        end
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
        if(clockArea_secondary_WR) begin
          clockArea_secondary_LBAMid <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l253_1) begin
        if(clockArea_secondary_HOB) begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_LBAHighPrev;
        end else begin
          clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_LBAHigh;
        end
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
        if(clockArea_secondary_WR) begin
          clockArea_secondary_LBAHigh <= io_IDE_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l269_1) begin
        clockArea_secondary_dataOutput[7 : 0] <= clockArea_secondary_DEVICE;
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l284_1) begin
        clockArea_secondary_dataOutput[7] <= clockArea_secondary_BSY;
        clockArea_secondary_dataOutput[6] <= clockArea_secondary_DRDY;
        clockArea_secondary_dataOutput[5] <= clockArea_secondary_DF;
        clockArea_secondary_dataOutput[3] <= clockArea_secondary_DRQ;
        clockArea_secondary_dataOutput[0] <= clockArea_secondary_ERR;
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l296_1) begin
        if(clockArea_secondary_WR) begin
          clockArea_secondary_COMMAND <= io_IDE_DATA_IN[7 : 0];
          clockArea_secondary_BSY <= 1'b1;
          clockArea_secondary_COMMAND_PEND <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l320_1) begin
        clockArea_secondary_dataOutput[7] <= clockArea_secondary_BSY;
        clockArea_secondary_dataOutput[6] <= clockArea_secondary_DRDY;
        clockArea_secondary_dataOutput[5] <= clockArea_secondary_DF;
        clockArea_secondary_dataOutput[3] <= clockArea_secondary_DRQ;
        clockArea_secondary_dataOutput[0] <= clockArea_secondary_ERR;
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
      end
      if(when_IDEDeviceRegs_l331_1) begin
        if(clockArea_secondary_WR) begin
          clockArea_secondary_HOB <= io_IDE_DATA_IN[7];
          clockArea_secondary_SRST <= io_IDE_DATA_IN[2];
          clockArea_secondary_nIEN <= io_IDE_DATA_IN[1];
        end
      end
      if(when_IDEDeviceRegs_l340_1) begin
        clockArea_secondary_dataOutput[15 : 0] <= clockArea_secondary_testReg;
        if(clockArea_secondary_RD) begin
          clockArea_secondary_dataOutputEn <= 1'b1;
        end
        if(clockArea_secondary_WR) begin
          clockArea_secondary_testReg <= clockArea_secondary_dataOutput[15 : 0];
        end
      end
      clockArea_secondary_mcuIRQ <= clockArea_secondary_irqDelay_3;
      if(when_IDEDeviceRegs_l362_1) begin
        clockArea_secondary_mcuDataOutput[0] <= 1'b1;
        clockArea_secondary_mcuDataOutput[1] <= clockArea_secondary_COMMAND_PEND;
        if(when_IDEDeviceRegs_l371_1) begin
          clockArea_secondary_COMMAND_PEND <= io_MCU_DATA_IN[1];
        end
      end
      if(when_IDEDeviceRegs_l389_1) begin
        clockArea_secondary_mcuDataOutput[0] <= clockArea_secondary_ERR;
        clockArea_secondary_mcuDataOutput[3] <= clockArea_secondary_DRQ;
        clockArea_secondary_mcuDataOutput[5] <= clockArea_secondary_DF;
        clockArea_secondary_mcuDataOutput[6] <= clockArea_secondary_DRDY;
        clockArea_secondary_mcuDataOutput[7] <= clockArea_secondary_BSY;
        if(when_IDEDeviceRegs_l396_1) begin
          clockArea_secondary_ERR <= io_MCU_DATA_IN[0];
          clockArea_secondary_DRQ <= io_MCU_DATA_IN[3];
          clockArea_secondary_DF <= io_MCU_DATA_IN[5];
          clockArea_secondary_DRDY <= io_MCU_DATA_IN[6];
          clockArea_secondary_BSY <= io_MCU_DATA_IN[7];
        end
      end
      if(when_IDEDeviceRegs_l405_1) begin
        clockArea_secondary_mcuDataOutput[7 : 0] <= clockArea_secondary_DEVICE;
        if(when_IDEDeviceRegs_l407_1) begin
          clockArea_secondary_DEVICE <= io_MCU_DATA_IN[7 : 0];
        end
      end
      if(when_IDEDeviceRegs_l412_1) begin
        clockArea_secondary_mcuDataOutput[3] <= clockArea_secondary_ABRT;
        if(when_IDEDeviceRegs_l414_1) begin
          clockArea_secondary_ABRT <= io_MCU_DATA_IN[3];
        end
      end
      if(when_IDEDeviceRegs_l419_1) begin
        clockArea_secondary_mcuDataOutput[7 : 0] <= clockArea_secondary_COMMAND;
        clockArea_secondary_mcuDataOutput[15 : 8] <= clockArea_secondary_Features;
        if(when_IDEDeviceRegs_l422_1) begin
          clockArea_secondary_COMMAND <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l425_1) begin
          clockArea_secondary_Features <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l430_1) begin
        clockArea_secondary_mcuDataOutput <= clockArea_secondary_SectorCount;
        if(when_IDEDeviceRegs_l433_1) begin
          clockArea_secondary_SectorCount[7 : 0] <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l436_1) begin
          clockArea_secondary_SectorCount[15 : 8] <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l441_1) begin
        clockArea_secondary_mcuDataOutput[7 : 0] <= clockArea_secondary_LBALow;
        clockArea_secondary_mcuDataOutput[15 : 8] <= clockArea_secondary_LBAMid;
        if(when_IDEDeviceRegs_l444_1) begin
          clockArea_secondary_LBALow <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l447_1) begin
          clockArea_secondary_LBAMid <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l452_1) begin
        clockArea_secondary_mcuDataOutput[7 : 0] <= clockArea_secondary_LBAHigh;
        clockArea_secondary_mcuDataOutput[15 : 8] <= clockArea_secondary_LBALowPrev;
        if(when_IDEDeviceRegs_l456_1) begin
          clockArea_secondary_LBAHigh <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l459_1) begin
          clockArea_secondary_LBALowPrev <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l464_1) begin
        clockArea_secondary_mcuDataOutput[7 : 0] <= clockArea_secondary_LBAMidPrev;
        clockArea_secondary_mcuDataOutput[15 : 8] <= clockArea_secondary_LBAHighPrev;
        if(when_IDEDeviceRegs_l468_1) begin
          clockArea_secondary_LBAMidPrev <= io_MCU_DATA_IN[7 : 0];
        end
        if(when_IDEDeviceRegs_l471_1) begin
          clockArea_secondary_LBAHighPrev <= io_MCU_DATA_IN[15 : 8];
        end
      end
      if(when_IDEDeviceRegs_l476_1) begin
        clockArea_secondary_mcuDataOutput <= clockArea_secondary_writeFIFOStream_io_pop_payload;
      end
      if(when_IDEDeviceRegs_l485_1) begin
        clockArea_secondary_mcuDataOutput[10 : 0] <= clockArea_secondary_readFIFOStream_io_occupancy;
      end
      if(when_IDEDeviceRegs_l489_1) begin
        clockArea_secondary_mcuDataOutput[10 : 0] <= clockArea_secondary_writeFIFOStream_io_occupancy;
      end
      if(when_IDEDeviceRegs_l493_1) begin
        clockArea_secondary_mcuDataOutput <= 16'hc0de;
      end
    end
  end

  always @(posedge io_MCU_CLK) begin
    clockArea_primary_WR_regNext <= clockArea_primary_WR;
    clockArea_primary_SRST_regNext <= clockArea_primary_SRST;
    clockArea_primary_irqDelay_1 <= clockArea_primary_irqDelay_0;
    clockArea_primary_irqDelay_2 <= clockArea_primary_irqDelay_1;
    clockArea_primary_irqDelay_3 <= clockArea_primary_irqDelay_2;
    clockArea_secondary_WR_regNext <= clockArea_secondary_WR;
    clockArea_secondary_SRST_regNext <= clockArea_secondary_SRST;
    clockArea_secondary_irqDelay_1 <= clockArea_secondary_irqDelay_0;
    clockArea_secondary_irqDelay_2 <= clockArea_secondary_irqDelay_1;
    clockArea_secondary_irqDelay_3 <= clockArea_secondary_irqDelay_2;
  end

  always @(posedge io_MCU_CLK) begin
    clockArea_primary_RD_regNext <= clockArea_primary_RD;
  end

  always @(posedge io_MCU_CLK) begin
    clockArea_primary_WR_regNext_1 <= clockArea_primary_WR;
  end

  always @(posedge io_MCU_CLK) begin
    clockArea_secondary_RD_regNext <= clockArea_secondary_RD;
  end

  always @(posedge io_MCU_CLK) begin
    clockArea_secondary_WR_regNext_1 <= clockArea_secondary_WR;
  end

  always @(posedge io_MCU_CLK or posedge io_MCU_RST) begin
    if(io_MCU_RST) begin
      _zz_io_MCU_DATA_OUT_1 <= 16'h5aa5;
    end else begin
      if(when_OpenGDEMU_l71) begin
        _zz_io_MCU_DATA_OUT_1[7 : 0] <= io_MCU_DATA_IN[7 : 0];
      end
      if(when_OpenGDEMU_l72) begin
        _zz_io_MCU_DATA_OUT_1[15 : 8] <= io_MCU_DATA_IN[15 : 8];
      end
    end
  end


endmodule

//MCURegisterManagerGenerator replaced by MCURegisterManagerGenerator

//IDEAddressDecode replaced by IDEAddressDecode

//StreamFifo replaced by StreamFifo

//StreamFifo replaced by StreamFifo

module MCURegisterManagerGenerator (
);



endmodule

module IDEAddressDecode (
  input      [1:0]    io_CSn,
  input      [2:0]    io_ADDR,
  output              io_DataRegister,
  output              io_ErrorRegister,
  output              io_FeaturesRegister,
  output              io_SectorCountRegister,
  output              io_LBALowRegister,
  output              io_LBAMidRegister,
  output              io_LBAHighRegister,
  output              io_DeviceRegister,
  output              io_CommandRegister,
  output              io_StatusRegister,
  output              io_DeviceControlRegister,
  output              io_AlternateStatusRegister,
  output              io_DebugTestpad
);

  wire                regBlock0;
  wire                regBlock1;
  wire                _zz_io_ErrorRegister;
  wire                _zz_io_DeviceRegister;
  wire                _zz_io_DeviceControlRegister;
  wire                _zz_io_CommandRegister;
  wire                _zz_io_CommandRegister_1;

  assign regBlock0 = (io_CSn == 2'b10);
  assign regBlock1 = (io_CSn == 2'b01);
  assign _zz_io_ErrorRegister = ((io_ADDR == 3'b001) && regBlock0);
  assign _zz_io_DeviceRegister = (io_ADDR == 3'b110);
  assign _zz_io_DeviceControlRegister = (_zz_io_DeviceRegister && regBlock1);
  assign _zz_io_CommandRegister = (io_ADDR == 3'b111);
  assign _zz_io_CommandRegister_1 = (_zz_io_CommandRegister && regBlock0);
  assign io_DataRegister = ((io_ADDR == 3'b000) && regBlock0);
  assign io_ErrorRegister = _zz_io_ErrorRegister;
  assign io_FeaturesRegister = _zz_io_ErrorRegister;
  assign io_SectorCountRegister = ((io_ADDR == 3'b010) && regBlock0);
  assign io_LBALowRegister = ((io_ADDR == 3'b011) && regBlock0);
  assign io_LBAMidRegister = ((io_ADDR == 3'b100) && regBlock0);
  assign io_LBAHighRegister = ((io_ADDR == 3'b101) && regBlock0);
  assign io_DeviceRegister = (_zz_io_DeviceRegister && regBlock0);
  assign io_CommandRegister = _zz_io_CommandRegister_1;
  assign io_StatusRegister = _zz_io_CommandRegister_1;
  assign io_DeviceControlRegister = _zz_io_DeviceControlRegister;
  assign io_AlternateStatusRegister = _zz_io_DeviceControlRegister;
  assign io_DebugTestpad = (_zz_io_CommandRegister && regBlock1);

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
  wire                when_Stream_l1075;
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
  assign when_Stream_l1075 = (logic_pushing != logic_popping);
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
      if(when_Stream_l1075) begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush) begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end


endmodule
