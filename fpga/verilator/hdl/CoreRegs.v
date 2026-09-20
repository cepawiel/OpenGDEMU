module CoreRegs (
  input               i_clk,

  input               i_reset,
  input               i_mcu_cs,
  input               i_mcu_rd,
  input               i_mcu_wr,
  input      [1:0]    i_mcu_bs,
  input      [8:0]    i_mcu_addr,
  input      [15:0]   i_mcu_data_in,
  output     [15:0]   o_mcu_data_out,
  output              o_mcu_irq

//   input               i_ide_rst,
//   input               i_ide_wr,
//   input               i_ide_rd,
//   input      [1:0]    i_ide_cs,
//   input      [2:0]    i_ide_addr,
//   input      [15:0]   i_ide_data_in,
//   output     [15:0]   o_ide_data_out,
//   output              o_ide_io_rdy,
//   output              o_ide_irq_rq,

//   input               i_ide_dma_ack,
//   output              o_ide_dma_req
);

    assign o_mcu_irq = i_mcu_rd;
    // assign o_ide_data_out = 16'h0000;
    // assign o_ide_io_rdy = 0;
    // assign o_ide_irq_rq = 0;
    // assign o_ide_dma_req = 0;

// ============================================================================
// Shared Registers
// ============================================================================
    // reg [7:0]   command_code_reg;
    // reg [7:0]   device_reg;
    // reg [7:0]   device_control_reg;
    // reg [7:0]   device_error_reg;
    // reg [7:0]   features_reg;
    // reg [7:0]   lba_high_reg;
    // reg [7:0]   lba_mid_reg;
    // reg [7:0]   lba_low_reg;
    // reg [7:0]   sector_count_reg;
    // reg [7:0]   status_reg;

// ============================================================================
// IDE Memory Map
// ============================================================================


// ============================================================================
// MCU Memory Map
// ============================================================================
    wire [1:0] mcu_wr = (i_mcu_cs & i_mcu_wr) ? i_mcu_bs : 2'b00;
    // wire [1:0] mcu_rd = (i_mcu_cs & i_mcu_rd) ? i_mcu_bs : 2'b00;

    wire mcu_address_00 = (i_mcu_addr == 9'h000);
    // wire mcu_address_02 = (i_mcu_addr == 9'h002);
    // wire mcu_address_04 = (i_mcu_addr == 9'h002);

    wire [15:0] mcu_data_out_00;

    assign o_mcu_data_out = (mcu_address_00) ? mcu_data_out_00 : 16'h0000;

    Register mcu_test_reg_l (
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_we(mcu_address_00 & mcu_wr[0]),
        .i_data_in(i_mcu_data_in[7:0]),
        .o_data_out(mcu_data_out_00[7:0])
    );
    Register mcu_test_reg_h (
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_we(mcu_address_00 & mcu_wr[1]),
        .i_data_in(i_mcu_data_in[15:8]),
        .o_data_out(mcu_data_out_00[15:8])
    );

    
    always @(posedge i_clk) begin
        // mcu_test_reg        <= mcu_test_reg_next;
        // mcu_data_out_reg    <= mcu_data_out_reg_next;
    end

    always @(*) begin
        // mcu_test_reg_next = mcu_test_reg;
        // mcu_data_out_reg_next = mcu_data_out_reg;

        if (i_reset) begin
            // mcu_data_out_reg_next = 16'hFFFF;
            // mcu_test_reg_next = 16'hC0DE;
        end
        
        // if (i_ide_cs) begin

        // end

        if (i_mcu_cs) begin
            if (i_mcu_wr) begin
                if (i_mcu_bs[0]) begin
                    // mcu_test_reg_next[7:0] = i_mcu_data_in[7:0];
                end
                if (i_mcu_bs[1]) begin
                    // mcu_test_reg_next[15:8] = i_mcu_data_in[15:8];
                end
            end


            // case (i_mcu_addr)
            //     8'h00:  mcu_test_reg <= i_mcu_data_in;
            //     default: mcu_test_reg <= 16'hDEAD;
            // endcase

            if (i_mcu_rd) begin
                // mcu_data_out_reg_next = mcu_test_reg;
            end
            
        end

    end


endmodule // OpenGDEMUSpinal
