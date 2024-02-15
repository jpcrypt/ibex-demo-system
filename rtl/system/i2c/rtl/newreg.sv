// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Description: I2C top level wrapper file

`include "prim_assert.sv"

module newreg
  import i2c_reg_pkg::*;
#(
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}}
) (
  input                     clk_i,
  input                     rst_ni,

  // Bus Interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o
);

  localparam int AW = 7;
  localparam int DW = 32;
  localparam int DBW = DW/8;                    // Byte Width

  // register signals
  logic           reg_we;
  logic           reg_re;
  logic [AW-1:0]  reg_addr;
  logic [DW-1:0]  reg_wdata;
  //logic [DBW-1:0] reg_be;
  logic [DW-1:0]  reg_rdata;
  logic reg_busy;

  logic [31:0]    reg0;
  logic [31:0]    reg1;
  logic [31:0]    reg2;

  // outgoing integrity generation (copied from i2c_reg_top.sv) as per Adrian:
  tlul_pkg::tl_d2h_t tl_o_pre;
  tlul_rsp_intg_gen #(
    .EnableRspIntgGen(1),
    .EnableDataIntgGen(1)
  ) u_rsp_intg_gen (
    .tl_i(tl_o_pre),
    .tl_o(tl_o)
  );


  tlul_adapter_reg #(
    .RegAw(AW),
    .RegDw(DW),
    .EnableDataIntgGen(0)
  ) u_reg_if (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),

    .tl_i (tl_i),
    .tl_o (tl_o_pre),

    .en_ifetch_i(prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    .we_o    (reg_we),
    .re_o    (reg_re),
    .addr_o  (reg_addr),
    .wdata_o (reg_wdata),
    .be_o    (),
    .busy_i  (reg_busy),
    .rdata_i (reg_rdata),
    .error_i (1'b0)
  );


  assign reg_busy = 1'b0;

  // register reads:
  /*
  always @(posedge clk_i) begin
      if (reg_re) begin
          case (reg_addr)
              8'h00: reg_rdata <= reg0;
              8'h04: reg_rdata <= reg1;
              8'h08: reg_rdata <= reg2;
              default: reg_rdata <= 32'haaaaaaaa;
          endcase
      end
      else
          reg_rdata <= 32'hbbbbbbbb;
  end
  */

  always @(*) begin
      if (reg_re) begin
          case (reg_addr)
              8'h00: reg_rdata = reg0;
              8'h04: reg_rdata = reg1;
              8'h08: reg_rdata = reg2;
              default: reg_rdata = 32'haaaaaaaa;
          endcase
      end
      else
          reg_rdata = 32'hbbbbbbbb;
  end



  // register writes:
  always @(posedge clk_i) begin
      if (~rst_ni) begin
          reg0 <= 0;
          reg1 <= 1;
          reg2 <= 2;
      end
      else if (reg_we) begin
          case (reg_addr)
              8'h00: reg0 <= reg_wdata;
              8'h04: reg1 <= reg_wdata;
              8'h08: reg2 <= reg_wdata;
          endcase
      end
  end


`ifdef ILA
    ila_reg U_ila_reg (
            .clk            (clk_i),
            .probe0         (reg_we),
            .probe1         (reg_re),
            .probe2         (reg_addr),     // 31:0
            .probe3         (reg_wdata),    // 31:0
            .probe4         (reg_rdata),    // 31:0
            .probe5         (reg_busy),
            .probe6         (reg0),         // 31:0
            .probe7         (reg1),         // 31:0
            .probe8         (reg2)          // 31:0
    );
`endif

endmodule
