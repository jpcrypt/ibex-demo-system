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

  // HR R/W test interface
  input wire I_auto_pass,
  input wire I_auto_fail, 
  input wire bresp_error, 
  input wire rresp_error,
  input wire [31:0] I_auto_errors,
  input wire [31:0] I_auto_error_addr,
  input wire [31:0] I_auto_iterations,
  input wire [31:0] I_auto_current_addr,
  input wire [31:0] hbmc_rdata,
  input wire  hbmc_idle,

  output reg O_lb_manual,
  output reg O_auto_lfsr_mode,
  output reg O_auto_clear_fail,
  output reg [31:0] O_auto_stop_addr,
  output reg [31:0] O_auto_start_addr,
  output wire  hbmc_write,
  output wire  hbmc_read,
  output reg [31:0] hbmc_wdata,


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

  reg [1:0] reg_hbmc_action;
  assign hbmc_write = reg_hbmc_action[0];
  assign hbmc_read  = reg_hbmc_action[1];


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
    .EnableDataIntgGen(0),
    .AccessLatency(1)
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

  `define REG0_ADDR 8'h00
  `define REG1_ADDR 8'h04
  `define REG2_ADDR 8'h08

  `define REG_LB_MANUAL         8'h10
  `define REG_HYPER_STATUS      8'h14
  `define REG_LB_ERRORS         8'h18
  `define REG_LB_ERROR_ADDR     8'h1c
  `define REG_LB_ITERATIONS     8'h20
  `define REG_LB_CURRENT_ADDR   8'h24
  `define REG_HBMC_SINGLE_DATA  8'h28
  `define REG_HBMC_ACTION       8'h2c
  `define REG_LB_STOP_ADDR      8'h30
  `define REG_LB_START_ADDR     8'h34

  // register reads:
  always @(posedge clk_i) begin
      if (reg_re) begin
          case (reg_addr)
              `REG0_ADDR: reg_rdata <= reg0;
              `REG1_ADDR: reg_rdata <= reg1;
              `REG2_ADDR: reg_rdata <= reg2;

              `REG_LB_MANUAL:             reg_rdata <= {31'b0, O_lb_manual};
              `REG_HYPER_STATUS:          reg_rdata <= {28'b0, I_auto_pass, I_auto_fail, bresp_error, rresp_error};
              `REG_LB_ERRORS:             reg_rdata <= I_auto_errors;
              `REG_LB_ERROR_ADDR:         reg_rdata <= I_auto_error_addr;
              `REG_LB_ITERATIONS:         reg_rdata <= I_auto_iterations;
              `REG_LB_CURRENT_ADDR:       reg_rdata <= I_auto_current_addr;
              `REG_HBMC_SINGLE_DATA:      reg_rdata <= hbmc_rdata;
              `REG_HBMC_ACTION:           reg_rdata <= hbmc_idle;

              default: reg_rdata <= 32'haaaaaaaa;
          endcase
      end
      else
          reg_rdata <= 32'hbbbbbbbb;
  end


  // register writes:
  always @(posedge clk_i) begin
      if (~rst_ni) begin
          reg0 <= 0;
          reg1 <= 1;
          reg2 <= 2;
          O_auto_lfsr_mode <= 0;
          O_auto_clear_fail <= 0;
          O_lb_manual <= 1;
          O_auto_stop_addr <= 32'h00004000;
          O_auto_start_addr <= 0;
          reg_hbmc_action <= 0;
          hbmc_wdata <= 0;
      end

      else if (reg_we) begin
          case (reg_addr)
              `REG0_ADDR: reg0 <= reg_wdata;
              `REG1_ADDR: reg1 <= reg_wdata;
              `REG2_ADDR: reg2 <= reg_wdata;

              `REG_LB_MANUAL:          {O_auto_lfsr_mode, O_auto_clear_fail, O_lb_manual} <= reg_wdata[2:0];
              `REG_LB_STOP_ADDR:       O_auto_stop_addr <= reg_wdata;
              `REG_LB_START_ADDR:      O_auto_start_addr <= reg_wdata;
              `REG_HBMC_ACTION:        reg_hbmc_action <= reg_wdata[1:0];
              `REG_HBMC_SINGLE_DATA:   hbmc_wdata <= reg_wdata;

          endcase
      end
  end


`ifdef ILA_REG
    ila_reg U_ila_reg (
            .clk            (clk_i),
            .probe0         (reg_we),
            .probe1         (reg_re),
            .probe2         (reg_addr),     // 31:0
            .probe3         (reg_wdata),    // 31:0
            .probe4         (reg_rdata),    // 31:0
            .probe5         (reg_busy),
            .probe6         (reg0),         // 31:0
            .probe7         (O_auto_start_addr), // 31:0
            .probe8         (O_auto_stop_addr) // 31:0
    );
`endif

endmodule
