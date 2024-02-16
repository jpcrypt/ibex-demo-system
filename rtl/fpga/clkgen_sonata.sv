// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module clkgen_sonata  #(
  // System Clock Frequency is parameterised, allowing it to be adjusted.
  parameter int unsigned SysClkFreq = 50_000_000,

  // Frequency of IO_CLK input on the FPGA board.
  parameter int unsigned IOClkFreq = 25_000_000
) (
    // Board clock signal
    input IO_CLK,
    output IO_CLK_BUF,
    input IO_RST_N,

    // System clock (CPU, RAM, system fabric) and reset
    output clk_sys,
    output rst_sys_n,

    // Peripheral clock and reset
    output clk_peri,
    output rst_peri_n,

    // USBDEV clock and reset
    output clk_usb,
    output rst_usb_n,

    // HyperRAM clock and reset
    output clk_hr,
    output clk_hr90p,
    output clk_hr3x,
    output reg rst_hr,

    // debug only:
    output hb,
    output hb90p,
    output hb3x,
    output locked
);

  // Required frequency of clk_usb output
  // - usbdev employs x4 oversampling for a 12Mbps Full Speed connection
  localparam int USBClkFreq = 48_000_000;

  // Required frequencey of clk_peri output
  localparam int PeriClkFreq = 24_000_000;

  // Required frequencey of clk_hr output
  localparam int HrClkFreq = 100_000_000;

  // Required frequencey of clk_hr3x output
  localparam int Hr3xClkFreq = 300_000_000;

  logic locked_pll;
  logic io_clk_buf;
  logic clk_peri_buf;
  logic clk_peri_unbuf;
  logic clk_usb_buf;
  logic clk_usb_unbuf;
  logic clk_sys_buf;
  logic clk_sys_unbuf;
  logic clk_fb_buf;
  logic clk_fb_unbuf;
  logic clk_hr_unbuf;
  logic clk_hr90p_unbuf;
  logic clk_hr3x_unbuf;


  // input buffer
  IBUF io_clk_ibuf(
    .I (IO_CLK),
    .O (io_clk_buf)
  );

  PLLE2_ADV #(
    .BANDWIDTH            ("OPTIMIZED"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT        (48),
    .CLKFBOUT_PHASE       (0.000),

    // clk_sys output
    .CLKOUT0_DIVIDE       ((48 * IOClkFreq) / SysClkFreq),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),

    // clk_usb output
    .CLKOUT1_DIVIDE       ((48 * IOClkFreq) / USBClkFreq),
    .CLKOUT1_PHASE        (0.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),

    // clk_peri_output
    .CLKOUT2_DIVIDE       ((48 * IOClkFreq) / PeriClkFreq),
    .CLKOUT2_PHASE        (0.000),
    .CLKOUT2_DUTY_CYCLE   (0.500),

    // clk_hr output
    .CLKOUT3_DIVIDE       ((48 * IOClkFreq) / HrClkFreq),
    .CLKOUT3_PHASE        (0.000),
    .CLKOUT3_DUTY_CYCLE   (0.500),

    // clk_hr90p output
    .CLKOUT4_DIVIDE       ((48 * IOClkFreq) / HrClkFreq),
    .CLKOUT4_PHASE        (90.000),
    .CLKOUT4_DUTY_CYCLE   (0.500),

    // clk_hr3x output
    .CLKOUT5_DIVIDE       ((48 * IOClkFreq) / Hr3xClkFreq),
    .CLKOUT5_PHASE        (0.000),
    .CLKOUT5_DUTY_CYCLE   (0.500),

    .CLKIN1_PERIOD        (40.000)
  ) pll (
    .CLKFBOUT            (clk_fb_unbuf),
    .CLKOUT0             (clk_sys_unbuf),
    .CLKOUT1             (clk_usb_unbuf),
    .CLKOUT2             (clk_peri_unbuf),
    .CLKOUT3             (clk_hr_unbuf),
    .CLKOUT4             (clk_hr90p_unbuf),
    .CLKOUT5             (clk_hr3x_unbuf),
     // Input clock control
    .CLKFBIN             (clk_fb_buf),
    .CLKIN1              (io_clk_buf),
    .CLKIN2              (1'b0),
     // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (),
    .DRDY                (),
    .DWE                 (1'b0),
    // Other control and status signals
    .LOCKED              (locked_pll),
    .PWRDWN              (1'b0),
    // Do not reset PLL on external reset, otherwise ILA disconnects at a reset
    .RST                 (1'b0));

  // output buffering
  BUFG clk_fb_bufg (
    .I (clk_fb_unbuf),
    .O (clk_fb_buf)
  );

  BUFG clk_sys_bufg (
    .I (clk_sys_unbuf),
    .O (clk_sys_buf)
  );

  BUFG clk_usb_bufg (
    .I (clk_usb_unbuf),
    .O (clk_usb_buf)
  );

  BUFG clk_peri_bufg (
    .I (clk_peri_unbuf),
    .O (clk_peri_buf)
  );

  BUFG clk_hr_bufg (
    .I (clk_hr_unbuf),
    .O (clk_hr_buf)
  );

  BUFG clk_hr90p_bufg (
    .I (clk_hr90p_unbuf),
    .O (clk_hr90p_buf)
  );

  BUFG clk_hr3x_bufg (
    .I (clk_hr3x_unbuf),
    .O (clk_hr3x_buf)
  );


  // outputs
  // clocks
  assign IO_CLK_BUF = io_clk_buf;

  assign clk_sys  = clk_sys_buf;
  assign clk_usb  = clk_usb_buf;
  assign clk_peri = clk_peri_buf;
  assign clk_hr = clk_hr_buf;
  assign clk_hr90p = clk_hr90p_buf;
  assign clk_hr3x = clk_hr3x_buf;

  // resets
  assign rst_sys_n  = locked_pll & IO_RST_N;
  assign rst_usb_n  = rst_sys_n;
  assign rst_peri_n = rst_sys_n;

  // make HR reset synchronous:
  (* ASYNC_REG = "TRUE" *) reg[1:0] rst_hr_pipe;
  always @(posedge clk_hr) begin
      if (~rst_sys_n) begin
          rst_hr_pipe <= 0;
          rst_hr <= 0;
      end
      else
          {rst_hr, rst_hr_pipe} <= {rst_hr_pipe, ~rst_sys_n};
  end

  // clock heartbeats (debug/development only):
  reg [25:0] clk_hr_heartbeat;
  always @(posedge clk_hr) clk_hr_heartbeat <= clk_hr_heartbeat + 1;

  // programmable clock Heartbeat
  reg [25:0] clk_hr90p_heartbeat;
  always @(posedge clk_hr90p) clk_hr90p_heartbeat <= clk_hr90p_heartbeat + 1;

  // programmable clock Heartbeat
  reg [26:0] clk_hr3x_heartbeat;
  always @(posedge clk_hr3x) clk_hr3x_heartbeat <= clk_hr3x_heartbeat + 1;

  // gentle heartbeat LEDs:
  //assign hb    = clk_hr_heartbeat[25]    ? clk_hr_heartbeat[4]    : clk_hr_heartbeat < {8{1'b1}};
  //assign hb90p = clk_hr90p_heartbeat[25] ? clk_hr90p_heartbeat[4] : clk_hr90p_heartbeat[4] < {8{1'b1}};
  //assign hb3x  = clk_hr3x_heartbeat[25]  ? clk_hr3x_heartbeat[4]  : clk_hr3x_heartbeat[4] < {8{1'b1}};

  assign hb    = clk_hr_heartbeat[25]    ? clk_hr_heartbeat[4]    : 1'b1;
  assign hb90p = clk_hr90p_heartbeat[25] ? 1'b1: clk_hr90p_heartbeat[4];
  assign hb3x  = clk_hr3x_heartbeat[26]  ? clk_hr3x_heartbeat[4]  : 1'b1;

  assign locked = locked_pll;


endmodule
