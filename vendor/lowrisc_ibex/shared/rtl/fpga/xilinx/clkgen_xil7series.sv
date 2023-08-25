// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module clkgen_xil7series #(
  parameter SysClkFreq = 50_000_000
) (
    input IO_CLK,
    input IO_RST_N,
    output clk_sys,
    output rst_sys_n,
    output clk_usb,
    output rst_usb_n
);
  // Frequency of IO_CLK input
  localparam int IOClkFreq = 100_000_000;

  // Required frequency of clk_usb output
  // - usbdev employs x4 oversampling for a 12Mbps Full Speed connection
  localparam int USBClkFreq = 48_000_000;

  logic locked_pll;
  logic io_clk_buf;
  logic clk_usb_buf;
  logic clk_usb_unbuf;
  logic clk_sys_buf;
  logic clk_sys_unbuf;
  logic clk_fb_buf;
  logic clk_fb_unbuf;

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
    .CLKFBOUT_MULT        (12),
    .CLKFBOUT_PHASE       (0.000),
    // clk_sys output
    .CLKOUT0_DIVIDE       ((12 * IOClkFreq) / SysClkFreq),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    // clk_usb output
    .CLKOUT1_DIVIDE       ((12 * IOClkFreq) / USBClkFreq),
    .CLKOUT1_PHASE        (0.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKIN1_PERIOD        (10)
  ) pll (
    .CLKFBOUT            (clk_fb_unbuf),
    .CLKOUT0             (clk_sys_unbuf),
    .CLKOUT1             (clk_usb_unbuf),
    .CLKOUT2             (),
    .CLKOUT3             (),
    .CLKOUT4             (),
    .CLKOUT5             (),
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

  // outputs
  // clocks
  assign clk_sys = clk_sys_buf;
  assign clk_usb = clk_usb_buf;

  // resets
  assign rst_sys_n = locked_pll & IO_RST_N;
  assign rst_usb_n = rst_sys_n;

endmodule
