// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level SystemVerilog file that connects the IO on the board to the Ibex Demo System.
module top_artya7 #(
  parameter SRAMInitFile = ""
) (
  // These inputs are defined in data/pins_artya7.xdc
  input         IO_CLK,
  input         IO_RST_N,
  input  [ 3:0] SW,
  input  [ 3:0] BTN,
  output [ 3:0] LED,
  output [11:0] RGB_LED,
  output [ 3:0] DISP_CTRL,
  input         UART_RX,
  output        UART_TX,
  input         SPI_RX,
  output        SPI_TX,
  output        SPI_SCK
);
  parameter int SysClkFreq = 50_000_000;

  logic clk_sys, rst_sys_n;

  // No support for USBDEV or I2C presently.
  wire clk_usb = 1'b0;
  wire rst_usb_n = rst_sys_n;
  wire clk_peri = 1'b0;
  wire rst_peri_n = rst_sys_n;

  wire scl0_i = 1'b1;
  wire sda0_i = 1'b1;
  wire scl1_i = 1'b1;
  wire sda1_i = 1'b1;

  // Instantiating the Ibex Demo System.
  ibex_demo_system #(
    .GpiWidth     ( 8            ),
    .GpoWidth     ( 8            ),
    .PwmWidth     ( 12           ),
    .SRAMInitFile ( SRAMInitFile )
  ) u_ibex_demo_system (
    //input
    .clk_sys_i (clk_sys),
    .rst_sys_ni(rst_sys_n),

    .clk_usb_i    (clk_usb),
    .rst_usb_ni   (rst_usb_n),

    .clk_peri_i   (clk_peri),
    .rst_peri_ni  (rst_peri_n),

    .gp_i({SW, BTN}),
    .uart_rx_i(UART_RX),

    //output
    .gp_o     ({LED, DISP_CTRL}),
    .pwm_o    (RGB_LED),
    .uart_tx_o(UART_TX),

    .spi_rx_i (SPI_RX),
    .spi_tx_o (SPI_TX),
    .spi_sck_o(SPI_SCK),

    // I2C bus 0
    .i2c0_scl_i       (scl0_i),
    .i2c0_scl_o       (),
    .i2c0_scl_en_o    (),
    .i2c0_sda_i       (sda0_i),
    .i2c0_sda_o       (),
    .i2c0_sda_en_o    (),

    // I2C bus 1
    .i2c1_scl_i       (scl1_i),
    .i2c1_scl_o       (),
    .i2c1_scl_en_o    (),
    .i2c1_sda_i       (sda1_i),
    .i2c1_sda_o       (),
    .i2c1_sda_en_o    (),

    .trst_ni(1'b1),
    .tms_i  (1'b0),
    .tck_i  (1'b0),
    .td_i   (1'b0),
    .td_o   ()
  );

  // Generating the system clock and reset for the FPGA.
  clkgen_xil7series clkgen(
    .IO_CLK,
    .IO_RST_N,
    .clk_sys,
    .rst_sys_n
  );

endmodule
