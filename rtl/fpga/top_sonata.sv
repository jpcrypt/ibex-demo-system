// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Drive I2C1 rather than I2C0?
// `define DRIVE_I2C1

`ifdef DRIVE_I2C1
// When driving I2C1, the I2C traffic may be indirected to one of the following.
// Define none or one of these...
//`define DRIVE_RPIHAT
//`define DRIVE_RPIID
//`define DRIVE_mkBUS
`endif

// Ibex demo system top level for the Sonata board
module top_sonata (
  input              main_clk,
  input              nrst_btn,

  output logic [7:0] led_user,
  output logic       led_bootok,
  output logic       led_halted,
  output logic       led_cheri,
  output logic       led_legacy,
  output logic [8:0] led_cherierr,

  input  logic [4:0] nav_sw,
  input  logic [7:0] user_sw,

  output logic lcd_rst,
  output logic lcd_dc,
  output logic lcd_copi,
  output logic lcd_clk,
  output logic lcd_cs,

  output logic ser0_tx,
  input  logic ser0_rx,

  // I2C buses
`ifdef DRIVE_RPIHAT
  output logic       SCL0,
  output logic       SDA0,
  output logic       SCL1,
  output logic       SDA1,
`else
  inout  logic       SCL0,  // Shared between Arduino and QWIIC (J1)
  inout  logic       SDA0,

  inout  logic       SCL1,  // QWIIC only (J7)
  inout  logic       SDA1,
`endif

  // mikroBUS Click
  inout  logic       MB5,
  inout  logic       MB6,

  // RaspberryPi HAT
  inout  logic       RPH_G2_SDA,
  inout  logic       RPH_G3_SCL,

  inout  logic       RPH_G1,      // ID_SC - I2C for HAT ID EEPROM
  inout  logic       RPH_G0,      // ID_SD
  input  logic tck_i,
  input  logic tms_i,
  input  logic td_i,
  output logic td_o
);
  parameter int SysClkFreq = 50_000_000;
  parameter SRAMInitFile = "";

  logic top_rst_n;
  logic main_clk_buf;
  logic clk_sys,  rst_sys_n;
  logic clk_peri, rst_peri_n;
  logic clk_usb,  rst_usb_n;
  logic [7:0] reset_counter;

  logic [4:0] nav_sw_n;
  logic [7:0] user_sw_n;

  initial begin
    reset_counter = 0;
  end

  always_ff @(posedge main_clk_buf) begin
    if (reset_counter != 8'hff) begin
      reset_counter <= reset_counter + 8'd1;
    end
  end

  assign top_rst_n = reset_counter < 8'd5   ? 1'b1 :
                     reset_counter < 8'd200 ? 1'b0 :
                                              nrst_btn;

  assign led_bootok = 1'b1;

  // Switch inputs have pull-ups and switches pull to ground when on. Invert here so CPU sees 1 for
  // on and 0 for off.
  assign nav_sw_n = ~nav_sw;
  assign user_sw_n = ~user_sw;

  // No LCD backlight FPGA IO on v0.2 board, so leave this unconnected
  logic lcd_backlight;

  logic scl0_o, scl0_oe;
  logic sda0_o, sda0_oe;
  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  // Feed all I2C traffic to the other port as a pure output (always enabled) since this allows
  // us very easily to attach the logic analyser.
`ifdef DRIVE_I2C1
  // Drive I2C1 bus (J7) - mirror its traffic on J1 (I2C0)
  logic scl0_out;
  logic sda0_out;

// ID EPROM on HAT
`ifdef DRIVE_RPIID
  always_ff @(posedge clk_sys) begin
    scl0_out <= RPH_G1;
    sda0_out <= RPH_G0;
  end
  // Open Drain drivers onto I2C bus.
  assign RPH_G1 = scl1_oe ? scl1_o : 1'bZ;
  assign RPH_G0 = sda1_oe ? sda1_o : 1'bZ;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = RPH_G1;
  wire sda1_i = RPH_G0;
`else
// General I2C bus on HAT
`ifdef DRIVE_RPIHAT
  always_ff @(posedge clk_sys) begin
    scl0_out <= RPH_G3_SCL;
    sda0_out <= RPH_G2_SDA;
  end
  // Open Drain drivers onto I2C bus.
  assign RPH_G3_SCL = scl1_oe ? scl1_o : 1'bZ;
  assign RPH_G2_SDA = sda1_oe ? sda1_o : 1'bZ;

  // Make it clear who is driving the I2C bus
  assign SCL1 = scl1_oe;
  assign SDA1 = sda1_oe;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = RPH_G3_SCL;
  wire sda1_i = RPH_G2_SDA;
`else
`ifdef DRIVE_mkBUS
  always_ff @(posedge clk_sys) begin
    scl0_out <= MB6;
    sda0_out <= MB5;
  end
  // Open Drain drivers onto I2C bus.
  assign MB6 = scl1_oe ? scl1_o : 1'bZ;
  assign MB5 = sda1_oe ? sda1_o : 1'bZ;

  // Make it clear who is driving the I2C bus
  assign SCL1 = scl1_oe;
  assign SDA1 = sda1_oe;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = MB6;
  wire sda1_i = MB5;
`else
  always_ff @(posedge clk_sys) begin
    scl0_out <= SCL1;
    sda0_out <= SDA1;
  end
  // Open Drain drivers onto I2C bus.
  assign SCL1 = scl1_oe ? scl1_o : 1'bZ;
  assign SDA1 = sda1_oe ? sda1_o : 1'bZ;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = SCL1;
  wire sda1_i = SDA1;
`endif
`endif
`endif
  // Output only, for logic analyser
  assign SCL0 = scl0_out;
  assign SDA0 = sda0_out;
`else
  logic scl1_out;
  logic sda1_out;

  // Driving I2C0
  always_ff @(posedge clk_sys) begin
    scl1_out <= SCL0;
    sda1_out <= SDA0;
  end
  // Open Drain drivers onto I2C bus.
  assign SCL0 = scl0_oe ? scl0_o : 1'bZ;
  assign SDA0 = sda0_oe ? sda0_o : 1'bZ;

  // Output only, for logic analyser
  assign SCL1 = scl1_out;
  assign SDA1 = sda1_out;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = SCL1;
  wire sda1_i = SDA1;
`endif
  ibex_demo_system #(
    .SysClkFreq(SysClkFreq),
    .GpiWidth(13),
    .GpoWidth(12),
    .PwmWidth(12),
    .SRAMInitFile(SRAMInitFile)
  ) u_ibex_demo_system (
    .clk_sys_i    (clk_sys),
    .rst_sys_ni   (rst_sys_n),

    .clk_usb_i    (clk_usb),
    .rst_usb_ni   (rst_usb_n),

    .clk_peri_i   (clk_peri),
    .rst_peri_ni  (rst_peri_n),

    .gp_i({user_sw_n, nav_sw_n}),
    .gp_o({led_user, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),

    .uart_rx_i(ser0_rx),
    .uart_tx_o(ser0_tx),

    .pwm_o({led_cherierr, led_legacy, led_cheri, led_halted}),

    .spi_rx_i(1'b0),
    .spi_tx_o(lcd_copi),
    .spi_sck_o(lcd_clk),

    // I2C bus 0
    .i2c0_scl_i       (scl0_i),
    .i2c0_scl_o       (scl0_o),
    .i2c0_scl_en_o    (scl0_oe),
    .i2c0_sda_i       (sda0_i),
    .i2c0_sda_o       (sda0_o),
    .i2c0_sda_en_o    (sda0_oe),

    // I2C bus 1
    .i2c1_scl_i       (scl1_i),
    .i2c1_scl_o       (scl1_o),
    .i2c1_scl_en_o    (scl1_oe),
    .i2c1_sda_i       (sda1_i),
    .i2c1_sda_o       (sda1_o),
    .i2c1_sda_en_o    (sda1_oe),

    .trst_ni(rst_sys_n),
    .tms_i,
    .tck_i,
    .td_i,
    .td_o
  );

  // Produce the system clock and 48 MHz USB clock from 25 MHz Sonata board clock
  clkgen_sonata  #(
    .SysClkFreq(SysClkFreq)
  ) clkgen(
    .IO_CLK     (main_clk),
    .IO_CLK_BUF (main_clk_buf),
    .IO_RST_N   (top_rst_n),
    .clk_sys    (clk_sys),
    .rst_sys_n  (rst_sys_n),
    .clk_usb    (clk_usb),
    .rst_usb_n  (rst_usb_n),
    .clk_peri   (clk_peri),
    .rst_peri_n (rst_peri_n)
  );

endmodule
