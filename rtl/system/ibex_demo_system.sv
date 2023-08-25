// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// The Ibex demo system, which instantiates and connects the following blocks:
// - Memory bus.
// - Ibex top module.
// - RAM memory to contain code and data.
// - GPIO driving logic.
// - UART for serial communication.
// - Timer.
// - Debug module.
// - SPI for driving LCD screen
module ibex_demo_system #(
  parameter int SysClkFreq   = 50_000_000,
  parameter int GpiWidth     = 8,
  parameter int GpoWidth     = 16,
  parameter int PwmWidth     = 12,
  parameter     SRAMInitFile = ""
) (
  input logic                 clk_sys_i,
  input logic                 rst_sys_ni,

  input logic                 clk_usb_i,
  input logic                 rst_usb_ni,

  input  logic [GpiWidth-1:0] gp_i,
  output logic [GpoWidth-1:0] gp_o,

  output logic [PwmWidth-1:0] pwm_o,

  input  logic                uart_rx_i,
  output logic                uart_tx_o,

  input  logic                spi_rx_i,
  output logic                spi_tx_o,
  output logic                spi_sck_o,

  output logic                usb_dp_o,
  output logic                usb_dp_en_o,
  output logic                usb_dn_o,
  output logic                usb_dn_en_o,

  input  logic                usb_dp_i,
  input  logic                usb_dn_i,

  input  logic                usb_sense_i,
  output logic                usb_dp_pullup_o,
  output logic                usb_dn_pullup_o
);
  localparam logic [31:0] MEM_SIZE      = 64 * 1024; // 64 KiB
  localparam logic [31:0] MEM_START     = 32'h00100000;
  localparam logic [31:0] MEM_MASK      = ~(MEM_SIZE-1);

  localparam logic [31:0] GPIO_SIZE     =  4 * 1024; //  4 KiB
  localparam logic [31:0] GPIO_START    = 32'h80000000;
  localparam logic [31:0] GPIO_MASK     = ~(GPIO_SIZE-1);

  localparam logic [31:0] DEBUG_START   = 32'h1a110000;
  localparam logic [31:0] DEBUG_SIZE    = 64 * 1024; // 64 KiB
  localparam logic [31:0] DEBUG_MASK    = ~(DEBUG_SIZE-1);

  localparam logic [31:0] UART_SIZE     =  4 * 1024; //  4 KiB
  localparam logic [31:0] UART_START    = 32'h80001000;
  localparam logic [31:0] UART_MASK     = ~(UART_SIZE-1);

  localparam logic [31:0] TIMER_SIZE    =  4 * 1024; //  4 KiB
  localparam logic [31:0] TIMER_START   = 32'h80002000;
  localparam logic [31:0] TIMER_MASK    = ~(TIMER_SIZE-1);

  localparam logic [31:0] PWM_SIZE      =  4 * 1024; //  4 KiB
  localparam logic [31:0] PWM_START     = 32'h80003000;
  localparam logic [31:0] PWM_MASK      = ~(PWM_SIZE-1);
  localparam int PwmCtrSize = 8;

  parameter logic [31:0] SPI_SIZE       = 1 * 1024; // 1kB
  parameter logic [31:0] SPI_START      = 32'h80004000;
  parameter logic [31:0] SPI_MASK       = ~(SPI_SIZE-1);

  localparam logic [31:0] USBDEV_SIZE   = 4 * 1024; // 4 KiB
  localparam logic [31:0] USBDEV_START  = 32'h80005000;
  localparam logic [31:0] USBDEV_MASK   = ~(USBDEV_SIZE-1);

  parameter logic [31:0] SIM_CTRL_SIZE  = 1 * 1024; // 1kB
  parameter logic [31:0] SIM_CTRL_START = 32'h20000;
  parameter logic [31:0] SIM_CTRL_MASK  = ~(SIM_CTRL_SIZE-1);

  // debug functionality is optional
  localparam bit DBG = 1;
  localparam int unsigned DbgHwBreakNum = (DBG == 1) ?    2 :    0;
  localparam bit          DbgTriggerEn  = (DBG == 1) ? 1'b1 : 1'b0;

  typedef enum int {
    CoreD,
    DbgHost
  } bus_host_e;

  typedef enum int {
    Ram = 0,
    Gpio,
    Pwm,
    Uart,
    Timer,
    Spi,
    SimCtrl,

    // Must be last
    DbgDev
  } bus_device_e;

  localparam int NrDevices = DBG ? (1 + DbgDev) : DbgDev;
  localparam int NrHosts = DBG ? 2 : 1;

  // interrupts
  logic timer_irq;
  logic uart_irq;

  // host and device signals
  logic           host_req      [NrHosts];
  logic           host_gnt      [NrHosts];
  logic [31:0]    host_addr     [NrHosts];
  logic           host_we       [NrHosts];
  logic [ 3:0]    host_be       [NrHosts];
  logic [31:0]    host_wdata    [NrHosts];
  logic           host_rvalid   [NrHosts];
  logic [31:0]    host_rdata    [NrHosts];
  logic           host_err      [NrHosts];

  // devices (slaves)
  logic           device_req    [NrDevices];
  logic [31:0]    device_addr   [NrDevices];
  logic           device_we     [NrDevices];
  logic [ 3:0]    device_be     [NrDevices];
  logic [31:0]    device_wdata  [NrDevices];
  logic           device_rvalid [NrDevices];
  logic [31:0]    device_rdata  [NrDevices];
  logic           device_err    [NrDevices];

  // requests to demo system bus
  logic           host_req_bus      [NrHosts];

  // responses from demo system bus
  logic           host_gnt_bus      [NrHosts];
  logic           host_rvalid_bus   [NrHosts];
  logic [31:0]    host_rdata_bus    [NrHosts];
  logic           host_err_bus      [NrHosts];

  logic tlul_gnt_o;
  logic tlul_rvalid_o;
  logic [31:0] tlul_rdata_o;
  logic tlul_err_o;

  wire host_req_usbdev = host_req[CoreD] & ((host_addr[CoreD] & USBDEV_MASK) == USBDEV_START);

  // TODO: quick hack - TL cannot provide an immediate response
  always_comb begin
    for (integer host = 0; host < NrHosts; host = host + 1) begin
      if (host == CoreD) begin
        bit usbdev_addr = ((host_addr[host] & USBDEV_MASK) == USBDEV_START);

        host_req_bus[host] = host_req[host] & ~usbdev_addr;
        host_gnt[host]     = tlul_gnt_o | host_gnt_bus[host];

        host_rvalid[host]  = tlul_rvalid_o | host_rvalid_bus[host];
        host_rdata[host]   = tlul_rvalid_o ? tlul_rdata_o : host_rdata_bus[host];
        host_err[host]     = tlul_err_o | host_err_bus[host];
      end else begin
        host_req_bus[host] = host_req[host];
        host_gnt[host]     = host_gnt_bus[host];
        host_rvalid[host]  = host_rvalid_bus[host];
        host_rdata[host]   = host_rdata_bus[host];
        host_err[host]     = host_err_bus[host];      
      end
    end
  end

  // Instruction fetch signals
  logic        core_instr_req;
  logic        core_instr_gnt;
  logic        core_instr_rvalid;
  logic [31:0] core_instr_addr;
  logic [31:0] core_instr_rdata;
  logic        core_instr_sel_dbg;

  logic        mem_instr_req;
  logic [31:0] mem_instr_rdata;
  logic        dbg_instr_req;

  logic        dbg_slave_req;
  logic [31:0] dbg_slave_addr;
  logic        dbg_slave_we;
  logic [ 3:0] dbg_slave_be;
  logic [31:0] dbg_slave_wdata;
  logic        dbg_slave_rvalid;
  logic [31:0] dbg_slave_rdata;

  // Internally generated resets cause IMPERFECTSCH warnings
  /* verilator lint_off IMPERFECTSCH */
  logic        rst_core_n;
  logic        ndmreset_req;
  logic        dm_debug_req;

  // Device address mapping
  logic [31:0] cfg_device_addr_base [NrDevices];
  logic [31:0] cfg_device_addr_mask [NrDevices];

  assign cfg_device_addr_base[Ram]     = MEM_START;
  assign cfg_device_addr_mask[Ram]     = MEM_MASK;
  assign cfg_device_addr_base[Gpio]    = GPIO_START;
  assign cfg_device_addr_mask[Gpio]    = GPIO_MASK;
  assign cfg_device_addr_base[Pwm]     = PWM_START;
  assign cfg_device_addr_mask[Pwm]     = PWM_MASK;
  assign cfg_device_addr_base[Uart]    = UART_START;
  assign cfg_device_addr_mask[Uart]    = UART_MASK;
  assign cfg_device_addr_base[Timer]   = TIMER_START;
  assign cfg_device_addr_mask[Timer]   = TIMER_MASK;
  assign cfg_device_addr_base[Spi]     = SPI_START;
  assign cfg_device_addr_mask[Spi]     = SPI_MASK;
  assign cfg_device_addr_base[SimCtrl] = SIM_CTRL_START;
  assign cfg_device_addr_mask[SimCtrl] = SIM_CTRL_MASK;

  if (DBG) begin : g_dbg_device_cfg
    assign cfg_device_addr_base[DbgDev] = DEBUG_START;
    assign cfg_device_addr_mask[DbgDev] = DEBUG_MASK;
    assign device_err[DbgDev] = 1'b0;
  end

  // Tie-off unused error signals
  assign device_err[Ram]     = 1'b0;
  assign device_err[Gpio]    = 1'b0;
  assign device_err[Pwm]     = 1'b0;
  assign device_err[Uart]    = 1'b0;
  assign device_err[Spi]     = 1'b0;
  assign device_err[SimCtrl] = 1'b0;

  bus #(
    .NrDevices    ( NrDevices ),
    .NrHosts      ( NrHosts   ),
    .DataWidth    ( 32        ),
    .AddressWidth ( 32        )
  ) u_bus (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    .host_req_i          (host_req_bus ),
    .host_gnt_o          (host_gnt_bus ),
    .host_addr_i         (host_addr    ),
    .host_we_i           (host_we      ),
    .host_be_i           (host_be      ),
    .host_wdata_i        (host_wdata   ),
    .host_rvalid_o       (host_rvalid_bus),
    .host_rdata_o        (host_rdata_bus ),
    .host_err_o          (host_err_bus   ),

    .device_req_o        (device_req   ),
    .device_addr_o       (device_addr  ),
    .device_we_o         (device_we    ),
    .device_be_o         (device_be    ),
    .device_wdata_o      (device_wdata ),
    .device_rvalid_i     (device_rvalid),
    .device_rdata_i      (device_rdata ),
    .device_err_i        (device_err   ),

    .cfg_device_addr_base,
    .cfg_device_addr_mask
  );

  assign mem_instr_req =
      core_instr_req & ((core_instr_addr & cfg_device_addr_mask[Ram]) == cfg_device_addr_base[Ram]);

  assign dbg_instr_req =
      core_instr_req & ((core_instr_addr & cfg_device_addr_mask[DbgDev]) == cfg_device_addr_base[DbgDev]);

  assign core_instr_gnt = mem_instr_req | (dbg_instr_req & ~device_req[DbgDev]);

  always @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      core_instr_rvalid  <= 1'b0;
      core_instr_sel_dbg <= 1'b0;
    end else begin
      core_instr_rvalid  <= core_instr_gnt;
      core_instr_sel_dbg <= dbg_instr_req;
    end
  end

  assign core_instr_rdata = core_instr_sel_dbg ? dbg_slave_rdata : mem_instr_rdata;

  assign rst_core_n = rst_sys_ni & ~ndmreset_req;

  ibex_top #(
    .RegFile         ( ibex_pkg::RegFileFPGA                   ),
    .MHPMCounterNum  ( 10                                      ),
    .RV32M           ( ibex_pkg::RV32MFast                     ),
    .RV32B           ( ibex_pkg::RV32BNone                     ),
    .DbgTriggerEn    ( DbgTriggerEn                            ),
    .DbgHwBreakNum   ( DbgHwBreakNum                           ),
    .DmHaltAddr      ( DEBUG_START + dm::HaltAddress[31:0]     ),
    .DmExceptionAddr ( DEBUG_START + dm::ExceptionAddress[31:0])
  ) u_top (
    .clk_i (clk_sys_i),
    .rst_ni(rst_core_n),

    .test_en_i  ('b0),
    .scan_rst_ni(1'b1),
    .ram_cfg_i  ('b0),

    .hart_id_i(32'b0),
    // First instruction executed is at 0x0 + 0x80
    .boot_addr_i(32'h00100000),

    .instr_req_o       (core_instr_req),
    .instr_gnt_i       (core_instr_gnt),
    .instr_rvalid_i    (core_instr_rvalid),
    .instr_addr_o      (core_instr_addr),
    .instr_rdata_i     (core_instr_rdata),
    .instr_rdata_intg_i('0),
    .instr_err_i       ('0),

    .data_req_o       (host_req[CoreD]),
    .data_gnt_i       (host_gnt[CoreD]),
    .data_rvalid_i    (host_rvalid[CoreD]),
    .data_we_o        (host_we[CoreD]),
    .data_be_o        (host_be[CoreD]),
    .data_addr_o      (host_addr[CoreD]),
    .data_wdata_o     (host_wdata[CoreD]),
    .data_wdata_intg_o(),
    .data_rdata_i     (host_rdata[CoreD]),
    .data_rdata_intg_i('0),
    .data_err_i       (host_err[CoreD]),

    .irq_software_i(1'b0),
    .irq_timer_i   (timer_irq),
    .irq_external_i(1'b0),
    .irq_fast_i    ({14'b0, uart_irq}),
    .irq_nm_i      (1'b0),

    .scramble_key_valid_i('0),
    .scramble_key_i      ('0),
    .scramble_nonce_i    ('0),
    .scramble_req_o      (),

    .debug_req_i        (dm_debug_req),
    .crash_dump_o       (),
    .double_fault_seen_o(),

    .fetch_enable_i        ('1),
    .alert_minor_o         (),
    .alert_major_internal_o(),
    .alert_major_bus_o     (),
    .core_sleep_o          ()
  );

  ram_2p #(
      .Depth       ( MEM_SIZE / 4 ),
      .MemInitFile ( SRAMInitFile )
  ) u_ram (
    .clk_i       (clk_sys_i),
    .rst_ni      (rst_sys_ni),

    .a_req_i     (device_req[Ram]),
    .a_we_i      (device_we[Ram]),
    .a_be_i      (device_be[Ram]),
    .a_addr_i    (device_addr[Ram]),
    .a_wdata_i   (device_wdata[Ram]),
    .a_rvalid_o  (device_rvalid[Ram]),
    .a_rdata_o   (device_rdata[Ram]),

    .b_req_i     (mem_instr_req),
    .b_we_i      (1'b0),
    .b_be_i      (4'b0),
    .b_addr_i    (core_instr_addr),
    .b_wdata_i   (32'b0),
    .b_rvalid_o  (),
    .b_rdata_o   (mem_instr_rdata)
  );

  gpio #(
    .GpiWidth ( GpiWidth ),
    .GpoWidth ( GpoWidth )
  ) u_gpio (
    .clk_i          (clk_sys_i),
    .rst_ni         (rst_sys_ni),

    .device_req_i   (device_req[Gpio]),
    .device_addr_i  (device_addr[Gpio]),
    .device_we_i    (device_we[Gpio]),
    .device_be_i    (device_be[Gpio]),
    .device_wdata_i (device_wdata[Gpio]),
    .device_rvalid_o(device_rvalid[Gpio]),
    .device_rdata_o (device_rdata[Gpio]),

    .gp_i,
    .gp_o
  );

  pwm_wrapper #(
    .PwmWidth   ( PwmWidth   ),
    .PwmCtrSize ( PwmCtrSize ),
    .BusWidth   ( 32         )
  ) u_pwm (
    .clk_i          (clk_sys_i),
    .rst_ni         (rst_sys_ni),

    .device_req_i   (device_req[Pwm]),
    .device_addr_i  (device_addr[Pwm]),
    .device_we_i    (device_we[Pwm]),
    .device_be_i    (device_be[Pwm]),
    .device_wdata_i (device_wdata[Pwm]),
    .device_rvalid_o(device_rvalid[Pwm]),
    .device_rdata_o (device_rdata[Pwm]),

    .pwm_o
  );

  uart #(
    .ClockFrequency (SysClkFreq)
  ) u_uart (
    .clk_i          (clk_sys_i),
    .rst_ni         (rst_sys_ni),

    .device_req_i   (device_req[Uart]),
    .device_addr_i  (device_addr[Uart]),
    .device_we_i    (device_we[Uart]),
    .device_be_i    (device_be[Uart]),
    .device_wdata_i (device_wdata[Uart]),
    .device_rvalid_o(device_rvalid[Uart]),
    .device_rdata_o (device_rdata[Uart]),

    .uart_rx_i,
    .uart_irq_o     (uart_irq),
    .uart_tx_o
  );

  spi_top #(
    .ClockFrequency(SysClkFreq),
    .CPOL(0),
    .CPHA(1)
  ) u_spi (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .device_req_i   (device_req[Spi]),
    .device_addr_i  (device_addr[Spi]),
    .device_we_i    (device_we[Spi]),
    .device_be_i    (device_be[Spi]),
    .device_wdata_i (device_wdata[Spi]),
    .device_rvalid_o(device_rvalid[Spi]),
    .device_rdata_o (device_rdata[Spi]),

    .spi_rx_i(spi_rx_i), // Data received from SPI device
    .spi_tx_o(spi_tx_o), // Data transmitted to SPI device
    .sck_o(spi_sck_o), // Serial clock pin

    .byte_data_o() // unused
  );

  tlul_pkg::tl_h2d_t tl_d_ibex2fifo;
  tlul_pkg::tl_d2h_t tl_d_fifo2ibex;

// TODO: still to be resolved - integrity generation and checking
// alert handling
  tlul_adapter_host #(
    .MAX_REQS(2),
    .EnableDataIntgGen(1'b1)
  ) tl_adapter_host_d_ibex (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),
    .req_i        (host_req_usbdev),
    .instr_type_i (prim_mubi_pkg::MuBi4False),
    .gnt_o        (tlul_gnt_o),
    .addr_i       (host_addr[CoreD]),
    .we_i         (host_we[CoreD]),
    .wdata_i      (host_wdata[CoreD]),
    .wdata_intg_i (7'b0),
    .be_i         (host_be[CoreD]),
    .valid_o      (tlul_rvalid_o),
    .rdata_o      (tlul_rdata_o),
    .rdata_intg_o (),
    .err_o        (tlul_err_o),
    .intg_err_o   (),
    .tl_o         (tl_d_ibex2fifo),
    .tl_i         (tl_d_fifo2ibex)
  );

  tlul_pkg::tl_h2d_t usb_tl_i;
  tlul_pkg::tl_d2h_t usb_tl_o;

  tlul_fifo_async #(
  ) fifo_d (
    .clk_h_i     (clk_sys_i),
    .rst_h_ni    (rst_sys_ni),
    .clk_d_i     (clk_usb_i),
    .rst_d_ni    (rst_usb_ni),
    .tl_h_i      (tl_d_ibex2fifo),
    .tl_h_o      (tl_d_fifo2ibex),
    .tl_d_o      (usb_tl_i),
    .tl_d_i      (usb_tl_o)
  );

  usbdev #(
    .Stub(1'b0)
  ) u_usbdev (
    .clk_i    (clk_usb_i),
    .rst_ni   (rst_usb_ni),

    // AON Wakeup functionality is not being used
    .clk_aon_i  (clk_usb_i),
    .rst_aon_ni (rst_usb_ni),

    .tl_i       (usb_tl_i),
    .tl_o       (usb_tl_o),

    // Alerts are unused
    .alert_rx_i (4'b0),
    .alert_tx_o (),

    // Data inputs
    .cio_usb_dp_i     (usb_dp_i),
    .cio_usb_dn_i     (usb_dn_i),
    .usb_rx_d_i       (usb_dp_i),  // We have no external diff receiver

    // Data outputs
    .cio_usb_dp_o     (usb_dp_o),
    .cio_usb_dp_en_o  (usb_dp_en_o),
    .cio_usb_dn_o     (usb_dn_o),
    .cio_usb_dn_en_o  (usb_dn_en_o),
    .usb_tx_se0_o     (),
    .usb_tx_d_o       (),
 
    // Non-data I/O
    .cio_sense_i        (usb_sense_i),
    .usb_dp_pullup_o    (usb_dp_pullup_o),
    .usb_dn_pullup_o    (usb_dn_pullup_o),
    .usb_rx_enable_o    (),
    .usb_tx_use_d_se0_o (),
    
    // Unused AON/Wakeup functionality
    .usb_aon_suspend_req_o  (),
    .usb_aon_wake_ack_o     (),

    .usb_aon_bus_reset_i          (1'b0),
    .usb_aon_sense_lost_i         (1'b0),
    .usb_aon_wake_detect_active_i (1'b0),

    .usb_ref_val_o    (),
    .usb_ref_pulse_o  (),

    .ram_cfg_i  ('b0),

    // Interrupts not required
    .intr_pkt_received_o    (),
    .intr_pkt_sent_o        (),
    .intr_powered_o         (),
    .intr_disconnected_o    (),
    .intr_host_lost_o       (),
    .intr_link_reset_o      (),
    .intr_link_suspend_o    (),
    .intr_link_resume_o     (),
    .intr_av_empty_o        (),
    .intr_rx_full_o         (),
    .intr_av_overflow_o     (),
    .intr_link_in_err_o     (),
    .intr_link_out_err_o    (),
    .intr_rx_crc_err_o      (),
    .intr_rx_pid_err_o      (),
    .intr_rx_bitstuff_err_o (),
    .intr_frame_o           ()
  );

  `ifdef VERILATOR
    simulator_ctrl #(
      .LogName("ibex_demo_system.log")
    ) u_simulator_ctrl (
      .clk_i     (clk_sys_i),
      .rst_ni    (rst_sys_ni),

      .req_i     (device_req[SimCtrl]),
      .we_i      (device_we[SimCtrl]),
      .be_i      (device_be[SimCtrl]),
      .addr_i    (device_addr[SimCtrl]),
      .wdata_i   (device_wdata[SimCtrl]),
      .rvalid_o  (device_rvalid[SimCtrl]),
      .rdata_o   (device_rdata[SimCtrl])
    );
  `endif

  timer #(
    .DataWidth    ( 32 ),
    .AddressWidth ( 32 )
  ) u_timer (
    .clk_i         (clk_sys_i),
    .rst_ni        (rst_sys_ni),

    .timer_req_i   (device_req[Timer]),
    .timer_we_i    (device_we[Timer]),
    .timer_be_i    (device_be[Timer]),
    .timer_addr_i  (device_addr[Timer]),
    .timer_wdata_i (device_wdata[Timer]),
    .timer_rvalid_o(device_rvalid[Timer]),
    .timer_rdata_o (device_rdata[Timer]),
    .timer_err_o   (device_err[Timer]),
    .timer_intr_o  (timer_irq)
  );

  assign dbg_slave_req         = device_req[DbgDev] | dbg_instr_req;
  assign dbg_slave_we          = device_req[DbgDev] & device_we[DbgDev];
  assign dbg_slave_addr        = device_req[DbgDev] ? device_addr[DbgDev] : core_instr_addr;
  assign dbg_slave_be          = device_be[DbgDev];
  assign dbg_slave_wdata       = device_wdata[DbgDev];
  assign device_rvalid[DbgDev] = dbg_slave_rvalid;
  assign device_rdata[DbgDev]  = dbg_slave_rdata;

  always @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      dbg_slave_rvalid <= 1'b0;
    end else begin
      dbg_slave_rvalid <= device_req[DbgDev];
    end
  end

  if (DBG) begin : g_dm_top
    dm_top #(
      .NrHarts ( 1 )
    ) u_dm_top (
      .clk_i             (clk_sys_i),
      .rst_ni            (rst_sys_ni),
      .testmode_i        (1'b0),
      .ndmreset_o        (ndmreset_req),
      .dmactive_o        (),
      .debug_req_o       (dm_debug_req),
      .unavailable_i     (1'b0),

      // bus device with debug memory (for execution-based debug)
      .slave_req_i       (dbg_slave_req),
      .slave_we_i        (dbg_slave_we),
      .slave_addr_i      (dbg_slave_addr),
      .slave_be_i        (dbg_slave_be),
      .slave_wdata_i     (dbg_slave_wdata),
      .slave_rdata_o     (dbg_slave_rdata),

      // bus host (for system bus accesses, SBA)
      .master_req_o      (host_req[DbgHost]),
      .master_add_o      (host_addr[DbgHost]),
      .master_we_o       (host_we[DbgHost]),
      .master_wdata_o    (host_wdata[DbgHost]),
      .master_be_o       (host_be[DbgHost]),
      .master_gnt_i      (host_gnt[DbgHost]),
      .master_r_valid_i  (host_rvalid[DbgHost]),
      .master_r_rdata_i  (host_rdata[DbgHost])
    );
  end else begin
    assign dm_debug_req = 1'b0;
    assign ndmreset_req = 1'b0;
  end

  `ifdef VERILATOR
    export "DPI-C" function mhpmcounter_get;

    function automatic longint unsigned mhpmcounter_get(int index);
      return u_top.u_ibex_core.cs_registers_i.mhpmcounter[index];
    endfunction
  `endif
endmodule
