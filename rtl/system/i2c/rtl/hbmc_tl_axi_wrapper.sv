// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Wrapper for OpenHBMC HyperRAM controller; bridges AXI to TL .
module hbmc_tl_axi_wrapper 
  import tlul_pkg::*;
(
  input logic               clk_peri_i,
  input logic               rst_peri_ni,

  input logic               clk_hr,
  input logic               clk_hr90p,
  input logic               clk_hr3x,
  input logic               rst_hr,

  // Bus Interface
  input  tl_h2d_t tl_i,
  output tl_d2h_t tl_o,

  // HyperRAM:
  inout  wire [7:0]           HYPERRAM_DQ,
  inout  wire                 HYPERRAM_RWDS,
  output wire                 HYPERRAM_CKP,
  output wire                 HYPERRAM_CKN,
  output wire                 HYPERRAM_nRST,
  output wire                 HYPERRAM_CS,

  // debug only:
  output reg  [31:0] awaddr,
  output reg  awvalid,
  output wire awready,
  // write data:
  output reg  [31:0] wdata,
  output reg  wvalid,
  output wire wready,
  // write response:
  output wire [1:0] bresp,
  output wire bvalid,
  output wire bready,
  // read address:
  output reg  [31:0] araddr,
  output reg  arvalid,
  output wire arready,
  // read data:
  output wire [31:0] rdata,
  output wire [1:0] rresp,
  output wire rvalid,
  output wire rready
);

tl_d2h_t tl_o_pre;

reg [1:0] state;
localparam pS_IDLE            = 2'd0;
localparam pS_WRITE           = 2'd1;
localparam pS_READ            = 2'd2;

reg axi_req_processed;

/*
// AXI signals:
// write address:
reg  [31:0] awaddr;
reg  awvalid;
wire awready;

// write data:
reg  [31:0] wdata;
reg  wvalid;
wire wready;

// write response:
wire [1:0] bresp;
wire bvalid;
wire bready = 1'b1;

// read address:
reg  [31:0] araddr;
reg  arvalid;
wire arready;

// read data:
wire [31:0] rdata;
wire [1:0] rresp;
wire rvalid;
wire rready = 1'b1;
*/
assign bready = 1'b1;
assign rready = 1'b1;


// immutables:
assign tl_o_pre.d_source =  tl_i.a_source;
assign tl_o_pre.d_size =    tl_i.a_size;
assign tl_o_pre.d_source =  tl_i.a_source;
assign tl_o_pre.d_param = 3'd0;
assign tl_o_pre.d_sink = 1'd0;


tlul_rsp_intg_gen #(
  .EnableRspIntgGen(1),
  .EnableDataIntgGen(1)
) u_rsp_intg_gen (
  .tl_i(tl_o_pre),
  .tl_o(tl_o)
);


wire awdone_condition = (awready && awvalid);
wire wdone_condition = (wready && wvalid);
wire ardone_condition = (arready && arvalid);
reg awdone_r;
reg wdone_r;
reg ardone_r;
wire awdone = awdone_r || awdone_condition;
wire wdone  = wdone_r  || wdone_condition;
wire ardone = ardone_r || ardone_condition;

always @(posedge clk_peri_i) begin
    if (state == pS_IDLE) begin
        awdone_r <= 1'b0;
        wdone_r <= 1'b0;
        ardone_r <= 1'b0;
    end
    else begin
        if (awdone_condition)
            awdone_r <= 1'b1;
        if (wdone_condition)
            wdone_r <= 1'b1;
        if (ardone_condition)
            ardone_r <= 1'b1;
    end
end


always @(posedge clk_peri_i) begin
    if (~rst_peri_ni) begin
        // AXI defaults:
        awvalid <= 0;
        wvalid <= 0;
        arvalid <= 0;

        // TL defaults:
        tl_o_pre.a_ready <= 1'b1; // drive this low when not ready for a new request
        tl_o_pre.d_valid <= 1'b0;

        state <= pS_IDLE;
    end

    else begin
        case (state)

            pS_IDLE: begin
                axi_req_processed <= 1'b0;
                if (tl_i.a_valid) begin
                    axi_req_processed <= 1'b0;
                    if ((tl_i.a_opcode == PutFullData) || (tl_i.a_opcode == PutPartialData)) begin
                        state <= pS_WRITE;
                        tl_o_pre.d_opcode <= AccessAck;
                        tl_o_pre.a_ready <= 1'b0;
                        //tl_o_pre.d_valid <= 1'b1; need to wait to see response first
                        wdata <= tl_i.a_data;
                        awaddr <= tl_i.a_address - 32'h8000_5000; // TODO: how to support full memory space?
                        awvalid <= 1'b1;
                        wvalid <= 1'b1;
                    end

                    else if (tl_i.a_opcode == Get) begin
                        state <= pS_READ;
                        araddr <= tl_i.a_address - 32'h8000_5000; // TODO: how to support full memory space?
                        arvalid <= 1'b1;
                        tl_o_pre.a_ready <= 1'b0;
                    end
                    // TODO: error response for other opcodes?
                end
                else begin
                    tl_o_pre.a_ready <= 1'b1;
                    tl_o_pre.d_valid <= 1'b0;
                end
            end

            pS_WRITE: begin
                // NOTE: could speed up here by assuming the write will go through? seems dangerous
                if (awready)
                    awvalid <= 0;
                if (wready)
                    wvalid <= 0;
                if (bvalid) begin
                    axi_req_processed <= 1'b1;
                    if (bresp == 0)
                        tl_o_pre.d_error <= 1'b0;
                    else
                        tl_o_pre.d_error <= 1'b1;
                end
                if (awdone && wdone && (bvalid || axi_req_processed))
                    tl_o_pre.d_valid <= 1'b1;
                if (tl_o_pre.d_valid && tl_i.d_ready) begin
                    state <= pS_IDLE;
                    tl_o_pre.d_valid <= 1'b0;
                    tl_o_pre.a_ready <= 1'b1;
                end
            end

            pS_READ: begin
                // this adds an additional cycle of latency on reads but keeps RTL simple
                if (arready)
                    arvalid <= 1'b0;
                if (rvalid) begin
                    tl_o_pre.d_data <= rdata;
                    axi_req_processed <= 1'b1;
                    if (rresp == 0)
                        tl_o_pre.d_error <= 1'b0;
                    else
                        tl_o_pre.d_error <= 1'b1;
                end
                if (tl_i.d_ready && ardone && (rvalid || axi_req_processed)) begin
                    state <= pS_IDLE;
                    tl_o_pre.d_opcode <= AccessAckData;
                    tl_o_pre.d_valid <= 1'b1;
                    tl_o_pre.a_ready <= 1'b1;
                end

            end

        endcase
    end
end


OpenHBMC U_HBMC (
  .clk_hbmc_0           (clk_hr         ),
  .clk_hbmc_90          (clk_hr90p      ),
  .clk_iserdes          (clk_hr3x       ),

  .s_axi_aclk           (clk_peri_i),
  .s_axi_aresetn        (~rst_hr),

  .s_axi_awid           (0),
  .s_axi_awaddr         (awaddr),
  .s_axi_awlen          (0),
  .s_axi_awsize         (4),
  .s_axi_awburst        (0),
  .s_axi_awlock         (0),
  .s_axi_awregion       (0),
  .s_axi_awcache        (0),
  .s_axi_awqos          (0),
  .s_axi_awprot         (0),
  .s_axi_awvalid        (awvalid),
  .s_axi_awready        (awready),

  .s_axi_wdata          (wdata  ),
  .s_axi_wstrb          (4'b1111),
  .s_axi_wlast          (1'b1   ),
  .s_axi_wvalid         (wvalid ),
  .s_axi_wready         (wready ),

  .s_axi_bid            (),             // unused (constant)
  .s_axi_bresp          (bresp  ),
  .s_axi_bvalid         (bvalid ),
  .s_axi_bready         (bready ),

  .s_axi_arid           (0),
  .s_axi_araddr         (araddr),
  .s_axi_arlen          (0),
  .s_axi_arsize         (4),
  .s_axi_arburst        (0),
  .s_axi_arlock         (0),
  .s_axi_arregion       (0),
  .s_axi_arcache        (0),
  .s_axi_arqos          (0),
  .s_axi_arprot         (0),
  .s_axi_arvalid        (arvalid),
  .s_axi_arready        (arready),

  .s_axi_rid            (),             // unused (constant)
  .s_axi_rdata          (rdata  ),
  .s_axi_rresp          (rresp  ),
  .s_axi_rlast          (),             // unused (constant)
  .s_axi_rvalid         (rvalid ),
  .s_axi_rready         (rready ),

  .hb_dq                (HYPERRAM_DQ   ),
  .hb_rwds              (HYPERRAM_RWDS ),
  .hb_ck_p              (HYPERRAM_CKP  ),
  .hb_ck_n              (HYPERRAM_CKN  ),
  .hb_reset_n           (HYPERRAM_nRST ),
  .hb_cs_n              (HYPERRAM_CS   ) 
);


endmodule
