/* 
 * ----------------------------------------------------------------------------
 *  Project:  OpenHBMC
 *  Filename: hbmc_ufifo.v
 *  Purpose:  Upstream data FIFO. Stores data read from the memory.
 * ----------------------------------------------------------------------------
 *  Copyright © 2020-2022, Vaagn Oganesyan <ovgn@protonmail.com>
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 * ----------------------------------------------------------------------------
 */

 
`default_nettype none
`timescale 1ps / 1ps

`ifndef PRIM_DEFAULT_IMPL
  `define PRIM_DEFAULT_IMPL prim_pkg::ImplGeneric
`endif

module hbmc_ufifo #
(
    parameter integer DATA_WIDTH = 32
)
(
    input   wire                        fifo_arst,
    
    input   wire                        fifo_wr_clk,
    input   wire    [15:0]              fifo_wr_din,
    input   wire                        fifo_wr_last,
    input   wire                        fifo_wr_ena,
    output  wire                        fifo_wr_full,
    
    input   wire                        fifo_rd_clk,
    output  wire    [DATA_WIDTH - 1:0]  fifo_rd_dout,
    output  wire    [9:0]               fifo_rd_free,
    output  wire                        fifo_rd_last,
    input   wire                        fifo_rd_ena,
    output  wire                        fifo_rd_empty
);

    parameter prim_pkg::impl_e Impl = `PRIM_DEFAULT_IMPL;
    localparam  FIFO_RD_DEPTH = 512;

    wire    [17:0]  din = {1'b0, fifo_wr_last, fifo_wr_din};
    wire    [8:0]   fifo_rd_used;

`ifdef XILINX_HBMC_UFIFO
    generate
        case (DATA_WIDTH)
            
            16: begin : uFIFO_18b_18b_512w
                
                wire    [17:0]  dout;
                
                assign  fifo_rd_dout = dout[15:0];
                assign  fifo_rd_last = dout[16];
            
                fifo_18b_18b_512w
                fifo_18b_18b_512w_inst
                (
                    .rst            ( fifo_arst     ),  // input rst
                    
                    .wr_clk         ( fifo_wr_clk   ),  // input wr_clk
                    .wr_en          ( fifo_wr_ena   ),  // input wr_en
                    .full           ( fifo_wr_full  ),  // output full
                    .din            ( din           ),  // input [17 : 0] din
                    
                    .rd_clk         ( fifo_rd_clk   ),  // input rd_clk
                    .rd_data_count  ( fifo_rd_used  ),  // output [8 : 0] rd_data_count
                    .rd_en          ( fifo_rd_ena   ),  // input rd_en
                    .empty          ( fifo_rd_empty ),  // output empty
                    .dout           ( dout          )   // output [17 : 0] dout
                );
            end

            /*--------------------------------------------------------------------*/
            
            32: begin : uFIFO_18b_36b_512w
                
                wire    [35:0]  dout;
                
                assign  fifo_rd_dout = {dout[15:0], dout[33:18]};
                assign  fifo_rd_last = dout[16];
            
                fifo_18b_36b_512w
                fifo_18b_36b_512w_inst
                (
                    .rst            ( fifo_arst     ),  // input rst
                    
                    .wr_clk         ( fifo_wr_clk   ),  // input wr_clk
                    .wr_en          ( fifo_wr_ena   ),  // input wr_en
                    .full           ( fifo_wr_full  ),  // output full
                    .din            ( din           ),  // input [17 : 0] din
                    
                    .rd_clk         ( fifo_rd_clk   ),  // input rd_clk
                    .rd_data_count  ( fifo_rd_used  ),  // output [8 : 0] rd_data_count
                    .rd_en          ( fifo_rd_ena   ),  // input rd_en
                    .empty          ( fifo_rd_empty ),  // output empty
                    .dout           ( dout          )   // output [35 : 0] dout
                );
                
            end

            /*--------------------------------------------------------------------*/
            
            64: begin : uFIFO_18b_72b_512w
                
                wire    [71:0]  dout;
                
                assign  fifo_rd_dout = {dout[15:0], dout[33:18], dout[51:36], dout[69:54]};
                assign  fifo_rd_last = dout[16];
                
                fifo_18b_72b_512w
                fifo_18b_72b_512w_inst
                (
                    .rst            ( fifo_arst     ),  // input rst
                    
                    .wr_clk         ( fifo_wr_clk   ),  // input wr_clk
                    .wr_en          ( fifo_wr_ena   ),  // input wr_en
                    .full           ( fifo_wr_full  ),  // output full
                    .din            ( din           ),  // input [17 : 0] din
                    
                    .rd_clk         ( fifo_rd_clk   ),  // input rd_clk
                    .rd_data_count  ( fifo_rd_used  ),  // output [8 : 0] rd_data_count
                    .rd_en          ( fifo_rd_ena   ),  // input rd_en
                    .empty          ( fifo_rd_empty ),  // output empty
                    .dout           ( dout          )   // output [71 : 0] dout
                );
            end
            
            /*--------------------------------------------------------------------*/
            
            default: begin
                INVALID_PARAMETER invalid_parameter_msg();
            end
            
        endcase
    endgenerate
    
    
    assign fifo_rd_free = FIFO_RD_DEPTH - fifo_rd_used;

`else
    // coding only the fifo_18b_36b_512w case that we need, for simplicity:
    localparam pDATA_IN_WIDTH = 18;
    localparam pDATA_OUT_WIDTH = 36;
    localparam pFIFO_WR_DEPTH = FIFO_RD_DEPTH * pDATA_OUT_WIDTH / pDATA_IN_WIDTH;

    wire    [pDATA_OUT_WIDTH-1:0]  dout;
    assign  fifo_rd_dout = {dout[15:0], dout[33:18]};
    assign  fifo_rd_last = dout[16];

    // handle I/O width conversion:
    reg wide_write_cnt = 1'b0;
    reg [pDATA_IN_WIDTH-1:0] din_r;
    always @(posedge fifo_wr_clk) begin
        if (fifo_wr_ena) begin
            wide_write_cnt <= ~wide_write_cnt;
            din_r <= din;
        end
    end
    wire fifo_wide_write = fifo_wr_ena && wide_write_cnt;
    wire [pDATA_OUT_WIDTH-1:0] fifo_wide_din = {din_r, din};

    if (Impl == prim_pkg::ImplGeneric) begin : gen_generic
        wire fifo_wready;
        wire fifo_rvalid;
        assign fifo_wr_full = ~fifo_wready;
        assign fifo_rd_empty = ~fifo_rvalid;

        prim_fifo_async #(
          .Width                (pDATA_OUT_WIDTH),
          .Depth                (FIFO_RD_DEPTH),
          // FWFT behaviour:
          .OutputZeroIfEmpty    (0),
          .OutputZeroIfInvalid  (0)
        ) U_fifo (
          // write port
          .clk_wr_i             (fifo_wr_clk),
          .rst_wr_ni            (~fifo_arst),
          .wvalid_i             (fifo_wide_write),
          .wready_o             (fifo_wready),
          .wdata_i              (fifo_wide_din),
          .wdepth_o             (),

          // read port
          .clk_rd_i             (fifo_rd_clk),
          .rst_rd_ni            (~fifo_arst),
          .rvalid_o             (fifo_rvalid),
          .rready_i             (fifo_rd_ena),
          .rdata_o              (dout),
          .rdepth_o             ()
        );

    end 
    else if (Impl == prim_pkg::ImplXilinx) begin : gen_xilinx

        // to avoid timing violations on reset net:
        wire fifo_arst_wsync;
        hbmc_arst_sync # (
            .C_SYNC_STAGES ( 3 )
        ) hbmc_arst_sync_inst (
            .clk  ( fifo_wr_clk   ),
            .arst ( fifo_arst     ),
            .rst  ( fifo_arst_wsync )
        );

        // NOTE: in theory it should be possible to instantiate xpm_fifo_async
        // with READ_DATA_WIDTH != WRITE_DATA_WIDTH (and avoid the *wide*
        // logic above) but the resulting implementation doesn't work? This
        // works, so didn't chase it down.
        xpm_fifo_async #(
            .CDC_SYNC_STAGES            (2),
            .DOUT_RESET_VALUE           ("0"),
            .ECC_MODE                   ("no_ecc"),
            .FIFO_MEMORY_TYPE           ("auto"),
            .FIFO_READ_LATENCY          (0),
            .FIFO_WRITE_DEPTH           (256),
            .FULL_RESET_VALUE           (0),
            .PROG_EMPTY_THRESH          (10),
            .PROG_FULL_THRESH           (10),
            .RD_DATA_COUNT_WIDTH        (1),
            .READ_DATA_WIDTH            (pDATA_OUT_WIDTH),
            .READ_MODE                  ("fwft"),
            .RELATED_CLOCKS             (0),
            .SIM_ASSERT_CHK             (0),
            .USE_ADV_FEATURES           ("0000"),
            .WAKEUP_TIME                (0),
            .WRITE_DATA_WIDTH           (pDATA_OUT_WIDTH),
            .WR_DATA_COUNT_WIDTH        (1)
        ) U_fifo (
            .almost_empty               (),
            .almost_full                (),
            .data_valid                 (),
            .dbiterr                    (),
            .dout                       (dout),
            .empty                      (fifo_rd_empty),
            .full                       (fifo_wr_full),
            .overflow                   (),
            .prog_empty                 (),
            .prog_full                  (),
            .rd_data_count              (),
            .rd_rst_busy                (),
            .sbiterr                    (),
            .underflow                  (),
            .wr_ack                     (),
            .wr_data_count              (),
            .wr_rst_busy                (),
            .din                        (fifo_wide_din),
            .injectdbiterr              (),
            .injectsbiterr              (),
            .rd_clk                     (fifo_rd_clk),
            .rd_en                      (fifo_rd_ena),
            .rst                        (fifo_arst_wsync),
            .sleep                      (1'b0),
            .wr_clk                     (fifo_wr_clk),
            .wr_en                      (fifo_wide_write)
        );

    end 
    else begin : gen_failure
        // TODO: Find code that works across tools and causes a compile failure
    end

`endif
    
endmodule

/*----------------------------------------------------------------------------------------------------------------------------*/

`default_nettype wire
