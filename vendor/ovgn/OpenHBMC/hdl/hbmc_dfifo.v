/* 
 * ----------------------------------------------------------------------------
 *  Project:  OpenHBMC
 *  Filename: hbmc_dfifo.v
 *  Purpose:  Downstream data FIFO. Stores data to be written to the memory.
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


module hbmc_dfifo #
(
    parameter integer DATA_WIDTH = 32
)
(
    input   wire                            fifo_arst,
    
    input   wire                            fifo_wr_clk,
    input   wire    [DATA_WIDTH - 1:0]      fifo_wr_din,
    input   wire    [DATA_WIDTH/8 - 1:0]    fifo_wr_strb,
    input   wire                            fifo_wr_ena,
    output  wire                            fifo_wr_full,
    
    input   wire                            fifo_rd_clk,
    output  wire    [15:0]                  fifo_rd_dout,
    output  wire    [1:0]                   fifo_rd_strb,
    input   wire                            fifo_rd_ena,
    output  wire                            fifo_rd_empty
);

    wire    [17:0]  dout;
    
`ifdef XILINX_HBMC_DFIFO
    assign  fifo_rd_dout = dout[15:0];
    assign  fifo_rd_strb = dout[17:16];

    generate
        case (DATA_WIDTH)
            
            16: begin : dFIFO_18b_18b_512w
                
                wire    [17:0]  din = {fifo_wr_strb[1:0], fifo_wr_din[15:0]};
                
                fifo_18b_18b_512w
                fifo_18b_18b_512w_inst
                (
                    .rst            ( fifo_arst     ),  // input rst
                    
                    .wr_clk         ( fifo_wr_clk   ),  // input wr_clk
                    .wr_en          ( fifo_wr_ena   ),  // input wr_en
                    .full           ( fifo_wr_full  ),  // output full
                    .din            ( din           ),  // input [17 : 0] din
                    
                    .rd_clk         ( fifo_rd_clk   ),  // input rd_clk
                    .rd_data_count  ( /*---NC---*/  ),  // output [8 : 0] rd_data_count
                    .rd_en          ( fifo_rd_ena   ),  // input rd_en
                    .empty          ( fifo_rd_empty ),  // output empty
                    .dout           ( dout          )   // output [17 : 0] dout
                );
            end

            /*--------------------------------------------------------------------*/
            
            32: begin : dFIFO_36b_18b_512w
                
                wire    [35:0]  din =   {
                                            fifo_wr_strb[1:0], fifo_wr_din[15:0],
                                            fifo_wr_strb[3:2], fifo_wr_din[31:16]
                                        };
                
                fifo_36b_18b_512w
                fifo_36b_18b_512w_inst
                (
                    .rst    ( fifo_arst     ),  // input rst
                    
                    .wr_clk ( fifo_wr_clk   ),  // input wr_clk
                    .wr_en  ( fifo_wr_ena   ),  // input wr_en
                    .full   ( fifo_wr_full  ),  // output full
                    .din    ( din           ),  // input [35 : 0] din
                    
                    .rd_clk ( fifo_rd_clk   ),  // input rd_clk
                    .rd_en  ( fifo_rd_ena   ),  // input rd_en
                    .empty  ( fifo_rd_empty ),  // output empty
                    .dout   ( dout          )   // output [17 : 0] dout
                );
            end

            /*--------------------------------------------------------------------*/
            
            64: begin : dFIFO_72b_18b_512w
                
                wire    [71:0]  din =   {
                                            fifo_wr_strb[1:0], fifo_wr_din[15:0],
                                            fifo_wr_strb[3:2], fifo_wr_din[31:16],
                                            fifo_wr_strb[5:4], fifo_wr_din[47:32],
                                            fifo_wr_strb[7:6], fifo_wr_din[63:48]
                                        };
                
                fifo_72b_18b_512w
                fifo_72b_18b_512w_inst
                (
                    .rst    ( fifo_arst     ),  // input rst
                    
                    .wr_clk ( fifo_wr_clk   ),  // input wr_clk
                    .wr_en  ( fifo_wr_ena   ),  // input wr_en
                    .full   ( fifo_wr_full  ),  // output full
                    .din    ( din           ),  // input [71 : 0] din
                    
                    .rd_clk ( fifo_rd_clk   ),  // input rd_clk
                    .rd_en  ( fifo_rd_ena   ),  // input rd_en
                    .empty  ( fifo_rd_empty ),  // output empty
                    .dout   ( dout          )   // output [17 : 0] dout
                );
            end
            
            /*--------------------------------------------------------------------*/
            
            default: begin
                INVALID_PARAMETER invalid_parameter_msg();
            end
            
        endcase
    endgenerate

`else
    // coding only the fifo_36b_18b_512w case that we need, for simplicity:
    localparam pDATA_IN_WIDTH = 36;
    localparam pDATA_OUT_WIDTH = 18;
    localparam  FIFO_RD_DEPTH = 512;
    wire fifo_wready;
    wire fifo_rvalid;

    assign fifo_wr_full = ~fifo_wready;
    assign fifo_rd_empty = ~fifo_rvalid;

    assign  fifo_rd_dout = dout[15:0];
    assign  fifo_rd_strb = dout[17:16];
    wire    [35:0]  din =   {
                                fifo_wr_strb[1:0], fifo_wr_din[15:0],
                                fifo_wr_strb[3:2], fifo_wr_din[31:16]
                            };
 
    // handle I/O width conversion:
    wire [pDATA_IN_WIDTH-1:0] dout_wide;
    reg  [pDATA_IN_WIDTH-1:0] dout_wide_r;
    reg wide_read_cnt = 1'b0; 
    wire fifo_wide_read = fifo_rd_ena & ~wide_read_cnt;
    assign dout = wide_read_cnt ? dout_wide_r[pDATA_OUT_WIDTH-1:0] : dout_wide[pDATA_OUT_WIDTH*2-1 : pDATA_OUT_WIDTH];
    always @(posedge fifo_rd_clk) begin
        if (fifo_rd_ena) begin
            wide_read_cnt <= ~wide_read_cnt;
            if (fifo_wide_read)
                dout_wide_r <= dout_wide;
        end
    end

    prim_fifo_async #(
      .Width                (pDATA_IN_WIDTH),
      .Depth                (FIFO_RD_DEPTH),
      // FWFT behaviour:
      .OutputZeroIfEmpty    (0),
      .OutputZeroIfInvalid  (0)
    ) U_lowrisc_fifo (
      // write port
      .clk_wr_i             (fifo_wr_clk),
      .rst_wr_ni            (~fifo_arst),
      .wvalid_i             (fifo_wr_ena),
      .wready_o             (fifo_wready),
      .wdata_i              (din),
      .wdepth_o             (),

      // read port
      .clk_rd_i             (fifo_rd_clk),
      .rst_rd_ni            (~fifo_arst),
      .rvalid_o             (fifo_rvalid),
      .rready_i             (fifo_wide_read),
      .rdata_o              (dout_wide),
      .rdepth_o             ()
    );
    
`endif

endmodule

/*----------------------------------------------------------------------------------------------------------------------------*/

`default_nettype wire
