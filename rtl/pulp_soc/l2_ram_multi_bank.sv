// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "soc_mem_map.svh"

module l2_ram_multi_bank #(
   parameter NB_BANKS                   = 4,
   parameter int unsigned BANK_SIZE_INTL_SRAM = 32768 //Number of 32-bit words
) (
   input logic             clk_i,
   input logic             rst_ni,
   input logic             init_ni,
   input logic             test_mode_i,
   XBAR_TCDM_BUS.Slave     mem_slave[NB_BANKS],
   XBAR_TCDM_BUS.Slave     mem_pri_slave[2]
);
    localparam int unsigned BANK_SIZE_PRI0       = 8192; //Number of 32-bit words
    localparam int unsigned BANK_SIZE_PRI1       = 8192; //Number of 32-bit words

    //Derived parameters
    localparam int unsigned INTL_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_INTL_SRAM);
    localparam int unsigned PRI0_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_PRI0);
    localparam int unsigned PRI1_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_PRI1);

    //Used in testbenches



    //INTERLEAVED Memory
    logic [31:0]           interleaved_addresses[NB_BANKS];
    for(genvar i=0; i<NB_BANKS; i++) begin : CUTS
        //Perform TCDM handshaking for constant 1 cycle latency
        //assign mem_slave[i].gnt = mem_slave[i].req;
        assign mem_slave[i].r_opc = 1'b0;
        always_ff @(posedge clk_i, negedge rst_ni) begin
            if (!rst_ni) begin
                mem_slave[i].r_valid <= 1'b0;
            end else begin
              mem_slave[i].r_valid <= mem_slave[i].req && mem_slave[i].gnt;
            end
        end
       //Remove Address offset
       assign interleaved_addresses[i] = mem_slave[i].add - `SOC_MEM_MAP_TCDM_START_ADDR;








       ecc_sram_wrap #(
                       .BankSize  ( BANK_SIZE_INTL_SRAM )
                       ) bank_i (
                                 .clk_i,
                                 .rst_ni,
                                 .tcdm_req_i   (  mem_slave[i].req                                  ),
                                 .tcdm_wen_i    ( mem_slave[i].wen                                  ),
                                 .tcdm_add_i  (  interleaved_addresses[i]),
                                 // and bank selection (log2(NB_BANKS) bits)
                                 .tcdm_wdata_i (  mem_slave[i].wdata                                ),
                                 .tcdm_be_i    (  mem_slave[i].be                                   ),
                                 .tcdm_rdata_o (  mem_slave[i].r_rdata                              ),
                                 .tcdm_gnt_o   (  mem_slave[i].gnt                                  )
                                 );



   end

    // PRIVATE BANK0
    //Perform TCDM handshaking for constant 1 cycle latency

    assign mem_pri_slave[0].r_opc = 1'b0;
    always_ff @(posedge clk_i, negedge rst_ni) begin
        if (!rst_ni) begin
            mem_pri_slave[0].r_valid <= 1'b0;
        end else begin
          mem_pri_slave[0].r_valid <= mem_pri_slave[0].req && mem_pri_slave[0].gnt;
        end
    end
    //Remove Address offset
    logic [31:0] pri0_address;
    assign pri0_address = mem_pri_slave[0].add - `SOC_MEM_MAP_PRIVATE_BANK0_START_ADDR;






    ecc_sram_wrap #(
                    .BankSize  ( BANK_SIZE_PRI0 )
                    ) bank_sram_pri0_i (
                              .clk_i,
                              .rst_ni,
                              .tcdm_req_i   ( mem_pri_slave[0].req                                  ),
                              .tcdm_wen_i    ( mem_pri_slave[0].wen                                 ),
                              .tcdm_add_i  (  pri0_address),
                              // and bank selection (log2(NB_BANKS) bits)
                              .tcdm_wdata_i (  mem_pri_slave[0].wdata                               ),
                              .tcdm_be_i    ( mem_pri_slave[0].be                                     ),
                              .tcdm_rdata_o (  mem_pri_slave[0].r_rdata                              ),
                              .tcdm_gnt_o   (   mem_pri_slave[0].gnt                                 )
                              );





    // PRIVATE BANK1
    //Perform TCDM handshaking for constant 1 cycle latency
    //assign mem_pri_slave[1].gnt = mem_pri_slave[1].req;
    assign mem_pri_slave[1].r_opc = 1'b0;
    always_ff @(posedge clk_i, negedge rst_ni) begin
        if (!rst_ni) begin
            mem_pri_slave[1].r_valid <= 1'b0;
        end else begin
          mem_pri_slave[1].r_valid <= mem_pri_slave[1].req && mem_pri_slave[1].gnt;
        end
    end
    //Remove Address offset
    logic [31:0] pri1_address;
    assign pri1_address = mem_pri_slave[1].add - `SOC_MEM_MAP_PRIVATE_BANK1_START_ADDR;


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // tc_sram #(                                                                                                                //
  //           .NumWords  ( BANK_SIZE_PRI1 ),                                                                                  //
  //           .DataWidth ( 32             ),                                                                                  //
  //           .NumPorts  ( 1              ),                                                                                  //
  //           .Latency   ( 1              )                                                                                   //
  //           ) bank_sram_pri1_i (                                                                                            //
  //                               .clk_i,                                                                                     //
  //                               .rst_ni,                                                                                    //
  //                               .req_i   (  mem_pri_slave[1].req                  ),                                        //
  //                               .we_i    ( ~mem_pri_slave[1].wen                  ),                                        //
  //                               .addr_i  (  pri1_address[PRI1_MEM_ADDR_WIDTH+1:2] ), //Convert from byte to word addressing //
  //                               .wdata_i (  mem_pri_slave[1].wdata                ),                                        //
  //                               .be_i    (  mem_pri_slave[1].be                   ),                                        //
  //                               .rdata_o (  mem_pri_slave[1].r_rdata              )                                         //
  //                               );                                                                                          //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





    ecc_sram_wrap #(
                    .BankSize  ( BANK_SIZE_PRI1 )
                    ) bank_sram_pri1_i (
                              .clk_i(clk_i),
                              .rst_ni(rst_ni),
                              .tcdm_req_i   ( mem_pri_slave[1].req                                  ),
                              .tcdm_wen_i    ( mem_pri_slave[1].wen                                 ),
                              .tcdm_add_i  (  pri1_address),
                              // and bank selection (log2(NB_BANKS) bits)
                              .tcdm_wdata_i (  mem_pri_slave[1].wdata                               ),
                              .tcdm_be_i    ( mem_pri_slave[1].be                                     ),
                              .tcdm_rdata_o (  mem_pri_slave[1].r_rdata                              ),
                              .tcdm_gnt_o   (   mem_pri_slave[1].gnt                                 )
                              //.tcdm_gnt_o ()
                              );





endmodule // l2_ram_multi_bank
