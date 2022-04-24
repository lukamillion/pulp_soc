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
`include "register_interface/typedef.svh"
`include "register_interface/assign.svh"

module l2_ram_multi_bank #(
   parameter NB_BANKS                   = 4,
   parameter int unsigned BANK_SIZE_INTL_SRAM = 32768 //Number of 32-bit words
) (
   input logic             clk_i,
   input logic             rst_ni,
   input logic             init_ni,
   input logic             test_mode_i,
   XBAR_TCDM_BUS.Slave     mem_slave[NB_BANKS],
   XBAR_TCDM_BUS.Slave     mem_pri_slave[2],
   APB_BUS.Slave apb_slave_ecc
);

    import ecc_manager_reg_pkg::* ;
    `REG_BUS_TYPEDEF_ALL(ecc, logic[31:0], logic[31:0], logic[3:0])

    ecc_req_t ecc_req;
    ecc_rsp_t ecc_rsp;

    localparam int unsigned BANK_SIZE_PRI0       = 8192; //Number of 32-bit words
    localparam int unsigned BANK_SIZE_PRI1       = 8192; //Number of 32-bit words

    //Derived parameters
    localparam int unsigned INTL_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_INTL_SRAM);
    localparam int unsigned PRI0_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_PRI0);
    localparam int unsigned PRI1_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_PRI1);

    //Used in testbenches



    //INTERLEAVED Memory
    logic [31:0]           interleaved_addresses[NB_BANKS];
    logic [NB_BANKS-1:0][1:0]            banks_error;
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
                                 .clk_i(clk_i),
                                 .rst_ni(rst_ni),
                                 .tcdm_req_i   (  mem_slave[i].req                                  ),
                                 .tcdm_wen_i    ( mem_slave[i].wen                                  ),
                                 .tcdm_add_i  (  interleaved_addresses[i]),
                                 // and bank selection (log2(NB_BANKS) bits)
                                 .tcdm_wdata_i (  mem_slave[i].wdata                                ),
                                 .tcdm_be_i    (  mem_slave[i].be                                   ),
                                 .tcdm_rdata_o (  mem_slave[i].r_rdata                              ),
                                 .tcdm_gnt_o   (  mem_slave[i].gnt                                  ),
                                 .error_o      (  banks_error[i]                                    )
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



  // register connections



  logic [1:0]    private0_error;



    ecc_sram_wrap #(
                    .BankSize  ( BANK_SIZE_PRI0 )
                    ) bank_sram_pri0_i (
                                        .clk_i(clk_i),
                                        .tcdm_req_i   ( mem_pri_slave[0].req                                  ),
                                        .tcdm_wen_i    ( mem_pri_slave[0].wen                                 ),
                                        .tcdm_add_i  (  pri0_address),
                                        // and bank selection (log2(NB_BANKS) bits)
                                        .tcdm_wdata_i (  mem_pri_slave[0].wdata                                ),
                                        .tcdm_be_i    ( mem_pri_slave[0].be                                    ),
                                        .tcdm_rdata_o (  mem_pri_slave[0].r_rdata                              ),
                                        .tcdm_gnt_o   (   mem_pri_slave[0].gnt                                 ),
                                        .error_o      ( private0_error                                         )
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



  logic [1:0]    private1_error;

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
                                      .tcdm_gnt_o   (   mem_pri_slave[1].gnt                                 ),
                                      .error_o      ( private1_error                                         )
                                      );


// Register Interface

  REG_BUS #(.ADDR_WIDTH(32), .DATA_WIDTH(32)) reg_ecc_bus ();


  apb_to_reg apb2reg (
                      .clk_i(clk_i),
                      .rst_ni(rst_ni),
                      .penable_i(apb_slave_ecc.penable),
                      .pwrite_i(apb_slave_ecc.pwrite),
                      .paddr_i(apb_slave_ecc.paddr),
                      .psel_i(apb_slave_ecc.psel),
                      .pwdata_i(apb_slave_ecc.pwdata),
                      .prdata_o(apb_slave_ecc.prdata),
                      .pready_o(apb_slave_ecc.pready),
                      .pslverr_o(apb_slave_ecc.pslverr),
                      .reg_o(reg_ecc_bus)
                      );

  `REG_BUS_ASSIGN_TO_REQ(ecc_req, reg_ecc_bus);
  `REG_BUS_ASSIGN_FROM_RSP(reg_ecc_bus, ecc_rsp);


  ecc_manager_reg2hw_t reg2hw;
  ecc_manager_hw2reg_t hw2reg;

  ecc_manager_reg_top #(
                        .reg_req_t ( ecc_req_t ),
                        .reg_rsp_t ( ecc_rsp_t )
                        ) i_registers (
                                       .clk_i     ( clk_i            ),
                                       .rst_ni    ( rst_ni           ),
                                       .reg_req_i ( ecc_req  ),
                                       .reg_rsp_o ( ecc_rsp ),
                                       .reg2hw    ( reg2hw           ),
                                       .hw2reg    ( hw2reg           ),
                                       .devmode_i ( '0               )
                                       );

  assign bank_be = 1'b1;

  assign hw2reg.private0.d = reg2hw.private0.q + 1;
  assign hw2reg.private0.de = private0_error[0];

  assign hw2reg.private1.d = reg2hw.private1.q + 1;
  assign hw2reg.private1.de = private1_error[0];

  assign hw2reg.cuts0.d = reg2hw.cuts0.q + 1;
  assign hw2reg.cuts0.de = banks_error[0][0];

  assign hw2reg.cuts1.d = reg2hw.cuts1.q + 1;
  assign hw2reg.cuts1.de = banks_error[1][0];

  assign hw2reg.cuts2.d = reg2hw.cuts2.q + 1;
  assign hw2reg.cuts2.de = banks_error[2][0];

  assign hw2reg.cuts3.d = reg2hw.cuts3.q + 1;
  assign hw2reg.cuts3.de = banks_error[3][0];

endmodule // l2_ram_multi_bank
