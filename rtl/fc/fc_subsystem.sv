// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "register_interface/typedef.svh"
// Peripheral communication signals


module fc_subsystem #(
    parameter CORE_TYPE           = 0,
    parameter USE_FPU             = 1,
    parameter USE_HWPE            = 1,
    parameter N_EXT_PERF_COUNTERS = 1,
    parameter EVENT_ID_WIDTH      = 8,
    parameter PER_ID_WIDTH        = 32,
    parameter NB_HWPE_PORTS       = 4,
    parameter PULP_SECURE         = 1,
    parameter TB_RISCV            = 0,
    parameter CORE_ID             = 4'h0,
    parameter CLUSTER_ID          = 6'h1F,
    parameter USE_ZFINX           = 1
)
(
    input logic                      clk_i,
    input logic                      rst_ni,
    input logic                      test_en_i,

    XBAR_TCDM_BUS.Master l2_data_master,
    XBAR_TCDM_BUS.Master l2_instr_master,
    XBAR_TCDM_BUS.Master l2_hwpe_master [NB_HWPE_PORTS-1:0],
    APB_BUS.Slave apb_slave_eu,
    APB_BUS.Slave apb_slave_hwpe,
    APB_BUS.Slave apb_slave_tcls,


    input logic                      fetch_en_i,
    input logic [31:0]               boot_addr_i,
    input logic                      debug_req_i,

    input logic                      event_fifo_valid_i,
    output logic                     event_fifo_fulln_o,
    input logic [EVENT_ID_WIDTH-1:0] event_fifo_data_i, // goes indirectly to core interrupt
    input logic [31:0]               events_i, // goes directly to core interrupt, should be called irqs
    output logic [1:0]               hwpe_events_o,

    output logic                     supervisor_mode_o
);

    import ctcls_manager_reg_pkg::* ;
    `REG_BUS_TYPEDEF_ALL(tcls, logic[31:0], logic[31:0], logic[3:0])

    localparam USE_IBEX   = CORE_TYPE == 1 || CORE_TYPE == 2;
    localparam IBEX_RV32M = CORE_TYPE == 1 ? ibex_pkg::RV32MFast : ibex_pkg::RV32MNone;
    localparam IBEX_RV32E = CORE_TYPE == 2;

    // Interrupt signals
    logic        core_irq_req   ;
    logic        core_irq_sec   ;
    logic [4:0]  core_irq_id    ;
    logic [4:0]  core_irq_ack_id;
    logic        core_irq_ack   ;
    logic [31:0] core_irq_x;

    // Boot address, core id, cluster id, fethc enable and core_status
    logic [31:0] boot_addr        ;
    logic        fetch_en_int     ;
    logic        core_busy_int    ;
    logic        perf_counters_int;
    logic [31:0] hart_id;

    //EU signals
    logic intc_clock_en;
    logic fetch_en_eu  ;

    //Core Instr Bus
    logic [31:0] core_instr_addr, core_instr_rdata;
    logic        core_instr_req, core_instr_gnt, core_instr_rvalid, core_instr_err;

    //Core Data Bus
    logic [31:0] core_data_addr, core_data_rdata, core_data_wdata;
    logic        core_data_req, core_data_gnt, core_data_rvalid, core_data_err;
    logic        core_data_we  ;
    logic [ 3:0]  core_data_be ;
    logic is_scm_instr_req, is_scm_data_req;

    assign perf_counters_int = 1'b0;
    assign fetch_en_int      = fetch_en_eu & fetch_en_i;

    assign hart_id = {21'b0, CLUSTER_ID[5:0], 1'b0, CORE_ID[3:0]};

    XBAR_TCDM_BUS core_data_bus ();
    XBAR_TCDM_BUS core_instr_bus ();

    //********************************************************
    //************ CORE DEMUX (TCDM vs L2) *******************
    //********************************************************
    assign l2_data_master.req    = core_data_req;
    assign l2_data_master.add    = core_data_addr;
    assign l2_data_master.wen    = ~core_data_we;
    assign l2_data_master.wdata  = core_data_wdata;
    assign l2_data_master.be     = core_data_be;
    assign core_data_gnt         = l2_data_master.gnt;
    assign core_data_rvalid      = l2_data_master.r_valid;
    assign core_data_rdata       = l2_data_master.r_rdata;
    assign core_data_err         = l2_data_master.r_opc;


    assign l2_instr_master.req   = core_instr_req;
    assign l2_instr_master.add   = core_instr_addr;
    assign l2_instr_master.wen   = 1'b1;
    assign l2_instr_master.wdata = '0;
    assign l2_instr_master.be    = 4'b1111;
    assign core_instr_gnt        = l2_instr_master.gnt;
    assign core_instr_rvalid     = l2_instr_master.r_valid;
    assign core_instr_rdata      = l2_instr_master.r_rdata;
    assign core_instr_err        = l2_instr_master.r_opc;


    //********************************************************
    //************ TCLS SIGNALS ******************************
    //********************************************************

    tcls_req_t tcls_req;
    tcls_rsp_t tcls_rsp;



    logic [2:0] red_rst_n;
    logic [2:0][ 31:0] red_hart_id;
    logic [2:0]       red_fetch_en_int;
    logic [2:0][ 31:0] red_boot_addr;

    logic [2:0][ 31:0] red_irq_x;
    logic [2:0]        red_irq_x_ack;
    logic [2:0][ 4:0]  red_irq_x_ack_id;

    logic [2:0]        red_instr_err;
    logic [2:0]        red_instr_req;
    logic [2:0]        red_instr_gnt;
    logic [2:0][ 31:0] red_instr_addr;
    logic [2:0][ 31:0] red_instr_rdata;
    logic [2:0]        red_instr_rvalid;

    logic [2:0]        red_debug_req;

    logic [2:0]        red_data_req;
    logic [2:0][ 31:0] red_data_addr;
    logic [2:0]        red_data_we;
    logic [2:0][ 31:0] red_data_wdata;
    logic [2:0][ 3:0]  red_data_be;
    logic [2:0]        red_data_gnt;
    logic [2:0][ 31:0] red_data_rdata;
    logic [2:0]        red_data_rvalid;
    logic [2:0]        red_data_err;

    logic [2:0][4:0]   red_perf_counters;

    logic              tcls_single_core_mismatch;

    TCLS_unit #(
                   .NExtPerfCounters(5)
    ) i_TCLS_unit (
                    .clk_i(clk_i),
                    .rst_ni(rst_ni),

                    .speriph_request(tcls_req),
                    .speriph_response(tcls_rsp),
                    .tcls_triple_core_mismatch(),
                    .tcls_single_core_mismatch(tcls_single_core_mismatch),

  // Ports to connect Interconnect/rest of system
                    .intc_hart_id_i(hart_id),

                    .intc_fetch_en_i(fetch_en_int),
                    .intc_boot_addr_i(boot_addr),

                    .intc_irq_x_i(core_irq_x),
                    .intc_irq_x_ack_o(core_irq_ack),
                    .intc_irq_x_ack_id_o(core_irq_ack_id),
                    .intc_instr_err_i(core_instr_err),
                    .intc_instr_req_o(core_instr_req),
                    .intc_instr_gnt_i(core_instr_gnt),
                    .intc_instr_addr_o(core_instr_addr),
                    .intc_instr_rdata_i(core_instr_rdata),
                    .intc_instr_rvalid_i(core_instr_rvalid),

                    .intc_debug_req_i(debug_req_i),

                    .intc_data_req_o(core_data_req),
                    .intc_data_addr_o(core_data_addr),
                    .intc_data_we_o(core_data_we),
                    .intc_data_wdata_o(core_data_wdata),
                    .intc_data_be_o(core_data_be),
                    .intc_data_gnt_i(core_data_gnt),
                    .intc_data_rdata_i(core_data_rdata),
                    .intc_data_rvalid_i(core_data_rvalid),
                    .intc_data_err_i(core_data_err),

                    .intc_perf_counters_i({ {16 - N_EXT_PERF_COUNTERS {'0}}, perf_counters_int }),

  // Ports to connect Cores
                    .core_rst_no(red_rst_n),

                    .core_hart_id_o(red_hart_id),


                    .core_fetch_en_o(red_fetch_en_int),
                    .core_boot_addr_o(red_boot_addr),

                    .core_irq_x_o(red_irq_x),
                    .core_irq_x_ack_i(red_irq_x_ack),
                    .core_irq_x_ack_id_i(red_irq_x_ack_id),

                    .core_instr_err_o(red_instr_err),
                    .core_instr_req_i(red_instr_req),
                    .core_instr_gnt_o(red_instr_gnt),
                    .core_instr_addr_i(red_instr_addr),
                    .core_instr_rdata_o(red_instr_rdata),
                    .core_instr_rvalid_o(red_instr_rvalid),

                    .core_debug_req_o(red_debug_req),

                    .core_data_req_i(red_data_req),
                    .core_data_addr_i(red_data_addr),
                    .core_data_we_i(red_data_we),
                    .core_data_wdata_i(red_data_wdata),
                    .core_data_be_i(red_data_be),
                    .core_data_gnt_o(red_data_gnt),
                    .core_data_rdata_o(red_data_rdata),
                    .core_data_rvalid_o(red_data_rvalid),
                    .core_data_err_o(red_data_err),

                    .core_perf_counters_o(red_perf_counters)

                    );



    //********************************************************
    //************ RISCV CORE ********************************
    //********************************************************

  assign boot_addr = boot_addr_i & 32'hFFFFFF00; // RI5CY expects 0x80 offset, Ibex expects 0x00 offset (adds reset offset 0x80 internally)
  for (genvar i = 0; i < 3; i++) begin :gen_core_inst
`ifdef VERILATOR
    ibex_core #(
`elsif TRACE_EXECUTION
    ibex_core_tracing #(
`else
    ibex_core #(
`endif
                .PMPEnable        ( 1'b0                ),
                .PMPGranularity   ( 0                   ),
                .PMPNumRegions    ( 4                   ),
                .MHPMCounterNum   ( 10                  ),
                .MHPMCounterWidth ( 40                  ),
                .RV32E            ( IBEX_RV32E          ),
                .RV32M            ( IBEX_RV32M          ),
                .RV32B            ( ibex_pkg::RV32BNone ),
                .RegFile          ( ibex_pkg::RegFileFF ),
                .BranchTargetALU  ( 1'b0                ),
                .WritebackStage   ( 1'b0                ),
                .ICache           ( 1'b0                ),
                .ICacheECC        ( 1'b0                ),
                .BranchPredictor  ( 1'b0                ),
                .DbgTriggerEn     ( 1'b1                ),
                .DbgHwBreakNum    ( 1                   ),
                .SecureIbex       ( 1'b0                ),
                .DmHaltAddr       ( 32'h1A110800        ),
                .DmExceptionAddr  ( 32'h1A110808        )
                ) i_lFC_CORE (
                              .clk_i                 ( clk_i             ),
                              .rst_ni                ( red_rst_n[i]     ),

                              .test_en_i             ( test_en_i         ),

                              .hart_id_i             ( red_hart_id[i]           ),
                              .boot_addr_i           ( red_boot_addr[i]         ),

                              // Instruction Memory Interface:  Interface to Instruction Logaritmic interconnect: Req->grant handshake
                              .instr_addr_o          ( red_instr_addr[i]   ),
                              .instr_req_o           ( red_instr_req[i]    ),
                              .instr_rdata_i         ( red_instr_rdata[i]  ),
                              .instr_gnt_i           ( red_instr_gnt[i]    ),
                              .instr_rvalid_i        ( red_instr_rvalid[i] ),
                              .instr_err_i           ( red_instr_err[i]    ),

                              // Data memory interface:
                              .data_addr_o           ( red_data_addr[i]    ),
                              .data_req_o            ( red_data_req[i]     ),
                              .data_be_o             ( red_data_be[i]      ),
                              .data_rdata_i          ( red_data_rdata[i]   ),
                              .data_we_o             ( red_data_we[i]      ),
                              .data_gnt_i            ( red_data_gnt[i]     ),
                              .data_wdata_o          ( red_data_wdata[i]   ),
                              .data_rvalid_i         ( red_data_rvalid[i]  ),
                              .data_err_i            ( red_data_err[i]     ),

                              .irq_software_i        ( 1'b0              ),
                              .irq_timer_i           ( 1'b0              ),
                              .irq_external_i        ( 1'b0              ),
                              .irq_fast_i            ( 15'b0             ),
                              .irq_nm_i              ( 1'b0              ),

                              .irq_x_i               ( red_irq_x[i]        ),
                              .irq_x_ack_o           ( red_irq_x_ack[i]      ),
                              .irq_x_ack_id_o        ( red_irq_x_ack_id[i]   ),

                              //.external_perf_i       ( { {16 - N_EXT_PERF_COUNTERS {'0}}, perf_counters_int } ),
                              .external_perf_i       ( red_perf_counters[i]     ),
                              .debug_req_i           ( red_debug_req[i]       ),

                              .fetch_enable_i        ( red_fetch_en_int[i]      ),
                              .alert_minor_o         (                   ),
                              .alert_major_o         (                   ),
                              .core_sleep_o          (                   )
                              );
  end // for loop


  // APB2REG converter

  REG_BUS #(.ADDR_WIDTH(32), .DATA_WIDTH(32))reg_tcls_bus ();


  apb_to_reg apb2reg (
                      .clk_i(clk_i),
                      .rst_ni(rst_ni),
                      .penable_i(apb_slave_tcls.penable),
                      .pwrite_i(apb_slave_tcls.pwrite),
                      .paddr_i(apb_slave_tcls.paddr),
                      .psel_i(apb_slave_tcls.psel),
                      .pwdata_i(apb_slave_tcls.pwdata),
                      .prdata_o(apb_slave_tcls.prdata),
                      .pready_o(apb_slave_tcls.pready),
                      .pslverr_o(apb_slave_tcls.pslverr),
                      .reg_o(reg_tcls_bus)
                      );

  assign tcls_req = '{ addr: reg_tcls_bus.addr,
               write: reg_tcls_bus.write,
               wdata: reg_tcls_bus.wdata,
               wstrb: reg_tcls_bus.wstrb,
               valid: reg_tcls_bus.valid
               };
  assign reg_tcls_bus.rdata = tcls_rsp.rdata;
  assign reg_tcls_bus.error = tcls_rsp.error;
  assign reg_tcls_bus.ready = tcls_rsp.ready;


    assign supervisor_mode_o = 1'b1;

    generate
    if ( USE_IBEX == 1) begin : convert_irqs
    // Ibex supports 32 additional fast interrupts and reads the interrupt lines directly.
    // Convert ID back to interrupt lines
    always_comb begin : gen_core_irq_x
        core_irq_x = '0;
        if (core_irq_req) begin
            core_irq_x[core_irq_id] = 1'b1;
        end
    end

    end
    endgenerate



    logic [31:0] events;
    assign events [24:0] = events_i[25:0];
    assign events [25] = tcls_single_core_mismatch;
    assign events [31:27] = events_i[31:27];





    apb_interrupt_cntrl #(.PER_ID_WIDTH(PER_ID_WIDTH)) fc_eu_i (
        .clk_i              ( clk_i              ),
        .rst_ni             ( rst_ni             ),
        .test_mode_i        ( test_en_i          ),
        .events_i           ( events             ),
        .event_fifo_valid_i ( event_fifo_valid_i ),
        .event_fifo_fulln_o ( event_fifo_fulln_o ),
        .event_fifo_data_i  ( event_fifo_data_i  ),
        .core_secure_mode_i ( 1'b0               ),
        .core_irq_id_o      ( core_irq_id        ),
        .core_irq_req_o     ( core_irq_req       ),
        .core_irq_ack_i     ( core_irq_ack       ),
        .core_irq_id_i      ( core_irq_ack_id    ),
        .core_irq_sec_o     ( /* SECURE IRQ */   ),
        .core_clock_en_o    ( core_clock_en      ),
        .fetch_en_o         ( fetch_en_eu        ),
        .apb_slave          ( apb_slave_eu       )
    );


    generate
    if(USE_HWPE) begin : fc_hwpe_gen
        fc_hwpe #(
            .N_MASTER_PORT ( NB_HWPE_PORTS ),
            .ID_WIDTH      ( 2             )
        ) i_fc_hwpe (
            .clk_i             ( clk_i          ),
            .rst_ni            ( rst_ni         ),
            .test_mode_i       ( test_en_i      ),
            .hwacc_xbar_master ( l2_hwpe_master ),
            .hwacc_cfg_slave   ( apb_slave_hwpe ),
            .evt_o             ( hwpe_events_o  ),
            .busy_o            (                )
        );
    end
    else begin : no_fc_hwpe_gen
        assign hwpe_events_o = '0;
        assign apb_slave_hwpe.prdata  = '0;
        assign apb_slave_hwpe.pready  = '0;
        assign apb_slave_hwpe.pslverr = '0;
        for(genvar ii=0; ii<NB_HWPE_PORTS; ii++) begin : no_fc_hwpe_gen_loop
            assign l2_hwpe_master[ii].req   = '0;
            assign l2_hwpe_master[ii].wen   = '0;
            assign l2_hwpe_master[ii].wdata = '0;
            assign l2_hwpe_master[ii].be    = '0;
            assign l2_hwpe_master[ii].add   = '0;
        end
    end
    endgenerate

endmodule
