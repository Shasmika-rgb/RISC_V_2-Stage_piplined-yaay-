// ID_Stage_rtl.sv  (modified for the 2-stage pipeline)
//
// Changes from the single-cycle version:
//   * The register file is now WRITTEN from stage 2: new inputs wb_en / wb_rd
//     (the write enable and destination of the instruction in stage 2) replace
//     the locally decoded reg_write / rd_sel on the write port.
//   * rs1_sel, rs2_sel, rd_sel and reg_write are now outputs, so the top level
//     can forward and carry them through the pipeline register.
//   * The stall input is removed (the pipeline never stalls).
// decoder, imm_generator, register_file and main_control are used unchanged.

module id_stage (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] instruction,
    // write-back port, driven by stage 2
    input  logic        wb_en,
    input  logic [4:0]  wb_rd,
    input  logic [31:0] wb_data,
    // decoded register numbers
    output logic [4:0]  rs1_sel,
    output logic [4:0]  rs2_sel,
    output logic [4:0]  rd_sel,
    // operands, immediate, control
    output logic [31:0] rs1_data,
    output logic [31:0] rs2_data,
    output logic [31:0] imm_out,
    output logic [6:0]  opcode_out_d,
    output logic [6:0]  imm11_5,
    output logic [2:0]  fn3_out_d,
    output logic        fn7_5,
    output logic [2:0]  aluop,
    output logic [1:0]  memtoreg,
    output logic        branch,
    output logic        mem_write,
    output logic        mem_read,
    output logic        alu_src,
    output logic        mux_inp,
    output logic        reg_write
);
    logic [11:0] imm;
    logic [19:0] imm_uj;

    decoder u_decoder (
        .instruction(instruction),
        .rs1(rs1_sel),
        .rs2(rs2_sel),
        .rd(rd_sel),
        .opcode(opcode_out_d),
        .imm11_5(imm11_5),
        .fn3(fn3_out_d),
        .imm(imm),
        .imm_uj(imm_uj),
        .fn7_5(fn7_5)
    );

    imm_generator u_imm_generator (
        .imm_input(imm),
        .fn3(fn3_out_d),
        .opcode(opcode_out_d),
        .imm_input_uj(imm_uj),
        .imm_output(imm_out)
    );

    // Read with this stage's rs1/rs2; write with stage 2's rd / enable / data.
    register_file u_register_file (
        .rs1_sel(rs1_sel),
        .rs2_sel(rs2_sel),
        .reg_write(wb_en),
        .clk(clk),
        .reset(reset),
        .wb_data(wb_data),
        .rd_sel(wb_rd),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );

    main_control u_main_control (
        .opcode(opcode_out_d),
        .fn3(fn3_out_d),
        .memtoreg(memtoreg),
        .branch(branch),
        .mux_inp(mux_inp),
        .memread(mem_read),
        .memwrite(mem_write),
        .alusrc(alu_src),
        .reg_write(reg_write),
        .aluop(aluop)
    );
endmodule
