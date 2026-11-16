// Pipelined_RISCV_rtl.sv
// Two-stage pipelined RV32I core built from the single-cycle stage modules.
//
//   Stage 1 (IF + ID)      : if_stage (PC), id_stage (decode, imm, control, reg read)
//   Stage 2 (EX + MEM + WB): ex_stage (ALU, branch/jump), data memory, wb_stage
//
// Hazards
//   * Data: stage 2's write-back value (ALU result, load data from mem_out, PC+4,
//     LUI/AUIPC value) is forwarded into stage 1's register read in the same
//     cycle, so the pipeline never stalls. x0 is never forwarded.
//   * Control: branches/jumps resolve in stage 2. A taken one redirects the PC
//     and the wrong-path instruction in stage 1 becomes a bubble (all control
//     signals zero, ex_valid = 0): no register write, no store, no redirect.
//
// Module and ports match the template spec (Pipelined_RISCV/README.md).

module pipelined_riscv (
    input  logic        clk,
    input  logic        reset,          // asynchronous, active high
    input  logic [31:0] instruction,    // combinational read of instruction memory at `address`
    input  logic [31:0] mem_out,        // combinational load data for alu_out / fn3 (already extended)
    output logic [31:0] address,        // PC of the instruction being fetched (stage 1)
    output logic [31:0] alu_out,        // stage 2: ALU result / data memory address
    output logic [31:0] rs2_data,       // stage 2: store data
    output logic        mem_write,      // stage 2: committed store (never for a squashed instruction)
    output logic        mem_read,       // stage 2: load
    output logic [2:0]  fn3             // stage 2: funct3 of the load/store
);

    // ------------------------------------------------------------------
    // Signals between the stages
    // ------------------------------------------------------------------
    logic        redirect;        // taken branch / jump in stage 2
    logic [31:0] pc_ex_out;       // its target (also AUIPC value)
    logic        and_out_ex;
    logic [31:0] pc_new_unused;

    logic        wb_en;
    logic [4:0]  wb_rd;
    logic [31:0] wb_data;

    // ==================================================================
    // Stage 1: IF + ID
    // ==================================================================
    if_stage u_if_stage (
        .clk(clk),
        .reset(reset),
        .stall(1'b0),
        .and_out(redirect),
        .pc_signed_offset(pc_ex_out),
        .address(address),
        .pc_new(pc_new_unused)
    );

    logic [4:0]  id_rs1_sel, id_rs2_sel, id_rd_sel;
    logic [31:0] id_rs1_rf, id_rs2_rf, id_imm;
    logic [6:0]  id_opcode, id_imm11_5;
    logic [2:0]  id_fn3, id_aluop;
    logic        id_fn7_5;
    logic [1:0]  id_memtoreg;
    logic        id_branch, id_mem_write, id_mem_read, id_alu_src, id_mux_inp, id_reg_write;

    id_stage u_id_stage (
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .wb_en(wb_en),
        .wb_rd(wb_rd),
        .wb_data(wb_data),
        .rs1_sel(id_rs1_sel),
        .rs2_sel(id_rs2_sel),
        .rd_sel(id_rd_sel),
        .rs1_data(id_rs1_rf),
        .rs2_data(id_rs2_rf),
        .imm_out(id_imm),
        .opcode_out_d(id_opcode),
        .imm11_5(id_imm11_5),
        .fn3_out_d(id_fn3),
        .fn7_5(id_fn7_5),
        .aluop(id_aluop),
        .memtoreg(id_memtoreg),
        .branch(id_branch),
        .mem_write(id_mem_write),
        .mem_read(id_mem_read),
        .alu_src(id_alu_src),
        .mux_inp(id_mux_inp),
        .reg_write(id_reg_write)
    );

    // Forwarding: stage 2 result -> stage 1 operands (wb_en is 0 for rd == x0)
    logic [31:0] id_rs1_val, id_rs2_val;
    assign id_rs1_val = (wb_en && (wb_rd == id_rs1_sel)) ? wb_data : id_rs1_rf;
    assign id_rs2_val = (wb_en && (wb_rd == id_rs2_sel)) ? wb_data : id_rs2_rf;

    // ==================================================================
    // Pipeline register: stage 1 -> stage 2
    // ==================================================================
    logic        ex_valid;
    logic [31:0] ex_pc, ex_rs1, ex_rs2, ex_imm;
    logic [4:0]  ex_rd;
    logic [6:0]  ex_opcode, ex_imm11_5;
    logic [2:0]  ex_fn3, ex_aluop;
    logic        ex_fn7_5;
    logic [1:0]  ex_memtoreg;
    logic        ex_branch, ex_mem_write, ex_mem_read, ex_alu_src, ex_mux_inp, ex_reg_write;

    // Bubble = everything cleared: opcode 0 and all control signals 0, so it can
    // never write a register, store, or redirect.
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            ex_valid     <= 1'b0;
            ex_pc        <= 32'd0;
            ex_rs1       <= 32'd0;
            ex_rs2       <= 32'd0;
            ex_imm       <= 32'd0;
            ex_rd        <= 5'd0;
            ex_opcode    <= 7'd0;
            ex_imm11_5   <= 7'd0;
            ex_fn3       <= 3'd0;
            ex_aluop     <= 3'd0;
            ex_fn7_5     <= 1'b0;
            ex_memtoreg  <= 2'd0;
            ex_branch    <= 1'b0;
            ex_mem_write <= 1'b0;
            ex_mem_read  <= 1'b0;
            ex_alu_src   <= 1'b0;
            ex_mux_inp   <= 1'b0;
            ex_reg_write <= 1'b0;
        end else if (redirect) begin
            // taken branch/jump in stage 2: squash the wrong-path instruction
            ex_valid     <= 1'b0;
            ex_pc        <= 32'd0;
            ex_rs1       <= 32'd0;
            ex_rs2       <= 32'd0;
            ex_imm       <= 32'd0;
            ex_rd        <= 5'd0;
            ex_opcode    <= 7'd0;
            ex_imm11_5   <= 7'd0;
            ex_fn3       <= 3'd0;
            ex_aluop     <= 3'd0;
            ex_fn7_5     <= 1'b0;
            ex_memtoreg  <= 2'd0;
            ex_branch    <= 1'b0;
            ex_mem_write <= 1'b0;
            ex_mem_read  <= 1'b0;
            ex_alu_src   <= 1'b0;
            ex_mux_inp   <= 1'b0;
            ex_reg_write <= 1'b0;
        end else begin
            ex_valid     <= 1'b1;
            ex_pc        <= address;
            ex_rs1       <= id_rs1_val;
            ex_rs2       <= id_rs2_val;
            ex_imm       <= id_imm;
            ex_rd        <= id_rd_sel;
            ex_opcode    <= id_opcode;
            ex_imm11_5   <= id_imm11_5;
            ex_fn3       <= id_fn3;
            ex_aluop     <= id_aluop;
            ex_fn7_5     <= id_fn7_5;
            ex_memtoreg  <= id_memtoreg;
            ex_branch    <= id_branch;
            ex_mem_write <= id_mem_write;
            ex_mem_read  <= id_mem_read;
            ex_alu_src   <= id_alu_src;
            ex_mux_inp   <= id_mux_inp;
            ex_reg_write <= id_reg_write;
        end
    end

    // ==================================================================
    // Stage 2: EX + MEM + WB
    // ==================================================================
    ex_stage u_ex_stage (
        .rs1_data(ex_rs1),
        .rs2_data(ex_rs2),
        .imm_out(ex_imm),
        .address(ex_pc),            // PC of the instruction in stage 2
        .branch(ex_branch),
        .alu_src(ex_alu_src),
        .fn7_5(ex_fn7_5),
        .mux_inp(ex_mux_inp),
        .fn3(ex_fn3),
        .imm11_5(ex_imm11_5),
        .aluop(ex_aluop),
        .opcode(ex_opcode),
        .and_out_ex(and_out_ex),
        .alu_out(alu_out),
        .pc_ex_out(pc_ex_out)
    );

    assign redirect = ex_valid && and_out_ex;

    // Data memory interface (memories are outside the core)
    assign rs2_data  = ex_rs2;
    assign mem_write = ex_valid && ex_mem_write;
    assign mem_read  = ex_valid && ex_mem_read;
    assign fn3       = ex_fn3;

    // Write back. return_addr is PC+4 of the jump in stage 2 (not the fetch PC+4).
    wb_stage u_wb_stage (
        .mem_out(mem_out),
        .alu_out(alu_out),
        .return_addr(ex_pc + 32'd4),
        .imm_out(ex_imm),
        .pc_signed_offset(pc_ex_out),
        .memtoreg(ex_memtoreg),
        .opcode_out_d(ex_opcode),
        .wb_data(wb_data)
    );

    assign wb_en = ex_valid && ex_reg_write && (ex_rd != 5'd0);
    assign wb_rd = ex_rd;

endmodule
