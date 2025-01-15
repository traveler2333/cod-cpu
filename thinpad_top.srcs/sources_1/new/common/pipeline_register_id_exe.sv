module pipline_register_id_exe(
    input wire clk,
    input wire rst,
    input wire stall_i, // 流水线暂停信号，来自hazard模块
    input wire nop_i, // nop信号，来自hazard模块

    // beq相关信号：
    input wire [4:0] inst_type_i,
    output reg [4:0] inst_type_o,
    input wire [31:0] pc_i,
    output reg [31:0] pc_o,
    input wire [31:0] imm_i,
    output reg [31:0] imm_o,
    // alu相关信号：
    input wire [ 3:0] alu_op_i,

    output reg [3:0] alu_op_o,

    // 前传相关信号（当然，我们决定在EXE中实现前传模块，因此还是给EXE的）：
    input wire [31:0] rdata_a_i,
    input wire [31:0] rdata_b_i,

    output reg [31:0] rdata_a_o,
    output reg [31:0] rdata_b_o,

    input wire [4:0] raddr_a_i,
    input wire [4:0] raddr_b_i,
    input wire [4:0] raddr_d_i,

    output reg [4:0] raddr_a_o,
    output reg [4:0] raddr_b_o,
    output reg [4:0] raddr_d_o,

    // 控制信号：
    input wire mem_read_i,
    input wire mem_we_i,
    input wire [1:0] mem_size_i,
    input wire rf_wen_i,

    output reg mem_read_o,
    output reg mem_we_o,
    output reg [1:0] mem_size_o,
    output reg rf_wen_o,

    // exception_code和page_fault
    input wire [3:0] exception_code_i,
    input wire page_fault_i,
    output reg [3:0] exception_code_o,
    output reg page_fault_o
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rdata_a_o <= 32'b0;
        rdata_b_o <= 32'b0;
        alu_op_o <= 4'b0;
        pc_o <= 32'b0;
        inst_type_o <= 5'b0;
        imm_o <= 32'b0;
        raddr_a_o <= 5'b0;
        raddr_b_o <= 5'b0;
        raddr_d_o <= 5'b0;
        mem_read_o <= 1'b0;
        mem_we_o <= 1'b0;
        mem_size_o <= 2'b0;
        rf_wen_o <= 1'b0;
        exception_code_o <= 4'b0;
        page_fault_o <= 1'b0;
    end else if (nop_i) begin
        rdata_a_o <= 32'b0;
        rdata_b_o <= 32'b0;
        alu_op_o <= 4'b0;
        pc_o <= 32'b0;
        inst_type_o <= 5'b0;
        imm_o <= 32'b0;
        raddr_a_o <= 5'b0;
        raddr_b_o <= 5'b0;
        raddr_d_o <= 5'b0;
        mem_read_o <= 1'b0;
        mem_we_o <= 1'b0;
        mem_size_o <= 2'b0;
        rf_wen_o <= 1'b0;
        exception_code_o <= 4'b0;
        page_fault_o <= 1'b0;
    end else if (!stall_i) begin
        rdata_a_o <= rdata_a_i;
        rdata_b_o <= rdata_b_i;
        alu_op_o <= alu_op_i;
        pc_o <= pc_i;
        inst_type_o <= inst_type_i;
        imm_o <= imm_i;
        raddr_a_o <= raddr_a_i;
        raddr_b_o <= raddr_b_i;
        raddr_d_o <= raddr_d_i;
        mem_read_o <= mem_read_i;
        mem_we_o <= mem_we_i;
        mem_size_o <= mem_size_i;
        rf_wen_o <= rf_wen_i;
        exception_code_o <= exception_code_i;
        page_fault_o <= page_fault_i;
    end
end

endmodule