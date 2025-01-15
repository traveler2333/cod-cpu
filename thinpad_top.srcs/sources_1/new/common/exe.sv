module EXE(
    input wire clk,
    input wire rst,
    input wire stall,

    input wire [4:0] inst_type_i, //指令类型
    output reg [4:0] inst_type_o, //指令类型
    input wire [31:0] pc_i, //pc
    input wire [31:0] imm_i, //立即数
    //alu相关 
    // input wire [31:0] operand1_i,//操作数1 a
    // input wire [31:0] operand2_i,//操作数2 b
    input wire [3:0] alu_op_i,//alu操作码

    //真正给alu的输入
    output reg [31:0] alu_a,//alu输入a
    output reg [31:0] alu_b,//alu输入b
    output reg [3:0] op,//alu操作码
    //alu输出
    input wire [31:0] alu_result_i,//alu结果

    //将alu给他的值再告诉 exe_mem 寄存器
    output reg [31:0] alu_result_o, 
    output reg [31:0] wdata_o,      //写入内存的数据
    output reg [11:0] csr_addr_o,    //csr地址
    output reg [31:0] pc_o,         //pc

    //前传需求信号,都是input-----------------------
    //来自ID/EXE寄存器：
    input wire [4:0] rf_raddr_a,
    input wire [4:0] rf_raddr_b,
    input wire [31:0] rdata_a_i,
    input wire [31:0] rdata_b_i,

    //来自EXE/MEM寄存器：
    input wire [4:0] next_raddr_d,
    input wire [31:0] next_exe_mem_result,

    input wire next_mem_read,
    input wire next_rf_wen,
    //来自MEM/WB寄存器：
    input wire [31:0] mem_wb_result,//将要给rd的数据
    input wire [4:0] mem_wb_raddr_rd,//将要写入的寄存器地址   

    input wire mem_wb_rf_wen,//是否写入寄存器
    //前传需求信号-----------------------

    //分支跳转相关-----------------
    output reg [31:0] next_pc, //给pc_mux,这里是分支预测的pc
    output reg branch_en,  //是否分支, 给hazard模块，同时也给pc_mux

    //BTB 查询接口
    input wire [31:0] btb_target_exe,   // BTB 提供的目标地址
    input wire btb_hit_exe,             // BTB 是否命中

    // BTB 更新接口
    output reg update_en,              // 更新使能
    output reg [31:0] update_pc,       // 更新的 PC
    output reg [31:0] update_target,   // 更新的目标地址
    output reg update_valid            // 更新有效位
    //分支跳转相关-----------------
);
localparam XLEN = 32;

typedef enum logic [4:0] {   
    ADD   = 0,  
    ADDI  = 1,   
    AND   = 2,  
    ANDI  = 3,  
    AUIPC = 4,  
    BEQ   = 5,  
    BNE   = 6,   
    JAL   = 7,  
    JALR  = 8,  
    LB    = 9,  
    LUI   = 10,  
    LW    = 11,  
    OR    = 12,  
    ORI   = 13,  
    SB    = 14,  
    SLLI  = 15,  
    SRLI  = 16,  
    SW    = 17,  
    XOR   = 18,  
    MINU  = 19,  
    PACK  = 20,  
    SBSET = 21,
    CSRRC = 22,
    CSRRS = 23,
    CSRRW = 24,
    EBREAK= 25,
    ECALL = 26,
    MRET  = 27,
    SLTU  = 28,
    NOP   = 29,
    SHA512SUN0R = 30
} instr_code;
logic [31:0] history_data;
logic [4:0] history_rd;
logic history_rf_wen;
logic use_history;

always_ff @(posedge clk) begin
    if (rst) begin
        history_data <= 1'b0;
        history_rf_wen <= 1'b0;
        history_rd <= 1'b0;
        use_history <= 1'b0;
    end else begin
        if(stall) begin
            history_data <= mem_wb_result;
            history_rd <= mem_wb_raddr_rd;
            history_rf_wen <= mem_wb_rf_wen;
            use_history <= mem_wb_rf_wen && (mem_wb_raddr_rd != 0);
        end else begin
            history_data <= 1'b0;
            history_rd <= 1'b0;
            history_rf_wen <= 1'b0;
            use_history <= 1'b0;
        end
    end
end

reg [31:0] rdata_a, rdata_b;
always_comb begin // 数据前传
    rdata_a = rdata_a_i;
    rdata_b = rdata_b_i;

    if(use_history) begin
        if (history_rd == rf_raddr_a) begin
            rdata_a = history_data;
        end
        if (history_rd == rf_raddr_b) begin
            rdata_b = history_data;
        end
    end

    if (mem_wb_rf_wen && (mem_wb_raddr_rd != 0)) begin
        if (mem_wb_raddr_rd == rf_raddr_a) begin
            rdata_a = mem_wb_result;
        end
        if (mem_wb_raddr_rd == rf_raddr_b) begin
            rdata_b = mem_wb_result;
        end
    end

    if (next_rf_wen && (next_raddr_d != 0) && !next_mem_read) begin
        if (next_raddr_d == rf_raddr_a) begin
            rdata_a = next_exe_mem_result;
        end
        if (next_raddr_d == rf_raddr_b) begin
            rdata_b = next_exe_mem_result;
        end
    end
end

always_comb begin // 指令解码
    alu_a = 32'b0;
    alu_b = 32'b0;
    op = alu_op_i;
    wdata_o = 32'b0;
    csr_addr_o = 12'b0;
    pc_o = pc_i;
    inst_type_o = inst_type_i;
    case (inst_type_i)
        LUI: begin
            alu_a = imm_i;
        end
        AUIPC: begin
            alu_a = pc_i;
            alu_b = imm_i;
        end
        LB, LW: begin
            alu_a = rdata_a;
            alu_b = imm_i;
        end
        ADDI, SLLI, SRLI, ORI, ANDI, JALR: begin
            alu_a = rdata_a;
            alu_b = imm_i;
        end
        SB, SW: begin
            alu_a = rdata_a;
            alu_b = imm_i;
            wdata_o = rdata_b;
        end
        ADD, AND, OR, XOR: begin
            alu_a = rdata_a;
            alu_b = rdata_b;
        end
        MINU: begin
            alu_a = (rdata_a < rdata_b) ? rdata_a : rdata_b;
        end
        PACK: begin
            alu_a = rdata_a[XLEN / 2 - 1:0];
            alu_b = rdata_b << (XLEN / 2);
        end
        SBSET: begin
            alu_a = rdata_a;
            alu_b = 1 << (rdata_b & (XLEN - 1));
        end
        JAL: begin
            alu_a = pc_i;
            alu_b = imm_i;
        end
        SLTU: begin
            if (rdata_a < rdata_b) begin
                alu_a = 1;
            end else begin
                alu_a = 0;
            end
            alu_b = 0;
        end
        CSRRC, CSRRS, CSRRW: begin
            csr_addr_o = imm_i[11:0];
            alu_a = rdata_a;
        end
        EBREAK, ECALL, MRET: begin

        end
        SHA512SUN0R: begin
        //     X(rd) = ((X(rs1) << 25) ^ (X(rs1) << 30) ^ (X(rs1) >> 28) ^
        // (X(rs2) >>  7) ^ (X(rs2) >>  2) ^ (X(rs2) <<  4) )
            alu_a = (rdata_a << 25) ^ (rdata_a << 30) ^ (rdata_a >> 28) ^ (rdata_b >> 7) ^ (rdata_b >> 2) ^ (rdata_b << 4); 
        end
    endcase
end

logic [31:0] target_pc;
logic branch_taken;

always_comb begin // 分支跳转
    alu_result_o = alu_result_i;
    // 初始化分支跳转信号
    next_pc = pc_i + 4;
    branch_en = 1'b0;

    update_en = 1'b0;
    update_pc = 32'b0;
    update_target = 32'b0;
    update_valid = 1'b0;
    
    target_pc = pc_i + 4;
    branch_taken = 0;
    // 分支和跳转指令处理
    case (inst_type_i)
        BEQ, BNE: begin
            // 计算分支条件
            if (inst_type_i == BEQ) begin
                branch_taken = (rdata_a == rdata_b);
            end else begin
                branch_taken = (rdata_a != rdata_b);
            end

            // 计算目标地址
            target_pc = pc_i + imm_i;

            // 检查BTB预测是否正确
            if (branch_taken==btb_hit_exe) begin
                // 预测正确，不需要改变PC
                branch_en = 1'b0;
            end else begin
                // 预测错误，更新PC
                next_pc = branch_taken ? target_pc : (pc_i + 4);
                branch_en = 1'b1;
                // 更新BTB
                // update_en = 1'b1;
                // update_pc = pc_i;
                // update_target = target_pc;
                // update_valid = branch_taken;           
            end
        end
        JAL: begin
            // 计算目标地址
            target_pc = alu_result_i;
            alu_result_o = pc_i+4;
            // 跳转
            next_pc = target_pc;
            branch_en = 1'b1;

        end
        JALR: begin
            // 计算目标地址
            target_pc = alu_result_i;
            alu_result_o = pc_i+4;
            // 跳转
            next_pc = target_pc;
            branch_en = 1'b1;
        end
        default: begin
            // 其他指令，不需处理
        end
    endcase
end



endmodule