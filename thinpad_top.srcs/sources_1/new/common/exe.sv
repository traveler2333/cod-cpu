module EXE(
    input wire clk,
    input wire rst,
    input wire stall,

    input wire [4:0] inst_type_i, //ָ������
    output reg [4:0] inst_type_o, //ָ������
    input wire [31:0] pc_i, //pc
    input wire [31:0] imm_i, //������
    //alu��� 
    // input wire [31:0] operand1_i,//������1 a
    // input wire [31:0] operand2_i,//������2 b
    input wire [3:0] alu_op_i,//alu������

    //������alu������
    output reg [31:0] alu_a,//alu����a
    output reg [31:0] alu_b,//alu����b
    output reg [3:0] op,//alu������
    //alu���
    input wire [31:0] alu_result_i,//alu���

    //��alu������ֵ�ٸ��� exe_mem �Ĵ���
    output reg [31:0] alu_result_o, 
    output reg [31:0] wdata_o,      //д���ڴ������
    output reg [11:0] csr_addr_o,    //csr��ַ
    output reg [31:0] pc_o,         //pc

    //ǰ�������ź�,����input-----------------------
    //����ID/EXE�Ĵ�����
    input wire [4:0] rf_raddr_a,
    input wire [4:0] rf_raddr_b,
    input wire [31:0] rdata_a_i,
    input wire [31:0] rdata_b_i,

    //����EXE/MEM�Ĵ�����
    input wire [4:0] next_raddr_d,
    input wire [31:0] next_exe_mem_result,

    input wire next_mem_read,
    input wire next_rf_wen,
    //����MEM/WB�Ĵ�����
    input wire [31:0] mem_wb_result,//��Ҫ��rd������
    input wire [4:0] mem_wb_raddr_rd,//��Ҫд��ļĴ�����ַ   

    input wire mem_wb_rf_wen,//�Ƿ�д��Ĵ���
    //ǰ�������ź�-----------------------

    //��֧��ת���-----------------
    output reg [31:0] next_pc, //��pc_mux,�����Ƿ�֧Ԥ���pc
    output reg branch_en,  //�Ƿ��֧, ��hazardģ�飬ͬʱҲ��pc_mux

    //BTB ��ѯ�ӿ�
    input wire [31:0] btb_target_exe,   // BTB �ṩ��Ŀ���ַ
    input wire btb_hit_exe,             // BTB �Ƿ�����

    // BTB ���½ӿ�
    output reg update_en,              // ����ʹ��
    output reg [31:0] update_pc,       // ���µ� PC
    output reg [31:0] update_target,   // ���µ�Ŀ���ַ
    output reg update_valid            // ������Чλ
    //��֧��ת���-----------------
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
always_comb begin // ����ǰ��
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

always_comb begin // ָ�����
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

always_comb begin // ��֧��ת
    alu_result_o = alu_result_i;
    // ��ʼ����֧��ת�ź�
    next_pc = pc_i + 4;
    branch_en = 1'b0;

    update_en = 1'b0;
    update_pc = 32'b0;
    update_target = 32'b0;
    update_valid = 1'b0;
    
    target_pc = pc_i + 4;
    branch_taken = 0;
    // ��֧����תָ���
    case (inst_type_i)
        BEQ, BNE: begin
            // �����֧����
            if (inst_type_i == BEQ) begin
                branch_taken = (rdata_a == rdata_b);
            end else begin
                branch_taken = (rdata_a != rdata_b);
            end

            // ����Ŀ���ַ
            target_pc = pc_i + imm_i;

            // ���BTBԤ���Ƿ���ȷ
            if (branch_taken==btb_hit_exe) begin
                // Ԥ����ȷ������Ҫ�ı�PC
                branch_en = 1'b0;
            end else begin
                // Ԥ����󣬸���PC
                next_pc = branch_taken ? target_pc : (pc_i + 4);
                branch_en = 1'b1;
                // ����BTB
                // update_en = 1'b1;
                // update_pc = pc_i;
                // update_target = target_pc;
                // update_valid = branch_taken;           
            end
        end
        JAL: begin
            // ����Ŀ���ַ
            target_pc = alu_result_i;
            alu_result_o = pc_i+4;
            // ��ת
            next_pc = target_pc;
            branch_en = 1'b1;

        end
        JALR: begin
            // ����Ŀ���ַ
            target_pc = alu_result_i;
            alu_result_o = pc_i+4;
            // ��ת
            next_pc = target_pc;
            branch_en = 1'b1;
        end
        default: begin
            // ����ָ����账��
        end
    endcase
end



endmodule