module ID(
    //输入,来自于IF/ID寄存器,指令和pc
    input wire [31:0] instr,
    input wire [31:0] pc,

    //与regfile交互（读），组合逻辑
    output reg [4:0] rf_raddr_a,
    output reg [4:0] rf_raddr_b,
    input wire [31:0] rf_rdata_a_i, //读取的数据a
    input wire [31:0] rf_rdata_b_i, //读取的数据b
    
    //向ID/EXE寄存器写入的数据
    output reg [4:0] inst_type_o,//指令类型
    output reg [31:0] imm_o,//立即数

    // output reg [31:0] operand1,//操作数1,根据指令类型，可能是寄存器数据，也可能是立即数
    // output reg [31:0] operand2,//操作数2
    // output reg [31:0] id_wdata, //需要写入内存的数据

    output reg [31:0] rdata_a,
    output reg [31:0] rdata_b,
    output reg [3:0] alu_op, //alu操作类型

    output reg [4:0] raddr_a, //写入寄存器地址
    output reg [4:0] raddr_b, //写入寄存器地址
    output reg [4:0] raddr_rd, //写入寄存器地址

    //控制信号，但是可以被nop清0
    output reg mem_read,//是否读内存
    output reg mem_we,//是否写内存
    output reg [1:0] mem_size,//内存操作的数据大小
    output reg rf_wen//是否启用寄存器写入（写入rd）
);
//实现对regfile的组合逻辑读取
//实现立即数的提取
//实现对alu_op的选择
//实现对mem_read/mem_we/mem_size的选择
//实现对rf_wen的选择
//实现对next_pc（跳转值，pc+imm）的计算
//在beq时，实现对a\b的比较，并给出分支预测的pc、是否分支的信号
//在jalr时，给出jalr的pc
localparam XLEN = 32;

typedef enum logic [3:0] {
    ALU_ADD = 4'b0001,
    ALU_SUB = 4'b0010,
    ALU_AND = 4'b0011,
    ALU_OR  = 4'b0100,
    ALU_XOR = 4'b0101,
    ALU_NOT = 4'b0110,
    ALU_SLL = 4'b0111,
    ALU_SRL = 4'b1000,
    ALU_SRA = 4'b1001,
    ALU_ROL = 4'b1010
} alu_op_t;

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

instr_code inst_type;
logic [31:0] imm;

assign inst_type_o = inst_type;
assign imm_o = imm;
assign raddr_a = rf_raddr_a;
assign raddr_b = rf_raddr_b;

always_comb begin
    inst_type = NOP;
    rf_raddr_a = 5'b00000;
    rf_raddr_b = 5'b00000;
    alu_op = 4'b0000;
    rdata_a = 32'b0;
    rdata_b = 32'b0;
    imm = 0;
    raddr_rd = 5'b00000;
    mem_read = 1'b0;
    mem_we = 1'b0;
    mem_size = 2'b00;
    rf_wen = 1'b0;
    case (instr[6:0])   // opcode
        7'b0110111: begin   // U-type lui
            inst_type = LUI;
            imm = {instr[31:12], 12'b0};
            raddr_rd = instr[11:7];
            alu_op = ALU_ADD;
            rf_wen = 1;
        end
        7'b0010111: begin   // U-type auipc
            inst_type = AUIPC;
            imm = {instr[31:12], 12'b0};
            raddr_rd = instr[11:7];
            alu_op = ALU_ADD;
            rf_wen = 1;
        end
        7'b1100011: begin   // B-type
            case (instr[14:12])
                3'b000: inst_type = BEQ;
                3'b001: inst_type = BNE;
            endcase
            imm = 32'(signed'({instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}));
            rf_raddr_a = instr[19:15];
            rf_raddr_b = instr[24:20];
            rdata_a = rf_rdata_a_i;
            rdata_b = rf_rdata_b_i;
            alu_op = ALU_SUB;
        end
        7'b0000011: begin   // I-type load
            case (instr[14:12])
                3'b000: inst_type = LB;
                3'b010: inst_type = LW;
            endcase
            imm = 32'(signed'(instr[31:20]));
            rf_raddr_a = instr[19:15];
            raddr_rd = instr[11:7];
            rdata_a = rf_rdata_a_i;
            alu_op = ALU_ADD;
            mem_read = 1;
            case (inst_type)
                LB: mem_size = 2'b00;
                LW: mem_size = 2'b10;
            endcase
            rf_wen = 1;
        end
        7'b0010011: begin   // I-type arith
            case (instr[14:12])
                3'b000: inst_type = ADDI;
                3'b001: inst_type = SLLI;
                3'b101: inst_type = SRLI;
                3'b110: inst_type = ORI;
                3'b111: inst_type = ANDI;
            endcase
            imm = 32'(signed'(instr[31:20]));
            rf_raddr_a = instr[19:15];
            raddr_rd = instr[11:7];
            rdata_a = rf_rdata_a_i;
            case (inst_type)
                ADDI: alu_op = ALU_ADD;
                SLLI: alu_op = ALU_SLL;
                SRLI: alu_op = ALU_SRL;
                ORI:  alu_op = ALU_OR;
                ANDI: alu_op = ALU_AND;
            endcase
            rf_wen = 1;
        end
        7'b1100111: begin   // I-type jalr
            inst_type = JALR;
            imm = 32'(signed'(instr[31:20]));
            rf_raddr_a = instr[19:15];
            raddr_rd = instr[11:7];
            rdata_a = rf_rdata_a_i;
            alu_op = ALU_ADD;
            rf_wen = 1;
        end
        7'b0100011: begin   // S-type
            case (instr[14:12])
                3'b000: inst_type = SB;
                3'b010: inst_type = SW;
            endcase
            imm = 32'(signed'({instr[31:25], instr[11:7]}));
            rf_raddr_a = instr[19:15];
            rf_raddr_b = instr[24:20];
            rdata_a = rf_rdata_a_i;
            rdata_b = rf_rdata_b_i;
            alu_op = ALU_ADD;
            mem_we = 1;
            case (inst_type)
                SB: mem_size = 2'b00;
                SW: mem_size = 2'b10;
            endcase
        end
        7'b0110011: begin   // R-type
            case (instr[31:25])
                7'b0000000: begin
                    case (instr[14:12])
                        3'b000: inst_type = ADD;
                        3'b011: inst_type = SLTU;
                        3'b100: inst_type = XOR;
                        3'b110: inst_type = OR;
                        3'b111: inst_type = AND;
                    endcase
                end
                7'b0000101: begin
                    case (instr[14:12])
                        3'b110: inst_type = MINU;
                    endcase
                end 
                7'b0000100: begin
                    case (instr[14:12])
                        3'b100: inst_type = PACK;
                    endcase
                end
                7'b0010100: begin
                    case (instr[14:12])
                        3'b001: inst_type = SBSET;
                    endcase
                end
                7'b0101000: begin
                    case (instr[14:12])
                        3'b000: inst_type = SHA512SUN0R;
                    endcase
                end
            endcase
            rf_raddr_a = instr[19:15];
            rf_raddr_b = instr[24:20];
            raddr_rd = instr[11:7];
            rdata_a = rf_rdata_a_i;
            rdata_b = rf_rdata_b_i;
            case (inst_type)
                MINU: alu_op = ALU_ADD;
                PACK: alu_op = ALU_OR;
                SBSET: alu_op = ALU_OR;
                ADD: alu_op = ALU_ADD;
                XOR: alu_op = ALU_XOR;
                OR:  alu_op = ALU_OR;
                AND: alu_op = ALU_AND;
                SLTU: alu_op = ALU_ADD;
                SHA512SUN0R: alu_op = ALU_ADD;
                default: alu_op = ALU_ADD;
            endcase
            rf_wen = 1;
        end
        7'b1101111: begin   // J-type
            inst_type = JAL;
            imm = 32'(signed'({instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}));
            raddr_rd = instr[11:7];
            alu_op = ALU_ADD;
            rf_wen = 1;
        end
        7'b1110011: begin   // I-type csr
            case (instr[14:12])
                3'b001: inst_type = CSRRW;
                3'b010: inst_type = CSRRS;
                3'b011: inst_type = CSRRC;
                3'b000: begin
                    case (instr[31:20])
                        12'b000000000000: inst_type = ECALL;
                        12'b000000000001: inst_type = EBREAK;
                        12'b001100000010: inst_type = MRET;
                    endcase
                end
            endcase
            imm = {20'b0, instr[31:20]}; // csr_addr
            rf_raddr_a = instr[19:15];
            raddr_rd = instr[11:7];
            rdata_a = rf_rdata_a_i;
            alu_op = ALU_ADD;
            case (inst_type)
                CSRRW, CSRRS, CSRRC: begin
                    rf_wen = 1;
                    mem_read = 1;
                end
            endcase
        end
    endcase
end

endmodule