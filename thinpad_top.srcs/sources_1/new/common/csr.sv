module Csr(
    input wire clk,
    input wire rst,

    input wire [4:0] inst_type,     // ָ������
    input wire [31:0] data_i,       // д������ݻ�pc
    input wire [11:0] csr_addr_i,   // csr��ַ
    output reg [31:0] data_o,        // ��ȡ������

    output reg time_interrupt,      // ʱ���ж�
    input wire timeout,             // ʱ���ж�
    input wire page_fault,          // ҳ����

    output reg [1:0] mode,          // ģʽ
    output reg [31:0] page_table_base  // ҳ���ַ
);
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
        NOP   = 29
    } instr_code;

    reg [31:0] mtvec;       // 0x305,�ж�������ַ�Ĵ�����MODE = mtvec[1:0]��BASE = mtvec[31:2]
    reg [31:0] mscratch;    // 0x340,����ģʽ�µ���ʱ�Ĵ���
    reg [31:0] mepc;        // 0x341,����ģʽ�µ��쳣���������
    reg [31:0] mcause;      // 0x342,����ģʽ�µ��쳣ԭ��Interrupt = 0, Exception = 1��Exception Code = mcause[30:0]
    reg [31:0] mstatus;     // 0x300,����ģʽ�µ�״̬�Ĵ�����MPP = mstatus[12:11]
    reg [31:0] mie;         // 0x304,����ģʽ�µ��ж�ʹ�ܣ�MTIE = mie[7]
    reg [31:0] mip;         // 0x344,����ģʽ�µ��жϹ���MTIP = mip[7]
    reg [31:0] satp;        // 0x180,ҳ���ַ�Ĵ���

    assign time_interrupt = mie[7] & !mip[7] & timeout;
    assign mode = mstatus[12:11];
    assign page_table_base = satp;

    always_comb begin
        data_o = 32'b0;
        if (time_interrupt | page_fault) begin
            data_o = 32'({mtvec[31:2], 2'b00});
        end else begin
            case (inst_type)
                CSRRC, CSRRS, CSRRW: begin
                    case (csr_addr_i)
                        12'h305: data_o = mtvec;
                        12'h340: data_o = mscratch;
                        12'h341: data_o = mepc;
                        12'h342: data_o = mcause;
                        12'h300: data_o = mstatus;
                        12'h304: data_o = mie;
                        12'h344: data_o = mip;
                        12'h180: data_o = satp;
                    endcase
                end
                ECALL, EBREAK: begin
                    data_o = 32'({mtvec[31:2], 2'b00});
                end
                MRET: begin
                    data_o = mepc;
                end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            mtvec <= 32'b0;
            mscratch <= 32'b0;
            mepc <= 32'b0;
            mcause <= 32'b0;
            mstatus <= 32'h00001800;
            mie <= 32'h80;
            mip <= 32'h80;
            satp <= 32'b0;
        end else if (time_interrupt) begin  // ʱ���жϣ������жϴ����������MTIP
            mcause <= 32'h80000007;
            mepc <= data_i;
            mip[7] <= 1;
        end else if (page_fault) begin  // ҳ���󣬴����жϴ����������MTIP
            mcause <= 32'h0000000D;
            mepc <= data_i;
        end else begin
            if (!timeout) begin
                mip[7] <= 0;
            end
            case (inst_type)
                CSRRC: begin
                    case (csr_addr_i)
                        12'h305: mtvec <= mtvec & ~data_i;
                        12'h340: mscratch <= mscratch & ~data_i;
                        12'h341: mepc <= mepc & ~data_i;
                        12'h342: mcause <= mcause & ~data_i;
                        12'h300: mstatus <= mstatus & ~data_i;
                        12'h304: mie <= mie & ~data_i;
                        12'h344: mip <= mip & ~data_i;
                        12'h180: satp <= satp & ~data_i;
                    endcase
                end
                CSRRS: begin
                    case (csr_addr_i)
                        12'h305: mtvec <= mtvec | data_i;
                        12'h340: mscratch <= mscratch | data_i;
                        12'h341: mepc <= mepc | data_i;
                        12'h342: mcause <= mcause | data_i;
                        12'h300: mstatus <= mstatus | data_i;
                        12'h304: mie <= mie | data_i;
                        12'h344: mip <= mip | data_i;
                        12'h180: satp <= satp | data_i;
                    endcase
                end
                CSRRW: begin
                    case (csr_addr_i)
                        12'h305: mtvec <= data_i;
                        12'h340: mscratch <= data_i;
                        12'h341: mepc <= data_i;
                        12'h342: mcause <= data_i;
                        12'h300: mstatus <= data_i;
                        12'h304: mie <= data_i;
                        12'h344: mip <= data_i;
                        12'h180: satp <= data_i;
                    endcase
                end
                ECALL: begin
                    mcause <= 32'({8 + mstatus[12:11]});
                    mepc <= data_i;
                end
                EBREAK: begin
                    mcause <= {30'b0, 2'b11};
                    mepc <= data_i;
                end
                MRET: begin

                end
            endcase
        end
    end

endmodule