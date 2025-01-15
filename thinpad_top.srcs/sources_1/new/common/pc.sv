module PC_MUX(  
    input wire clk,                    // ϵͳʱ���ź�  
    input wire reset,                  // �첽��λ�ź�  
    input wire branch_en,              // beq�������֧�Ƿ񱻲���  
    input wire [31:0] branch_addr,     // ��֧Ŀ���ַ

    // ���� BTB ���ź�
    input wire [31:0] btb_target,      // BTB �ṩ��Ŀ���ַ
    input wire btb_hit,                // BTB �Ƿ�����

    // ����hazard���쳣��ת�ź�
    input wire error_branch_en,        // �쳣��ת�ź�
    input wire [31:0] error_branch_target,// �쳣��ת��ַ

    input wire stall,                  // ��ˮ����ͣ�ź�
    output reg [31:0] pc               // ��ǰ�ĳ��������ֵ  
);  
//�ڲ�����pc+4, ���շ�֧Ԥ����ѡ��pc+4���߷�֧Ŀ���ַ

logic [31:0] next_pc;
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        pc <= 32'h8000_0000;
        next_pc <= 32'h8000_0004;
    end else begin
        if (error_branch_en) begin
            next_pc <= error_branch_target;
        end else if (branch_en) begin
            next_pc <= branch_addr;
        end else if (btb_hit) begin
            next_pc <= btb_target;
        end

        if (stall) begin
            pc <= pc;
        end else if (error_branch_en) begin
            pc <= error_branch_target;
            next_pc <= error_branch_target+4;
        end else if(branch_en)begin
            pc <= branch_addr;
            next_pc <= branch_addr+4;
        end else if(btb_hit) begin
            pc <= btb_target;
            next_pc <= btb_target+4;
        end else begin
            pc <= next_pc;
            next_pc <= next_pc+4;
        end
    end
end

endmodule